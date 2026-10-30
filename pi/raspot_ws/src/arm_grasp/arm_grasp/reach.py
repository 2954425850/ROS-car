# -*- coding: utf-8 -*-
"""可达性：全 α 包络枚举 + 整段下扎路径余量 + 字典序选解。

这一层是**纯函数**（不碰 ROS、不碰硬件、不碰相机）。它只回答两个问题：

1. 对目标点 O，哪些 (α, 肘支) 能让**整段** `s_stop → s_pre` 的爪尖姿态都在
   固件 field 钳位内？        → `enumerate_feasible()`
2. 在这些可行解里，哪个最稳、最不跳、行程最少？  → `select()`

为什么要有它（2026-10-28）：
- 旧 `pick_target` 只在 `alpha0 ± alpha_span` 里扫，桌面目标的真解常在这个窗外
  ⇒ 假不可达；
- 旧选解只看末态（或预抓/接触两个点）的余量，不看整段下扎 ⇒ 末态可行、中途顶死；
- 旧 `find_grasp_solution` 返回第一个合法解，可能余量极薄。

选解是**字典序**，不是加权和：
    ① 整段最小余量最大（不贴限位，安全项不能被别的项换掉）
    ② α 离上一拍最近（不跳解）
    ③ 关节行程最少（少让舵机跑）
    ④ |α − prefer|、α 本身（确定性 tie-break）

注意：这里**不含**关节居中/可操作度；它们是后续可选的 tie-breaker，不能压过 ①。
"""
import math
from collections import namedtuple

from .arm_kin import Unreachable
from .servo import field_slack, fields_in_range, ik_open_m

JOINT_KEYS = ('base', 'shoulder', 'elbow', 'wrist_pitch')

# 一个可行解。slack = 在 s_m 这一点的余量；min_slack = 整段路径的最小余量。
Candidate = namedtuple('Candidate', 'alpha branch joints fields slack min_slack')


class NoFeasible(Exception):
    """全包络里没有任何整段路径可行的 (α, 肘支)。"""


def axis_point(O_m, s_m, alpha_deg):
    """过 O、沿下扎轴 ẑ_T(α) 退 s 的那一点（米）。s=0 就是 O 本身。

    ⚠️ ẑ_T 的水平方向取决于**爪尖自己的方位角**，不是目标点的方位角（两者差
    atan(s·cosα / r)，s=5cm、r=20cm 时约 7.8°，不是小量）⇒ 迭代一次即可收敛。

    与 `grasp.axis_point` 同一实现；放在这里是为了让 reach 不 import grasp
    （grasp 要 import reach，避免循环）。
    """
    tb = math.atan2(O_m[1], O_m[0])
    for _ in range(2):
        a = math.radians(alpha_deg)
        ux, uy, uz = (math.cos(a) * math.cos(tb),
                      math.cos(a) * math.sin(tb), math.sin(a))
        T = (O_m[0] - s_m * ux, O_m[1] - s_m * uy, O_m[2] - s_m * uz)
        tb = math.atan2(T[1], T[0])
    return T


def path_samples(cfg, n=9, s_lo=None, s_hi=None):
    """留量路径的采样（米），含两端。

    默认是下扎路径 `s_stop → s_pre`；**抬起相要传 `s_hi=s_stop+lift_m`** ——
    抬起只走 `s_stop → s_stop+lift`，不需要整段下扎路径都可行（2026-10-30 真机：
    用整段路径检查把抬起解误拒成 REFUSED）。
    """
    lo = float(cfg.s_stop_m if s_lo is None else s_lo)
    hi = float(cfg.s_pre_m if s_hi is None else s_hi)
    if hi <= lo or n < 2:
        return [lo]
    return [lo + i * (hi - lo) / float(n - 1) for i in range(n)]


def _candidate_at(O_m, cfg, s_m, alpha, branch):
    """算 s_m 这一点的解；不可行返回 None。"""
    T = axis_point(O_m, s_m, alpha)
    try:
        j = ik_open_m(T[0], T[1], T[2], alpha, branch=branch)
    except Unreachable:
        return None
    ok, f = fields_in_range(j, cfg.gripper, cfg.wrist_roll)
    if not ok:
        return None
    return j, f, field_slack(f)


def enumerate_feasible(O_m, cfg, s_m, alpha_lo=None, alpha_hi=None,
                       step_deg=1.0, branches=(-1, 1), s_samples=None):
    """枚举所有**整段路径**可行的 (α, 肘支)。

    参数
    ----
    O_m : 目标点（米，车体安装面系）
    cfg : GraspConfig（只用 s_pre_m / s_stop_m / gripper / wrist_roll / alpha_lo / alpha_hi）
    s_m : 当前要求的爪尖留量（米）；候选的 joints/fields/slack 是这一点的
    alpha_lo/hi : 默认取 cfg 的包络
    step_deg : α 粗扫步长（默认 1°）
    branches : 肘支，默认两支都试
    s_samples : 路径留量采样；默认 `path_samples(cfg)`（9 点）

    返回 `list[Candidate]`，按 α 升序。**不做选解**，选解交给 `select()`。
    """
    lo = float(cfg.alpha_lo if alpha_lo is None else alpha_lo)
    hi = float(cfg.alpha_hi if alpha_hi is None else alpha_hi)
    if hi < lo:
        lo, hi = hi, lo
    if s_samples is None:
        s_samples = path_samples(cfg)
    # 先把 s_m 也算进路径最小值里（s_m 可能不在 [s_stop, s_pre]，例如 plan_verdict 的 s=0）
    n = int(math.floor((hi - lo) / step_deg + 1e-9))
    out = []
    for i in range(n + 1):
        alpha = round(lo + i * step_deg, 9)   # 去掉 −62.99999999999999 这类浮点尾巴
        if alpha > hi + 1e-9:
            break
        for branch in branches:
            # 先查 s_m：不可行就没必要再查整段路径
            at_m = _candidate_at(O_m, cfg, s_m, alpha, branch)
            if at_m is None:
                continue
            j_m, f_m, slack_m = at_m
            path_slacks = []
            ok = True
            for s in s_samples:
                got = _candidate_at(O_m, cfg, s, alpha, branch)
                if got is None:
                    ok = False
                    break
                path_slacks.append(got[2])
            if not ok:
                continue
            # ★ 排序键只取**固定路径采样**的最小值，**不把 s_m 的瞬时余量并进去**：
            #   s_m 每拍都在动，并进去会让同一个 α 的排序键每拍抖一下 ⇒ 选解跟着
            #   在 −63/−64 之间来回跳（正是“跳解”）。s_m 的 field 合法性仍单独检查。
            out.append(Candidate(
                alpha=alpha, branch=branch, joints=j_m, fields=f_m,
                slack=slack_m,
                min_slack=min(path_slacks) if path_slacks else slack_m))
    return out


def feasible_alpha_intervals(cands):
    """把候选 α 折成连续区间，给拒绝信息/诊断用。返回 [[lo,hi], ...]。"""
    xs = sorted(set(c.alpha for c in cands))
    if not xs:
        return []
    out = [[xs[0], xs[0]]]
    for a in xs[1:]:
        if a - out[-1][1] <= 1.0001:      # 1° 步长下相邻
            out[-1][1] = a
        else:
            out.append([a, a])
    return out


def select(cands, cfg, alpha_prev=None, prefer_alpha=None, j_prev=None):
    """在可行解里按字典序选一个。

    ① 整段最小余量最大（`min_slack`）；只在 top − cfg.slack_tie 的并列档里比后面的项。
    ② α 离 `alpha_prev` 最近（不跳解）。
    ③ 关节行程最少（相对 `j_prev`；没给 j_prev 时该项恒 0）。
    ④ |α − prefer_alpha|、α 本身（确定性 tie-break）。

    硬门：优先只考虑 `min_slack >= cfg.warn_slack` 的候选；一个都没有时退回全部
    （保留旧的“余量薄但可先试”语义，由 `tier()` 去判 warn/refuse）。
    """
    if not cands:
        raise NoFeasible('没有任何整段路径可行的 (α, 肘支)')

    preferred = [c for c in cands if c.min_slack >= cfg.warn_slack]
    pool = preferred or list(cands)

    top = max(c.min_slack for c in pool)
    near = [c for c in pool if c.min_slack >= top - cfg.slack_tie]
    ap = prefer_alpha if alpha_prev is None else alpha_prev
    pref = prefer_alpha if prefer_alpha is not None else ap

    def travel(c):
        if j_prev is None:
            return 0.0
        return sum(abs(c.joints[k] - j_prev[k]) for k in JOINT_KEYS)

    near.sort(key=lambda c: (abs(c.alpha - ap), travel(c),
                             abs(c.alpha - pref), c.alpha))
    return near[0]
