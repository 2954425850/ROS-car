# arm_grasp/grasp.py
# -*- coding: utf-8 -*-
"""抓取控制层：估计器 + 目标/姿态生成 + 相位机 + 10Hz 限速流式参考。**不碰硬件**。

设计见 `计划文档/【有效】2026-09-30-抓取全流程-design.md`。与 09-29 `servo.py` 的
根本差别只有一条，但它是根本的：

    目标点  T = O + s·(−ẑ_T(α))      ← 爪尖在**过目标点的下扎轴上**
                               ↑ 不再要求"爪尖落在视线（相机→目标那条线）上"

那条视线约束把臂锁在一条一维曲线上，是肘关节顶死固件限位 125 的根因（design §2.1）。
相机只要"看得见"目标，不再要求目标落在某个像素。

I/O 全部由调用方注入（`Link` 协议）：测试注入假臂，工具注入 ROS + K230。
"""
import math
from collections import namedtuple

from . import geom, ltrace
from .arm_kin import FIELD_HI, FIELD_LO, Unreachable, from_fields, to_fields
from .servo import (ServoRefused as Refused, field_slack, fields_in_range,
                    ik_open_m, limit_step, tip_open_m)


def _refuse(msg):
    return Refused(msg)


Obs = namedtuple('Obs', 't_arrive u_ai v_ai frame src box_norm')
Vis = namedtuple('Vis', 'u_lo u_hi v_lo v_hi')
Target = namedtuple('Target', 'alpha s_m joints fields slack err_m visible upx vpx')

GraspConfig = namedtuple('GraspConfig',
    'alpha_lo alpha_hi alpha_span alpha_step alpha0 '   # α 搜索
    'min_slack warn_slack slack_tie '                   # S0 三级 + 并列宽容
    'gripper wrist_roll '                               # p1/p2（张开 240 / 出厂 496）
    'vis '                                              # 看得见区域（AI 帧像素）
    's_pre_m s_stop_m s_min_step s_frac '               # 留量与下扎
    'max_step_deg hz max_seconds max_ticks '            # 流式
    'tol_m tol_px_err patience min_gain_m '             # 判据
    'z_plane_m latency_s k_ewma gate_m '                # 尺度 / 时间 / 估计器
    'close_field lift_m')
GraspConfig.__new__.__defaults__ = (
    -88.0, -45.0, 14.0, 1.0, -58.0,
    40.0, 10.0, 5.0,
    240.0, 496.0,
    Vis(0.08 * 320.0, 0.92 * 320.0, 0.12 * 180.0, 0.86 * 180.0),
    # s_pre_m = 8cm 的来由（2026-09-30 实测 + 一条已证明的性质，见 pick_target docstring）：
    #   目标投影像素 v(s) = cy + fy·|dy|/(|dz|+s) = 97.1 + 9.394/(0.119+s) px —— **只由留量 s 决定**，
    #   与目标在哪无关。实测：s=0 → v=177.4（瞄准像素）；5cm → 153.3；8cm → 144.8；10cm → 140.4。
    #   跟踪器可锁带 v ≤ 160.7 ⇒ **目标中心在 s ≥ 2.9cm 时才可锁** ⇒ 末段 2.9cm 必然无观测
    #   （几何决定，改 α 没用；只有改相机安装才能动这条）。
    #   s_pre 的取舍：越大 → 目标在画面里越靠上（跟踪余量越大），但**接近姿态的余量越小**：
    #     5cm → slack 77 但 v=153（可锁带只剩 8px）；8cm → slack 58、v=145（16px）；10cm → slack 35。
    #   取 8cm：跟踪余量够，且 slack 仍在 min_slack(40) 之上。
    0.08, 0.003, 0.002, 0.30,
    1.5, 10.0, 40.0, 400,
    0.002, 10.0, 4, 0.0005,
    0.020, 0.040, 0.35, 0.03,
    578.0, 0.04,
)


# --------------------------------------------------------------------------
# 目标 / 姿态
# --------------------------------------------------------------------------

def axis_point(O_m, s_m, alpha_deg):
    """过 O、沿下扎轴 ẑ_T(α) 退 s 的那一点（米）。s=0 就是 O 本身。

    ⚠️ ẑ_T 的水平方向取决于**爪尖自己的方位角**，不是目标点的方位角（两者差
    atan(s·cosα / r)，s=5cm、r=20cm 时约 7.8°，不是小量）⇒ 迭代一次即可收敛。
    """
    tb = math.atan2(O_m[1], O_m[0])
    for _ in range(2):
        a = math.radians(alpha_deg)
        ux, uy, uz = (math.cos(a) * math.cos(tb),
                      math.cos(a) * math.sin(tb), math.sin(a))
        T = (O_m[0] - s_m * ux, O_m[1] - s_m * uy, O_m[2] - s_m * uz)
        tb = math.atan2(T[1], T[0])
    return T


def visible(joints, O_m, vis):
    """此刻相机还看得见 O 吗。返回 (ok, (u_ai, v_ai))。"""
    u, v = geom.project(joints, O_m)
    return (vis.u_lo <= u <= vis.u_hi and vis.v_lo <= v <= vis.v_hi), (u, v)


def pick_target(O_m, cfg, s_m, prefer_alpha, vis=None):
    """在 α 上搜一个把**张开态爪尖**放到 `axis_point(O, s, α)` 的可行姿态。

    硬门：p3/p4/p5 ∈ [125,875]。排序：余量最大 → α 最接近 prefer_alpha（连续性，避免每拍换解）。

    ★★ **"看得见"不能当选择判据** —— 2026-09-30 由变异自检证明，**别再加回来**：
       候选之间 `visible` **恒相同**。证明：`ik_open_m` 恰好把爪尖放在过 O 的下扎轴上，于是
       `O − C = (O − T) − 1.7cm·ẑ − R·CAM_CLOSED = (s − 0.017)·ẑ − R·CAM_CLOSED`，在**工具系**里
       = `(0, 0, s−0.017) − CAM_CLOSED` —— **只含 s，与 α 无关，也与 O 在哪儿无关**。
       ⇒ 按 α 枚举的候选里，目标投影像素是同一个 ⇒ `[c for c in cands if c.visible]` 要么全留、
       要么全空，全空时 `or cands` 又原样补回来 ⇒ **恒等变换**（穷举 5 个 O × 7 个 s 验证过）。
       推论（有用）：**目标在画面里的位置是留量 s 的纯函数** ⇒ "接近阶段看得见吗"由 `s_pre` 决定，
       与目标在哪无关；`visible/upx/vpx` 只当**诊断量**（S0 报给人看），不参与选择。
    """

    vis = cfg.vis if vis is None else vis
    cands = []
    k = 0.0
    while k <= cfg.alpha_span + 1e-9:
        for a in ((prefer_alpha,) if k == 0.0
                  else (prefer_alpha - k, prefer_alpha + k)):
            if not (cfg.alpha_lo - 1e-9 <= a <= cfg.alpha_hi + 1e-9):
                continue
            T = axis_point(O_m, s_m, a)
            try:
                j = ik_open_m(T[0], T[1], T[2], a)
            except Unreachable:
                continue
            ok, f = fields_in_range(j, cfg.gripper, cfg.wrist_roll)
            if not ok:
                continue
            vis_ok, px = visible(j, O_m, vis)
            cands.append(Target(a, s_m, j, f, field_slack(f),
                                math.dist(tip_open_m(j), O_m), vis_ok, px[0], px[1]))
        k += cfg.alpha_step
    if not cands:
        raise _refuse('α∈[%.0f°,%.0f°]、留量 %.1fcm 下无可行姿态（不可解或越界）'
                      % (cfg.alpha_lo, cfg.alpha_hi, s_m * 100))
    good = cands          # ← 别加 visible 过滤：那是恒等变换（证明见 docstring）
    top = max(c.slack for c in good)
    near = [c for c in good if c.slack >= top - cfg.slack_tie]
    near.sort(key=lambda c: (abs(c.alpha - prefer_alpha), c.alpha))
    return near[0]


def tier(slack, cfg):
    """余量三级（**纯函数，单独测**，不用去凑一个工作空间边缘的点）：
    ≥min_slack → 'ok'；warn_slack~min_slack → 'warn'；<warn_slack → 'refuse'。
    """
    if slack < cfg.warn_slack:
        return 'refuse'
    if slack < cfg.min_slack:
        return 'warn'
    return 'ok'


def plan_verdict(O_m, cfg, vis=None):
    """S0 预检：末态（爪尖就在 O）能不能干。返回 (level, Target|None, 说明)。

    level：'ok'＝余量 ≥min_slack；'warn'＝warn_slack~min_slack（照跑但告警）；
           'refuse'＝解不出或余量 <warn_slack（拒绝并报该往哪挪）。
    """
    try:
        t = pick_target(O_m, cfg, 0.0, cfg.alpha0, vis)
    except Refused as e:
        return 'refuse', None, '末态解不出：%s' % e
    # 诊断（见 pick_target docstring 的推论）：目标投影像素只由留量 s 决定 ⇒
    # "接近阶段看得见吗 / 在不在可锁带里"是 s_pre 的函数，与目标在哪无关。S0 就报出来给人看。
    try:
        tp = pick_target(O_m, cfg, cfg.s_pre_m, cfg.alpha0, vis)
        pix = '；s_pre=%.1fcm 处：预测像素 (%.0f, %.0f) %s有效区（可锁带 v ≤ %.0f）、slack %.0f' % (
            cfg.s_pre_m * 100, tp.upx, tp.vpx,
            '在' if tp.visible else '**不在**', 0.893 * 180.0, tp.slack)
    except Refused as e:
        pix = '；但 s_pre=%.1fcm 处解不出姿态：%s' % (cfg.s_pre_m * 100, e)
    lvl = tier(t.slack, cfg)
    if lvl == 'refuse':
        return ('refuse', t,
                '余量只剩 %d count（< %d）——臂已到工作空间边缘，'
                '把目标往底座方向挪 5~8cm 再来%s' % (t.slack, cfg.warn_slack, pix))
    if lvl == 'warn':
        return ('warn', t, '余量 %d count（< %d）——接近限位，成功率可能下降%s'
                % (t.slack, cfg.min_slack, pix))
    return 'ok', t, '余量 %d count%s' % (t.slack, pix)


def standoff_along_axis(tip_m, O_m, alpha_deg):
    """爪尖离目标**沿下扎轴**还有多远（m）。下扎的限速用这个（用观测+回读算，不用指令）。"""
    tb = math.atan2(O_m[1], O_m[0])
    a = math.radians(alpha_deg)
    uz = math.sin(a)
    ux = math.cos(a) * math.cos(tb)
    uy = math.cos(a) * math.sin(tb)
    return (tip_m[0] - O_m[0]) * ux + (tip_m[1] - O_m[1]) * uy + (tip_m[2] - O_m[2]) * uz
