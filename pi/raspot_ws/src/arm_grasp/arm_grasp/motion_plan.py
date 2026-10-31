# -*- coding: utf-8 -*-
"""spec + 当前姿态 → 关键帧（设计 §7）。纯函数，不 import rclpy。

    plan(spec, cur_fields, history, cfg) -> Plan(keyframes, notes, partial, history_used)

* `cur_fields`：当前回读 [p1..p6]（p1 夹爪、p2 腕旋、p3 腕俯、p4 肘、p5 肩、p6 底座）。
* `history`：执行器维护的「动作前姿态」栈（每项一组 [p1..p6]，**最后一项是最近的**）。
  本函数不改它；`Plan.history_used` 告诉调用方弹掉了几项（spec 里每出现一次「上一个姿态」用掉一项）。
* 每个 step 合成**一帧**（toward → turn → shift → approach → gripper 依次作用在同一个姿态上）；
  oscillate 单独展开成 2×times+1 帧，最后一帧回到起始姿态。`repeat` = 整串 steps 再来几遍
  （相对量按"上一遍做完的姿态"继续累加）。
* 越限一律截到极限，原因写进 notes，`partial=True`。用户本来就说「到底」的，到边上不算截断。
* p2（腕旋）原样保持。
"""
import math
from dataclasses import dataclass, field
from typing import List

from arm_grasp.arm_kin import FACTOR, Unreachable, fk, from_fields, ikine, to_fields
from arm_grasp.gaze import el_limits, gaze_of, joint_bounds, joints_ok, solve_gaze
from arm_grasp.motion_spec import MAX, TOWARD_PREV, Spec, parse_spec, resolve_amount

_JK = ('base', 'shoulder', 'elbow', 'wrist_pitch')
_TRAVEL_EPS_CM = 0.05        # 小于这个算"没挪动"
_BISECT_ITERS = 30
_SHIFT_MAX_BASE_JUMP = 90.0  # 一次平移底座角跳变超过它 ⇒ 视为穿过底座轴线，判不可行

_SHIFT_VEC = {'前': (1, 0, 0), '后': (-1, 0, 0), '左': (0, 1, 0), '右': (0, -1, 0),
              '上': (0, 0, 1), '下': (0, 0, -1)}


@dataclass
class Keyframe:
    fields: List[float]     # [p1..p6]
    t_ms: int
    label: str


@dataclass
class Plan:
    keyframes: List[Keyframe]
    notes: List[str]
    partial: bool
    history_used: int = 0


@dataclass
class _Pose:
    j: dict           # base/shoulder/elbow/wrist_pitch（度）
    p1: float
    p2: float

    def fields(self):
        return to_fields(self.j, self.p1, self.p2)

    def copy(self):
        return _Pose(dict(self.j), self.p1, self.p2)


@dataclass
class _Ctx:
    cfg: object
    history: list
    notes: List[str] = field(default_factory=list)
    partial: bool = False
    history_used: int = 0

    def note(self, text, partial=True):
        if text and text not in self.notes:
            self.notes.append(text)
        if text and partial:
            self.partial = True


def _wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def _sanitize(fields, cfg):
    """回读 → _Pose，并把略越界的关节钳回限位内（回读有 ±1 抖动）。"""
    if fields is None or len(fields) != 6:
        raise ValueError('姿态应是 6 个 field，收到 %r' % (fields,))
    j = from_fields([float(v) for v in fields])
    b = joint_bounds()
    for k, (lo, hi) in b.items():
        j[k] = min(max(j[k], lo), hi)
    j['base'] = min(max(j['base'], -cfg.base_limit_deg), cfg.base_limit_deg)
    return _Pose(j, float(fields[0]), float(fields[1]))


def nearest_equivalent_az(az, cur_base, limit):
    """绝对方位 az 的等价角（az、az±360）里，取在 ±limit 内、离当前底座角最近的（避免绕圈）。"""
    cands = [az + k * 360.0 for k in (-1, 0, 1)]
    inside = [c for c in cands if abs(c) <= limit + 1e-9]
    if not inside:          # limit < 180 时可能一个都不在，截到最近的边
        return min(max(az, -limit), limit)
    return min(inside, key=lambda c: (abs(c - cur_base), abs(c)))


def frame_t_ms(prev_fields, new_fields, speed_dps, min_ms):
    """t_ms = max(min_ms, max|Δ关节°| / 档速 × 1000)。夹爪/腕旋也是 LX 舵机，按 240°/1000 折成度。"""
    a, b = from_fields(prev_fields), from_fields(new_fields)
    d = [abs(a[k] - b[k]) for k in _JK]
    d += [abs(prev_fields[i] - new_fields[i]) / FACTOR for i in (0, 1)]
    return max(int(min_ms), int(math.ceil(max(d) / float(speed_dps) * 1000.0 - 1e-9)))


# --------------------------------------------------------------------------- 各字段

def _toward(name, pose, ctx):
    cfg = ctx.cfg
    if name == TOWARD_PREV:
        if ctx.history_used >= len(ctx.history):
            ctx.note('没有记录到上一个姿态，没法回去')
            return pose
        prev = _sanitize(ctx.history[-1 - ctx.history_used], cfg)
        ctx.history_used += 1
        return _Pose(prev.j, pose.p1, pose.p2)     # 只回关节，夹爪保持现状
    az, el = cfg.directions[name]
    az = nearest_equivalent_az(az, pose.j['base'], cfg.base_limit_deg)
    j, notes = solve_gaze(az, el, pose.j, cfg)
    for n in notes:
        ctx.note(n)
    return _Pose(j, pose.p1, pose.p2)


def _turn(moves, pose, ctx):
    cfg = ctx.cfg
    az, el = gaze_of(pose.j)
    el_lo, el_hi = el_limits(cfg)
    daz = del_ = 0.0
    az_abs = el_abs = None
    maxed = []
    for m in moves:
        v = resolve_amount('turn', m.amount, m.number, cfg)
        if v == MAX:
            maxed.append(m.dir)
            if m.dir == '左':
                az_abs = cfg.base_limit_deg
            elif m.dir == '右':
                az_abs = -cfg.base_limit_deg
            elif m.dir == '上':
                el_abs = el_hi
            else:
                el_abs = el_lo
            continue
        if m.dir == '左':
            daz += v
        elif m.dir == '右':
            daz -= v
        elif m.dir == '上':
            del_ += v
        else:
            del_ -= v
    az_t = az_abs if az_abs is not None else az + daz
    el_t = el_abs if el_abs is not None else el + del_
    j, notes = solve_gaze(az_t, el_t, pose.j, cfg)
    for n in notes:
        ctx.note(n)
    words = {'左': '转到最左了', '右': '转到最右了', '上': '抬到最高了', '下': '低到最低了'}
    for d in maxed:
        ctx.note(words[d], partial=False)
    return _Pose(j, pose.p1, pose.p2)


def _ik_near(tip, el, ref, cfg):
    """IK 两个肘支里取 field 合法、且离 ref 关节最近的；都不行返回 None。"""
    best, best_cost = None, None
    for br in (-1, 1):
        try:
            c = ikine(tip[0], tip[1], tip[2], el, branch=br)
        except Unreachable:
            continue
        c['base'] = ref['base'] + _wrap180(c['base'] - ref['base'])
        if abs(c['base'] - ref['base']) > _SHIFT_MAX_BASE_JUMP:
            continue
        if not joints_ok(c, cfg):
            continue
        cost = sum(abs(c[k] - ref[k]) for k in _JK)
        if best is None or cost < best_cost:
            best, best_cost = c, cost
    return best


def el_search_order(el0, cfg):
    """从 el0 起向两侧按 1° 扩展的俯仰候选（不含 el0 本身），都在 el_range 内；|Δel| 小的在前。"""
    lo, hi = el_limits(cfg)
    out = []
    for d in range(1, int(math.ceil(hi - lo)) + 1):
        for el in (el0 - d, el0 + d):          # 同 |Δel| 先试往下
            if lo - 1e-9 <= el <= hi + 1e-9:
                out.append(el)
    return out


def _translate(delta, pose, ctx, verb, is_max):
    """爪尖沿 delta（cm，车体系）平移（设计 §7 shift，2026-10-31 修订）：

    1. 保持当前俯仰能完整到达 ⇒ 用它；
    2. 否则在俯仰上从当前值起向两侧按 1° 扩展，取能完整到达、|Δel| 最小的解
       （同 |Δel| 取关节总改动最小）；位置要求满足了 ⇒ 不算截断，只说明俯仰变了多少；
    3. 任何俯仰都到不了 ⇒ 二分找"某个俯仰下"能挪的最大比例，置 partial。
    """
    cfg = ctx.cfg
    tip0, _ = fk(pose.j)
    el0 = gaze_of(pose.j)[1]
    want = math.sqrt(sum(d * d for d in delta))
    if want < 1e-9:
        return pose
    others = el_search_order(el0, cfg)

    def cost(c):
        return sum(abs(c[k] - pose.j[k]) for k in _JK)

    def at(ratio):
        """该比例下 |Δel| 最小（同档取关节改动最小）的解；都不行 None。"""
        tip = [tip0[i] + ratio * delta[i] for i in range(3)]
        c = _ik_near(tip, el0, pose.j, cfg)
        if c is not None:
            return c
        best, best_d = None, None
        for el in others:
            d = abs(el - el0)
            if best is not None and d > best_d + 1e-9:
                break
            c = _ik_near(tip, el, pose.j, cfg)
            if c is not None and (best is None or cost(c) < cost(best)):
                best, best_d = c, d
        return best

    j = at(1.0)
    ratio = 1.0
    if j is None:
        lo, hi, j = 0.0, 1.0, None
        for _ in range(_BISECT_ITERS):
            mid = 0.5 * (lo + hi)
            c = at(mid)
            if c is not None:
                lo, j = mid, c
            else:
                hi = mid
        ratio = lo
    if j is not None:
        d_el = gaze_of(j)[1] - el0
        if abs(d_el) > 0.5:
            ctx.note('为了够到，视线俯仰变了 %.0f°（%s）' % (abs(d_el), '往上' if d_el > 0 else '往下'),
                     partial=False)
    moved = ratio * want
    if is_max:
        if moved < _TRAVEL_EPS_CM:
            ctx.note('%s已经到头了，没动' % verb, partial=False)
        else:
            ctx.note('%s到头了（挪了 %.1fcm）' % (verb, moved), partial=False)
    elif ratio < 1.0:
        if j is None or moved < _TRAVEL_EPS_CM:
            ctx.note('%s够不着了，没动' % verb)
        else:
            ctx.note('只%s挪了 %.1fcm（要 %.1fcm），再远够不着' % (verb, moved, want))
    if j is None:
        return pose
    return _Pose(j, pose.p1, pose.p2)


def _shift(moves, pose, ctx):
    cfg = ctx.cfg
    delta = [0.0, 0.0, 0.0]
    is_max = False
    for m in moves:
        v = resolve_amount('shift', m.amount, m.number, cfg)
        if v == MAX:
            v, is_max = cfg.max_travel_cm, True
        for i in range(3):
            delta[i] += v * _SHIFT_VEC[m.dir][i]
    verb = '、'.join('往' + m.dir for m in moves)
    return _translate(delta, pose, ctx, verb, is_max)


def _approach(m, pose, ctx):
    cfg = ctx.cfg
    v = resolve_amount('approach', m.amount, m.number, cfg)
    is_max = v == MAX
    if is_max:
        v = cfg.max_travel_cm
    sign = 1.0 if m.dir == '靠近' else -1.0
    _, axis = fk(pose.j)
    delta = [sign * v * a for a in axis]
    return _translate(delta, pose, ctx, '凑近' if sign > 0 else '退远', is_max)


def _oscillate(osc, pose, ctx):
    """返回 [(pose, label)]：2×times 个来回帧 + 1 帧回到起始。"""
    cfg = ctx.cfg
    times = osc.times or cfg.oscillate_times
    out = []
    if osc.axis == '夹爪':
        g_open, g_close = cfg.gripper['张开'], cfg.gripper['合上']
        first_close = abs(pose.p1 - g_open) <= abs(pose.p1 - g_close)
        seq = [(g_close, '合上'), (g_open, '张开')] if first_close else [(g_open, '张开'), (g_close, '合上')]
        for t in range(times):
            for p1, word in seq:
                out.append((_Pose(dict(pose.j), p1, pose.p2), 'oscillate:夹爪 %d/%d %s' % (t + 1, times, word)))
    else:
        a = resolve_amount('oscillate', osc.amount, None, cfg)
        if osc.axis == '上下':
            key, (lo, hi), words = 'wrist_pitch', joint_bounds()['wrist_pitch'], ('下', '上')
            first, second = pose.j[key] - a, pose.j[key] + a       # 点头先往下
        else:
            key, lo, hi, words = 'base', -cfg.base_limit_deg, cfg.base_limit_deg, ('左', '右')
            first, second = pose.j[key] + a, pose.j[key] - a       # 摇头先往左
        c1, c2 = min(max(first, lo), hi), min(max(second, lo), hi)
        for want, got, w in ((first, c1, words[0]), (second, c2, words[1])):
            if abs(want - got) > 1e-9:
                ctx.note('%s幅度到限位了：往%s只能动 %.0f°（要 %.0f°）'
                         % ('点头' if key == 'wrist_pitch' else '摇头', w, abs(got - pose.j[key]), a))
        for t in range(times):
            for v, w in ((c1, words[0]), (c2, words[1])):
                p = pose.copy()
                p.j[key] = v
                out.append((p, 'oscillate:%s %d/%d %s' % (osc.axis, t + 1, times, w)))
    out.append((pose.copy(), 'oscillate:%s 回原位' % osc.axis))
    return out


def _step_label(st):
    parts = []
    if st.toward:
        parts.append('toward:' + st.toward)
    for key in ('turn', 'shift'):
        for m in getattr(st, key):
            parts.append('%s:%s%s' % (key, m.dir, ('%g' % m.number) if m.number is not None else (m.amount or '')))
    if st.approach:
        m = st.approach
        parts.append('approach:%s%s' % (m.dir, ('%g' % m.number) if m.number is not None else (m.amount or '')))
    if st.gripper:
        parts.append('gripper:' + st.gripper)
    return ' + '.join(parts)


# --------------------------------------------------------------------------- 入口

def plan(spec, cur_fields, history, cfg):
    """spec（Spec / JSON 字符串 / dict）+ 当前 [p1..p6] + 动作前姿态栈 → Plan。"""
    if not isinstance(spec, Spec):
        spec = parse_spec(spec)
    ctx = _Ctx(cfg=cfg, history=list(history or []))
    pose = _sanitize(cur_fields, cfg)
    prev_fields = pose.fields()
    frames = []
    for _ in range(spec.repeat):
        for st in spec.steps:
            if st.oscillate is not None:
                speed = cfg.speeds[spec.speed or cfg.oscillate_speed]
                seq = _oscillate(st.oscillate, pose, ctx)
            else:
                speed = cfg.speeds[spec.speed or cfg.default_speed]
                p = pose
                if st.toward:
                    p = _toward(st.toward, p, ctx)
                if st.turn:
                    p = _turn(st.turn, p, ctx)
                if st.shift:
                    p = _shift(st.shift, p, ctx)
                if st.approach:
                    p = _approach(st.approach, p, ctx)
                if st.gripper:
                    p = _Pose(dict(p.j), cfg.gripper[st.gripper], p.p2)
                seq = [(p, _step_label(st))]
            for p, label in seq:
                f = p.fields()
                frames.append(Keyframe(f, frame_t_ms(prev_fields, f, speed, cfg.min_frame_ms), label))
                prev_fields = f
                pose = p
    return Plan(frames, ctx.notes, ctx.partial, ctx.history_used)
