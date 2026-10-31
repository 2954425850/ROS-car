# -*- coding: utf-8 -*-
"""视线求解（设计 §3 §7）。纯函数，不 import rclpy。

视线 = 夹爪轴（相机光轴与之平行）：
  * 方位 az = 底座角，左正、0=车头；
  * 俯仰 el = shoulder + elbow + wrist_pitch，下负、0=水平。

`solve_gaze` 的求解顺序：
  1. base = az，截到 ±base_limit_deg（截了 ⇒ note「还差 Y°」）；
  2. el 截到 cfg.el_range_deg（截了 ⇒ note「已经低到最低了」/「已经抬到最高了」）；
  3. 先试只动腕：肩、肘不变，wrist = el − shoulder − elbow；
  4. 不行就在肩×肘 1° 网格里按 |Δ肩|+|Δ肘| 从小到大找第一个满足 field 限位的解；
     同代价的多个解里取爪尖位移最小的（动作看起来最"原地"）。
"""
import math

from arm_grasp.arm_kin import FACTOR, FIELD_HI, FIELD_LO, fk

_EPS = 1e-9         # 浮点容差（回读略越界的情况由 motion_plan 先把输入钳回限位内）


def joint_bounds():
    """由 field 限位 [125, 875] 反推的关节角范围（度）。"""
    w_lo, w_hi = (FIELD_LO - 500.0) / FACTOR, (FIELD_HI - 500.0) / FACTOR
    s_lo, s_hi = 90.0 - (FIELD_HI - 500.0) / FACTOR, 90.0 - (FIELD_LO - 500.0) / FACTOR
    return {'wrist_pitch': (w_lo, w_hi), 'elbow': (w_lo, w_hi), 'shoulder': (s_lo, s_hi)}


def _in(v, lohi):
    return lohi[0] - _EPS <= v <= lohi[1] + _EPS


def joints_ok(j, cfg):
    """肩/肘/腕 field 在 [125,875]，底座在 ±base_limit_deg 内。"""
    b = joint_bounds()
    return (all(_in(j[k], b[k]) for k in ('shoulder', 'elbow', 'wrist_pitch'))
            and abs(j['base']) <= cfg.base_limit_deg + _EPS)


def gaze_of(joints):
    """关节角 → (az, el)，度。"""
    return (joints['base'],
            joints['shoulder'] + joints['elbow'] + joints['wrist_pitch'])


def clamp_base(base, cfg):
    """底座角截到 ±base_limit_deg。返回 (base, note 或 None)。"""
    lim = cfg.base_limit_deg
    if base > lim:
        return lim, '左边最多转到 %.0f°，还差 %.0f°（要的话车可以转）' % (lim, base - lim)
    if base < -lim:
        return -lim, '右边最多转到 %.0f°，还差 %.0f°（要的话车可以转）' % (lim, -lim - base)
    return base, None


def el_limits(cfg):
    """可用的俯仰范围 = 配置范围 ∩ 关节物理能凑出的范围。"""
    b = joint_bounds()
    phys_lo = sum(b[k][0] for k in b)
    phys_hi = sum(b[k][1] for k in b)
    return max(cfg.el_range_deg[0], phys_lo), min(cfg.el_range_deg[1], phys_hi)


def clamp_el(el, cfg):
    lo, hi = el_limits(cfg)
    if el < lo - _EPS:
        return lo, '已经低到最低了（视线最多向下 %.0f°）' % -lo
    if el > hi + _EPS:
        return hi, '已经抬到最高了（视线最多向上 %.0f°）' % hi
    return el, None


def _tip_dist(a, b):
    return math.dist(fk(a)[0], fk(b)[0])


def solve_gaze(az, el, cur_joints, cfg):
    """目标视线 (az, el) + 当前关节 → (新关节 dict, notes)。"""
    notes = []
    base, n = clamp_base(float(az), cfg)
    if n:
        notes.append(n)
    el, n = clamp_el(float(el), cfg)
    if n:
        notes.append(n)

    s0, e0 = cur_joints['shoulder'], cur_joints['elbow']
    b = joint_bounds()

    def cand(s, e):
        return {'base': base, 'shoulder': s, 'elbow': e, 'wrist_pitch': el - s - e}

    # 1) 只动腕（像转脖子）
    j = cand(s0, e0)
    if joints_ok(j, cfg):
        return j, notes

    # 2) 肩×肘 1° 网格，按 L1 改动量一圈圈往外找
    i_lo, i_hi = math.ceil(b['shoulder'][0] - s0), math.floor(b['shoulder'][1] - s0)
    k_lo, k_hi = math.ceil(b['elbow'][0] - e0), math.floor(b['elbow'][1] - e0)
    ref = dict(cur_joints, base=base)
    max_r = (i_hi - i_lo) + (k_hi - k_lo)
    for r in range(0, max_r + 1):
        found = []
        for i in range(-r, r + 1):
            if not i_lo <= i <= i_hi:
                continue
            rest = r - abs(i)
            for k in ((rest, -rest) if rest else (0,)):
                if not k_lo <= k <= k_hi:
                    continue
                c = cand(s0 + i, e0 + k)
                if joints_ok(c, cfg):
                    found.append(c)
        if found:
            return min(found, key=lambda c: _tip_dist(c, ref)), notes

    # 3) 理论上到不了这里（el 已截到物理可达范围内）；防御：不动肩肘，腕截到限位
    w = min(max(el - s0 - e0, b['wrist_pitch'][0]), b['wrist_pitch'][1])
    j = {'base': base, 'shoulder': s0, 'elbow': e0, 'wrist_pitch': w}
    notes.append('这个角度够不着，只转到了俯仰 %.0f°' % gaze_of(j)[1])
    return j, notes
