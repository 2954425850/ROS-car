# -*- coding: utf-8 -*-
"""LeArm 6 轴臂的运动学（从幻尔官方 C 源码移植，2026-09-28 移植并自检通过）。

## 关节对应（与 l150pro_driver_node 的 ARM_JOINT_NAMES 逐字一致）
    knot[0] -> KNOT6 -> servo6 = 底座 yaw
    knot[1] -> KNOT5 -> servo5 = 肩
    knot[2] -> KNOT4 -> servo4 = 肘
    knot[3] -> KNOT3 -> servo3 = 腕俯仰

## ⚠️ 两个必须记住的坑

1. 官方 `ikine()` 是**二维**的：里面 knots[0]=0、`sqrt(x*x)` 把 y 丢了。
   三维要自己补 `theta_base = atan2(y, x)` / `r = hypot(x, y)`。

2. 官方 `k1 = atan2(c,d) - atan2(a,b)` 看着像参数顺序写反了，**其实不是**——
   它是恒等式 `atan2(c,d) = 90 - delta` 的结果。按"它写错了"去推前向运动学，
   会得到 **23 cm** 的误差。前向必须用下面的 `fk()`，它已用数值反推验证过
   （IK->FK 往返最大误差 7.1e-15 cm）。

## 比例尺（2026-09-28 执行时已对着官方源码逐字核对）
- 肩/肘/腕：官方 `1000/240`
  `robot_arm.c:38`  `serial_servo_set_position(&c, 6-i, 500 + (int)(SERIAL_ANGLE_FACTOR * target_angle[i]), time)`
  `robot_arm.c:32-35` `target_angle = [θ6, 90-θ5, θ4, θ3]`
  `robot_arm.h:39`  `SERIAL_ANGLE_FACTOR = 4.166666666666667`
- **底座：本固件是 `1000/360`**（±1000 <-> ±360 度，见 `arm_cal.h` ARM_SOFT_LIMIT_DEG），
  已实测物理验证（指令 +20.16 度，pot_raw 实测 +19.39 度）
"""
import math

L1, L2, L3, L4 = 2.89, 10.43, 8.9, 17.7      # cm
FACTOR = 1000.0 / 240.0                        # 肩/肘/腕
BASE_SCALE = 1000.0 / 360.0                    # 底座（本固件）
FIELD_LO, FIELD_HI = 125.0, 875.0              # 固件真实钳位

_KEY = ('base', 'shoulder', 'elbow', 'wrist_pitch')


class Unreachable(Exception):
    pass


def ikine(x, y, z, alpha_deg):
    """(x, y, z) 是夹爪尖目标，alpha_deg 是末端下扎角（度，负=往下）。

    返回 {'base','shoulder','elbow','wrist_pitch'}，单位度。
    """
    theta_base = math.degrees(math.atan2(y, x))
    r = math.hypot(x, y)
    a = r - L4 * math.cos(math.radians(alpha_deg))
    b = z - L1 - L4 * math.sin(math.radians(alpha_deg))
    ck2 = (a * a + b * b - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    if not (-1.0 <= ck2 <= 1.0):
        raise Unreachable('no solution, ck2=%.6f' % ck2)
    k2 = math.atan2(-math.sqrt(1.0 - ck2 * ck2), ck2)
    c = L2 + L3 * math.cos(k2)
    d = L3 * math.sin(k2)
    k1 = math.atan2(c, d) - math.atan2(a, b)
    k3 = math.radians(alpha_deg) - k1 - k2
    return dict(base=theta_base,
                shoulder=math.degrees(k1),
                elbow=math.degrees(k2),
                wrist_pitch=math.degrees(k3))


def fk(joints):
    """返回 (tip, axis)。

    tip  = (x, y, z)，相对底座安装面，cm
    axis = 夹爪下扎方向单位向量（相机光轴与它绝对平行）
    """
    k1 = math.radians(joints['shoulder'])
    k2 = math.radians(joints['elbow'])
    k3 = math.radians(joints['wrist_pitch'])
    tb = math.radians(joints['base'])
    alpha = k1 + k2 + k3
    er, ez = L2 * math.cos(k1), L2 * math.sin(k1)
    wr = er + L3 * math.cos(k1 + k2)
    wz = ez + L3 * math.sin(k1 + k2)
    tr = wr + L4 * math.cos(alpha)
    tz = wz + L4 * math.sin(alpha) + L1
    tip = (tr * math.cos(tb), tr * math.sin(tb), tz)
    axis = (math.cos(alpha) * math.cos(tb),
            math.cos(alpha) * math.sin(tb),
            math.sin(alpha))
    return tip, axis


def find_grasp_solution(x, y, z, alpha_lo=-89.0, alpha_hi=-20.0, step=1.0,
                        field_ok=True):
    """在 alpha 范围里逐度扫，返回第一个「舵机 field 都在 125..875 内」的解。

    为什么不直接用官方限位：官方把肘限在 (-90,90)，而**本臂出厂姿态的肘
    已经贴到那条线附近**（用官方映射解出来是 -89.28）。真正的保护是固件的
    125..875 钳位。
    """
    n = int(round((alpha_hi - alpha_lo) / step))
    for i in range(n + 1):
        alpha = alpha_lo + i * step
        if not (alpha_lo <= alpha <= alpha_hi):
            continue
        try:
            j = ikine(x, y, z, alpha)
        except Unreachable:
            continue
        if not field_ok:
            return alpha, j
        f = to_fields(j, 240.0, 496.0)
        if all(FIELD_LO <= v <= FIELD_HI for v in (f[2], f[3], f[4])):
            return alpha, j
    raise Unreachable('no alpha in [%.0f, %.0f] works for (%.2f,%.2f,%.2f)'
                      % (alpha_lo, alpha_hi, x, y, z))


def to_fields(joints, gripper, wrist_roll):
    """关节角 -> [p1..p6]（p1 夹爪、p2 夹爪自转原样带过）。"""
    return [float(gripper), float(wrist_roll),
            500.0 + FACTOR * joints['wrist_pitch'],
            500.0 + FACTOR * joints['elbow'],
            500.0 + FACTOR * (90.0 - joints['shoulder']),
            joints['base'] * BASE_SCALE]


def from_fields(fields):
    """[p1..p6] -> 关节角（p1/p2 丢掉）。"""
    p1, p2, p3, p4, p5, p6 = fields
    return dict(base=p6 / BASE_SCALE,
                shoulder=90.0 - (p5 - 500.0) / FACTOR,
                elbow=(p4 - 500.0) / FACTOR,
                wrist_pitch=(p3 - 500.0) / FACTOR)
