"""小车运动学：方向→速度、云台角度→脉宽。**纯函数，不 import ROS。**

刻意与 CarController 分开：这里每个决策（斜向怎么合成、超限怎么截、云台到没到
限位）都能在没接车的机器上测，而 CarController 要碰 ROS。
"""

from __future__ import annotations

import math

from car.types import CarLimits

# 每个方向 = (前进分量, 转向分量)，各取 -1 / 0 / +1。
# 与 teleop_l150pro.py 的 MOTION_KEYS 一致。
# 角速度为正 = 逆时针 = 左转（REP-103）。
#
# **没有 left / right。** 差速车没有横移，「往左」要么是原地转（car_turn），
# 要么是弧线（forward_left）。「往左走两米」里的「两米」对原地转毫无意义。
DIRECTIONS: dict[str, tuple[int, int]] = {
    "forward": (+1, 0),
    "backward": (-1, 0),
    "forward_left": (+1, +1),
    "forward_right": (+1, -1),
    "backward_left": (-1, +1),
    "backward_right": (-1, -1),
}

# 弧线（同时有前进和转向）时，转向速率相对原地转向的倍数。teleop 用 0.5，照抄。
ARC_TURN_RATIO = 0.5

# 标准舵机：中位 1500us，±90° 对应 ±1000us。
_US_PER_DEG = 1000.0 / 90.0


def direction_names() -> list[str]:
    """给工具 schema 用的方向枚举。

    工具的 JSON Schema 直接用它，**不要再在 tools/ 里抄一份** ——
    抄一份会静默漂移：schema 是发给模型的，漂了就变成「模型传一个
    控制器不认的方向」。有没有漂，靠 tests/test_car_tools.py 里的同步断言兜。
    """
    return list(DIRECTIONS)


# 云台动作。注意 `center` 是回中位，不是方向。
CAMERA_DIRECTIONS: tuple[str, ...] = ("left", "right", "up", "down", "center")


def camera_direction_names() -> list[str]:
    """给工具 schema 用的云台方向枚举。理由同 direction_names()。"""
    return list(CAMERA_DIRECTIONS)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def clamp_speed(speed: float, limits: CarLimits) -> tuple[float, bool]:
    """返回 (生效速度, 是否被截断)。取绝对值 —— 方向由 direction 决定。"""
    magnitude = abs(float(speed))
    if magnitude > limits.max_speed:
        return limits.max_speed, True
    return magnitude, False


def clamp_turn_speed(dps: float, limits: CarLimits) -> tuple[float, bool]:
    """返回 (生效角速度 度/秒, 是否被截断)。"""
    magnitude = abs(float(dps))
    if magnitude > limits.max_turn_speed_dps:
        return limits.max_turn_speed_dps, True
    return magnitude, False


def direction_to_velocity(
    direction: str, speed: float, turn_speed_dps: float
) -> tuple[float, float]:
    """方向 + 线速度 → (vx m/s, wz rad/s)。未知方向抛 KeyError。"""
    fwd, turn = DIRECTIONS[direction]
    arc = fwd != 0 and turn != 0
    angular_dps = turn_speed_dps * (ARC_TURN_RATIO if arc else 1.0)
    vx = fwd * speed
    wz = turn * angular_dps * math.pi / 180.0
    return vx, wz


def camera_target_us(
    direction: str,
    degrees: float | None,
    current_us: int,
    limits: CarLimits,
) -> tuple[int, bool]:
    """算出云台动作后的目标脉宽。

    direction == "center" 时忽略 degrees，直接回中位。

    Returns:
        (目标脉宽 us, 是否撞到行程限位)

    Raises:
        KeyError: direction 不是 left/right/up/down/center
    """
    if direction == "center":
        return limits.camera_center_us, False

    if direction in ("left", "right"):
        sign = +1 if direction == "right" else -1
    elif direction in ("up", "down"):
        sign = +1 if direction == "up" else -1
    else:
        raise KeyError(direction)

    # 方向是权威：degrees 只取大小，符号由 direction 决定。
    # 这样模型把「左转 30 度」写成 (left, -30) 也不会转到反方向去。
    delta_deg = limits.camera_step_deg if degrees is None else abs(float(degrees))
    target = current_us + int(round(sign * delta_deg * _US_PER_DEG))
    clamped = int(clamp(target, limits.camera_min_us, limits.camera_max_us))
    return clamped, clamped != target
