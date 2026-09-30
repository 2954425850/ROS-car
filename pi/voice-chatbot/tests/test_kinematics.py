"""纯运动学测试 —— 不碰 ROS、不碰硬件。"""

import math

import pytest

from car.kinematics import (
    DIRECTIONS,
    camera_target_us,
    clamp_speed,
    clamp_turn_speed,
    direction_names,
    direction_to_velocity,
)
from car.types import CarLimits, RobotState, describe_faults

LIMITS = CarLimits()


# ---------------- 方向表 ----------------

def test_move_directions_have_no_pure_turn():
    """car_move 只管带前进分量的方向；纯转向归 car_turn。"""
    assert "left" not in DIRECTIONS
    assert "right" not in DIRECTIONS
    assert set(direction_names()) == {
        "forward", "backward",
        "forward_left", "forward_right",
        "backward_left", "backward_right",
    }


def test_forward_is_positive_vx():
    vx, wz = direction_to_velocity("forward", 0.3, 60.0)
    assert vx == pytest.approx(0.3)
    assert wz == pytest.approx(0.0)


def test_backward_is_negative_vx():
    vx, _ = direction_to_velocity("backward", 0.3, 60.0)
    assert vx == pytest.approx(-0.3)


def test_left_turn_is_positive_wz():
    """REP-103：角速度为正 = 逆时针 = 左转。"""
    _, wz = direction_to_velocity("forward_left", 0.3, 90.0)
    assert wz > 0


def test_arc_turn_is_half_of_plain_turn():
    """弧线的转向速率是原地转向的一半 —— 与 teleop 的 arc_gain 一致。"""
    _, wz_arc = direction_to_velocity("forward_left", 0.3, 90.0)
    expected = 90.0 * 0.5 * math.pi / 180.0
    assert wz_arc == pytest.approx(expected)


def test_direction_to_velocity_rejects_unknown_direction():
    with pytest.raises(KeyError):
        direction_to_velocity("sideways", 0.3, 60.0)


# ---------------- 限幅 ----------------

def test_clamp_speed_passes_through_when_within_limit():
    value, clamped = clamp_speed(0.3, LIMITS)
    assert value == pytest.approx(0.3)
    assert clamped is False


def test_clamp_speed_reports_clamping():
    """超限必须报出来，不能静默截断 —— 否则模型以为车真按 0.8 在跑。"""
    value, clamped = clamp_speed(0.8, LIMITS)
    assert value == pytest.approx(0.5)
    assert clamped is True


def test_clamp_speed_uses_magnitude():
    value, _ = clamp_speed(-0.3, LIMITS)
    assert value == pytest.approx(0.3)


def test_clamp_turn_speed_reports_clamping():
    value, clamped = clamp_turn_speed(999.0, LIMITS)
    assert value == pytest.approx(120.0)
    assert clamped is True


# ---------------- 云台 ----------------

def test_camera_center_goes_to_center_regardless_of_degrees():
    target, hit = camera_target_us("center", 45.0, 2000, LIMITS)
    assert target == 1500
    assert hit is False


def test_camera_right_increases_pulse_width():
    target, _ = camera_target_us("right", None, 1500, LIMITS)
    assert target > 1500


def test_camera_left_decreases_pulse_width():
    target, _ = camera_target_us("left", None, 1500, LIMITS)
    assert target < 1500


def test_camera_default_step_is_symmetric():
    right, _ = camera_target_us("right", None, 1500, LIMITS)
    left, _ = camera_target_us("left", None, 1500, LIMITS)
    assert right - 1500 == 1500 - left


def test_camera_explicit_degrees_overrides_step():
    """说了角度就按说的做：30 度 ≈ 333us，与默认 15 度的步进不同。"""
    explicit, _ = camera_target_us("right", 30.0, 1500, LIMITS)
    default, _ = camera_target_us("right", None, 1500, LIMITS)
    assert explicit != default
    assert explicit - 1500 == pytest.approx(30.0 * 1000.0 / 90.0, abs=1)


def test_camera_up_and_down_map_to_tilt():
    up, _ = camera_target_us("up", None, 1500, LIMITS)
    down, _ = camera_target_us("down", None, 1500, LIMITS)
    assert up > 1500 > down


def test_camera_reports_hitting_the_limit():
    target, hit = camera_target_us("right", 900.0, 1500, LIMITS)
    assert target == LIMITS.camera_max_us
    assert hit is True


def test_camera_rejects_unknown_direction():
    with pytest.raises(KeyError):
        camera_target_us("diagonal", None, 1500, LIMITS)


# ---------------- 配置与故障码 ----------------

def test_car_limits_read_from_config():
    class _FakeConfig:
        def get(self, key, default=None):
            return {"car.max_speed": 0.42}.get(key, default)

    limits = CarLimits.from_config(_FakeConfig())
    assert limits.max_speed == pytest.approx(0.42)
    assert limits.camera_center_us == 1500


def test_describe_faults_normal():
    assert describe_faults(0) == "正常"


def test_describe_faults_names_the_bits():
    assert describe_faults(1 << 0) == "欠压"
    assert "欠压" in describe_faults((1 << 0) | (1 << 4))
    assert "下行超时" in describe_faults((1 << 0) | (1 << 4))


def test_describe_faults_handles_missing_data():
    assert describe_faults(None) == "读不到故障码"


def test_robot_state_is_frozen():
    state = RobotState(voltage=12.0, fault=0, wheel_speeds=None,
                       yaw_rate=0.0, traveled=0.0, online=True)
    with pytest.raises(Exception):
        state.voltage = 11.0
