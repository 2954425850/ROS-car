"""CarController 的闭环逻辑测试 —— 用假桥，不接车、不碰 ROS。"""

import re
import time

import pytest

from car.controller import CarController
from car.types import CarLimits
from tests.fakes import FakeBridge, FakeClock

LIMITS = CarLimits()


class _FakeConfig:
    def get(self, key, default=None):
        return default


def _make(**bridge_kwargs):
    clock = FakeClock()
    bridge = FakeBridge(clock, **bridge_kwargs)
    controller = CarController(
        _FakeConfig(), bridge_factory=lambda **_: bridge,
        clock=clock, sleeper=clock.sleep,
    )
    # 先把桥建起来 —— 模拟「车被用过」。stop_all() 刻意**不**建桥
    # （它跑在唤醒词路径上，不该为了发几帧零速去 rclpy.init()），
    # 所以测「急停连发 8 帧」必须先有一次真实使用。
    controller.start()
    return controller, bridge, clock


# ---------------- 走距离 ----------------

def test_move_distance_does_not_block():
    """★★ 这是急停能用的前提，别删。

    `move_distance` 跑在语音助手的**唤醒线程**上，而唤醒模块的串口监听
    也在那个线程（wakeword/engine.py 的 `_listen_loop -> _handle_detection`
    是内联回调，它自己的注释就写着「回调里同步跑完了整条流水线」）。
    它只要阻塞，串口就没人读，用户喊唤醒词进不来，**急停失效**。
    2026-09-17 实车复现：「他在往前走的时候，喊他没用」。

    ★ 必须用**真实时钟 + 真实 sleep**。用 FakeClock 是抓不到的 ——
    假的 sleep 只拨时间不真等，阻塞实现也会在假时间里「瞬间」跑完，
    测试照样绿（实测过，第一版就是这个坑）。这条跑一次约 0.8 秒，值。
    """
    bridge = FakeBridge(FakeClock())
    controller = CarController(
        _FakeConfig(),
        bridge_factory=lambda **_: bridge,
        clock=time.monotonic,      # ← 真实时间
        sleeper=time.sleep,        # ← 真的睡
    )
    controller.start()

    started = time.monotonic()
    controller.move_distance("forward", 0.4, speed=0.5)   # 真跑要 0.8 秒
    elapsed = time.monotonic() - started
    assert elapsed < 0.3, (
        f"阻塞了 {elapsed:.2f} 秒 —— 唤醒线程会被占住，喊唤醒词停不下来"
    )

    controller.stop_all()
    controller.wait_idle()


def test_stop_all_interrupts_a_running_motion():
    """★ 唤醒词路径调的就是 stop_all() —— 车走着的时候必须能立刻停。"""
    controller, bridge, _ = _make()
    controller.move_distance("forward", 100.0, speed=0.5)
    controller.stop_all()
    controller.wait_idle()
    assert "被打断" in controller.last_outcome, controller.last_outcome
    assert bridge.velocity_calls[-1] == (0.0, 0.0)


def test_new_command_preempts_the_old_one():
    """新指令顶掉旧指令，而不是回一句「正忙」。"""
    controller, _, _ = _make()
    controller.move_distance("forward", 100.0, speed=0.5)
    text = controller.move_distance("backward", 0.5, speed=0.5)
    assert text.startswith("已开始后退"), text
    controller.wait_idle()
    assert "后退" in controller.last_outcome, controller.last_outcome


def test_move_distance_stops_after_covering_the_distance():
    controller, bridge, _ = _make()
    controller.move_distance("forward", 1.0, speed=0.5)
    controller.wait_idle()
    # 前进总里程应当略大于等于 1.0（最后一次采样会过冲一点）
    assert bridge.state().traveled >= 1.0


def test_move_distance_ends_with_zero_velocity():
    """走完必须收尾归零，否则车会一直跑。"""
    controller, bridge, _ = _make()
    controller.move_distance("forward", 0.5, speed=0.5)
    controller.wait_idle()
    assert bridge.velocity_calls[-1] == (0.0, 0.0)


def test_move_distance_publishes_at_the_configured_rate():
    """0.5 米 @ 0.5 m/s = 1 秒；25Hz 下应当约 25 帧，而不是一帧。"""
    controller, bridge, _ = _make()
    controller.move_distance("forward", 0.5, speed=0.5)
    controller.wait_idle()
    moving = [c for c in bridge.velocity_calls if c != (0.0, 0.0)]
    assert len(moving) >= 20


def test_move_distance_uses_default_speed_when_omitted():
    controller, bridge, _ = _make()
    controller.move_distance("forward", 0.3)
    first = bridge.velocity_calls[0]
    assert first[0] == pytest.approx(LIMITS.default_speed)


def test_move_distance_clamps_and_says_so():
    """超限要报出来，不能静默截断。"""
    controller, bridge, _ = _make()
    text = controller.move_distance("forward", 0.2, speed=0.8)
    assert "0.5" in text
    assert bridge.velocity_calls[0][0] == pytest.approx(LIMITS.max_speed)


def test_backward_is_negative():
    controller, bridge, _ = _make()
    controller.move_distance("backward", 0.3, speed=0.3)
    assert bridge.velocity_calls[0][0] < 0


def test_move_distance_gives_up_when_odom_never_moves():
    """驱动没在跑时里程计恒为 0，必须超时退出并记下原因。"""
    controller, bridge, _ = _make(dead=True)
    controller.move_distance("forward", 1.0, speed=0.5)
    controller.wait_idle()
    assert "超时" in controller.last_outcome, controller.last_outcome
    assert bridge.velocity_calls[-1] == (0.0, 0.0)


def test_move_distance_rejects_unknown_direction():
    controller, _, _ = _make()
    text = controller.move_distance("sideways", 1.0)
    assert "方向" in text or "sideways" in text


# ---------------- 转向 ----------------

def test_turn_by_integrates_yaw_to_the_target_angle():
    """90 度 @ 60 度/秒 ≈ 1.5 秒。

    ★ 下面两条「已左转 / 90」的断言是**必须的**：只查首帧符号和末帧归零的话，
    「度/弧度混用」那个 bug 会照样通过 —— 它的表现是永远跑满超时，再报一个
    假的「陀螺仪一直读到 0 —— IMU 数据没上来」。测试必须验到**走的是完成分支**。
    """
    controller, bridge, _ = _make()
    controller.turn_by(90.0)
    controller.wait_idle()
    assert bridge.velocity_calls[-1] == (0.0, 0.0)
    assert bridge.velocity_calls[0][1] > 0        # 正角 = 逆时针 = 左转
    # ★ 必须验到**结局是「完成」**，不是超时。单位混用的 bug 表现就是
    # 永远跑满超时，只查首帧符号和末帧归零的话它会照样通过。
    assert "完成" in controller.last_outcome, controller.last_outcome
    assert "左转 90 度" in controller.last_outcome, controller.last_outcome


def test_turn_by_finishes_in_about_the_expected_time():
    """90 度 @ 60 度/秒 = 1.5 秒；25Hz 下约 37 帧。

    这条是给「度当弧度用」上的第二道锁：那个 bug 会让帧数变成几十上百倍
    （要积 90 弧度才够），所以卡一个量级就抓住了。
    """
    controller, bridge, _ = _make()
    controller.turn_by(90.0)
    controller.wait_idle()
    moving = [c for c in bridge.velocity_calls if c != (0.0, 0.0)]
    assert 30 <= len(moving) <= 45, f"发了 {len(moving)} 帧，与 1.5 秒的预期差太远"


def test_turn_by_360_degrees_finishes():
    """整整一圈也该正常收尾（角度大了更容易暴露单位问题）。"""
    controller, _, _ = _make()
    controller.turn_by(360.0)
    controller.wait_idle()
    assert "完成" in controller.last_outcome, controller.last_outcome
    assert "360" in controller.last_outcome, controller.last_outcome


def test_turn_by_negative_angle_turns_the_other_way():
    controller, bridge, _ = _make()
    controller.turn_by(-90.0)
    assert bridge.velocity_calls[0][1] < 0


def test_turn_by_clamps_turn_speed():
    controller, bridge, _ = _make()
    text = controller.turn_by(360.0, speed=999.0)
    assert "120" in text


# ---------------- 急停 ----------------

def test_stop_all_publishes_zero_frames_repeatedly():
    """急停要连发几帧，确保驱动真收到（teleop 同款做法）。"""
    controller, bridge, _ = _make()
    controller.stop_all()
    assert len(bridge.velocity_calls) >= 8
    assert all(c == (0.0, 0.0) for c in bridge.velocity_calls)


def test_stop_all_is_safe_before_bridge_exists():
    """唤醒词路径会直接调它 —— 那条路径上绝不能抛异常。"""
    clock = FakeClock()
    controller = CarController(
        _FakeConfig(), bridge_factory=lambda **_: FakeBridge(clock),
        clock=clock, sleeper=clock.sleep,
    )
    controller.stop_all()          # 不该抛


def test_stop_and_describe_reports_stopping():
    controller, _, _ = _make()
    assert "停" in controller.stop_and_describe()


# ---------------- 云台 ----------------

def test_camera_move_sends_six_channel_frame():
    controller, bridge, _ = _make()
    controller.camera_move("right")
    frame = bridge.servo_calls[-1]
    assert len(frame) == 6
    assert frame[4] > 1500          # ch5 = 水平
    assert frame[5] == 0            # ch6 没动 → 0 = 保持不动


def test_camera_center_returns_to_1500():
    controller, bridge, _ = _make()
    controller.camera_move("right")
    controller.camera_move("center")
    assert bridge.servo_calls[-1][4] == 1500


def test_camera_up_moves_tilt_channel():
    controller, bridge, _ = _make()
    controller.camera_move("up")
    frame = bridge.servo_calls[-1]
    assert frame[4] == 0            # ch5 没动
    assert frame[5] > 1500          # ch6 = 俯仰


def test_camera_reports_hitting_the_limit():
    controller, _, _ = _make()
    text = controller.camera_move("right", degrees=900.0)
    assert "限位" in text or "到头" in text


# ---- camera_nudge：给控制回路用的结构化版本 ----
# 为什么必须有它：`camera_move` 返回的是**给人念的中文串**，`hit_limit` 埋在文字里。
# 2026-09-19 做云台居中闭环时要用这个值做判断（撞限位就该放弃），
# 让回路去解析中文串是不可接受的。


def test_camera_nudge_returns_structured_pulse_and_limit():
    controller, bridge, _ = _make()
    us, hit = controller.camera_nudge("right", degrees=15.0)
    assert us > LIMITS.camera_center_us       # 往右 = 脉宽变大
    assert hit is False
    assert bridge.servo_calls[-1][4] == us    # 发出去的帧就是它
    assert bridge.servo_calls[-1][5] == 0     # 俯仰那路没动


def test_camera_nudge_reports_limit_as_a_value_not_text():
    controller, _, _ = _make()
    us, hit = controller.camera_nudge("right", degrees=900.0)
    assert hit is True
    assert us == LIMITS.camera_max_us         # 被夹到行程上限


def test_camera_nudge_moves_tilt_channel_for_up():
    controller, bridge, _ = _make()
    _us, hit = controller.camera_nudge("up", degrees=10.0)
    assert hit is False
    assert bridge.servo_calls[-1][4] == 0     # 水平那路没动
    assert bridge.servo_calls[-1][5] > LIMITS.camera_center_us


def test_camera_nudge_raises_keyerror_for_unknown_direction():
    controller, _, _ = _make()
    with pytest.raises(KeyError):
        controller.camera_nudge("diagonal")


def test_camera_move_text_unchanged_after_refactor():
    """重构不能改变 camera_move 的对外行为 —— 这句话是要念给用户听的。"""
    controller, _, _ = _make()
    text = controller.camera_move("right", degrees=15.0)
    assert text.startswith("云台往右了（水平轴 ")
    assert text.endswith("微秒）。")
    assert "到头" not in text
    assert "到头" in controller.camera_move("right", degrees=900.0)


# ---------------- 状态 ----------------

def test_status_text_includes_voltage_and_fault():
    controller, _, _ = _make(voltage=11.6, fault=0)
    text = controller.status_text()
    assert "11.6" in text
    assert "正常" in text


def test_status_text_says_so_when_offline():
    controller, _, _ = _make(dead=True)
    assert "读不到" in controller.status_text() or "没在跑" in controller.status_text()
