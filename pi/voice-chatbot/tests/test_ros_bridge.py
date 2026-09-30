"""RosBridge 测试。

不接车也能测的部分：环境检查的报错、发布的帧真的能被子节点收到。
需要 rclpy 的用例在没有 ROS 的机器上自动跳过。

**桥是 module 级共享的** —— rclpy.init() 是进程级的，只能调一次，
每个用例各建一次桥会从第二个开始报错。
"""

import builtins
import sys
import threading
import time

import pytest


def test_ensure_ros_available_gives_actionable_error(monkeypatch):
    """rclpy 不在时，报错必须告诉使用者怎么办，不能只说 ModuleNotFoundError。"""
    from car import ros_bridge

    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "rclpy":
            raise ImportError("No module named 'rclpy'")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    with pytest.raises(ros_bridge.RosUnavailable) as excinfo:
        ros_bridge.ensure_ros_available()
    assert "source /opt/ros/jazzy/setup.bash" in str(excinfo.value)
    assert "run.sh" in str(excinfo.value)


@pytest.fixture(scope="module")
def bridge():
    """整个模块共用一个桥 —— rclpy.init() 进程内只能调一次。

    `importorskip` 放在 fixture 里，**不放在模块级**：模块级的 Skipped 会在
    collection 阶段把整个文件吞掉，连 test_ensure_ros_available_gives_actionable_error
    这个**根本不依赖 ROS** 的用例都跑不了（实测：不带 ROS 时整文件 1 skipped）。
    """
    pytest.importorskip("rclpy", reason="需要 ROS 2 环境")
    from car.ros_bridge import RosBridge

    b = RosBridge(cmd_rate_hz=25.0)
    yield b
    b.close()


def test_bridge_publishes_velocity_that_others_can_hear(bridge):
    """自己发的帧，同一 DDS 域内的另一个节点应当收得到。"""
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
    listener = Node("test_car_listener")
    received: list[tuple[float, float]] = []
    done = threading.Event()

    def _on_msg(msg):
        received.append((msg.linear.x, msg.angular.z))
        done.set()

    listener.create_subscription(Twist, "cmd_vel", _on_msg, qos)

    # 给 DDS 发现留时间：反复发，同时自旋收
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not done.is_set():
        bridge.publish_velocity(0.25, 0.0)
        rclpy.spin_once(listener, timeout_sec=0.1)

    listener.destroy_node()
    assert received, "5 秒内没收到自己发的 cmd_vel —— DDS 发现或发布有问题"
    assert received[-1] == pytest.approx((0.25, 0.0))


def test_publish_servo_rejects_wrong_length(bridge):
    with pytest.raises(ValueError):
        bridge.publish_servo([1500, 1500])


def test_state_reports_offline_before_any_uplink(bridge):
    state = bridge.state()
    assert state.online is False
    assert state.traveled == 0.0
