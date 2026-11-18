"""ROS 话题新鲜度。

纯存储部分可单测；rclpy 后端只在 start() 里导入，ROS 不可用时只影响话题类探针。
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass

from ..types import ProbeResult

_TOF_FIELDS = ("connected", "connecting", "frames", "fps", "reconnects",
               "age_s", "device", "port", "last_error", "frame_id")
_ARM_HEALTH_FIELDS = ("busy", "pid", "uptime_s", "state", "feedback_age_s",
                      "last_goal_wall")
_DRIVER_HEALTH_FIELDS = (
    "arm_online", "arm_estop", "arm_status", "ctrl_live", "arm_cmd_age_s",
    "chassis_faults", "battery_voltage", "seen55", "seen56",
    "feedback_age_s", "telemetry_age_s", "cmd_age_s",
    "cmd_vx", "cmd_wz", "odom_vx", "odom_wz",
)


def _pick(data: dict, fields) -> dict:
    return {k: data[k] for k in fields if k in data}


def tof_status_evidence(data: dict) -> dict:
    return _pick(data, _TOF_FIELDS)


def arm_health_evidence(data: dict) -> dict:
    return _pick(data, _ARM_HEALTH_FIELDS)


def driver_health_evidence(data: dict) -> dict:
    return _pick(data, _DRIVER_HEALTH_FIELDS)


@dataclass
class _Record:
    last_mono: float | None = None
    count: int = 0
    payload: dict | None = None


class RosTopicStore:
    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._recs: dict[str, _Record] = {}
        self._publisher_count = None
        self._blind: str | None = None

    def set_publisher_counter(self, fn) -> None:
        """rclpy 后端注入 node.count_publishers；没有后端时为 None。"""
        self._publisher_count = fn

    def set_blind(self, reason: str) -> None:
        """后端起不来（没 source ROS / rclpy 缺失）时，话题探针必须是 UNKNOWN，
        不能把「我看不见」伪装成「组件没数据」。"""
        self._blind = reason

    def record(self, topic: str, payload: dict | None = None) -> None:
        with self._lock:
            rec = self._recs.setdefault(topic, _Record())
            rec.last_mono = self._clock()
            rec.count += 1
            rec.payload = dict(payload or {})

    def _publishers(self, topic: str):
        if self._publisher_count is None:
            return None
        try:
            return int(self._publisher_count(topic))
        except Exception:                                   # noqa: BLE001
            return None

    def snapshot(self, topic: str) -> ProbeResult:
        if self._blind is not None:
            return ProbeResult(ok=None, error=self._blind)
        publishers = self._publishers(topic)
        with self._lock:
            rec = self._recs.get(topic)
            if rec is None or rec.last_mono is None:
                return ProbeResult(ok=True, evidence={
                    "age_s": None, "count": 0, "publishers": publishers,
                })
            age = max(0.0, self._clock() - rec.last_mono)
            ev = dict(rec.payload or {})
            ev["age_s"] = round(age, 3)
            ev["count"] = rec.count
            ev["publishers"] = publishers
            return ProbeResult(ok=True, evidence=ev)


class TopicProbe:
    def __init__(self, name: str, topic: str, store: RosTopicStore, interval_s: float = 0.5):
        self.name = name
        self.topic = topic
        self.interval_s = interval_s
        self._store = store

    def poll(self) -> ProbeResult:
        return self._store.snapshot(self.topic)


class RosNodeBackend:
    """单订阅节点 + 独立 spin 线程。"""

    def __init__(self, store: RosTopicStore):
        self._store = store
        self._node = None
        self._executor = None
        self._thread = None
        self.reason: str | None = None

    def start(self) -> bool:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import String
        except Exception as e:                              # noqa: BLE001
            self.reason = "rclpy/消息包不可用: %s" % e
            return False

        try:
            rclpy.init(args=None)
            node = rclpy.create_node("carhealth_probe")
            self._node = node

            def sub_json(topic, convert):
                def cb(msg):
                    try:
                        data = json.loads(msg.data)
                    except ValueError:
                        data = {}
                    if isinstance(data, dict):
                        self._store.record(topic, convert(data))
                node.create_subscription(String, topic, cb, 10)

            self._store.set_publisher_counter(node.count_publishers)

            sub_json("/tof/status", tof_status_evidence)
            sub_json("/health/arm_grasp", arm_health_evidence)

            node.create_subscription(String, "/k230/result",
                                     lambda m: self._store.record("/k230/result", {}), 10)
            # ★ 只订阅契约真正用到的低/中频话题。
            # ★ 2026-11-18 Pi 实测：直接订阅 25/50Hz 话题时，rclpy 每条消息的固定
            #   开销让守护稳定吃 18~28% CPU。驱动节点现在自己发 1Hz /health/driver
            #   摘要；守护只订阅低/中频话题，总量约 13 条/秒。
            sub_json("/health/driver", driver_health_evidence)

            self._executor = SingleThreadedExecutor()
            self._executor.add_node(node)
            self._thread = threading.Thread(target=self._executor.spin,
                                            name="carhealth-ros", daemon=True)
            self._thread.start()
            return True
        except Exception as e:                              # noqa: BLE001
            self.reason = "ROS 节点启动失败: %s" % e
            return False

    def stop(self) -> None:
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:                                   # noqa: BLE001
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:                                   # noqa: BLE001
            pass
        try:
            import rclpy
            rclpy.shutdown()
        except Exception:                                   # noqa: BLE001
            pass
