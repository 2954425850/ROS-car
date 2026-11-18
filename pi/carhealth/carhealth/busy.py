"""动作互锁。二期恢复动作的唯一开关；一期只写进日志。"""

from __future__ import annotations

from dataclasses import dataclass

from .types import ProbeResult


@dataclass(frozen=True)
class BusyState:
    arm: bool
    chassis: bool
    grasp: bool
    voice: bool

    @property
    def any(self) -> bool:
        return self.arm or self.chassis or self.grasp or self.voice


def _fresh(ev: dict, max_age: float) -> bool:
    age = ev.get("age_s")
    return isinstance(age, (int, float)) and age <= max_age


def _fresh_scalar(value, max_age: float) -> bool:
    return isinstance(value, (int, float)) and value <= max_age


def _num(ev: dict, key: str) -> float:
    try:
        return float(ev.get(key) or 0.0)
    except (TypeError, ValueError):
        return 0.0


def compute(results: dict[str, ProbeResult], voice_evidence: dict | None) -> BusyState:
    # 驱动把 /cmd_vel_mux、/wheel_odom、/arm/command 的最近值汇总到 1Hz
    # /health/driver；守护不再直接订阅这些 25/50Hz 话题。
    health = results.get("topic:/health/driver")
    ev = health.evidence if health and health.ok and _fresh(health.evidence, 2.0) else {}

    chassis = False
    if _fresh_scalar(ev.get("cmd_age_s"), 1.0):
        chassis = abs(_num(ev, "cmd_vx")) > 0.02 or abs(_num(ev, "cmd_wz")) > 0.05
    if _fresh_scalar(ev.get("telemetry_age_s"), 1.0):
        chassis = chassis or abs(_num(ev, "odom_vx")) > 0.02
        chassis = chassis or abs(_num(ev, "odom_wz")) > 0.05

    arm = _fresh_scalar(ev.get("arm_cmd_age_s"), 1.5)

    grasp = False
    health = results.get("topic:/health/arm_grasp")
    if health and health.ok and _fresh(health.evidence, 3.0):
        grasp = bool(health.evidence.get("busy"))
    arm = arm or grasp

    voice = bool((voice_evidence or {}).get("busy"))
    return BusyState(arm=arm, chassis=chassis, grasp=grasp, voice=voice)
