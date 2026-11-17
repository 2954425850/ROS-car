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


def _num(ev: dict, key: str) -> float:
    try:
        return float(ev.get(key) or 0.0)
    except (TypeError, ValueError):
        return 0.0


def compute(results: dict[str, ProbeResult], voice_evidence: dict | None) -> BusyState:
    arm_cmd = results.get("topic:/arm/command")
    arm = bool(arm_cmd and arm_cmd.ok and _fresh(arm_cmd.evidence, 1.5))

    chassis = False
    cmd = results.get("topic:/cmd_vel_mux")
    if cmd and cmd.ok and _fresh(cmd.evidence, 1.0):
        chassis = abs(_num(cmd.evidence, "linear_x")) > 0.02 or \
                  abs(_num(cmd.evidence, "angular_z")) > 0.05
    odom = results.get("topic:/wheel_odom")
    if odom and odom.ok and _fresh(odom.evidence, 1.0):
        chassis = chassis or abs(_num(odom.evidence, "linear_x")) > 0.02
        chassis = chassis or abs(_num(odom.evidence, "angular_z")) > 0.05

    grasp = False
    health = results.get("topic:/health/arm_grasp")
    if health and health.ok and _fresh(health.evidence, 3.0):
        grasp = bool(health.evidence.get("busy"))
    arm = arm or grasp

    voice = bool((voice_evidence or {}).get("busy"))
    return BusyState(arm=arm, chassis=chassis, grasp=grasp, voice=voice)
