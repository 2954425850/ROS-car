"""健康判定公共类型。纯标准库，不依赖 ROS / systemd。"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Any, Protocol


class Verdict(enum.Enum):
    OK = "OK"
    DEGRADED = "DEGRADED"
    STALE = "STALE"
    ABSENT = "ABSENT"
    BLOCKED = "BLOCKED"
    UNKNOWN = "UNKNOWN"


# 只在「多个 check 失败时挑最严重的那个」时使用。
# BLOCKED 代表环境根因（欠压 / 板子离线），比任何单点症状都优先。
CHECK_SEVERITY = {
    Verdict.DEGRADED: 1,
    Verdict.STALE: 2,
    Verdict.ABSENT: 3,
    Verdict.BLOCKED: 4,
}


@dataclass
class ProbeResult:
    ok: bool | None = None
    evidence: dict[str, Any] = field(default_factory=dict)
    error: str | None = None


class Probe(Protocol):
    name: str
    interval_s: float

    def poll(self) -> ProbeResult: ...


@dataclass(frozen=True)
class Check:
    probe: str
    op: str
    value: Any = None
    field: str | None = None
    level: Verdict = Verdict.STALE
    reason: str = ""


@dataclass(frozen=True)
class ComponentSpec:
    name: str
    display: str
    checks: tuple[Check, ...]
    blocked_by: tuple[str, ...] = ()


@dataclass(frozen=True)
class Diagnosis:
    """已知故障库的命中结果。只给结论与指令，不改变六态判定。"""
    id: str
    title: str
    action_class: str      # wait / restart_unit / restart_stack / human / fix_config
    hint: str


@dataclass
class ComponentState:
    name: str
    display: str
    verdict: Verdict
    reason: str
    evidence: dict[str, dict]
    root_cause: str | None
    failing: tuple[str, ...]
    diagnoses: tuple[Diagnosis, ...]
    since: float
    updated: float


def compare(actual: Any, op: str, expected: Any) -> bool:
    """判据比较。字段缺省由调用方处理成「不知道」，这里只处理已取到的值。"""
    if op == "exists":
        return actual is not None
    if op == "truthy":
        return bool(actual)
    if op == "falsy":
        return not bool(actual)
    if op == "is_none":
        return actual is None
    if op == "eq":
        return actual == expected
    if op == "ne":
        return actual != expected
    if op in ("lt", "le", "gt", "ge"):
        try:
            a, b = float(actual), float(expected)
        except (TypeError, ValueError):
            return False
        if op == "lt":
            return a < b
        if op == "le":
            return a <= b
        if op == "gt":
            return a > b
        return a >= b
    raise ValueError(f"未知比较操作: {op}")
