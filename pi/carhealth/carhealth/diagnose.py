"""把已知故障库求值成 Diagnosis。规则异常不得影响主流程。"""

from __future__ import annotations

from .faults import FaultContext, RULES
from .types import Diagnosis


def diagnose(ctx: FaultContext) -> dict[str | None, list[Diagnosis]]:
    out: dict[str | None, list[Diagnosis]] = {}
    for rule in RULES:
        try:
            hit = rule.when(ctx)
        except Exception:                                   # noqa: BLE001
            hit = False
        if not hit:
            continue
        diagnosis = Diagnosis(rule.id, rule.title, rule.action_class, rule.hint)
        for target in (rule.components or (None,)):
            out.setdefault(target, []).append(diagnosis)
    return out
