"""把探针结果判成组件状态，并挂载环境根因。纯函数，方便单测。"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, replace

from .types import (
    CHECK_SEVERITY, Check, ComponentSpec, ProbeResult, Verdict, compare,
)


@dataclass(frozen=True)
class Evaluation:
    verdict: Verdict
    reason: str
    evidence: dict[str, dict]          # probe 名 -> 该探针的 evidence（避免 age_s 互相覆盖）
    failing_reasons: tuple[str, ...]
    failing_probes: tuple[str, ...]
    probes_ok: dict[str, bool | None]
    root_cause: str | None = None


def _dedupe(items):
    return tuple(dict.fromkeys(items))


def evaluate(spec: ComponentSpec, results: dict[str, ProbeResult],
             states: dict[str, Evaluation]) -> Evaluation:
    probes_ok: dict[str, bool | None] = {}
    failing: list[tuple[Check, str]] = []
    unknown: list[str] = []

    for check in spec.checks:
        r = results.get(check.probe)
        probes_ok[check.probe] = None if r is None else r.ok
        if r is None or r.ok is not True:
            unknown.append(check.probe)
            continue
        if check.field is not None:
            if check.field not in r.evidence:
                unknown.append(check.probe)
                continue
            actual = r.evidence[check.field]
        else:
            actual = r.evidence
        if not compare(actual, check.op, check.value):
            failing.append((check, check.probe))

    evidence = {}
    for probe_name in _dedupe(c.probe for c in spec.checks):
        r = results.get(probe_name)
        if r is not None:
            evidence[probe_name] = dict(r.evidence)

    blocked_dep = None
    blocked_reason = ""
    for dep in spec.blocked_by:
        dep_ev = states.get(dep)
        if dep_ev is not None and dep_ev.verdict is Verdict.BLOCKED:
            blocked_dep, blocked_reason = dep, dep_ev.reason
            break

    if failing:
        worst = max(CHECK_SEVERITY[c.level] for c, _ in failing)
        top = [c for c, _ in failing if CHECK_SEVERITY[c.level] == worst]
        base = Evaluation(
            verdict=top[0].level,
            reason=top[0].reason or ("%s 判据不满足" % top[0].probe),
            evidence=evidence,
            failing_reasons=_dedupe(c.reason or c.probe for c, _ in failing),
            failing_probes=_dedupe(p for _, p in failing),
            probes_ok=probes_ok,
        )
        if blocked_dep is None:
            return base
        # 只有自己确实失败/失明时，才把根因挂上去；健康的组件不许被 BLOCKED 掩盖。
        return replace(
            base,
            verdict=Verdict.BLOCKED,
            root_cause=blocked_dep,
            reason="被 %s 压住（%s）；本组件判据：%s" % (
                blocked_dep, blocked_reason, base.reason),
        )

    if unknown:
        reason = "探针不可用：%s" % "、".join(_dedupe(unknown))
        if blocked_dep is not None:
            return Evaluation(
                verdict=Verdict.BLOCKED,
                reason="被 %s 压住（%s）；%s" % (blocked_dep, blocked_reason, reason),
                evidence=evidence, failing_reasons=(), failing_probes=(),
                probes_ok=probes_ok, root_cause=blocked_dep,
            )
        return Evaluation(
            verdict=Verdict.UNKNOWN, reason=reason, evidence=evidence,
            failing_reasons=(), failing_probes=(), probes_ok=probes_ok,
        )

    return Evaluation(Verdict.OK, "正常", evidence, (), (), probes_ok)


def attach_graph_root_cause(evals: dict[str, Evaluation],
                            specs: tuple[ComponentSpec, ...]) -> dict[str, Evaluation]:
    """≥3 个「单元健康但话题全停」的组件同时异常时，用发布者数量区分根因。

    - 所有失败话题 `count_publishers == 0` → `ros_discovery`（DDS 发现/网络）
    - 任一失败话题有发布者但没数据   → `ros_data_path`（QoS/MTU/网络）
    - 发布者数量未知（没有 rclpy 后端）→ **不归因**，宁可 UNKNOWN，不靠症状数量猜
    """
    by_name = {s.name: s for s in specs}
    candidates: list[str] = []
    pub_counts: list[int] = []

    for name, ev in evals.items():
        if ev.verdict not in (Verdict.STALE, Verdict.DEGRADED):
            continue
        spec = by_name[name]
        has_topic_check = any(c.probe.startswith("topic:") for c in spec.checks)
        if not has_topic_check:
            continue
        other_probes = {c.probe for c in spec.checks if not c.probe.startswith("topic:")}
        if any(ev.probes_ok.get(p) is not True for p in other_probes):
            continue
        if not ev.failing_probes or not all(p.startswith("topic:") for p in ev.failing_probes):
            continue
        for probe_name in ev.failing_probes:
            publishers = ev.evidence.get(probe_name, {}).get("publishers")
            if not isinstance(publishers, int):
                return evals          # 证据不足，不归因
            pub_counts.append(publishers)
        candidates.append(name)

    if len(candidates) < 3:
        return evals

    if any(count > 0 for count in pub_counts):
        cause = "ros_data_path"
        reason = "有发布者但收不到数据：查 QoS 兼容、MTU 分片、WiFi 质量"
    else:
        cause = "ros_discovery"
        reason = "多个话题 0 发布者：判 DDS 发现/网络故障，通常要整条 ROS 栈重启"

    out = dict(evals)
    for name in candidates:
        out[name] = replace(
            evals[name], verdict=Verdict.BLOCKED, root_cause=cause, reason=reason,
        )
    return out


def fingerprint(component: str, verdict: Verdict, root_cause: str | None,
                failing: list[str] | tuple[str, ...]) -> str:
    """同一组件、同一状态、同一组判据失败原因 -> 同一指纹。用于二期熔断。"""
    raw = "|".join([component, verdict.value, root_cause or "", *sorted(failing)])
    return hashlib.sha1(raw.encode("utf-8")).hexdigest()[:12]
