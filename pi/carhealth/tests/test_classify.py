from dataclasses import replace

from carhealth.classify import (
    Evaluation, attach_graph_root_cause, evaluate, fingerprint,
)
from carhealth.types import Check, ComponentSpec, ProbeResult, Verdict


def spec(*checks, blocked_by=()):
    return ComponentSpec(name="x", display="X", checks=tuple(checks), blocked_by=blocked_by)


def ok_probe(**evidence):
    return ProbeResult(ok=True, evidence=evidence)


def test_all_checks_pass_is_ok():
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="太旧"))
    ev = evaluate(s, {"p": ok_probe(age_s=0.2)}, {})
    assert ev.verdict is Verdict.OK
    assert ev.failing_reasons == ()


def test_stale_check_reports_level_and_reason():
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="太旧"))
    ev = evaluate(s, {"p": ok_probe(age_s=3.0)}, {})
    assert ev.verdict is Verdict.STALE
    assert ev.reason == "太旧"
    assert ev.failing_probes == ("p",)


def test_worst_level_wins_but_all_reasons_are_kept():
    s = spec(
        Check("p", "gt", 0, field="fps", level=Verdict.DEGRADED, reason="fps 掉了"),
        Check("p", "lt", 2.0, field="age_s", level=Verdict.STALE, reason="没数据"),
    )
    ev = evaluate(s, {"p": ok_probe(fps=0, age_s=5)}, {})
    assert ev.verdict is Verdict.STALE
    assert "fps 掉了" in ev.failing_reasons
    assert "没数据" in ev.failing_reasons


def test_missing_field_is_unknown_not_ok():
    """探针没报这个字段 ≠ 这个字段合格。这条是防假绿的关键。"""
    s = spec(Check("p", "eq", True, field="connected", level=Verdict.STALE, reason="没连上"))
    ev = evaluate(s, {"p": ok_probe()}, {})
    assert ev.verdict is Verdict.UNKNOWN
    assert "探针不可用" in ev.reason


def test_probe_itself_failed_is_unknown():
    s = spec(Check("p", "eq", True, field="x", level=Verdict.STALE, reason="坏"))
    ev = evaluate(s, {"p": ProbeResult(ok=None, error="systemctl 挂了")}, {})
    assert ev.verdict is Verdict.UNKNOWN


def test_blocked_dependency_wins_over_local_symptoms():
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
             blocked_by=("power",))
    power = Evaluation(Verdict.BLOCKED, "欠压", {}, (), (), {}, root_cause="power")
    ev = evaluate(s, {"p": ok_probe(age_s=9)}, {"power": power})
    assert ev.verdict is Verdict.BLOCKED
    assert ev.root_cause == "power"
    assert "欠压" in ev.reason


def test_healthy_component_is_not_masked_by_blocked_dependency():
    """欠压期间底盘完全正常时不能显示 BLOCKED——否则真故障会被掩盖。"""
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
             blocked_by=("power",))
    power = Evaluation(Verdict.BLOCKED, "欠压", {}, (), (), {}, root_cause="power")
    ev = evaluate(s, {"p": ok_probe(age_s=0.1)}, {"power": power})
    assert ev.verdict is Verdict.OK
    assert ev.root_cause is None


def test_unknown_component_with_blocked_dependency_is_blocked():
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
             blocked_by=("power",))
    power = Evaluation(Verdict.BLOCKED, "欠压", {}, (), (), {}, root_cause="power")
    ev = evaluate(s, {"p": ProbeResult(ok=None, error="失明")}, {"power": power})
    assert ev.verdict is Verdict.BLOCKED
    assert ev.root_cause == "power"


def test_blocked_dependency_not_blocked_falls_through():
    s = spec(Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
             blocked_by=("power",))
    power = Evaluation(Verdict.OK, "正常", {}, (), (), {})
    ev = evaluate(s, {"p": ok_probe(age_s=0.1)}, {"power": power})
    assert ev.verdict is Verdict.OK


def test_is_none_operator():
    s = spec(Check("p", "is_none", None, field="last_error", level=Verdict.DEGRADED, reason="有错误"))
    assert evaluate(s, {"p": ok_probe(last_error=None)}, {}).verdict is Verdict.OK
    assert evaluate(s, {"p": ok_probe(last_error="boom")}, {}).verdict is Verdict.DEGRADED


def _graph_case(publishers, n=3, unit_active=True):
    specs = []
    results = {"unit": ProbeResult(ok=True, evidence={"active": unit_active})}
    for name in ("a", "b", "c")[:n]:
        specs.append(ComponentSpec(name, name, (
            Check("unit", "eq", True, field="active", level=Verdict.ABSENT, reason="unit 不在"),
            Check("topic:" + name, "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
        )))
        results["topic:" + name] = ok_probe(age_s=9, publishers=publishers)
    evals = {s.name: evaluate(s, results, {}) for s in specs}
    return specs, evals


def test_zero_publishers_means_discovery_fault():
    specs, evals = _graph_case(publishers=0)
    fixed = attach_graph_root_cause(evals, specs)
    assert all(fixed[n].verdict is Verdict.BLOCKED for n in ("a", "b", "c"))
    assert all(fixed[n].root_cause == "ros_discovery" for n in ("a", "b", "c"))


def test_publishers_alive_but_no_data_means_data_path_fault():
    specs, evals = _graph_case(publishers=2)
    fixed = attach_graph_root_cause(evals, specs)
    assert all(fixed[n].root_cause == "ros_data_path" for n in ("a", "b", "c"))


def test_unknown_publisher_count_does_not_attribute():
    """没有 rclpy 后端时 publishers 缺失；不能靠症状数量硬猜。"""
    specs, evals = _graph_case(publishers=None)
    assert attach_graph_root_cause(evals, specs) == evals


def test_graph_fault_needs_three_candidates():
    specs, evals = _graph_case(publishers=0, n=2)
    assert attach_graph_root_cause(evals, specs) == evals


def test_graph_fault_ignores_components_whose_unit_is_down():
    specs, evals = _graph_case(publishers=0, unit_active=False)
    fixed = attach_graph_root_cause(evals, specs)
    assert fixed["a"].verdict is Verdict.ABSENT


def test_fingerprint_is_stable_and_sensitive():
    a = fingerprint("tof", Verdict.STALE, None, ["没数据"])
    b = fingerprint("tof", Verdict.STALE, None, ["没数据"])
    c = fingerprint("tof", Verdict.STALE, None, ["连不上"])
    assert a == b
    assert a != c
    assert len(a) == 12
