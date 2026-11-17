import pytest

from carhealth.types import (
    CHECK_SEVERITY, Check, ComponentSpec, ComponentState, Diagnosis,
    ProbeResult, Verdict, compare,
)


def test_compare_numeric_operators():
    assert compare(3, "gt", 2)
    assert compare(2, "ge", 2)
    assert compare(1, "lt", 2)
    assert compare(2, "le", 2)
    assert not compare(None, "lt", 2)


def test_compare_equality_and_specials():
    assert compare(True, "eq", True)
    assert compare(True, "truthy", None)
    assert compare(None, "is_none", None)
    assert compare("00", "eq", "00")
    assert not compare(None, "eq", False)     # 探针没报 = 不知道，不是 false


def test_compare_rejects_unknown_op():
    with pytest.raises(ValueError):
        compare(1, "approximately", 1)


def test_verdict_strings_are_stable():
    """日志里存的是 verdict.value，改名等于破坏历史日志兼容性。"""
    assert [v.value for v in Verdict] == [
        "OK", "DEGRADED", "STALE", "ABSENT", "BLOCKED", "UNKNOWN",
    ]


def test_check_severity_only_contains_failure_levels():
    assert CHECK_SEVERITY[Verdict.ABSENT] > CHECK_SEVERITY[Verdict.STALE]
    assert CHECK_SEVERITY[Verdict.STALE] > CHECK_SEVERITY[Verdict.DEGRADED]
    assert CHECK_SEVERITY[Verdict.BLOCKED] > CHECK_SEVERITY[Verdict.ABSENT]


def test_component_state_is_constructible():
    d = Diagnosis(id="usb.disconnect_storm", title="USB 在反复重枚举",
                  action_class="human", hint="换线/换口")
    st = ComponentState(
        name="tof", display="ToF", verdict=Verdict.OK, reason="正常",
        evidence={"topic:/tof/status": {"age_s": 0.1}}, root_cause=None,
        failing=(), diagnoses=(d,), since=1.0, updated=1.0,
    )
    assert st.verdict is Verdict.OK
    assert st.diagnoses[0].action_class == "human"
