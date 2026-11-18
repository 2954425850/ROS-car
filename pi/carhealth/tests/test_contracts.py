from carhealth.contracts import build_specs
from carhealth.types import CHECK_SEVERITY, Verdict


def test_component_names_unique_and_ordered_for_blocking():
    specs = build_specs()
    names = [s.name for s in specs]
    assert len(names) == len(set(names))
    seen = set()
    for s in specs:
        for dep in s.blocked_by:
            assert dep in seen, f"{s.name} 的根因 {dep} 必须排在它前面"
        seen.add(s.name)


def test_every_check_has_known_level_and_probe_name():
    known_probes = {
        "power", "host",
        "unit:ps2-teleop", "unit:tof-a010", "unit:k230-resultd",
        "unit:k230-bridge", "unit:k230-framed", "unit:rosbridge",
        "unit:arm-grasp",
        "udev:l150pro", "udev:a010", "udev:js0",
        "file:resultd-status", "file:framed-status", "file:latest-idr",
        "file:voice-heartbeat",
        "net:board", "net:rosbridge",
        "topic:/tof/status", "topic:/health/driver", "topic:/k230/result",
        "topic:/health/arm_grasp",
    }
    for spec in build_specs():
        assert spec.display
        assert spec.checks
        for c in spec.checks:
            assert c.probe in known_probes, f"{spec.name} 引用了未知探针 {c.probe}"
            assert c.level in CHECK_SEVERITY
            assert c.reason, f"{spec.name} 的 {c.probe} 判据缺中文原因"


def test_power_and_board_are_root_causes_with_no_dependencies():
    specs = {s.name: s for s in build_specs()}
    assert specs["power"].blocked_by == ()
    assert specs["board"].blocked_by == ()
    assert specs["arm"].blocked_by == ("power",)
    assert specs["k230_link"].blocked_by == ("board",)
    assert specs["k230_video"].blocked_by == ("board",)


def test_tof_has_interface_number_check():
    """同颗 FT2232 的通道 B 也能被打开、但一个字节都收不到（2026-10-14 实测）。"""
    tof = {s.name: s for s in build_specs()}["tof"]
    fields = {(c.probe, c.field, c.op, c.value) for c in tof.checks}
    assert ("udev:a010", "interface", "eq", "00") in fields


def test_tof_does_not_check_last_error():
    """tof_a010 重连成功后不清 last_error（serial_dev._disconnect 只写不清），
    拿它当判据会永久误报。"""
    tof = {s.name: s for s in build_specs()}["tof"]
    assert all(c.field != "last_error" for c in tof.checks)


def test_tof_reports_by_id_fallback_as_degraded():
    tof = {s.name: s for s in build_specs()}["tof"]
    fields = {(c.probe, c.field, c.op, c.value, c.level) for c in tof.checks}
    assert ("udev:a010", "resolved_by", "eq", "short", Verdict.DEGRADED) in fields


def test_battery_component_warns_below_threshold():
    battery = {s.name: s for s in build_specs()}["battery"]
    fields = {(c.probe, c.field, c.op, c.value) for c in battery.checks}
    assert ("topic:/health/driver", "battery_voltage", "gt", 11.5) in fields
