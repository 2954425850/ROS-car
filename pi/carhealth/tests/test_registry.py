from carhealth.contracts import build_specs
from carhealth.registry import build_engine, build_probes


def test_all_contract_probes_are_built():
    bundle = build_probes(ros=False, board_host="127.0.0.1")
    try:
        needed = {c.probe for s in build_specs() for c in s.checks}
        missing = needed - set(bundle.probes)
        assert not missing, missing
    finally:
        if bundle.closer:
            bundle.closer()


def test_build_engine_wires_specs_and_probes(tmp_path):
    eng = build_engine(state_dir=tmp_path, journal_dir=tmp_path,
                       ros=False, board_host="127.0.0.1")
    try:
        assert {s.name for s in eng.specs} == {s.name for s in build_specs()}
        assert "topic:/tof/status" in eng.probes
        assert "unit:ps2-teleop" in eng.probes
        assert {"service", "usb", "dds", "config_drift", "udev:js0"} <= set(eng.probes)
    finally:
        eng.close()


def test_ros_false_does_not_import_rclpy():
    bundle = build_probes(ros=False, board_host="127.0.0.1")
    try:
        assert bundle.closer is None
        # 没有后端时必须标 UNKNOWN，不能把「看不见」当成「没数据」。
        assert bundle.probes["topic:/tof/status"].poll().ok is None
    finally:
        if bundle.closer:
            bundle.closer()
