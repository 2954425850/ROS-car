from carhealth.classify import Evaluation
from carhealth.diagnose import diagnose
from carhealth.faults import FaultContext
from carhealth.types import ProbeResult, Verdict


def ctx(evals=None, **probes):
    results = {k: ProbeResult(ok=True, evidence=v) for k, v in probes.items()}
    return FaultContext(results=results, evals=evals or {}, now_mono=100.0)


def hits(context):
    return {d.id for ds in diagnose(context).values() for d in ds}


def root_eval(root):
    return Evaluation(Verdict.BLOCKED, "x", {}, (), (), {}, root_cause=root)


def test_undervoltage():
    assert "power.undervoltage" in hits(ctx(power={"uv_now": True}))
    assert "power.undervoltage" in hits(ctx(power={"uv_recent": True}))
    assert "power.undervoltage" not in hits(ctx(power={"uv_since_boot": True}))


def test_power_probe_unavailable_is_actionable():
    """Pi 5 实测：/dev/vcio 是 root:video，cy 不在 video 组 ⇒ vcgencmd 读不到。"""
    c = FaultContext(
        results={"power": ProbeResult(ok=None, error="Can't open device file")},
        evals={}, now_mono=0.0)
    assert "power.probe_unavailable" in hits(c)


def test_usb_disconnect_storm():
    assert "usb.disconnect_storm" in hits(
        ctx(usb={"disconnects": {"0403:6010": 3}}))
    assert "usb.disconnect_storm" not in hits(
        ctx(usb={"disconnects": {"0403:6010": 2}}))


def test_dds_stale_ip():
    assert "dds.stale_ip" in hits(ctx(dds={"stale_binds": ["192.168.1.108"]}))


def test_crash_loop_and_config_drift():
    assert "service.crash_loop" in hits(
        ctx(service={"crash_loop_units": ["tof-a010"]}))
    assert "service.ordering_cycle" in hits(
        ctx(config_drift={"ordering_cycles": 1}))
    assert "udev.rules_missing" in hits(
        ctx(config_drift={"missing_udev_rules": ["99-tof-a010.rules"]}))
    assert "build.stale" in hits(
        ctx(config_drift={"stale_packages": ["arm_grasp"]}))


def test_k230_host_unknown():
    assert "k230.host_unknown" in hits(ctx(**{"net:board": {"host": None}}))


def test_battery_low():
    assert "battery.low" in hits(ctx(**{"topic:/battery_voltage": {"data": 11.1}}))
    assert "battery.low" not in hits(ctx(**{"topic:/battery_voltage": {"data": 12.0}}))


def test_cpu_rules():
    assert "host.cpu_high" in hits(ctx(host={"cpu_pct": 90.0}))
    assert "service.cpu_high" in hits(ctx(service={"cpu_pct": {"x": 250.0}}))
    assert "host.cpu_high" not in hits(ctx(host={"cpu_pct": 40.0}))


def test_gateway_latency_rule():
    assert "host.gateway_latency" in hits(
        ctx(**{"net:gateway": {"rtt_avg_ms": 141.0}}))
    assert "host.gateway_latency" not in hits(
        ctx(**{"net:gateway": {"rtt_avg_ms": 3.9, "rtt_max_ms": 10.0,
                               "loss_pct": 0.0}}))


def test_asr_error_rule():
    assert "voice.asr_error" in hits(
        ctx(**{"file:voice-heartbeat": {
            "asr_last_error": "status=44: request timeout"}}))


def test_wakeword_lost_and_api_stuck():
    assert "voice.wakeword_lost" in hits(
        ctx(**{"file:voice-heartbeat": {"wakeword_ready": False}}))
    assert "voice.api_stuck" in hits(
        ctx(**{"file:voice-heartbeat": {"busy_for_s": 200.0}}))
    assert "voice.mic_error" in hits(
        ctx(**{"file:voice-heartbeat": {"mic_last_error": "Input overflow"}}))


def test_tof_channel_b_and_by_id_only():
    assert "udev.tof_channel_b" in hits(ctx(**{"udev:a010": {"interface": "01"}}))
    assert "udev.tof_byid_only" in hits(
        ctx(**{"udev:a010": {"resolved_by": "by_id"}}))


def test_ros_root_causes():
    evals = {"a": root_eval("ros_discovery")}
    assert "ros.discovery" in hits(ctx(evals=evals))
    evals = {"a": root_eval("ros_data_path")}
    assert "ros.data_path" in hits(ctx(evals=evals))


def test_diagnosis_targets_component():
    out = diagnose(ctx(power={"uv_now": True}))
    assert out["power"][0].id == "power.undervoltage"
    assert out["power"][0].action_class == "wait"


def test_clean_context_has_no_diagnosis():
    assert diagnose(ctx(power={"uv_now": False, "uv_recent": False},
                        host={"load1": 0.5, "wifi_link": 60.0},
                        dds={"stale_binds": []})) == {}


def test_rule_exception_does_not_break_diagnose():
    class Boom:
        id = "x"
        title = "x"
        action_class = "none"
        hint = "x"
        components = ()
        def when(self, c):
            raise RuntimeError("bug")

    import carhealth.diagnose as d
    old = d.RULES
    d.RULES = (Boom(),)
    try:
        assert d.diagnose(ctx()) == {}
    finally:
        d.RULES = old
