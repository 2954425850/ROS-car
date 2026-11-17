from carhealth.probes.service_health import ServiceHealthProbe
from carhealth.types import ProbeResult


class FakeUnit:
    def __init__(self):
        self.n = 0
        self.mem = 10.0
        self.cpu_ns = 0

    def poll(self):
        return ProbeResult(ok=True, evidence={"n_restarts": self.n,
                                              "memory_mb": self.mem,
                                              "cpu_usage_ns": self.cpu_ns})


def test_crash_loop_detected_and_baseline_resets():
    now = [0.0]
    a, b = FakeUnit(), FakeUnit()
    p = ServiceHealthProbe({"a": a, "b": b}, clock=lambda: now[0], window_s=60)
    p.poll()
    a.n = 2
    now[0] = 10.0
    r = p.poll()
    assert r.evidence["crash_loop_units"] == ["a"]
    assert r.evidence["memory_mb"]["a"] == 10.0
    assert r.evidence["n_restarts"]["a"] == 2


def test_crash_loop_window_expires():
    now = [0.0]
    a = FakeUnit()
    p = ServiceHealthProbe({"a": a}, clock=lambda: now[0], window_s=60)
    p.poll()
    now[0] = 120.0
    a.n = 2
    assert p.poll().evidence["crash_loop_units"] == []


def test_unavailable_unit_is_skipped():
    class Bad:
        def poll(self):
            return ProbeResult(ok=None, error="no systemctl")

    r = ServiceHealthProbe({"x": Bad()}, clock=lambda: 0.0).poll()
    assert r.evidence["n_restarts"] == {}


def test_cpu_pct_from_cpu_usage_delta():
    now = [0.0]
    a = FakeUnit()
    p = ServiceHealthProbe({"a": a}, clock=lambda: now[0])
    p.poll()
    a.cpu_ns = 2_000_000_000          # 2 秒 CPU 时间
    now[0] = 2.0                      # 墙钟 2 秒 -> 100%
    r = p.poll()
    assert r.evidence["cpu_pct"]["a"] == 100.0
