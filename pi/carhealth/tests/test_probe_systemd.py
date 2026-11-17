import subprocess

from carhealth.probes.systemd import SystemdCache, UnitProbe

SHOW_OUTPUT = """\
Id=ps2-teleop.service
ActiveState=active
SubState=running
NRestarts=2
MemoryCurrent=125829120
CPUUsageNSec=9876543210

Id=tof-a010.service
ActiveState=failed
SubState=failed
NRestarts=7
MemoryCurrent=[not set]
CPUUsageNSec=[not set]
"""


class FakeRunner:
    def __init__(self, out=SHOW_OUTPUT, rc=0):
        self.out = out
        self.rc = rc
        self.calls = 0

    def __call__(self, cmd, **kw):
        self.calls += 1
        return subprocess.CompletedProcess(cmd, self.rc, self.out, "")


def test_batches_one_call_for_all_units():
    runner = FakeRunner()
    cache = SystemdCache(["ps2-teleop", "tof-a010"], runner=runner)
    a = UnitProbe("unit:ps2-teleop", "ps2-teleop", cache).poll()
    b = UnitProbe("unit:tof-a010", "tof-a010", cache).poll()
    assert runner.calls == 1
    assert a.evidence["active"] is True
    assert a.evidence["n_restarts"] == 2
    assert a.evidence["memory_mb"] == 120.0
    assert a.evidence["cpu_usage_ns"] == 9876543210
    assert b.evidence["active"] is False
    assert b.evidence["active_state"] == "failed"
    assert b.evidence["memory_mb"] is None


def test_systemctl_failure_is_unknown():
    runner = FakeRunner(out="", rc=1)
    cache = SystemdCache(["ps2-teleop"], runner=runner)
    r = UnitProbe("unit:ps2-teleop", "ps2-teleop", cache).poll()
    assert r.ok is None
    assert "systemctl" in r.error


def test_refresh_interval_is_respected():
    now = [0.0]
    runner = FakeRunner()
    cache = SystemdCache(["ps2-teleop"], runner=runner, interval_s=1.0, clock=lambda: now[0])
    probe = UnitProbe("unit:ps2-teleop", "ps2-teleop", cache)
    probe.poll()
    now[0] = 0.5
    probe.poll()
    assert runner.calls == 1
    now[0] = 1.1
    probe.poll()
    assert runner.calls == 2
