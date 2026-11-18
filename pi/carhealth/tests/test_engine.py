import json

from carhealth.engine import Engine, EngineConfig
from carhealth.types import Check, ComponentSpec, ProbeResult, Verdict


class FakeProbe:
    def __init__(self, name, evidence, interval_s=0.0, ok=True):
        self.name = name
        self.interval_s = interval_s
        self.evidence = evidence
        self.ok = ok
        self.polls = 0

    def poll(self):
        self.polls += 1
        return ProbeResult(ok=self.ok, evidence=dict(self.evidence))


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def make(probe_evidence, specs=None, tmp_path=None):
    clock = Clock()
    probes = {p.name: p for p in probe_evidence}
    specs = specs or (
        ComponentSpec("a", "A", (
            Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
        )),
    )
    cfg = EngineConfig(state_dir=tmp_path / "state", journal_dir=tmp_path / "journal")
    return Engine(specs, probes, cfg, clock=clock, wall=lambda: 1763388000.0 + clock.now), clock


def test_ok_state(tmp_path):
    eng, _ = make([FakeProbe("p", {"age_s": 0.1})], tmp_path=tmp_path)
    states = eng.run_once()
    assert states["a"].verdict is Verdict.OK
    payload = json.loads((tmp_path / "state" / "state.json").read_text())
    assert payload["components"]["a"]["verdict"] == "OK"


def test_stale_state_and_reason(tmp_path):
    eng, _ = make([FakeProbe("p", {"age_s": 9.0})], tmp_path=tmp_path)
    assert eng.run_once()["a"].verdict is Verdict.STALE


def test_probe_exception_becomes_unknown(tmp_path):
    class Boom(FakeProbe):
        def poll(self):
            raise RuntimeError("boom")

    eng, _ = make([Boom("p", {})], tmp_path=tmp_path)
    assert eng.run_once()["a"].verdict is Verdict.UNKNOWN


def test_probe_interval_is_respected(tmp_path):
    probe = FakeProbe("p", {"age_s": 0.1}, interval_s=10.0)
    eng, clock = make([probe], tmp_path=tmp_path)
    eng.run_once()
    clock.now += 1.0
    eng.run_once()
    assert probe.polls == 1
    clock.now += 10.0
    eng.run_once()
    assert probe.polls == 2


def test_hysteresis_needs_two_bad_and_five_good(tmp_path):
    probe = FakeProbe("p", {"age_s": 0.1})
    eng, clock = make([probe], tmp_path=tmp_path)
    assert eng.run_once()["a"].verdict is Verdict.OK

    probe.evidence["age_s"] = 9.0
    clock.now += 1
    assert eng.run_once()["a"].verdict is Verdict.OK        # 1 次坏：不动
    clock.now += 1
    assert eng.run_once()["a"].verdict is Verdict.STALE     # 2 次坏：降级

    probe.evidence["age_s"] = 0.1
    for _ in range(4):
        clock.now += 1
        assert eng.run_once()["a"].verdict is Verdict.STALE
    clock.now += 1
    assert eng.run_once()["a"].verdict is Verdict.OK        # 5 次好：恢复


def test_since_resets_only_when_committed_verdict_changes(tmp_path):
    probe = FakeProbe("p", {"age_s": 0.1})
    eng, clock = make([probe], tmp_path=tmp_path)
    first = eng.run_once()["a"]
    clock.now += 5
    second = eng.run_once()["a"]
    assert first.since == second.since
    probe.evidence["age_s"] = 9.0
    clock.now += 1
    eng.run_once()            # 第一次坏，仍 OK
    clock.now += 1
    third = eng.run_once()["a"]
    assert third.since == clock.now


def test_diagnosis_is_attached_to_component(tmp_path):
    specs = (
        ComponentSpec("power", "供电", (
            Check("power", "eq", False, field="uv_now", level=Verdict.BLOCKED,
                  reason="欠压"),
        )),
    )
    eng, _ = make([FakeProbe("power", {"uv_now": True})], specs=specs,
                  tmp_path=tmp_path)
    st = eng.run_once()["power"]
    assert st.verdict is Verdict.BLOCKED
    assert [d.id for d in st.diagnoses] == ["power.undervoltage"]


def test_default_state_dir_respects_env(monkeypatch, tmp_path):
    import carhealth.engine as engine_mod
    monkeypatch.setenv("CARHEALTH_STATE_DIR", str(tmp_path))
    assert engine_mod.default_state_dir() == tmp_path


def test_busy_is_computed_and_written(tmp_path):
    eng, _ = make(
        [FakeProbe("p", {"age_s": 0.1}),
         FakeProbe("topic:/health/driver", {
             "age_s": 0.1, "cmd_age_s": 0.1, "cmd_vx": 0.3, "cmd_wz": 0.0})],
        tmp_path=tmp_path)
    eng.run_once()
    assert eng.last_busy.chassis is True
    payload = json.loads((tmp_path / "state" / "state.json").read_text())
    assert payload["busy"]["chassis"] is True


def _events(tmp_path):
    eng_journal = EngineConfig(state_dir=tmp_path / "state", journal_dir=tmp_path / "journal")
    from carhealth.journal import Journal
    return Journal(eng_journal.journal_dir).read()


def test_open_and_close_incident(tmp_path):
    probe = FakeProbe("p", {"age_s": 9.0})
    eng, clock = make([probe], tmp_path=tmp_path)
    eng.run_once()
    evs = _events(tmp_path)
    assert [e["event"] for e in evs] == ["incident_open"]
    assert evs[0]["component"] == "a"
    assert evs[0]["to"] == "STALE"
    assert evs[0]["action"] is None
    probe.evidence["age_s"] = 0.1
    for _ in range(5):          # 迟滞：5 次好才恢复
        clock.now += 1
        eng.run_once()
    evs = _events(tmp_path)
    assert [e["event"] for e in evs] == ["incident_open", "incident_close"]
    assert evs[1]["duration_s"] == 5.0


def test_unknown_does_not_open_component_incident(tmp_path):
    class Boom(FakeProbe):
        def poll(self):
            raise RuntimeError("probe blind")

    eng, clock = make([Boom("p", {})], tmp_path=tmp_path)
    eng.run_once()
    evs = _events(tmp_path)
    assert [e["event"] for e in evs] == ["monitor_blind"]
    clock.now += 10
    eng.run_once()
    assert [e["event"] for e in _events(tmp_path)] == ["monitor_blind"]  # 限流


def test_same_incident_not_duplicated(tmp_path):
    eng, clock = make([FakeProbe("p", {"age_s": 9.0})], tmp_path=tmp_path)
    eng.run_once()
    eng.run_once()
    assert len(_events(tmp_path)) == 1


def test_fingerprint_change_emits_update(tmp_path):
    probe = FakeProbe("p", {"age_s": 9.0})
    from carhealth.types import Check, ComponentSpec, Verdict
    specs = (
        ComponentSpec("a", "A", (
            Check("p", "lt", 1.0, field="age_s", level=Verdict.STALE, reason="没数据"),
            Check("p", "gt", 10.0, field="age_s", level=Verdict.DEGRADED, reason="帧率低"),
        )),
    )
    eng, _ = make([probe], specs=specs, tmp_path=tmp_path)
    eng.run_once()
    probe.evidence["age_s"] = 20.0
    eng.run_once()
    evs = _events(tmp_path)
    assert [e["event"] for e in evs] == ["incident_open", "incident_update"]
    assert evs[0]["fingerprint"] != evs[1]["fingerprint"]


def test_interlock_is_written_into_incident(tmp_path):
    eng, _ = make(
        [FakeProbe("p", {"age_s": 9.0}),
         FakeProbe("topic:/health/driver", {
             "age_s": 0.1, "cmd_age_s": 0.1, "cmd_vx": 0.3, "cmd_wz": 0.0})],
        tmp_path=tmp_path)
    eng.run_once()
    assert _events(tmp_path)[0]["interlock"]["chassis"] is True
