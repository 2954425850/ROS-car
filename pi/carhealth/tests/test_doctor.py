import argparse
import json
import time

from carhealth import cli
from carhealth.types import ComponentState, Verdict


class FakeEngine:
    def __init__(self, verdict):
        self.verdict = verdict
        self.closed = False

    def run_once(self):
        return {
            "power": ComponentState("power", "供电", self.verdict, "欠压", {}, "power",
                                    ("欠压",), (), 0.0, 0.0),
        }

    def close(self):
        self.closed = True


def args(tmp_path, **kw):
    base = dict(state_dir=str(tmp_path), journal_dir=str(tmp_path),
                no_ros=True, no_daemon=True, warmup=0.0,
                board_host=None, fake_throttled=None)
    base.update(kw)
    return argparse.Namespace(**base)


def write_state(tmp_path, verdict="OK", wall_ts=None):
    state_dir = tmp_path / "state"
    state_dir.mkdir(exist_ok=True)
    (state_dir / "state.json").write_text(json.dumps({
        "wall_ts": time.time() if wall_ts is None else wall_ts,
        "components": {"power": {"display": "供电", "verdict": verdict,
                                 "reason": "欠压", "root_cause": "power",
                                 "diagnoses": []}},
        "diagnoses_global": [],
    }))
    return state_dir


def test_doctor_pass(tmp_path, capsys):
    eng = FakeEngine(Verdict.OK)
    code = cli.cmd_doctor(args(tmp_path), engine_factory=lambda a: eng,
                          sleeper=lambda s: None)
    assert code == 0
    assert "PASS" in capsys.readouterr().out
    assert eng.closed is True


def test_doctor_fail_on_blocked(tmp_path, capsys):
    eng = FakeEngine(Verdict.BLOCKED)
    code = cli.cmd_doctor(args(tmp_path), engine_factory=lambda a: eng,
                          sleeper=lambda s: None)
    assert code == 2
    assert "FAIL" in capsys.readouterr().out


def test_doctor_warn_on_degraded(tmp_path, capsys):
    eng = FakeEngine(Verdict.DEGRADED)
    code = cli.cmd_doctor(args(tmp_path), engine_factory=lambda a: eng,
                          sleeper=lambda s: None)
    assert code == 1
    assert "WARN" in capsys.readouterr().out


def test_doctor_reuses_fresh_daemon_state(tmp_path, capsys):
    state_dir = write_state(tmp_path, verdict="STALE")
    def factory(a):
        raise AssertionError("守护在跑时不应自己起 ROS 节点")
    code = cli.cmd_doctor(args(state_dir, no_daemon=False), engine_factory=factory,
                          sleeper=lambda s: None)
    assert code == 2
    out = capsys.readouterr().out
    assert "carhealthd" in out
    assert "FAIL" in out


def test_doctor_ignores_stale_daemon_state(tmp_path, capsys):
    state_dir = write_state(tmp_path, verdict="STALE", wall_ts=time.time() - 100)
    eng = FakeEngine(Verdict.OK)
    code = cli.cmd_doctor(args(state_dir, no_daemon=False),
                          engine_factory=lambda a: eng, sleeper=lambda s: None)
    assert code == 0
    assert eng.closed is True
