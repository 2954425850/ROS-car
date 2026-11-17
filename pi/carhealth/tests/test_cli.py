import json

from carhealth import cli


def write_state(tmp_path, verdict="OK", stale=False, wall_ts=None):
    state_dir = tmp_path / "state"
    state_dir.mkdir()
    payload = {
        "wall": "2026-11-17T14:00:00",
        "wall_ts": (1763388000.0 if wall_ts is None else wall_ts),
        "pid": 123,
        "interval_s": 1.0,
        "busy": {"arm": False, "chassis": False, "grasp": False, "voice": False},
        "diagnoses_global": [],
        "components": {
            "tof": {"display": "ToF 深度", "verdict": verdict, "reason": "没数据",
                    "root_cause": None, "failing": ["没数据"], "since_s": 12.0,
                    "diagnoses": [{"id": "usb.disconnect_storm",
                                   "title": "USB 在反复重枚举",
                                   "action_class": "human",
                                   "hint": "换线/换口/查供电"}],
                    "evidence": {}},
        },
    }
    (state_dir / "state.json").write_text(json.dumps(payload))
    return state_dir


def test_status_prints_table(tmp_path, capsys):
    state_dir = write_state(tmp_path, verdict="STALE")
    code = cli.main(["status", "--state-dir", str(state_dir),
                     "--now", "1763388005"])
    out = capsys.readouterr().out
    assert code == 0
    assert "tof" in out
    assert "STALE" in out
    assert "没数据" in out
    assert "USB 在反复重枚举" in out
    assert "换线/换口/查供电" in out


def test_status_reports_stale_daemon(tmp_path, capsys):
    state_dir = write_state(tmp_path, wall_ts=1763388000.0)
    code = cli.main(["status", "--state-dir", str(state_dir),
                     "--now", "1763388100"])
    out = capsys.readouterr().out
    assert code == 2
    assert "守护未运行" in out


def test_status_json(tmp_path, capsys):
    state_dir = write_state(tmp_path)
    code = cli.main(["status", "--state-dir", str(state_dir),
                     "--now", "1763388005", "--json"])
    assert code == 0
    assert json.loads(capsys.readouterr().out)["components"]["tof"]["verdict"] == "OK"


def test_history_and_why(tmp_path, capsys):
    journal_dir = tmp_path / "journal"
    journal_dir.mkdir()
    line = json.dumps({"ts": 1763388001.0, "event": "incident_open",
                       "incident": "I-1", "component": "tof", "to": "STALE",
                       "reason": "没数据", "evidence": {"x": 1}})
    (journal_dir / "journal-20261117.jsonl").write_text(line + "\n")

    assert cli.main(["history", "--journal-dir", str(journal_dir),
                     "--since", "all"]) == 0
    assert "incident_open" in capsys.readouterr().out

    assert cli.main(["why", "tof", "--journal-dir", str(journal_dir)]) == 0
    out = capsys.readouterr().out
    assert "I-1" in out
    assert "没数据" in out


def test_why_unknown_component(tmp_path, capsys):
    journal_dir = tmp_path / "journal"
    journal_dir.mkdir()
    assert cli.main(["why", "nope", "--journal-dir", str(journal_dir)]) == 1
    assert "没有记录" in capsys.readouterr().out


def test_parse_since():
    assert cli.parse_since("1h") == 3600
    assert cli.parse_since("30m") == 1800
    assert cli.parse_since("2d") == 172800
