import json
import os

from carhealth.journal import Journal


def test_record_appends_timestamped_json_line(tmp_path):
    j = Journal(tmp_path, clock=lambda: 1763388000.0)
    event = j.record({"event": "incident_open", "component": "tof"})
    assert event["ts"] == 1763388000.0
    path = next(tmp_path.glob("journal-*.jsonl"))
    lines = path.read_text().splitlines()
    assert len(lines) == 1
    assert json.loads(lines[0])["component"] == "tof"


def test_record_keeps_explicit_ts(tmp_path):
    j = Journal(tmp_path, clock=lambda: 200.0)
    assert j.record({"ts": 123.0, "event": "x"})["ts"] == 123.0


def test_record_adds_boot_id_and_mono(tmp_path):
    """Pi 无 RTC：墙钟在对时前不可信，排序/过滤要靠 boot_id + mono。"""
    j = Journal(tmp_path, clock=lambda: 100.0,
                boot_id_fn=lambda: "boot-a", mono_fn=lambda: 12.5)
    event = j.record({"event": "x"})
    assert event["boot_id"] == "boot-a"
    assert event["mono"] == 12.5


def test_read_filters_by_since(tmp_path):
    times = iter([100.0, 200.0, 300.0])
    j = Journal(tmp_path, clock=lambda: next(times))
    for i in range(3):
        j.record({"event": "e", "n": i})
    got = j.read(since_ts=150.0)
    assert [e["n"] for e in got] == [1, 2]


def test_read_skips_bad_lines(tmp_path):
    j = Journal(tmp_path)
    j.record({"event": "ok"})
    path = next(tmp_path.glob("journal-*.jsonl"))
    with open(path, "a", encoding="utf-8") as f:
        f.write("not json\n")
    assert [e["event"] for e in j.read()] == ["ok"]


def test_read_since_uses_mono_for_current_boot(tmp_path):
    """NTP 对时前墙钟可能差几小时；当前 boot 的事件必须按 mono 过滤。"""
    j = Journal(tmp_path, clock=lambda: 1_000_000.0,
                boot_id_fn=lambda: "b1", mono_fn=lambda: 900.0)
    j.record({"event": "new", "mono": 850.0})
    j.record({"event": "old", "mono": 100.0})
    j.record({"event": "previous_boot", "ts": 999_990.0, "boot_id": "b0", "mono": 5.0})
    got = j.read_since(120.0, boot_id="b1", now_mono=900.0)
    assert [e["event"] for e in got] == ["new", "previous_boot"]


def test_prune_removes_old_files_only(tmp_path):
    old = tmp_path / "journal-20200101.jsonl"
    old.write_text("{}\n")
    new = tmp_path / "journal-20990101.jsonl"
    new.write_text("{}\n")
    Journal(tmp_path, clock=lambda: 1763388000.0).prune()
    assert not old.exists()
    assert new.exists()
