import json
import os
import time

from carhealth.probes.file_status import FileStatusProbe


def test_reads_json_and_age(tmp_path):
    path = tmp_path / "status.json"
    path.write_text(json.dumps({"connected": True, "fps": 12.5}))
    r = FileStatusProbe("file:x", [path], wall=lambda: os.path.getmtime(path) + 1.5).poll()
    assert r.evidence["exists"] is True
    assert r.evidence["connected"] is True
    assert r.evidence["fps"] == 12.5
    assert r.evidence["age_s"] == 1.5


def test_missing_file_has_none_age_so_checks_fail(tmp_path):
    r = FileStatusProbe("file:x", [tmp_path / "nope.json"]).poll()
    assert r.evidence["exists"] is False
    assert r.evidence["age_s"] is None


def test_malformed_json_keeps_age_and_marks_error(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("{not json")
    r = FileStatusProbe("file:x", [path]).poll()
    assert "parse_error" in r.evidence
    assert r.evidence["age_s"] is not None


def test_binary_file_without_json(tmp_path):
    path = tmp_path / "idr.h264"
    path.write_bytes(b"\x00\x00\x00\x01")
    r = FileStatusProbe("file:x", [path], parse_json=False).poll()
    assert r.evidence["exists"] is True
    assert "parse_error" not in r.evidence


def test_picks_newest_of_candidate_paths(tmp_path):
    old = tmp_path / "old.json"
    new = tmp_path / "new.json"
    old.write_text("{}")
    new.write_text("{}")
    os.utime(old, (1000, 1000))
    r = FileStatusProbe("file:x", [old, new], wall=lambda: os.path.getmtime(new) + 2).poll()
    assert r.evidence["path"] == str(new)
    assert r.evidence["age_s"] == 2.0
