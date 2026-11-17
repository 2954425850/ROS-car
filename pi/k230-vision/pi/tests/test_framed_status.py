import json

import k230pi


def test_payload_shape():
    p = k230pi.framed_status_payload(True, "10.0.0.5:1234", 2048, 7, wall=1000.0)
    assert p["connected"] is True
    assert p["peer"] == "10.0.0.5:1234"
    assert p["bytes"] == 2048
    assert p["idr_frames"] == 7
    assert p["wall"] == 1000.0
    json.dumps(p)


def test_disconnected_payload_has_no_peer():
    p = k230pi.framed_status_payload(False, None, 0, 0, wall=1.0)
    assert p["connected"] is False
    assert p["peer"] is None
