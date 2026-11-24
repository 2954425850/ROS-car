import json
import threading
import time
import urllib.error
import urllib.request

from server import State, build_server


def _serve(tmp_path, state):
    index = tmp_path / "index.html"
    index.write_text("<html>ok</html>", encoding="utf-8")
    srv = build_server("127.0.0.1", 0, state, str(index))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, "http://127.0.0.1:%d" % srv.server_address[1]


def test_index_and_health(tmp_path):
    state = State()
    state.set_status(mode="mapping", tf_ok=True, ros="ok")
    srv, base = _serve(tmp_path, state)
    try:
        assert urllib.request.urlopen(base + "/").read() == b"<html>ok</html>"
        body = json.loads(urllib.request.urlopen(base + "/api/health").read())
        assert body["mode"] == "mapping"
        assert body["tf_ok"] is True
        assert body["map"] is None
        assert body["scan_points"] == 0
    finally:
        srv.shutdown()


def test_unknown_path_is_404(tmp_path):
    srv, base = _serve(tmp_path, State())
    try:
        try:
            urllib.request.urlopen(base + "/nope")
            assert False, "expected 404"
        except urllib.error.HTTPError as e:
            assert e.code == 404
    finally:
        srv.shutdown()


def test_sse_pushes_status_and_map(tmp_path):
    state = State()
    state.set_map({"w": 2, "h": 1, "res": 0.05, "ox": 0.0, "oy": 0.0,
                   "data": "AAA="})
    srv, base = _serve(tmp_path, state)
    resp = None
    try:
        resp = urllib.request.urlopen(base + "/events", timeout=5)
        assert resp.headers["Content-Type"].startswith("text/event-stream")
        buf = b""
        deadline = time.time() + 3
        while time.time() < deadline:
            if b"event: map" in buf and b"event: status" in buf:
                break
            line = resp.readline()
            if not line:
                break
            buf += line
        assert b"event: map" in buf
        assert b"event: status" in buf
    finally:
        if resp is not None:
            resp.close()
        srv.shutdown()


def test_sse_delivers_later_map_update(tmp_path):
    state = State()
    srv, base = _serve(tmp_path, state)
    resp = None
    try:
        resp = urllib.request.urlopen(base + "/events", timeout=5)
        resp.readline()  # 先让连接建立，拿到第一行
        state.set_map({"w": 1, "h": 1, "res": 0.05, "ox": 0.0, "oy": 0.0,
                       "data": "/w=="})
        buf = b""
        deadline = time.time() + 3
        while time.time() < deadline and b"event: map" not in buf:
            line = resp.readline()
            if not line:
                break
            buf += line
        assert b"event: map" in buf
    finally:
        if resp is not None:
            resp.close()
        srv.shutdown()
