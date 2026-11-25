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


class FakeController:
    maps_dir = "/tmp/maps"

    def __init__(self, ok=True):
        self.ok = ok
        self.calls = []

    def list_maps(self):
        return [{"name": "a.yaml", "size": 1, "mtime": 2}]

    def start(self, mode, map_name=None):
        self.calls.append(("start", mode, map_name))
        return (True, "started") if self.ok else (False, "外部实例在跑")

    def stop(self):
        self.calls.append(("stop",))
        return True, "stopped"

    def save_map(self, name):
        self.calls.append(("save", name))
        return True, "saved"

    def send_goal(self, x, y, yaw):
        self.calls.append(("goal", x, y, yaw))
        return True, "goal sent"

    def cancel_goal(self):
        self.calls.append(("cancel",))
        return True, "canceled"


def _serve_ctl(tmp_path, controller):
    index = tmp_path / "index.html"
    index.write_text("<html>ok</html>", encoding="utf-8")
    srv = build_server("127.0.0.1", 0, State(), str(index), controller)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, "http://127.0.0.1:%d" % srv.server_address[1]


def _post(base, path, obj):
    req = urllib.request.Request(
        base + path, data=json.dumps(obj).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    return urllib.request.urlopen(req)


def test_maps_and_mode_with_controller(tmp_path):
    ctl = FakeController()
    srv, base = _serve_ctl(tmp_path, ctl)
    try:
        maps = json.loads(urllib.request.urlopen(base + "/api/maps").read())
        assert maps["maps"][0]["name"] == "a.yaml"
        body = json.loads(_post(base, "/api/mode",
                                {"mode": "slam"}).read())
        assert body["ok"] is True
        assert ctl.calls[-1] == ("start", "slam", None)
    finally:
        srv.shutdown()


def test_mode_error_is_409(tmp_path):
    ctl = FakeController(ok=False)
    srv, base = _serve_ctl(tmp_path, ctl)
    try:
        try:
            _post(base, "/api/mode", {"mode": "nav", "map": "a.yaml"})
            assert False, "expected 409"
        except urllib.error.HTTPError as e:
            assert e.code == 409
            assert "外部实例" in json.loads(e.read())["error"]
    finally:
        srv.shutdown()


def test_goal_and_cancel(tmp_path):
    ctl = FakeController()
    srv, base = _serve_ctl(tmp_path, ctl)
    try:
        body = json.loads(_post(base, "/api/goal",
                                {"x": 1.5, "y": -2.0, "yaw": 0.5}).read())
        assert body["ok"] is True
        assert ctl.calls[-1] == ("goal", 1.5, -2.0, 0.5)
        assert json.loads(_post(base, "/api/goal/cancel", {}).read())["ok"]
    finally:
        srv.shutdown()


def test_bad_body_is_400(tmp_path):
    srv, base = _serve_ctl(tmp_path, FakeController())
    try:
        req = urllib.request.Request(
            base + "/api/mode", data=b"not json",
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            urllib.request.urlopen(req)
            assert False, "expected 400"
        except urllib.error.HTTPError as e:
            assert e.code == 400
    finally:
        srv.shutdown()


def test_save_map_calls_tools_save_map_snapshot(tmp_path, monkeypatch):
    import os
    import server as srv_mod
    from server import ModeController

    tools = tmp_path / "tools"
    tools.mkdir()
    (tools / "save_map_snapshot.py").write_text("")
    launch = tmp_path / "launch"
    launch.mkdir()
    maps = tmp_path / "maps"
    maps.mkdir()
    ctl = ModeController(State(), None, str(maps), str(launch))
    assert ctl.tools_dir == str(tools)

    seen = {}

    def fake_run(cmd, **kw):
        seen["cmd"] = cmd

        class R:
            returncode = 0
            stdout = "saved"
            stderr = ""
        return R()

    monkeypatch.setattr(srv_mod.subprocess, "run", fake_run)
    ok, msg = ctl.save_map("abc")
    assert ok, msg
    assert seen["cmd"][1] == os.path.join(str(tools), "save_map_snapshot.py")


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


def test_detect_mode_three_states(monkeypatch):
    import subprocess
    import server as srv_mod

    alive = {"slam_toolbox", "nav2_controller"}

    def fake_run(cmd, **kw):
        class R:
            returncode = 0 if cmd[2] in alive else 1
        return R()

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert srv_mod.detect_mode() == "fusion"
    alive = {"slam_toolbox"}
    assert srv_mod.detect_mode() == "mapping"
    alive = {"nav2_controller"}
    assert srv_mod.detect_mode() == "navigating"
    alive = set()
    assert srv_mod.detect_mode() == "idle"


class FusionController(FakeController):
    def __init__(self):
        super().__init__()
        self.boundary = None

    def set_explore(self, enabled):
        self.calls.append(("explore", enabled))
        return True, "explore %s" % enabled

    def set_boundary(self, obj):
        self.calls.append(("boundary", obj))
        self.boundary = obj
        return True, "boundary ok"


def test_explore_endpoint(tmp_path):
    ctl = FusionController()
    srv, base = _serve_ctl(tmp_path, ctl)
    try:
        body = json.loads(_post(base, "/api/explore",
                                {"enabled": True}).read())
        assert body["ok"] is True
        assert ctl.calls[-1] == ("explore", True)
    finally:
        srv.shutdown()


def test_boundary_post_and_get(tmp_path):
    ctl = FusionController()
    srv, base = _serve_ctl(tmp_path, ctl)
    try:
        obj = {"anchor": [0.0, 0.0],
               "segments": [[2.0, -1.0, 2.0, 1.0]]}
        assert json.loads(_post(base, "/api/boundary", obj).read())["ok"]
        got = json.loads(urllib.request.urlopen(base + "/api/boundary").read())
        assert got["boundary"] == obj
    finally:
        srv.shutdown()


def test_mode_accepts_fusion(tmp_path):
    ctl = FakeController()
    srv, base = _serve_ctl(tmp_path, ctl)
    try:
        body = json.loads(_post(base, "/api/mode",
                                {"mode": "fusion"}).read())
        assert body["ok"] is True
        assert ctl.calls[-1] == ("start", "fusion", None)
    finally:
        srv.shutdown()


def test_save_map_writes_pose(tmp_path, monkeypatch):
    import json as _json
    import server as srv_mod
    from server import ModeController, State

    tools = tmp_path / "tools"
    tools.mkdir()
    (tools / "save_map_snapshot.py").write_text("")
    launch = tmp_path / "launch"
    launch.mkdir()
    maps = tmp_path / "maps"
    maps.mkdir()
    state = State()
    state.set_pose(1.25, -0.5, 0.75)
    ctl = ModeController(state, None, str(maps), str(launch))

    def fake_run(cmd, **kw):
        class R:
            returncode = 0
            stdout = "saved"
            stderr = ""
        return R()

    monkeypatch.setattr(srv_mod.subprocess, "run", fake_run)
    monkeypatch.setattr(srv_mod, "detect_mode", lambda: "fusion")
    ok, msg = ctl.save_map("trace")
    assert ok, msg
    with open(maps / "trace_last_pose.json") as f:
        d = _json.load(f)
    assert d == {"x": 1.25, "y": -0.5, "yaw": 0.75}
