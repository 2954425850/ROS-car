import json

from utils.heartbeat import Heartbeat, default_path


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def test_payload_tracks_state_and_busy_duration(tmp_path):
    clock = Clock()
    hb = Heartbeat(tmp_path / "hb.json", clock=clock, wall=lambda: 1000.0)
    hb.update("idle", busy=False)
    clock.now = 10.0
    hb.update("thinking", busy=True)
    clock.now = 25.0
    p = hb.payload()
    assert p["state"] == "thinking"
    assert p["busy"] is True
    assert p["busy_for_s"] == 15.0
    assert p["state_age_s"] == 15.0


def test_busy_for_resets_when_idle(tmp_path):
    clock = Clock()
    hb = Heartbeat(tmp_path / "hb.json", clock=clock)
    hb.update("thinking", busy=True)
    clock.now = 10.0
    hb.update("idle", busy=False)
    p = hb.payload()
    assert p["busy"] is False
    assert p["busy_for_s"] == 0.0


def test_status_provider_is_merged(tmp_path):
    hb = Heartbeat(
        tmp_path / "hb.json", clock=lambda: 1.0,
        status_provider=lambda: {"wakeword_ready": False, "mic_open": True})
    p = hb.payload()
    assert p["wakeword_ready"] is False
    assert p["mic_open"] is True


def test_status_provider_exception_is_ignored(tmp_path):
    def boom():
        raise RuntimeError("device driver exploded")

    hb = Heartbeat(tmp_path / "hb.json", clock=lambda: 1.0, status_provider=boom)
    assert hb.payload()["busy"] is False


def test_write_once_is_atomic_json(tmp_path):
    path = tmp_path / "hb.json"
    hb = Heartbeat(path, clock=lambda: 1.0, wall=lambda: 1000.0)
    hb.update("listening", busy=False)
    hb.write_once()
    data = json.loads(path.read_text())
    assert data["state"] == "listening"
    assert data["busy"] is False
    assert not (tmp_path / "hb.json.tmp").exists()


def test_write_failure_is_swallowed(tmp_path):
    blocker = tmp_path / "blocker"
    blocker.write_text("x")          # 拿文件当父目录，mkdir 必失败
    hb = Heartbeat(blocker / "hb.json", clock=lambda: 1.0)
    hb.update("idle", busy=False)
    hb.write_once()          # 不抛异常


def test_default_path_returns_first_writable(tmp_path, monkeypatch):
    monkeypatch.setattr("os.access", lambda p, mode: False)
    monkeypatch.setattr("pathlib.Path.home", lambda: tmp_path)
    path = default_path()
    assert str(path).startswith(str(tmp_path))
    assert path.name == "heartbeat.json"
