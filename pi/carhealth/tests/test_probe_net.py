import json
import subprocess

from carhealth.probes.net import PingProbe, TcpProbe, resolve_board_host


def test_tcp_ok():
    r = TcpProbe("net:x", "127.0.0.1", 9090, connect=lambda addr, timeout: _Closer()).poll()
    assert r.evidence["tcp_ok"] is True


def test_tcp_failure_is_measured_not_raised():
    def boom(addr, timeout):
        raise OSError("refused")

    r = TcpProbe("net:x", "127.0.0.1", 9090, connect=boom).poll()
    assert r.ok is True
    assert r.evidence["tcp_ok"] is False
    assert "refused" in r.evidence["error"]


class _Closer:
    def close(self):
        pass


def test_ping_uses_host_fn_each_poll():
    hosts = iter(["10.0.0.1", "10.0.0.2"])
    seen = []

    def runner(cmd, **kw):
        seen.append(cmd[-1])
        return subprocess.CompletedProcess(cmd, 0, "ok", "")

    p = PingProbe("net:board", host_fn=lambda: next(hosts), runner=runner)
    assert p.poll().evidence["ping_ok"] is True
    assert p.poll().evidence["ping_ok"] is True
    assert seen == ["10.0.0.1", "10.0.0.2"]


def test_ping_exception_is_false():
    def runner(cmd, **kw):
        raise OSError("no ping")

    r = PingProbe("net:board", host_fn=lambda: "10.0.0.1", runner=runner).poll()
    assert r.evidence["ping_ok"] is False


def test_ping_without_known_host_is_unknown_not_offline():
    """Pi 重启后 /tmp 清空、resultd 还没有 peer；不能拿写死的旧 IP 判板子离线。"""
    r = PingProbe("net:board", host_fn=lambda: None,
                  runner=lambda *a, **k: (_ for _ in ()).throw(AssertionError("不应 ping"))).poll()
    assert r.ok is None
    assert "地址未知" in r.error


def test_resolve_board_host_prefers_env_then_peer_then_none(tmp_path):
    status = tmp_path / "status.json"
    status.write_text(json.dumps({"peer": "10.64.35.77:54321"}))
    assert resolve_board_host(env={"K230_HOST": "1.2.3.4"}, status_path=status) == "1.2.3.4"
    assert resolve_board_host(env={}, status_path=status) == "10.64.35.77"
    assert resolve_board_host(env={}, status_path=tmp_path / "nope.json") is None
