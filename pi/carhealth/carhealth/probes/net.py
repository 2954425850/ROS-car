"""TCP / ping 探活。不做任何协议交互，连上就关。"""

from __future__ import annotations

import json
import os
import socket
import subprocess

from ..types import ProbeResult


def resolve_board_host(env=None, status_path="/tmp/k230/resultd-status.json") -> str | None:
    """返回 None = 地址未知。**绝不写死回退 IP**：网段变过（192.168.1.x → 10.198.146.x），
    写死的旧地址只会把「不知道」伪装成「板子离线」。"""
    env = os.environ if env is None else env
    if "K230_HOST" in env:
        return env["K230_HOST"]
    try:
        with open(status_path, "r", encoding="utf-8") as f:
            peer = json.load(f).get("peer")
        if isinstance(peer, str) and ":" in peer:
            return peer.rsplit(":", 1)[0]
    except (OSError, ValueError, AttributeError):
        pass
    return None


class TcpProbe:
    def __init__(self, name: str, host: str, port: int, timeout: float = 1.0,
                 connect=socket.create_connection, interval_s: float = 5.0):
        self.name = name
        self.interval_s = interval_s
        self.host = host
        self.port = port
        self.timeout = timeout
        self._connect = connect

    def poll(self) -> ProbeResult:
        try:
            sock = self._connect((self.host, self.port), timeout=self.timeout)
            sock.close()
            return ProbeResult(ok=True, evidence={"tcp_ok": True, "error": None})
        except OSError as e:
            return ProbeResult(ok=True, evidence={"tcp_ok": False, "error": str(e)})


class PingProbe:
    def __init__(self, name: str, host_fn, runner=subprocess.run,
                 timeout: float = 2.0, interval_s: float = 5.0):
        self.name = name
        self.interval_s = interval_s
        self._host_fn = host_fn
        self._runner = runner
        self._timeout = timeout

    def poll(self) -> ProbeResult:
        host = self._host_fn()
        if not host:
            return ProbeResult(
                ok=None,
                error="K230 地址未知（resultd 无 peer 且未设 K230_HOST）",
                evidence={"ping_ok": None, "host": None},
            )
        try:
            r = self._runner(["ping", "-c", "1", "-W", "1", host],
                             capture_output=True, text=True, timeout=self._timeout)
            ok = r.returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            ok = False
        return ProbeResult(ok=True, evidence={"ping_ok": ok, "host": host})
