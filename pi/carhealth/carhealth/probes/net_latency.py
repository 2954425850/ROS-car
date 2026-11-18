"""默认网关 RTT。

信号满格 ≠ 链路好：旧 TP-LINK 2.4G 实测信号 -53dBm、网关 RTT 平均 141ms/
最大 322ms，云端 ASR 直接 23s 超时。只看 WiFi 信号强度会漏掉这类故障。
"""

from __future__ import annotations

import re
import subprocess
import time
from collections import deque

from ..types import ProbeResult

_RTT_RE = re.compile(
    r"rtt min/avg/max/mdev = ([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+) ms")
_LOSS_RE = re.compile(r"([\d.]+)% packet loss")


def parse_ping(text: str) -> dict:
    ev = {}
    m = _LOSS_RE.search(text or "")
    if m:
        ev["loss_pct"] = float(m.group(1))
    m = _RTT_RE.search(text or "")
    if m:
        ev["rtt_min_ms"] = float(m.group(1))
        ev["rtt_avg_ms"] = float(m.group(2))
        ev["rtt_max_ms"] = float(m.group(3))
        ev["rtt_mdev_ms"] = float(m.group(4))
    return ev


class GatewayLatencyProbe:
    name = "net:gateway"
    interval_s = 10.0

    def __init__(self, runner=subprocess.run, clock=time.monotonic):
        self._runner = runner
        self._clock = clock
        # 单次 ping 的尖峰很常见（尤其手机热点）；规则看 5 次中位数。
        self._avg_hist: deque[float] = deque(maxlen=5)
        self._mdev_hist: deque[float] = deque(maxlen=5)

    def _gateway(self) -> str | None:
        try:
            r = self._runner(["ip", "-4", "route", "show", "default"],
                             capture_output=True, text=True, timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            return None
        if r.returncode != 0:
            return None
        for line in r.stdout.splitlines():
            if "default via " in line:
                return line.split("default via ", 1)[1].split()[0]
        return None

    def poll(self) -> ProbeResult:
        gateway = self._gateway()
        if not gateway:
            return ProbeResult(ok=None, error="找不到默认网关")
        try:
            r = self._runner(["ping", "-c", "3", "-W", "1", gateway],
                             capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired) as e:
            return ProbeResult(ok=None, error="ping 网关失败: %r" % (e,))
        ev = {"gateway": gateway}
        ev.update(parse_ping(r.stdout))
        if "rtt_avg_ms" in ev:
            self._avg_hist.append(ev["rtt_avg_ms"])
            ev["rtt_avg_ms"] = round(
                sorted(self._avg_hist)[len(self._avg_hist) // 2], 1)
        if "rtt_mdev_ms" in ev:
            self._mdev_hist.append(ev["rtt_mdev_ms"])
            ev["rtt_mdev_ms"] = round(
                sorted(self._mdev_hist)[len(self._mdev_hist) // 2], 1)
        ev["samples"] = len(self._avg_hist)
        return ProbeResult(ok=True, evidence=ev)
