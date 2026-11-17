"""DDS 是否还绑着旧网段的地址。换网段后只重启一个节点修不好，必须整条 ROS 栈重启。"""

from __future__ import annotations

import subprocess
import time

from ..types import ProbeResult


def parse_ss_local_addrs(text: str) -> set[str]:
    addrs = set()
    for line in (text or "").splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        local = parts[3]
        if ":" not in local:          # 表头 / 非地址列
            continue
        host = local.rsplit(":", 1)[0].strip("[]")
        if host in ("*", "0.0.0.0", "::", "::1", "127.0.0.1"):
            continue
        if host.startswith("127."):
            continue
        addrs.add(host.split("%", 1)[0])
    return addrs


class DdsProbe:
    name = "dds"
    interval_s = 10.0

    def __init__(self, runner=subprocess.run, clock=time.monotonic):
        self._runner = runner
        self._clock = clock

    def poll(self) -> ProbeResult:
        try:
            ip = self._runner(["hostname", "-I"], capture_output=True,
                              text=True, timeout=3)
            ss = self._runner(["ss", "-uan"], capture_output=True,
                              text=True, timeout=3)
        except (OSError, subprocess.TimeoutExpired) as e:
            return ProbeResult(ok=None, error="ss/hostname 不可用: %r" % (e,))
        if ip.returncode != 0 or ss.returncode != 0:
            return ProbeResult(ok=None, error="ss/hostname 返回非零")
        current = [a for a in ip.stdout.split() if a]
        stale = sorted(parse_ss_local_addrs(ss.stdout) - set(current))
        return ProbeResult(ok=True, evidence={
            "current_ips": current,
            "stale_binds": stale,
        })
