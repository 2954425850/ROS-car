"""USB disconnect 计数。K230 24 小时 13 次重枚举这类问题在这里现形。"""

from __future__ import annotations

import re
import subprocess
import time

from ..types import ProbeResult

_NEW_RE = re.compile(
    r"usb (\S+): New USB device found, idVendor=([0-9a-fA-F]{4}), idProduct=([0-9a-fA-F]{4})")
_DISC_RE = re.compile(r"usb (\S+): USB disconnect")


def parse_usb_disconnects(text: str) -> dict[str, int]:
    known: dict[str, str] = {}
    counts: dict[str, int] = {}
    for line in (text or "").splitlines():
        m = _NEW_RE.search(line)
        if m:
            known[m.group(1)] = "%s:%s" % (m.group(2).lower(), m.group(3).lower())
            continue
        m = _DISC_RE.search(line)
        if m:
            key = known.get(m.group(1), "unknown")
            counts[key] = counts.get(key, 0) + 1
    return counts


class UsbEventsProbe:
    name = "usb"
    interval_s = 10.0

    def __init__(self, runner=subprocess.run, clock=time.monotonic):
        self._runner = runner
        self._clock = clock

    def poll(self) -> ProbeResult:
        cmd = ["journalctl", "-k", "-b", "--since", "-10min",
               "--no-pager", "-o", "cat"]
        try:
            r = self._runner(cmd, capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired) as e:
            return ProbeResult(ok=None, error="USB 事件读取失败: %r" % (e,))
        if r.returncode != 0:
            return ProbeResult(ok=None, error="USB 事件读取失败: journalctl rc=%d" % r.returncode)
        counts = parse_usb_disconnects(r.stdout)
        return ProbeResult(ok=True, evidence={
            "disconnects": counts,
            "total": sum(counts.values()),
        })
