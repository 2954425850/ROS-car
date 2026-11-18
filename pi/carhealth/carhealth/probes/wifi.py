"""WiFi 省电模式。iwconfig 只读查询；关闭动作由 recovery 执行。"""

from __future__ import annotations

import re
import subprocess

from ..types import ProbeResult

_PM_RE = re.compile(r"Power Management:(\w+)")


class WifiProbe:
    name = "wifi"
    interval_s = 30.0

    def __init__(self, iface: str = "wlan0", runner=subprocess.run):
        self.iface = iface
        self._runner = runner

    def poll(self) -> ProbeResult:
        try:
            r = self._runner(["iwconfig", self.iface],
                             capture_output=True, text=True, timeout=3)
        except (OSError, subprocess.TimeoutExpired) as e:
            return ProbeResult(ok=None, error="iwconfig 不可用: %r" % (e,))
        if r.returncode != 0:
            return ProbeResult(ok=None, error="iwconfig 返回 %d" % r.returncode)
        ev = {"iface": self.iface}
        m = _PM_RE.search(r.stdout or "")
        if m:
            ev["power_save"] = m.group(1).strip().lower() == "on"
        return ProbeResult(ok=True, evidence=ev)
