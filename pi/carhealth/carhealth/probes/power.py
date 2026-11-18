"""Pi 供电状态。掉压会打掉 USB 设备（2026-10-14 实测：连臂串口一起掉）。"""

from __future__ import annotations

import re
import subprocess
import time

from ..types import ProbeResult

_THROTTLED_RE = re.compile(r"throttled=(0x[0-9a-fA-F]+|\d+)")


def parse_throttled(text: str) -> int | None:
    m = _THROTTLED_RE.search(text or "")
    if not m:
        return None
    try:
        return int(m.group(1), 0)
    except ValueError:
        return None


class PowerProbe:
    name = "power"
    interval_s = 2.0

    def __init__(self, runner=subprocess.run, fake_output: str | None = None,
                 clock=time.monotonic, recent_s: float = 600.0):
        self._runner = runner
        self._fake = fake_output
        self._clock = clock
        self._recent_s = recent_s
        self._uv_seen: float | None = None

    def _read(self) -> str | None:
        if self._fake is not None:
            # CLI 的 --fake-throttled 允许只写 0x1；补上真实 vcgencmd 的格式。
            return self._fake if "throttled" in self._fake else "throttled=" + self._fake
        try:
            r = self._runner(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return r.stdout if r.returncode == 0 else None

    def poll(self) -> ProbeResult:
        text = self._read()
        bits = parse_throttled(text) if text is not None else None
        if bits is None:
            return ProbeResult(ok=None, error="vcgencmd get_throttled 不可用")
        now = self._clock()
        uv_now = bool(bits & 0x1)
        uv_since_boot = bool(bits & 0x10000)
        # 只有**亲眼看到**当前欠压才更新 recent；历史位时间未知，只算 Degraded。
        if uv_now:
            self._uv_seen = now
        uv_recent = uv_now or (
            self._uv_seen is not None and now - self._uv_seen < self._recent_s
        )
        return ProbeResult(ok=True, evidence={
            "throttled": hex(bits),
            "uv_now": uv_now,
            "uv_recent": uv_recent,
            "uv_since_boot": uv_since_boot,
            "freq_capped": bool(bits & 0x2),
            "throttled_now": bool(bits & 0x4),
            "soft_temp": bool(bits & 0x8),
        })
