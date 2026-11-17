"""批量 systemctl show，避免每个单元每秒起一个子进程。"""

from __future__ import annotations

import subprocess
import time

from ..types import ProbeResult


def _parse_show(text: str) -> dict[str, dict]:
    units: dict[str, dict] = {}
    block: dict[str, str] = {}
    for line in (text or "").splitlines() + [""]:
        if not line.strip():
            if "Id" in block:
                units[block["Id"]] = block
            block = {}
            continue
        key, _, value = line.partition("=")
        block[key.strip()] = value.strip()
    return units


class SystemdCache:
    def __init__(self, units, runner=subprocess.run, interval_s: float = 1.0,
                 clock=time.monotonic):
        self.units = tuple(units)
        self._runner = runner
        self._interval = interval_s
        self._clock = clock
        self._last: float | None = None
        self._data: dict[str, dict] = {}
        self._error: str | None = None

    def _refresh(self) -> None:
        self._last = self._clock()
        cmd = ["systemctl", "show", "-p", "Id", "-p", "ActiveState",
               "-p", "SubState", "-p", "NRestarts", "-p", "MemoryCurrent",
               "-p", "CPUUsageNSec", *self.units]
        try:
            r = self._runner(cmd, capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired) as e:
            self._data, self._error = {}, "systemctl 执行失败: %r" % (e,)
            return
        if r.returncode != 0:
            self._data, self._error = {}, "systemctl 返回 %d" % r.returncode
            return
        self._data = _parse_show(r.stdout)
        self._error = None

    @staticmethod
    def _int(value):
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def get(self, unit: str, full_unit: str) -> ProbeResult:
        if self._last is None or self._clock() - self._last >= self._interval:
            self._refresh()
        if self._error:
            return ProbeResult(ok=None, error=self._error)
        data = self._data.get(full_unit)
        if data is None:
            return ProbeResult(ok=None, error="systemctl 没有返回 %s" % full_unit)
        mem_bytes = self._int(data.get("MemoryCurrent"))
        return ProbeResult(ok=True, evidence={
            "active": data.get("ActiveState") == "active",
            "active_state": data.get("ActiveState"),
            "sub_state": data.get("SubState"),
            "n_restarts": self._int(data.get("NRestarts")) or 0,
            "memory_mb": None if mem_bytes is None else round(mem_bytes / 1048576.0, 1),
            "cpu_usage_ns": self._int(data.get("CPUUsageNSec")),
        })


class UnitProbe:
    def __init__(self, name: str, unit: str, cache: SystemdCache, interval_s: float = 1.0):
        self.name = name
        self.interval_s = interval_s
        self._unit = unit
        self._full = unit if unit.endswith(".service") else unit + ".service"
        self._cache = cache

    def poll(self) -> ProbeResult:
        return self._cache.get(self._unit, self._full)
