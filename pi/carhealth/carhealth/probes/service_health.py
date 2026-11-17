"""服务资源与崩溃循环。Restart=always 会把崩溃循环伪装成 active。"""

from __future__ import annotations

import time

from ..types import ProbeResult


class ServiceHealthProbe:
    name = "service"
    interval_s = 5.0

    def __init__(self, unit_probes: dict, clock=time.monotonic, window_s: float = 60.0):
        self._units = dict(unit_probes)
        self._clock = clock
        self._window = window_s
        self._baseline: dict[str, tuple[int, float]] = {}
        self._cpu_prev: dict[str, tuple[int, float]] = {}

    def poll(self) -> ProbeResult:
        now = self._clock()
        crash, memory, restarts, cpu_pct = [], {}, {}, {}
        for name, probe in self._units.items():
            result = probe.poll()
            if not result.ok:
                continue
            n = int(result.evidence.get("n_restarts") or 0)
            memory[name] = result.evidence.get("memory_mb")
            restarts[name] = n

            cpu_ns = result.evidence.get("cpu_usage_ns")
            if isinstance(cpu_ns, int):
                prev = self._cpu_prev.get(name)
                if prev is not None and now > prev[1]:
                    cpu_pct[name] = round(
                        (cpu_ns - prev[0]) / 1e9 / (now - prev[1]) * 100.0, 1)
                self._cpu_prev[name] = (cpu_ns, now)

            base = self._baseline.get(name)
            if base is None or now - base[1] > self._window:
                self._baseline[name] = (n, now)
                continue
            if n - base[0] >= 2:
                crash.append(name)
                self._baseline[name] = (n, now)
        return ProbeResult(ok=True, evidence={
            "crash_loop_units": crash,
            "memory_mb": memory,
            "n_restarts": restarts,
            "cpu_pct": cpu_pct,
        })
