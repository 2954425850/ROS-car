"""内存 / 磁盘 / 温度 / load / WiFi。只读 /proc、statvfs、thermal。"""

from __future__ import annotations

import os
import time

from ..types import ProbeResult


def parse_cpu_stat(text: str) -> tuple[int, int]:
    """返回 (总 jiffies, 空闲 jiffies)。只看 aggregate 的 `cpu` 行。"""
    for line in (text or "").splitlines():
        if line.startswith("cpu "):
            values = [int(v) for v in line.split()[1:9]]
            total = sum(values)
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            return total, idle
    raise ValueError("没有 cpu 行")


def parse_wireless(text: str) -> dict:
    for line in (text or "").splitlines():
        if ":" not in line or "|" in line:
            continue
        name, _, rest = line.partition(":")
        parts = rest.split()
        if len(parts) < 3:
            continue
        try:
            return {
                "wifi_iface": name.strip(),
                "wifi_link": float(parts[1]),
                "wifi_signal_dbm": float(parts[2]),
            }
        except ValueError:
            continue
    return {}


class HostProbe:
    name = "host"
    interval_s = 5.0

    def __init__(self, meminfo: str = "/proc/meminfo", disk_path: str = "/",
                 thermal: str = "/sys/class/thermal/thermal_zone0/temp",
                 wireless: str = "/proc/net/wireless", stat_path: str = "/proc/stat",
                 getloadavg=os.getloadavg, clock=time.monotonic):
        self.meminfo = meminfo
        self.disk_path = disk_path
        self.thermal = thermal
        self.wireless = wireless
        self.stat_path = stat_path
        self._getloadavg = getloadavg
        self._clock = clock
        self._prev_cpu = None

    def poll(self) -> ProbeResult:
        ev: dict = {}
        try:
            values = {}
            with open(self.meminfo, "r", encoding="utf-8") as f:
                for line in f:
                    key, _, rest = line.partition(":")
                    values[key.strip()] = int(rest.split()[0])
            ev["mem_available_mb"] = round(values["MemAvailable"] / 1024.0, 1)
            ev["swap_free_mb"] = round(values.get("SwapFree", 0) / 1024.0, 1)
        except (OSError, KeyError, ValueError, IndexError):
            pass
        try:
            st = os.statvfs(self.disk_path)
            ev["disk_free_pct"] = round(100.0 * st.f_bavail / st.f_blocks, 1)
        except OSError:
            pass
        try:
            with open(self.thermal, "r", encoding="utf-8") as f:
                ev["soc_temp_c"] = round(int(f.read().strip()) / 1000.0, 1)
        except (OSError, ValueError):
            pass
        try:
            ev["load1"] = round(float(self._getloadavg()[0]), 2)
        except (OSError, ValueError, IndexError):
            pass
        try:
            with open(self.wireless, "r", encoding="utf-8") as f:
                ev.update(parse_wireless(f.read()))
        except OSError:
            pass
        try:
            with open(self.stat_path, "r", encoding="utf-8") as f:
                total, idle = parse_cpu_stat(f.read())
            if self._prev_cpu is not None:
                d_total = total - self._prev_cpu[0]
                d_idle = idle - self._prev_cpu[1]
                if d_total > 0:
                    ev["cpu_pct"] = round(100.0 * (1 - d_idle / d_total), 1)
            self._prev_cpu = (total, idle)
        except (OSError, ValueError, IndexError):
            pass
        return ProbeResult(ok=True, evidence=ev)
