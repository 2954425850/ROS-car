"""udev 稳定名、by-id 兜底与 USB 接口号。

同一个 FT2232 出两个 tty；只按 vid/pid 匹配会抢同一个名字，
接到通道 B 时「节点存在、能打开、但一个字节都收不到」（2026-10-14 实测）。
接口号用 `udevadm info -q property` 读 `ID_USB_INTERFACE_NUM`——
从 sysfs 读对 FTDI 的 ttyUSB 要经过 usb-serial 端口再上一级，容易读错。
"""

from __future__ import annotations

import glob as _glob
import os
import subprocess

from ..types import ProbeResult


class UdevProbe:
    def __init__(self, name: str, paths, expect_interface: str | None = None,
                 realpath=os.path.realpath, exists=os.path.exists,
                 glob_fn=_glob.glob, runner=subprocess.run,
                 interval_s: float = 5.0):
        self.name = name
        self.interval_s = interval_s
        self.paths = (paths,) if isinstance(paths, str) else tuple(paths)
        self.expect_interface = expect_interface
        self._realpath = realpath
        self._exists = exists
        self._glob = glob_fn
        self._runner = runner

    def _resolve(self):
        for index, pattern in enumerate(self.paths):
            kind = "short" if index == 0 else "by_id"
            if any(ch in pattern for ch in "*?["):
                matches = self._glob(pattern)
                if matches:
                    return matches[0], kind
            elif self._exists(pattern):
                return pattern, kind
        return None, None

    def _interface(self, real: str) -> str | None:
        try:
            r = self._runner(["udevadm", "info", "-q", "property", "-n", real],
                             capture_output=True, text=True, timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            return None
        if r.returncode != 0:
            return None
        for line in r.stdout.splitlines():
            if line.startswith("ID_USB_INTERFACE_NUM="):
                return line.split("=", 1)[1].strip()
        return None

    def poll(self) -> ProbeResult:
        path, resolved_by = self._resolve()
        if path is None:
            return ProbeResult(ok=True, evidence={"exists": False, "real": None})
        real = self._realpath(path)
        ev: dict = {"exists": True, "real": real, "path": path,
                    "resolved_by": resolved_by}
        if self.expect_interface is not None:
            iface = self._interface(real)
            if iface is not None:
                ev["interface"] = iface
        return ProbeResult(ok=True, evidence=ev)
