"""配置漂移：ordering cycle、enabled 但 inactive、udev 规则、git HEAD、build 过期。"""

from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path

from ..types import ProbeResult

EXPECTED_RULES = ("99-k230.rules", "99-tof-a010.rules", "99-l150pro.rules",
                  "99-myspeech.rules")


def parse_ordering_cycles(text: str) -> int:
    return sum(1 for line in (text or "").splitlines() if "ordering cycle" in line)


def build_stale_packages(repo: str, packages, exists=os.path.exists,
                         mtime=os.path.getmtime) -> list[str]:
    stale = []
    for pkg in packages:
        src = Path(repo) / "pi/raspot_ws/src" / pkg
        install = Path(repo) / "pi/raspot_ws/install" / pkg / "share" / pkg / "package.xml"
        if not exists(str(install)):
            continue
        sources = [src / "package.xml", src / "setup.py", src / "setup.cfg"]
        latest = max((mtime(str(p)) for p in sources if exists(str(p))), default=0.0)
        if latest > mtime(str(install)):
            stale.append(pkg)
    return stale


class ConfigDriftProbe:
    name = "config_drift"
    interval_s = 60.0

    def __init__(self, repo: str, rules_dir: str = "/etc/udev/rules.d",
                 packages=("arm_grasp", "k230_ros", "tof_a010"),
                 units=("ps2-teleop", "tof-a010", "k230-resultd", "k230-bridge",
                        "k230-framed", "rosbridge", "arm-grasp"),
                 runner=subprocess.run, clock=time.monotonic):
        self.repo = repo
        self.rules_dir = Path(rules_dir)
        self.packages = tuple(packages)
        self.units = tuple(units)
        self._runner = runner
        self._clock = clock
        self._ordering = None
        self._ordering_retry = 0.0

    def _run(self, cmd, timeout=5):
        try:
            return self._runner(cmd, capture_output=True, text=True, timeout=timeout)
        except (OSError, subprocess.TimeoutExpired):
            return None

    def poll(self) -> ProbeResult:
        ev: dict = {"errors": []}

        # `journalctl -g` 在全量 boot 日志上可能超过 5s，而且「没有匹配」会返回 1
        # （不是错误）。改成有界的最近 5000 行 + Python 过滤，并在本次启动内缓存：
        # ordering cycle 是开机事件，不需要每次轮询都重扫。
        now = self._clock()
        if self._ordering is None and now >= self._ordering_retry:
            r = self._run(["journalctl", "-b", "-n", "5000", "--no-pager", "-o", "cat"],
                          timeout=10)
            if r is not None and r.returncode == 0:
                self._ordering = parse_ordering_cycles(r.stdout)
            else:
                self._ordering_retry = now + 300
                ev["errors"].append("journalctl 不可读（5 分钟后重试）")
        if self._ordering is not None:
            ev["ordering_cycles"] = self._ordering

        dead = []
        for unit in self.units:
            r = self._run(["systemctl", "show", "-p", "ActiveState",
                           "-p", "UnitFileState", unit + ".service"])
            if r is None or r.returncode != 0:
                continue
            values = dict(
                line.split("=", 1) for line in r.stdout.splitlines() if "=" in line)
            if (values.get("UnitFileState") == "enabled"
                    and values.get("ActiveState") == "inactive"):
                dead.append(unit)
        ev["enabled_but_dead"] = dead

        ev["missing_udev_rules"] = [
            name for name in EXPECTED_RULES
            if not (self.rules_dir / name).exists()
        ]

        branch = self._run(["git", "-C", self.repo, "rev-parse", "--abbrev-ref", "HEAD"])
        head = self._run(["git", "-C", self.repo, "rev-parse", "--short", "HEAD"])
        ev["git_branch"] = branch.stdout.strip() if branch and branch.returncode == 0 else None
        ev["git_head"] = head.stdout.strip() if head and head.returncode == 0 else None

        ev["stale_packages"] = build_stale_packages(self.repo, self.packages)
        return ProbeResult(ok=True, evidence=ev)
