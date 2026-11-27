"""nav2 overlay 是否被真正解析到（而不是 /opt 的系统 nav2）。

ros2 pkg prefix 用的是进程的 AMENT_PREFIX_PATH；这里显式按 nav.service 的
启动顺序 source（/opt 之后 source ~/nav2_ws/install），所以能同时发现
「补丁编了但服务没 source」和「根本没编」两种漂移。
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

from ..types import ProbeResult


class Nav2OverlayProbe:
    name = "cmd:nav2-overlay"
    interval_s = 60.0

    def __init__(self, prefix=None, runner=subprocess.run):
        self._prefix = Path(prefix) if prefix else Path.home() / "nav2_ws/install"
        self._runner = runner

    def poll(self) -> ProbeResult:
        cmd = (
            "source /opt/ros/jazzy/setup.bash; "
            'if [ -f "$HOME/nav2_ws/install/setup.bash" ]; then '
            'source "$HOME/nav2_ws/install/setup.bash"; fi; '
            "ros2 pkg prefix nav2_smac_planner"
        )
        try:
            r = self._runner(["/bin/bash", "-c", cmd],
                             capture_output=True, text=True, timeout=20)
        except (OSError, subprocess.TimeoutExpired) as e:
            # 探测本身失败 ≠ 判据通过：交给 check 把 is_overlay=false 标出来。
            return ProbeResult(ok=True, evidence={
                "prefix": None, "is_overlay": False, "error": str(e)})

        out = (r.stdout or "").strip().splitlines()
        prefix = out[-1] if out else ""
        want = os.path.realpath(str(self._prefix))
        real = os.path.realpath(prefix) if prefix else ""
        is_overlay = bool(real) and (real == want or real.startswith(want + os.sep))
        return ProbeResult(ok=True, evidence={
            "prefix": prefix,
            "expected_prefix": str(self._prefix),
            "is_overlay": is_overlay,
            "returncode": r.returncode,
            "error": None if r.returncode == 0 else (r.stderr or "").strip()[:200],
        })
