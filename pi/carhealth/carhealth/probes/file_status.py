"""status 文件的 mtime 新鲜度 + JSON 内容。沿用 k230ctl status 的语义。"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

from ..types import ProbeResult


class FileStatusProbe:
    def __init__(self, name: str, paths, parse_json: bool = True,
                 clock=time.monotonic, wall=time.time, interval_s: float = 1.0):
        self.name = name
        self.interval_s = interval_s
        if isinstance(paths, (str, Path)):
            paths = [paths]
        self.paths = tuple(Path(p) for p in paths)
        self.parse_json = parse_json
        self._clock = clock
        self._wall = wall

    def poll(self) -> ProbeResult:
        best: tuple[Path, float] | None = None
        for path in self.paths:
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                continue
            if best is None or mtime > best[1]:
                best = (path, mtime)

        if best is None:
            return ProbeResult(ok=True, evidence={
                "exists": False, "age_s": None, "path": None,
            })

        path, mtime = best
        age = max(0.0, self._wall() - mtime)
        ev: dict = {"exists": True}
        if self.parse_json:
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    ev.update(data)
            except (OSError, ValueError) as e:
                ev["parse_error"] = str(e)
        ev["age_s"] = round(age, 3)
        ev["path"] = str(path)
        return ProbeResult(ok=True, evidence=ev)
