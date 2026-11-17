"""按天滚动的 JSONL 故障日志。append-only，永不重写。"""

from __future__ import annotations

import json
import os
import re
import time
from pathlib import Path

_NAME_RE = re.compile(r"^journal-(\d{8})\.jsonl$")


def read_boot_id(path: str = "/proc/sys/kernel/random/boot_id") -> str:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read().strip() or "unknown"
    except OSError:
        return "unknown"


class Journal:
    def __init__(self, dir_path, clock=time.time, keep_days: int = 30,
                 boot_id_fn=read_boot_id, mono_fn=time.monotonic):
        self.dir = Path(dir_path)
        self.dir.mkdir(parents=True, exist_ok=True)
        self._clock = clock
        self._boot_id = boot_id_fn
        self._mono = mono_fn
        self.keep_days = keep_days

    def _path_for(self, ts: float) -> Path:
        return self.dir / ("journal-%s.jsonl" % time.strftime("%Y%m%d", time.localtime(ts)))

    def record(self, event: dict) -> dict:
        event = dict(event)
        event.setdefault("ts", self._clock())
        event.setdefault("boot_id", self._boot_id())
        event.setdefault("mono", round(self._mono(), 3))
        line = json.dumps(event, ensure_ascii=False, sort_keys=True, default=str)
        with open(self._path_for(event["ts"]), "a", encoding="utf-8") as f:
            f.write(line + "\n")
            f.flush()
            os.fsync(f.fileno())
        return event

    def read(self, since_ts: float | None = None) -> list[dict]:
        out: list[dict] = []
        for path in sorted(self.dir.glob("journal-*.jsonl")):
            try:
                lines = path.read_text(encoding="utf-8").splitlines()
            except OSError:
                continue
            for line in lines:
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if since_ts is not None and event.get("ts", 0) < since_ts:
                    continue
                out.append(event)
        out.sort(key=lambda e: e.get("ts", 0))
        return out

    def read_since(self, seconds: float, boot_id: str | None = None,
                   now_mono: float | None = None) -> list[dict]:
        """当前 boot 用 mono 过滤（墙钟可能被 NTP 校正），其它 boot 用墙钟。"""
        boot = self._boot_id() if boot_id is None else boot_id
        now_mono = self._mono() if now_mono is None else now_mono
        cutoff_wall = self._clock() - seconds
        anchor_mono = now_mono - seconds
        out = []
        for event in self.read():
            mono = event.get("mono")
            if event.get("boot_id") == boot and isinstance(mono, (int, float)):
                if mono >= anchor_mono:
                    out.append(event)
            elif event.get("ts", 0) >= cutoff_wall:
                out.append(event)
        out.sort(key=lambda e: (
            e.get("boot_id") != boot, e.get("mono", 0), e.get("ts", 0)))
        return out

    def prune(self) -> None:
        cutoff = self._clock() - self.keep_days * 86400
        cutoff_day = time.strftime("%Y%m%d", time.localtime(cutoff))
        for path in self.dir.glob("journal-*.jsonl"):
            m = _NAME_RE.match(path.name)
            if m and m.group(1) < cutoff_day:
                try:
                    path.unlink()
                except OSError:
                    pass
