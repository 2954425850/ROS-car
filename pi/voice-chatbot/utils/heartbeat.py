"""给 carhealth 的 1Hz 心跳。写失败绝不打断语音助手。"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path


def default_path() -> Path:
    shm = Path("/dev/shm/voice-heartbeat.json")
    if os.access("/dev/shm", os.W_OK):
        return shm
    base = Path.home() / ".local/state/voice-chatbot"
    base.mkdir(parents=True, exist_ok=True)
    return base / "heartbeat.json"


class Heartbeat:
    def __init__(self, path, clock=time.monotonic, wall=time.time,
                 interval_s: float = 1.0, status_provider=None):
        self.path = Path(path)
        self._clock = clock
        self._wall = wall
        self._interval = interval_s
        self._status_provider = status_provider
        now = clock()
        self._t0 = now
        self._state = "idle"
        self._state_since = now
        self._busy = False
        self._busy_since = now
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def update(self, state: str, busy: bool | None = None) -> None:
        now = self._clock()
        if state != self._state:
            self._state = state
            self._state_since = now
        if busy is not None and bool(busy) != self._busy:
            self._busy = bool(busy)
            if self._busy:
                self._busy_since = now

    def payload(self) -> dict:
        now = self._clock()
        data = {
            "pid": os.getpid(),
            "state": self._state,
            "busy": self._busy,
            "state_age_s": round(now - self._state_since, 1),
            "busy_for_s": round(now - self._busy_since, 1) if self._busy else 0.0,
            "uptime_s": round(now - self._t0, 1),
            "beat_wall": round(self._wall(), 3),
        }
        if self._status_provider is not None:
            try:
                extra = self._status_provider()
            except Exception:                               # noqa: BLE001
                extra = None
            if isinstance(extra, dict):
                data.update(extra)
        return data

    def write_once(self) -> None:
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp.write_text(json.dumps(self.payload(), ensure_ascii=False),
                           encoding="utf-8")
            os.replace(tmp, self.path)
        except OSError:
            pass

    def _run(self) -> None:
        while not self._stop.wait(self._interval):
            self.write_once()

    def start(self) -> None:
        if self._thread is not None:
            return
        self.write_once()
        self._thread = threading.Thread(target=self._run, name="heartbeat", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
