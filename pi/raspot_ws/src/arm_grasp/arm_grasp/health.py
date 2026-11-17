"""抓取服务的 1Hz 心跳载荷。纯函数，方便单测。"""

from __future__ import annotations

import os
import time


def health_payload(busy, uptime_s: float, wall: float | None = None,
                   feedback_age_s=None, last_goal_wall=None) -> dict:
    return {
        "component": "arm_grasp",
        "busy": bool(busy),
        "uptime_s": round(float(uptime_s), 1),
        "pid": os.getpid(),
        "wall": round(time.time() if wall is None else float(wall), 3),
        "feedback_age_s": (None if feedback_age_s is None
                           else round(float(feedback_age_s), 2)),
        "last_goal_wall": (None if last_goal_wall is None
                           else round(float(last_goal_wall), 3)),
    }
