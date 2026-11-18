"""l150pro 驱动的 1Hz 健康摘要载荷。

为什么要单独发一条 1Hz JSON：carhealth 直接订阅 /wheel_odom(50Hz)、
/chassis_faults(25Hz)、/arm/online(25Hz) 等话题时，rclpy 每条消息的固定
开销会把守护 CPU 推到 ~18-28%（2026-11-18 Pi 实测）。驱动本来就在 25Hz
定时器里持有全部状态，汇总成一条 1Hz 消息后，守护只剩 ~13 条消息/秒。
"""

from __future__ import annotations

import os
import time


def build_driver_health(*, arm_online: bool, arm_estop: bool, arm_status: int,
                        ctrl_live: bool, arm_cmd_age_s, chassis_faults: int,
                        battery_voltage: float, seen55: bool, seen56: bool,
                        feedback_age_s, telemetry_age_s, cmd_age_s,
                        cmd_vx: float, cmd_wz: float,
                        odom_vx: float, odom_wz: float,
                        wall: float | None = None) -> dict:
    def _age(value):
        return None if value is None else round(float(value), 3)

    return {
        "component": "l150pro_driver",
        "arm_online": bool(arm_online),
        "arm_estop": bool(arm_estop),
        "arm_status": int(arm_status),
        "ctrl_live": bool(ctrl_live),
        "arm_cmd_age_s": _age(arm_cmd_age_s),
        "chassis_faults": int(chassis_faults),
        "battery_voltage": round(float(battery_voltage), 2),
        "seen55": bool(seen55),
        "seen56": bool(seen56),
        "feedback_age_s": _age(feedback_age_s),
        "telemetry_age_s": _age(telemetry_age_s),
        "cmd_age_s": _age(cmd_age_s),
        "cmd_vx": round(float(cmd_vx), 3),
        "cmd_wz": round(float(cmd_wz), 3),
        "odom_vx": round(float(odom_vx), 3),
        "odom_wz": round(float(odom_wz), 3),
        "pid": os.getpid(),
        "wall": round(time.time() if wall is None else float(wall), 3),
    }
