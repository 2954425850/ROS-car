"""组件契约：唯一阈值来源。改判据只改这里。

判据设计见 docs/plans/2026-11-17-car-health-guard-design.md §5。
"""

from __future__ import annotations

from pathlib import Path

from .types import Check, ComponentSpec, Verdict

VOICE_HEARTBEAT_PATHS = (
    Path("/dev/shm/voice-heartbeat.json"),
    Path.home() / ".local/state/voice-chatbot/heartbeat.json",
)
RESULTD_STATUS = Path("/tmp/k230/resultd-status.json")
FRAMED_STATUS = Path("/tmp/k230/framed-status.json")
LATEST_IDR = Path("/dev/shm/k230/latest-idr.h264")
ROSBRIDGE_ADDR = ("127.0.0.1", 9090)


def _unit(unit: str, name: str | None = None) -> Check:
    return Check(
        probe="unit:" + (name or unit),
        op="eq", value=True, field="active",
        level=Verdict.ABSENT,
        reason="%s 单元不在运行（systemd ActiveState != active）" % unit,
    )


def build_specs() -> tuple[ComponentSpec, ...]:
    return (
        ComponentSpec("power", "供电", (
            Check("power", "eq", False, field="uv_now", level=Verdict.BLOCKED,
                  reason="当前欠压（vcgencmd bit0）"),
            Check("power", "eq", False, field="uv_recent", level=Verdict.BLOCKED,
                  reason="进程观察到 10 分钟内出现过欠压（USB 设备可能已掉）"),
            Check("power", "eq", False, field="uv_since_boot", level=Verdict.DEGRADED,
                  reason="本次开机出现过欠压（历史位，时间未知）"),
            Check("power", "eq", False, field="freq_capped", level=Verdict.DEGRADED,
                  reason="CPU 正在降频（throttled bit1）"),
            Check("power", "eq", False, field="soft_temp", level=Verdict.DEGRADED,
                  reason="软温度上限触发（throttled bit3）"),
        )),

        ComponentSpec("board", "K230 板", (
            Check("net:board", "eq", True, field="ping_ok", level=Verdict.BLOCKED,
                  reason="K230 板 ping 不通（掉电 / 掉 WiFi / IP 变了）"),
        )),

        ComponentSpec("host", "主机资源", (
            Check("host", "gt", 10, field="disk_free_pct", level=Verdict.DEGRADED,
                  reason="根分区剩余 <10%"),
            Check("host", "gt", 150, field="mem_available_mb", level=Verdict.DEGRADED,
                  reason="可用内存 <150MB"),
            Check("host", "lt", 80, field="soc_temp_c", level=Verdict.DEGRADED,
                  reason="SoC 温度 ≥80°C"),
            Check("host", "lt", 6.0, field="load1", level=Verdict.DEGRADED,
                  reason="1 分钟负载 ≥6（实测出现过 17.8，会放大抖动）"),
        )),

        ComponentSpec("scan_gate", "导航扫描门控", (
            Check("unit:scan-gate", "eq", True, field="active",
                  level=Verdict.ABSENT,
                  reason="scan-gate 不在运行：collision_monitor 会因 "
                         "/scan_motion_guard 超时停车（看起来像被墙挡住）"),
        ), blocked_by=("power",)),

        ComponentSpec("battery", "电池", (
            Check("topic:/health/driver", "lt", 3.0, field="age_s", level=Verdict.STALE,
                  reason="/health/driver 超过 3s 没有数据"),
            Check("topic:/health/driver", "gt", 11.5, field="battery_voltage",
                  level=Verdict.DEGRADED, reason="电池电压 <11.5V"),
        )),

        ComponentSpec("chassis", "底盘", (
            _unit("ps2-teleop"),
            Check("udev:l150pro", "eq", True, field="exists", level=Verdict.ABSENT,
                  reason="底盘板串口 /dev/l150pro 不存在"),
            Check("udev:js0", "eq", True, field="exists", level=Verdict.ABSENT,
                  reason="PS2 接收器 /dev/input/js0 不存在"),
            Check("topic:/health/driver", "lt", 2.0, field="age_s", level=Verdict.STALE,
                  reason="/health/driver 超过 2s 没有数据"),
            Check("topic:/health/driver", "lt", 1.0, field="telemetry_age_s",
                  level=Verdict.STALE, reason="底盘遥测 0x55 超过 1s 没有数据"),
            Check("topic:/health/driver", "eq", 0, field="chassis_faults",
                  level=Verdict.DEGRADED,
                  reason="底盘固件报告故障位（看 /chassis_faults 与驱动日志）"),
        ), blocked_by=("power",)),

        ComponentSpec("arm", "机械臂", (
            _unit("ps2-teleop"),
            Check("udev:l150pro", "eq", True, field="exists", level=Verdict.ABSENT,
                  reason="底盘/臂共用串口 /dev/l150pro 不存在"),
            Check("topic:/health/driver", "lt", 2.0, field="age_s", level=Verdict.STALE,
                  reason="/health/driver 超过 2s 没有数据"),
            Check("topic:/health/driver", "lt", 0.5, field="feedback_age_s",
                  level=Verdict.STALE, reason="机械臂回读 0x56 超过 0.5s 没有数据"),
            Check("topic:/health/driver", "eq", True, field="arm_online",
                  level=Verdict.STALE, reason="机械臂未 READY（arm_online=false）"),
            Check("topic:/health/driver", "eq", False, field="arm_estop",
                  level=Verdict.DEGRADED, reason="机械臂固件急停位置位"),
        ), blocked_by=("power",)),

        ComponentSpec("tof", "ToF 深度", (
            _unit("tof-a010"),
            Check("udev:a010", "eq", True, field="exists", level=Verdict.ABSENT,
                  reason="ToF 设备节点不存在（/dev/a010 与 by-id if00 都没有）"),
            Check("udev:a010", "eq", "short", field="resolved_by", level=Verdict.DEGRADED,
                  reason="短名 /dev/a010 不存在，走了 by-id（99-tof-a010.rules 可能未装）"),
            Check("udev:a010", "eq", "00", field="interface", level=Verdict.ABSENT,
                  reason="ToF 接到了 FT2232 的通道 B（接口号应为 00）"),
            Check("topic:/tof/status", "lt", 2.0, field="age_s", level=Verdict.STALE,
                  reason="/tof/status 超过 2s 没有更新"),
            Check("topic:/tof/status", "eq", True, field="connected", level=Verdict.DEGRADED,
                  reason="ToF 串口未连接"),
            Check("topic:/tof/status", "gt", 0, field="fps", level=Verdict.DEGRADED,
                  reason="ToF 帧率为 0"),
        ), blocked_by=("power",)),

        ComponentSpec("grasp", "抓取服务", (
            _unit("arm-grasp"),
            Check("topic:/health/arm_grasp", "lt", 3.0, field="age_s", level=Verdict.STALE,
                  reason="/health/arm_grasp 心跳超过 3s"),
            Check("topic:/health/arm_grasp", "lt", 1.0, field="feedback_age_s",
                  level=Verdict.STALE,
                  reason="抓取服务收不到 /arm/feedback（臂驱动可能已停）"),
        ), blocked_by=("power",)),

        ComponentSpec("k230_link", "K230 结果链路", (
            _unit("k230-resultd"),
            _unit("k230-bridge"),
            Check("file:resultd-status", "lt", 7.0, field="age_s", level=Verdict.STALE,
                  reason="resultd 状态文件超过 7s 未更新"),
            Check("topic:/k230/result", "lt", 3.0, field="age_s", level=Verdict.STALE,
                  reason="/k230/result 超过 3s 没有数据"),
        ), blocked_by=("board",)),

        ComponentSpec("k230_video", "K230 视频链路", (
            _unit("k230-framed"),
            Check("file:framed-status", "lt", 3.0, field="age_s", level=Verdict.STALE,
                  reason="framed 状态文件超过 3s 未更新"),
            Check("file:framed-status", "eq", True, field="connected", level=Verdict.DEGRADED,
                  reason="板子没有连上 8555 推流口"),
            Check("file:latest-idr", "lt", 5.0, field="age_s", level=Verdict.STALE,
                  reason="最新 IDR 关键帧超过 5s（GOP 间隔需现场确认）"),
        ), blocked_by=("board",)),

        ComponentSpec("rosbridge", "rosbridge", (
            _unit("rosbridge"),
            Check("net:rosbridge", "eq", True, field="tcp_ok", level=Verdict.DEGRADED,
                  reason="127.0.0.1:9090 连不上"),
        )),

        ComponentSpec("voice", "语音助手", (
            Check("file:voice-heartbeat", "lt", 5.0, field="age_s", level=Verdict.STALE,
                  reason="语音助手心跳超过 5s（进程卡死或退出）"),
            Check("file:voice-heartbeat", "lt", 180.0, field="busy_for_s", level=Verdict.STALE,
                  reason="语音助手在忙状态超过 180s（疑似 API 卡死）"),
            Check("file:voice-heartbeat", "eq", True, field="wakeword_ready",
                  level=Verdict.STALE,
                  reason="唤醒模块串口不可用（USB 重枚举后喊它没反应）"),
            Check("file:voice-heartbeat", "eq", True, field="wakeword_thread_alive",
                  level=Verdict.STALE,
                  reason="唤醒词监听线程不在"),
            Check("file:voice-heartbeat", "eq", True, field="mic_open",
                  level=Verdict.DEGRADED,
                  reason="麦克风句柄未打开"),
        )),
    )
