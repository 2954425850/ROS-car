"""已知故障库。每条规则只做「特征 → 结论 + 动作类别 + 指令」。

规则不改变六态判定；它们让 status/doctor/why 直接给出人话结论。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .types import Diagnosis, ProbeResult, Verdict


@dataclass
class FaultContext:
    results: dict[str, ProbeResult]
    evals: dict
    now_mono: float


def ev(ctx: FaultContext, probe: str) -> dict:
    r = ctx.results.get(probe)
    return r.evidence if r is not None and r.ok is True else {}


def raw_ev(ctx: FaultContext, probe: str) -> dict:
    """连 ok=None 的探针证据也要看（例如「地址未知」本身就是结论）。"""
    r = ctx.results.get(probe)
    return dict(r.evidence or {}) if r is not None else {}


def _root(ctx: FaultContext, component: str):
    return getattr(ctx.evals.get(component), "root_cause", None)


def _any_root(ctx: FaultContext, cause: str) -> bool:
    return any(_root(ctx, name) == cause for name in ctx.evals)


def _count_at_least(ctx: FaultContext, probe: str, key: str, threshold: int) -> bool:
    counts = ev(ctx, probe).get(key) or {}
    return any(int(value) >= threshold for value in counts.values())


def _number(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


@dataclass(frozen=True)
class Rule:
    id: str
    title: str
    action_class: str          # wait / restart_unit / restart_stack / human / fix_config / none
    hint: str
    components: tuple[str, ...]  # 空 = 全局
    when: Callable[[FaultContext], bool]


RULES = (
    Rule("power.undervoltage", "Pi 供电欠压", "wait",
         "检查电源线、接插件与驱动板压降；欠压会连带打掉 USB 设备",
         ("power",),
         lambda c: bool(ev(c, "power").get("uv_now") or ev(c, "power").get("uv_recent"))),

    Rule("usb.disconnect_storm", "USB 设备在反复重枚举", "human",
         "10 分钟内同一设备断开 ≥3 次：换线、换口、检查供电",
         ("host",),
         lambda c: _count_at_least(c, "usb", "disconnects", 3)),

    Rule("udev.l150pro_missing", "底盘/臂串口稳定名不存在", "fix_config",
         "检查 99-l150pro.rules 是否安装、serial 是否匹配",
         ("chassis", "arm"),
         lambda c: ev(c, "udev:l150pro").get("exists") is False),

    Rule("udev.tof_byid_only", "ToF 短名丢失，只走了 by-id", "fix_config",
         "装 99-tof-a010.rules；功能可能仍可用，但节点名不再稳定",
         ("tof",),
         lambda c: ev(c, "udev:a010").get("resolved_by") == "by_id"),

    Rule("udev.tof_channel_b", "ToF 接到了 FT2232 的通道 B", "human",
         "插回通道 A；通道 B 能打开但一个字节都收不到",
         ("tof",),
         lambda c: (ev(c, "udev:a010").get("interface") not in (None, "00"))),

    Rule("udev.rules_missing", "udev 规则未安装", "fix_config",
         "按 pi/systemd/*.rules 安装并 udevadm reload/trigger",
         (),
         lambda c: bool(ev(c, "config_drift").get("missing_udev_rules"))),

    Rule("dds.stale_ip", "DDS 仍绑着旧网段地址", "restart_stack",
         "换网段后只重启一个单元修不好：整条 ROS 栈一起重启",
         ("chassis",),
         lambda c: bool(ev(c, "dds").get("stale_binds"))),

    Rule("ros.discovery", "ROS 发现层故障（0 个发布者）", "restart_stack",
         "检查 ROS_DOMAIN_ID / 网卡 / DDS 发现；再整栈重启",
         (),
         lambda c: _any_root(c, "ros_discovery")),

    Rule("ros.data_path", "有发布者但收不到数据", "human",
         "查 QoS 兼容、MTU 分片、WiFi 质量",
         (),
         lambda c: _any_root(c, "ros_data_path")),

    Rule("service.crash_loop", "服务在崩溃循环（被 Restart=always 掩盖）", "fix_config",
         "看 journalctl -u <unit>；NRestarts 在涨说明它一直在死",
         (),
         lambda c: bool(ev(c, "service").get("crash_loop_units"))),

    Rule("service.ordering_cycle", "开机时出现 systemd ordering cycle", "fix_config",
         "修 unit 的 After/Wants；arm-grasp inactive (dead) 常见于此",
         (),
         lambda c: (_number(ev(c, "config_drift").get("ordering_cycles"), 0) > 0)),

    Rule("service.enabled_but_dead", "enabled 的服务处于 inactive (dead)", "fix_config",
         "systemctl status 看失败原因；别只重启，先修根因",
         (),
         lambda c: bool(ev(c, "config_drift").get("enabled_but_dead"))),

    Rule("build.stale", "install 比源码旧", "fix_config",
         "colcon build --symlink-install，再重启服务",
         (),
         lambda c: bool(ev(c, "config_drift").get("stale_packages"))),

    Rule("k230.host_unknown", "K230 地址未知", "wait",
         "确认板子上电/连 WiFi；Pi 重启后 /tmp 清空、尚无 peer 属正常",
         ("board",),
         lambda c: ("host" in raw_ev(c, "net:board")
                    and raw_ev(c, "net:board").get("host") is None)),

    Rule("k230.push_stall", "板子连着推流口但关键帧不更新", "human",
         "检查 K230 推流与 WiFi；板子不能主动请求 IDR",
         ("k230_video",),
         lambda c: (ev(c, "file:framed-status").get("connected") is True
                    and _number(ev(c, "file:latest-idr").get("age_s"), 0) > 5.0)),

    Rule("battery.low", "电池电压低", "wait",
         "11.2–12.3V 性能差近一倍，别误判成控制环路问题",
         ("battery",),
         lambda c: (_number(ev(c, "topic:/battery_voltage").get("data"), 99) < 11.2)),

    Rule("arm.estop", "机械臂固件急停位置位", "human",
         "检查急停开关/固件状态；/arm/status bit7",
         ("arm",),
         lambda c: bool(int(ev(c, "topic:/arm/status").get("data") or 0) & 0x80)),

    Rule("chassis.faults", "底盘固件报告故障位", "human",
         "看 /chassis_faults 与驱动日志；欠压/IMU/链路超时是常见位",
         ("chassis",),
         lambda c: (ev(c, "topic:/chassis_faults").get("data") not in (None, 0))),

    Rule("host.load_high", "主机负载高", "wait",
         "查是不是软解视频/编译/多个节点叠加；高负载会放大抖动",
         ("host",),
         lambda c: (_number(ev(c, "host").get("load1"), 0) >= 6.0)),

    Rule("host.cpu_high", "主机 CPU 使用率高", "wait",
         "持续 ≥85% 会放大所有超时判据的抖动；先找是谁在吃 CPU",
         ("host",),
         lambda c: (_number(ev(c, "host").get("cpu_pct"), 0) >= 85.0)),

    Rule("service.cpu_high", "有服务 CPU 占用高", "human",
         "单服务 >200%（约 2 核）通常是软解/死循环",
         (),
         lambda c: any((_number(v, 0) or 0) > 200
                       for v in (ev(c, "service").get("cpu_pct") or {}).values())),

    Rule("host.wifi_weak", "WiFi 信号弱", "human",
         "信号 < -75dBm 或链路质量 <30：换信道/靠近 AP/换 5G",
         ("host",),
         lambda c: ((_number(ev(c, "host").get("wifi_signal_dbm"), 0) < -75)
                    or (_number(ev(c, "host").get("wifi_link"), 99) < 30))),

    Rule("service.memory_high", "有服务内存占用过高", "human",
         "看是否内存泄漏；>1GB 通常会拖垮整机",
         (),
         lambda c: any((_number(v, 0) or 0) > 1024
                       for v in (ev(c, "service").get("memory_mb") or {}).values())),

    Rule("voice.wakeword_lost", "唤醒模块掉线", "human",
         "唤醒模块被拔/重枚举；按路径重连已实现，仍不行就换口",
         ("voice",),
         lambda c: (ev(c, "file:voice-heartbeat").get("wakeword_ready") is False
                    or ev(c, "file:voice-heartbeat").get("wakeword_thread_alive") is False)),

    Rule("voice.api_stuck", "语音助手在忙状态卡住", "restart_unit",
         "看 ASR/LLM 报错；历史上 API 400 会卡在 THINKING",
         ("voice",),
         lambda c: (_number(ev(c, "file:voice-heartbeat").get("busy_for_s"), 0) >= 180)),

    Rule("voice.mic_error", "麦克风读取失败", "human",
         "USB 重枚举/设备被抢占；换口或重启语音助手",
         ("voice",),
         lambda c: bool(ev(c, "file:voice-heartbeat").get("mic_last_error"))),

    Rule("tof.last_error_stale", "ToF 连着但保留着旧错误", "none",
         "仅提示：该字段重连后不清零，不代表当前故障",
         ("tof",),
         lambda c: (ev(c, "topic:/tof/status").get("connected") is True
                    and bool(ev(c, "topic:/tof/status").get("last_error")))),
)

RULES_BY_ID = {rule.id: rule for rule in RULES}
