"""把契约、探针、引擎装配起来。daemon 与 doctor 共用。"""

from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass
from pathlib import Path

from .contracts import (
    FRAMED_STATUS, LATEST_IDR, RESULTD_STATUS, ROSBRIDGE_ADDR,
    VOICE_HEARTBEAT_PATHS, build_specs,
)
from .engine import Engine, EngineConfig, default_journal_dir, default_state_dir
from .recovery import RecoveryConfig
from .probes.config_drift import ConfigDriftProbe
from .probes.dds import DdsProbe
from .probes.file_status import FileStatusProbe
from .probes.host import HostProbe
from .probes.net import PingProbe, TcpProbe, resolve_board_host
from .probes.net_latency import GatewayLatencyProbe
from .probes.power import PowerProbe
from .probes.ros_topic import RosNodeBackend, RosTopicStore, TopicProbe
from .probes.service_health import ServiceHealthProbe
from .probes.systemd import SystemdCache, UnitProbe
from .probes.udev import UdevProbe
from .probes.usb_events import UsbEventsProbe
from .probes.wifi import WifiProbe

UNITS = ("ps2-teleop", "tof-a010", "k230-resultd", "k230-bridge",
         "k230-framed", "rosbridge", "arm-grasp")

# 只订阅低频摘要。驱动把 25/50Hz 话题汇总成 1Hz /health/driver；
# 守护总量约 13 条/秒，CPU ~1-2%（2026-11-18 Pi 实测）。
TOPICS = ("/tof/status", "/health/driver", "/health/arm_grasp", "/k230/result")


@dataclass
class ProbeBundle:
    probes: dict
    closer: object = None


def build_probes(ros: bool = True, board_host: str | None = None,
                 fake_throttled: str | None = None,
                 systemd_runner=subprocess.run, repo: str | None = None) -> ProbeBundle:
    store = RosTopicStore()
    probes: dict = {}

    probes["power"] = PowerProbe(fake_output=fake_throttled)
    probes["host"] = HostProbe()

    cache = SystemdCache(UNITS, runner=systemd_runner)
    unit_probes = {}
    for unit in UNITS:
        probe = UnitProbe("unit:" + unit, unit, cache)
        probes[probe.name] = probe
        unit_probes[unit] = probe
    probes["service"] = ServiceHealthProbe(unit_probes)

    probes["udev:l150pro"] = UdevProbe("udev:l150pro", ["/dev/l150pro"])
    probes["udev:a010"] = UdevProbe(
        "udev:a010", ["/dev/a010", "/dev/serial/by-id/*SIPEED*if00-port0"],
        expect_interface="00")
    probes["udev:js0"] = UdevProbe("udev:js0", ["/dev/input/js0"])

    probes["file:resultd-status"] = FileStatusProbe("file:resultd-status", [RESULTD_STATUS])
    probes["file:framed-status"] = FileStatusProbe("file:framed-status", [FRAMED_STATUS])
    probes["file:latest-idr"] = FileStatusProbe("file:latest-idr", [LATEST_IDR],
                                                parse_json=False)
    probes["file:voice-heartbeat"] = FileStatusProbe(
        "file:voice-heartbeat", VOICE_HEARTBEAT_PATHS)

    if board_host is None:
        host_fn = lambda: resolve_board_host(  # noqa: E731
            status_path=str(RESULTD_STATUS))
    else:
        host_fn = lambda: board_host          # noqa: E731
    probes["net:board"] = PingProbe("net:board", host_fn=host_fn)
    probes["net:rosbridge"] = TcpProbe("net:rosbridge", ROSBRIDGE_ADDR[0],
                                       ROSBRIDGE_ADDR[1])
    probes["net:gateway"] = GatewayLatencyProbe()
    probes["wifi"] = WifiProbe()

    probes["usb"] = UsbEventsProbe()
    probes["dds"] = DdsProbe()
    probes["config_drift"] = ConfigDriftProbe(
        repo=repo or os.environ.get(
            "CARHEALTH_REPO", str(Path(__file__).resolve().parents[3])))

    for topic in TOPICS:
        probes["topic:" + topic] = TopicProbe("topic:" + topic, topic, store)

    closer = None
    if ros:
        backend = RosNodeBackend(store)
        if backend.start():
            closer = backend.stop
        else:
            store.set_blind(backend.reason or "ROS 后端未启动")
    else:
        store.set_blind("ROS 后端未启用（--no-ros）")
    return ProbeBundle(probes=probes, closer=closer)


def build_engine(state_dir=None, journal_dir=None, ros: bool = True,
                 board_host: str | None = None, fake_throttled: str | None = None,
                 interval_s: float = 1.0, systemd_runner=subprocess.run,
                 repo: str | None = None, recovery=None) -> Engine:
    state_dir = Path(state_dir) if state_dir else default_state_dir()
    journal_dir = Path(journal_dir) if journal_dir else default_journal_dir()
    bundle = build_probes(ros=ros, board_host=board_host,
                          fake_throttled=fake_throttled,
                          systemd_runner=systemd_runner, repo=repo)
    engine = Engine(build_specs(), bundle.probes,
                    EngineConfig(state_dir=state_dir, journal_dir=journal_dir,
                                 interval_s=interval_s,
                                 recovery=recovery or RecoveryConfig()))
    engine.closer = bundle.closer
    return engine
