# -*- coding: utf-8 -*-
"""`tof_a010.config.resolve_device` 的回归测试。**不碰硬件、不碰 ROS。**

这个文件的核心是第一条 —— 2026-11-18 那次"雷达转一会儿停三四秒"的真凶：
ToF 节点找不到自己的串口时，旧代码兜底到裸 `/dev/ttyUSB0`，而那个口当时是
RPLIDAR 的 CP2102，于是每 ~20 秒把雷达电机停一次。
"""
import os

from tof_a010 import config


def _empty_by_id(tmp_path, monkeypatch):
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    monkeypatch.setattr(config, "BY_ID_DIR", str(by_id), raising=False)
    return by_id


def _empty_sysfs(tmp_path, monkeypatch):
    sysfs = tmp_path / "usb-devices"
    sysfs.mkdir()
    monkeypatch.setattr(config, "SYSFS_USB", str(sysfs), raising=False)
    return sysfs


# --------------------------------------------------------------------------
# ★ 事故回归：绝不按"位置"兜底
# --------------------------------------------------------------------------

def test_never_falls_back_to_a_bare_ttyusb(tmp_path, monkeypatch):
    """找不到身份匹配的路径时，**绝不返回任何 /dev/ttyUSB***。

    旧代码在这一刻返回 `/dev/ttyUSB0` —— 那次那个口是雷达的 CP2102，
    打开串口会把 DTR 拉高，而 DTR 就是那块适配器的电机开关。
    """
    _empty_by_id(tmp_path, monkeypatch)
    _empty_sysfs(tmp_path, monkeypatch)
    # 假装 /dev/ttyUSB0 存在（Pi 上它真的存在，只是**不是我们的设备**）
    monkeypatch.setattr(config.os.path, "exists", lambda p: p == "/dev/ttyUSB0")

    path, _why = config.resolve_device({"device": "/dev/a010"})

    assert "ttyUSB" not in path, "不许按位置兜底 —— 那会开到别人的串口上"
    assert path == "/dev/a010", "找不到就原样返回，让调用方只报错不开串口"


def test_config_path_still_wins_when_it_exists(tmp_path, monkeypatch):
    """显式配置的路径存在时，照旧用它（不带兜底逻辑）。"""
    _empty_by_id(tmp_path, monkeypatch)
    _empty_sysfs(tmp_path, monkeypatch)
    monkeypatch.setattr(config.os.path, "exists", lambda p: p == "/dev/a010")

    path, why = config.resolve_device({"device": "/dev/a010"})

    assert path == "/dev/a010"
    assert why == "配置指定"


def test_uses_meta_sense_by_id_channel_a(tmp_path, monkeypatch):
    """模块在的时候，走 by-id 的通道 A（`if00` = 数据口）。"""
    by_id = _empty_by_id(tmp_path, monkeypatch)
    _empty_sysfs(tmp_path, monkeypatch)
    real = tmp_path / "ttyUSB3"
    real.write_text("")
    link = by_id / "usb-FTDI_FT2232H_Device_Meta_Sense_A-if00-port0"
    try:
        os.symlink(real, link)
    except (OSError, NotImplementedError):       # Windows 上建不了软链
        link.write_text("")
    monkeypatch.setattr(config.os.path, "exists", lambda p: False)

    path, why = config.resolve_device({"device": "/dev/a010"})

    assert path.endswith("if00-port0")
    assert "by-id" in why


# --------------------------------------------------------------------------
# 报错要说人话：枚举失败 ≠ 没插
# --------------------------------------------------------------------------

def test_explains_usb_enumeration_failure(tmp_path, monkeypatch):
    """FTDI 在总线上、却没生成 tty ⇒ 要说"枚举失败、去拔插"，不是"没插"。"""
    _empty_by_id(tmp_path, monkeypatch)
    sysfs = _empty_sysfs(tmp_path, monkeypatch)
    dev = sysfs / "2-2"
    dev.mkdir()
    (dev / "idVendor").write_text("0403\n")
    monkeypatch.setattr(config.os.path, "exists", lambda p: False)

    path, why = config.resolve_device({"device": "/dev/a010"})

    assert path == "/dev/a010"
    assert "没生成 ttyUSB" in why
    assert "拔插" in why


def test_says_module_missing_when_no_ftdi_at_all(tmp_path, monkeypatch):
    _empty_by_id(tmp_path, monkeypatch)
    _empty_sysfs(tmp_path, monkeypatch)
    monkeypatch.setattr(config.os.path, "exists", lambda p: False)

    _path, why = config.resolve_device({"device": "/dev/a010"})

    assert "没有 FTDI" in why
