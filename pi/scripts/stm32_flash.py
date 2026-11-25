#!/usr/bin/env python3
"""stm32_flash.py — 在树莓派上通过板载 USB 串口给 STM32F407 烧固件。

走的链路：Pi ── Type-C ── CH9102F(CDC-ACM) ── USART1(PA9/PA10) ── STM32F407VE
烧写协议是 ST 的 ROM bootloader（AN3155），工具用 stm32flash。

能这么干的前提（PC 上用 FlyMcu 已验证）：
    板载 CH9102 的 DTR -> NRST （低电平复位）
                   RTS -> BOOT0（高电平进 bootloader）
所以这里用 pyserial 把这两根"虚拟线"拉一遍，就把芯片送进 bootloader 了。
BOOT0 只在 NRST 上升沿被采样，所以摆完线可以放心关掉串口交给 stm32flash。

用法
----
    sudo systemctl stop ps2-teleop        # 必须先让驱动让开串口（否则会抢数据）
    python3 ~/ROS-car/pi/scripts/stm32_flash.py info          # 只读：连 bootloader 读版本/器件 ID
    python3 ~/ROS-car/pi/scripts/stm32_flash.py write fw.bin  # 烧写 + 回读校验 + 复位运行
    sudo systemctl start ps2-teleop

    boot   只进 bootloader 并停在那里（配合手动 stm32flash）
    run    让 bootloader 用 GO 跳到 App（芯片必须还在 bootloader 里）

注意
----
* **退出 bootloader 必须用 GO 命令（`-g 0x08000000`），不要靠 DTR/RTS 复位。**
  2026-11-25 实测：这块板 DTR/RTS 能进 bootloader，但**退不出来** —— rts/dtr
  四种电平组合都试过，芯片始终停在 ROM bootloader（遥测 0 字节）。GO 是唯一可靠的出口。
* 任何一次复位都会让 STM32 跑 INIT_HOME —— **机械臂会归位**，先把臂放到安全位置。
* 烧录期间别断电；ROM bootloader 烧不坏，失败可以重来。
* 镜像优先用 .bin（基址 0x08000000）；.hex 也能烧（stm32flash 自带 HEX 解析）。
* 本工具不负责启停 systemd 服务，串口占用检查不过就直接退出。
"""

from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
import time

import serial

# CH9102 的接法（与 PC 上 FlyMcu「DTR的低电平复位，RTS高电平进BootLoader」一致）
PIN_NRST = "dtr"
PIN_BOOT0 = "rts"

DEFAULT_BAUD = 115200


def find_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    if os.path.exists("/dev/l150pro"):
        return "/dev/l150pro"
    hits = sorted(glob.glob("/dev/serial/by-id/usb-1a86_USB_Single_Serial_*-if00"))
    if hits:
        return hits[0]
    sys.exit(
        "找不到 STM32 串口。用 --port 指定，或先确认板子在线：\n"
        "    lsusb | grep 1a86\n"
        "    ls -l /dev/serial/by-id/ /dev/l150pro"
    )


def port_holders(port: str) -> list[str]:
    """fuser 查占用（同用户下的进程不需要 root）。"""
    r = subprocess.run(["fuser", port], capture_output=True, text=True)
    # fuser 把文件名写到 stderr、PID 写到 stdout；这里只要 PID
    return [p for p in r.stdout.split() if p.isdigit()]


def warn_and_wait(action: str, assume_yes: bool) -> None:
    if action == "run":
        return
    print("⚠️  即将复位 STM32（固件会跑 INIT_HOME，机械臂会归位）。", flush=True)
    print("   臂附近别放东西，车别在动。", flush=True)
    if assume_yes:
        return
    for n in (3, 2, 1):
        print(f"   {n}...", end="\r", flush=True)
        time.sleep(1)
    print(" " * 12, end="\r")


def _pulse(port: str, baud: int, boot0: bool) -> None:
    s = serial.Serial(port, baud, timeout=0.2)
    try:
        setattr(s, PIN_NRST, True)  # 先放开复位
        time.sleep(0.05)
        setattr(s, PIN_BOOT0, boot0)  # 采样前的 BOOT0 电平
        time.sleep(0.05)
        setattr(s, PIN_NRST, False)  # 拉低复位
        time.sleep(0.05)
        setattr(s, PIN_NRST, True)  # 放开：此刻锁存 BOOT0
        time.sleep(0.2)
    finally:
        # 关串口后 CDC 可能把 DTR/RTS 拉回默认，但 BOOT0 已经锁存，不影响
        s.close()


def enter_bootloader(port: str, baud: int) -> None:
    _pulse(port, baud, boot0=True)


def go_app(port: str, baud: int) -> int:
    """让 bootloader 用 GO 命令跳到 0x08000000。

    这是**唯一可靠**的退出方式：实测 DTR/RTS 的复位脉冲能把芯片送进
    bootloader，却送不回 App（四种电平组合均无效）。GO 不依赖任何引脚电平。
    """
    return run_stm32flash(["-g", "0x08000000"], port, baud)


def run_stm32flash(args: list[str], port: str, baud: int) -> int:
    cmd = ["stm32flash", "-b", str(baud), "-m", "8e1", *args, port]
    print("$ " + " ".join(cmd), flush=True)
    return subprocess.run(cmd).returncode


def main() -> int:
    ap = argparse.ArgumentParser(description="Pi 侧串口烧录 STM32F407（ROM bootloader）")
    ap.add_argument("action", choices=["info", "write", "boot", "run"])
    ap.add_argument("image", nargs="?", help="write 时的固件文件（.bin / .hex）")
    ap.add_argument("--port", help="默认 /dev/l150pro，找不到就按 by-id 自动找")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("-y", "--yes", action="store_true", help="跳过 3 秒倒计时")
    a = ap.parse_args()

    if a.action == "write" and not a.image:
        ap.error("write 需要固件文件")
    if a.image and not os.path.isfile(a.image):
        ap.error(f"找不到固件：{a.image}")

    port = find_port(a.port)

    held = port_holders(port)
    if held:
        sys.exit(
            f"{port} 正被占用（PID: {', '.join(held)}）。先让驱动让开：\n"
            "    sudo systemctl stop ps2-teleop\n"
            "    fuser -v " + port
        )

    warn_and_wait(a.action, a.yes)

    if a.action == "run":
        rc = go_app(port, a.baud)
        if rc != 0:
            print("GO 失败：芯片可能不在 bootloader（已经在跑 App？）。",
                  file=sys.stderr, flush=True)
        return rc

    enter_bootloader(port, a.baud)
    print(f"已请求进入 bootloader（{port}）。", flush=True)

    if a.action == "boot":
        return 0

    if a.action == "info":
        rc = run_stm32flash([], port, a.baud)  # 不带 -w/-r 只连接读信息
    else:
        rc = run_stm32flash(["-w", a.image, "-v"], port, a.baud)

    if rc != 0:
        print(f"stm32flash 返回 {rc}；芯片停在 bootloader，可直接重试。",
              file=sys.stderr, flush=True)
        return rc

    go_app(port, a.baud)
    print("已让 bootloader 跳到 App。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
