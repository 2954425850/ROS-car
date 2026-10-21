# -*- coding: utf-8 -*-
r"""A010 的串口层（pyserial）：开/初始化/命令/后台泵帧。

这一层**只依赖 pyserial**，不依赖 ROS —— 所以 `tof_probe` 能拿它单独排障。

## 2026-10-14 在 Pi 上实测（USB 路径，与 K230 侧的真 UART 不同！）

| | K230 侧（座 2 真 UART） | **Pi 侧（模块自带 USB）** |
|---|---|---|
| 设备 | 庐山派 UART2 | `/dev/ttyUSB0`（FT2232 通道 A） |
| 提速 | **必须** `AT+BAUD=5`(921600)，否则 1.1 fps | **不需要**：115200 实测 **9.0 fps** |
| DISP 位 | `5` = LCD(1)\|**UART(4)** | **`3` = LCD(1)\|USB(2)** |
| 开机默认 | BAUD=2 (115200)、DISP=1 (仅 LCD) | 同左，**掉电不保存，每次上电都回默认** |

⇒ **DISP 位选错 = 一个字节都收不到**，而且屏会亮（"屏亮"只说明有电，不说明信号通）。
  这是 K230 侧踩过的同一个坑的另一半。

## 三个必须照抄的细节（都是踩出来的）

1. **每条命令前置 `\\r\\r\\r`**。上一次命令如果波特率/模式不对，模块的接收缓冲里会
   留着没有 `\\r` 收尾的残渣；不先用裸 CR 把它冲掉，模块在你说话的正确 baud 上
   **什么都不会回**。这不是装饰。
2. **`cmd()` 不能长排空**。设完 `AT+DISP=3` 的下一毫秒模块就开始推 10022 字节/帧，
   长排空会把整帧吃掉、还会把下一帧切碎。所以：读到 `OK` 就立刻收手，
   **`OK` 之后的字节全部当作帧数据留给上层**。
3. **打开串口会翻转 DTR ⇒ 模块复位**。开完必须等（`INIT_WAIT_MS=1500`）再发命令。
"""
from __future__ import annotations

import os
import threading
import time

try:
    import serial
    from serial.serialutil import SerialException
except ImportError:                      # 允许没有 pyserial 的环境做静态检查
    serial = None

    class SerialException(Exception):
        pass

from . import a010

# 波特率**索引**表（★ `AT+BAUD=` 收的是索引，不是波特率数值！发数值 = 静默无回显）
BAUD_TABLE = (9600, 57600, 115200, 230400, 460800, 921600, 1000000, 2000000, 3000000)
BAUD_115200_INDEX = 2
BAUD_921600_INDEX = 5

CR_FLUSH = b"\r\r\r"
INIT_WAIT_MS = 1500
READ_CHUNK = 8192


class A010(object):
    """一个模块。**不是线程安全的**：命令期和泵帧期不能重叠。"""

    def __init__(self, device="/dev/a010", baud=115200, timeout=0.15):
        self.device = device
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self.unit_mm = a010.DEFAULT_UNIT_MM
        self.opened_at = None
        self.actual_port = None
        self.last_response = b""          # 最近一条命令的应答正文（排障用）

    # -- 生命周期 -----------------------------------------------------------

    def open(self):
        if serial is None:
            raise RuntimeError("没有 pyserial：pip install pyserial / apt install python3-serial")
        # ★ **独占打开**（TIOCEXCL）。默认 Linux 允许多个进程同时开同一个 tty，
        #   于是"第二个进程也能 open 成功"，直到读的时候才炸成
        #   `device reports readiness to read but returned no data
        #    (device disconnected or multiple access on port?)`
        #   —— 这个报错**极具误导性**：它看起来像"模块挂了"，实际是"有两个进程在抢"。
        #   独占之后第二个 opener 立刻拿到干净的 "Device or resource busy"。
        self.ser = serial.Serial(self.device, self.baud, timeout=self.timeout,
                                 write_timeout=1.0, exclusive=True)
        self.opened_at = time.monotonic()
        # 记下**真实**占用的 ttyUSB（by-id → ttyUSBx）。通道号会平移，
        # 出问题时"到底开到了哪个口"是第一件要看的，别让它只存在于软链里。
        try:
            self.actual_port = os.path.realpath(self.device)
        except Exception:                          # noqa: BLE001
            self.actual_port = self.device
        time.sleep(INIT_WAIT_MS / 1000.0)      # ★ DTR 复位，等模块起来
        self.drain(quiet_s=0.3, max_s=1.0)
        return self

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:                  # noqa: BLE001
                pass
            self.ser = None

    @property
    def is_open(self):
        try:
            return self.ser is not None and self.ser.is_open
        except Exception:                      # noqa: BLE001
            return False

    # -- 底层读写 -----------------------------------------------------------

    def _read(self, seconds):
        """固定时间窗读取。**故意不做"有数据就续命"** —— 模块一旦推流就是连续的，
        续命会让这个函数永远不返回。"""
        if not self.is_open:
            return b""
        t0 = time.monotonic()
        out = bytearray()
        while time.monotonic() - t0 < seconds:
            try:
                d = self.ser.read(READ_CHUNK)
            except SerialException:
                raise
            if d:
                out += d
        return bytes(out)

    def drain(self, quiet_s=0.3, max_s=1.5):
        """排掉异步日志 / 残帧，直到连续 `quiet_s` 没有新数据。

        模块会在命令之间插 `Dragonfly ISP stop/start tranfer` 这类日志；
        不排掉的话它会被当成**当前**命令的应答，让整条初始化错位一步
        （K230 侧实测：`AT+UNIT=4` 读到了 `AT+COEFF?` 的答案）。
        """
        t0 = time.monotonic()
        last = time.monotonic()
        while time.monotonic() - last < quiet_s and time.monotonic() - t0 < max_s:
            try:
                d = self.ser.read(READ_CHUNK)
            except SerialException:
                raise
            if d:
                last = time.monotonic()

    # -- 命令 ---------------------------------------------------------------

    def cmd(self, text, timeout=0.8, keep_leftover=True):
        """发一条 AT 命令。返回 `(ok, leftover)`。

        - `ok`：在 `timeout` 内看到了 `OK`（看到 `ERR` 直接 False）。
        - `leftover`：`OK` **之后**收到的字节。设 `AT+DISP=` 之后这些就是**帧数据**，
          绝不能丢 —— 丢了就是每设一次参数丢一帧。
        """
        if not self.is_open:
            return False, b""
        self.drain(quiet_s=0.05, max_s=0.2)
        try:
            self.ser.write(CR_FLUSH + text.encode("ascii") + b"\r")
        except SerialException:
            raise
        buf = b""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            d = self.ser.read(READ_CHUNK)
            if not d:
                if buf:
                    break
                continue
            buf += d
            i = buf.find(b"OK")
            if i >= 0:
                self.last_response = buf[:i]     # 排障用：`OK` 之前那半才是应答正文
                tail = buf[i + 2:]
                tail = tail[2:] if tail.startswith(b"\r\n") else tail
                return True, (tail if keep_leftover else b"")
            if b"ERR" in buf:
                self.last_response = buf
                return False, b""
        self.last_response = buf
        return False, buf

    def ping(self, tries=2, timeout=0.6):
        """`AT` 握手。**发两次**：第一次可能只是在冲掉缓冲里的残渣。"""
        for _ in range(tries):
            ok, _left = self.cmd("AT", timeout=timeout)
            if ok:
                return True
            time.sleep(0.2)
        return False

    def init(self, unit_mm=a010.DEFAULT_UNIT_MM, binn=1, disp=3, fps=None):
        """★ 初始化。任何一条失败都返回 False（**不要让调用方带着半个配置跑**）。

        顺序不能动：
          `AT` 探活 → `AT+DISP=0` 停流 → 设 UNIT/BINN/FPS → `AT+DISP=<disp>` 开流。

        先停流再设参数：不停的话命令回显会被深度帧淹没，判定不了 OK。
        """
        self.drain(quiet_s=0.2, max_s=0.8)
        if not self.ping():
            return False
        if not self.cmd("AT+DISP=0")[0]:
            return False
        if not self.cmd("AT+UNIT=%d" % int(unit_mm))[0]:
            return False
        self.unit_mm = float(unit_mm)
        if binn is not None:
            self.cmd("AT+BINN=%d" % int(binn))
        if fps is not None:
            self.cmd("AT+FPS=%d" % int(fps))
        ok, leftover = self.cmd("AT+DISP=%d" % int(disp), timeout=1.2)
        if not ok:
            return False
        if leftover:
            self._leftover = leftover
        return True

    def stop_stream(self):
        try:
            return self.cmd("AT+DISP=0", timeout=0.8)[0]
        except SerialException:
            return False

    # -- 读帧 ---------------------------------------------------------------

    def read_frame(self, timeout=1.0):
        """阻塞读到**一个**合法帧（或超时 → None）。泵帧线程用它。"""
        t0 = time.monotonic()
        buf = getattr(self, "_leftover", b"")
        self._leftover = b""
        while time.monotonic() - t0 < timeout:
            try:
                d = self.ser.read(READ_CHUNK)
            except SerialException:
                raise
            if d:
                buf += d
            fr, buf = a010.parse_frame(buf)
            if fr is not None:
                self._leftover = buf          # 半包留给下一次，别丢
                return fr
        self._leftover = buf if len(buf) < a010.HEAD_SIZE else b""
        return None

    # -- 便捷 ---------------------------------------------------------------

    def query(self, text, timeout=0.6):
        """发查询、把应答当文本摘出来（排障用，不参与数据通路）。"""
        ok, left = self.cmd(text, timeout=timeout)
        return ok, left


# --------------------------------------------------------------------------
# 后台泵帧线程
# --------------------------------------------------------------------------

class TofReader(threading.Thread):
    """独占串口、只留**最新帧**的后台线程。掉线自己重连。

    ★ 为什么必须有这个线程：模块 2026-10-14 实测枚举抖过 3 次（换 USB 口），
      而且 ROS 节点的定时器里**绝不能**做阻塞读（厂商那份 C++ 就是这么写的：
      `O_NONBLOCK` 被清掉 + `VTIME=10` ⇒ 单线程执行器最长卡 1 秒）。
    """

    def __init__(self, dev, unit_mm=a010.DEFAULT_UNIT_MM, binn=1, disp=3,
                 reopen_delay=1.0, max_backoff=20.0):
        super().__init__(daemon=True, name="tof-reader")
        self.dev = dev
        self.unit_mm = unit_mm
        self.binn = binn
        self.disp = disp
        self.reopen_delay = reopen_delay
        self.max_backoff = max_backoff

        self.latest = None
        self.t_latest = None
        self.frames = 0
        self.reconnects = 0
        self.last_error = None
        self.connected = False
        self.connecting = False      # 正在 open+init（启动时那 2~3s，**不是**掉线）

        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._fps_t = time.monotonic()
        self._fps_n = 0
        self.fps = 0.0

    # -- 给外面看 -----------------------------------------------------------

    def snapshot(self):
        """`(frame, age_s)`。没有帧时返回 `(None, None)`。"""
        with self._lock:
            fr, t = self.latest, self.t_latest
        if fr is None:
            return None, None
        return fr, time.monotonic() - t

    def status(self):
        fr, age = self.snapshot()
        return {
            "connected": self.connected,
            "connecting": self.connecting,
            "frames": self.frames,
            "fps": round(self.fps, 2),
            "reconnects": self.reconnects,
            "age_s": None if age is None else round(age, 3),
            "device": self.dev.device,
            "port": getattr(self.dev, "actual_port", None),
            "unit_mm": self.unit_mm,
            "last_error": self.last_error,
            "frame_id": None if fr is None else fr.frame_id,
        }

    def stop(self):
        self._stop.set()

    # -- 线程体 -------------------------------------------------------------

    def run(self):
        """★ 重连必须**退避**，不能死循环猛敲。

        2026-10-14 踩到：模块卡住不出声时，1 秒一敲的循环会**每 ~3 秒开一次串口**，
        而开串口会翻转 DTR。等于**把模块按住反复复位**，它永远等不到缓过来的机会
        （实测连敲了 39 次）。退避让每次失败的间隔 1→2→4→…→20s，
        既给模块恢复的时间，也不会在日志里刷成噪声山。
        """
        backoff = self.reopen_delay
        while not self._stop.is_set():
            try:
                # 设备压根不在总线上时**只等，不开**：这种 open 纯粹是浪费时间，
                # 还会在日志里刷一堆 "No such file or directory"。
                if not os.path.exists(self.dev.device):
                    self.connected = False
                    self.last_error = "设备不在总线上（%s）" % self.dev.device
                    self.reconnects += 1
                    self._stop.wait(backoff)
                    backoff = min(backoff * 2, self.max_backoff)
                    continue

                if not self.dev.is_open:
                    self._connect()
                    backoff = self.reopen_delay          # 连上了 ⇒ 退避归零

                fr = self.dev.read_frame(timeout=0.5)
                if fr is not None:
                    with self._lock:
                        self.latest = fr
                        self.t_latest = time.monotonic()
                        self.frames += 1
                        self._fps_n += 1
                    now = time.monotonic()
                    if now - self._fps_t >= 1.0:
                        self.fps = self._fps_n / (now - self._fps_t)
                        self._fps_n, self._fps_t = 0, now
            except SerialException as e:
                self._disconnect("%r" % (e,))
                self._stop.wait(backoff)
                backoff = min(backoff * 2, self.max_backoff)
            except Exception as e:                  # noqa: BLE001
                self._disconnect("%r" % (e,))
                self._stop.wait(backoff)
                backoff = min(backoff * 2, self.max_backoff)

    def _connect(self):
        # ★ 每次重连都**重新解析设备路径**，不要用打开时的那个快照。
        #   实测 2026-10-14：重枚举后 FT2232 的通道号会平移
        #   （`if00` 从 ttyUSB0 挪到 ttyUSB1）。by-id 软链是 udev 在维护的，
        #   重新解析就能跟住；拿旧快照会开到**通道 B** —— 症状是"口能开、
        #   但一个字节都收不到"，极难和"模块坏了"区分。
        try:
            from .config import resolve_device
            path, _why = resolve_device({"device": self.dev.device})
            if path != self.dev.device:
                self.dev.device = path
        except Exception:                          # noqa: BLE001
            pass
        self.dev.open()
        if not self.dev.init(unit_mm=self.unit_mm, binn=self.binn, disp=self.disp):
            raise SerialException("初始化失败（AT 不回应 / DISP 位不对）@ %s"
                                  % self.dev.device)
        self.connected = True

    def _disconnect(self, msg):
        self.connected = False
        self.last_error = msg
        self.reconnects += 1
        try:
            self.dev.close()
        except Exception:                          # noqa: BLE001
            pass
