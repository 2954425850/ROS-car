# /sdcard/k230vision/tof.py —— MaixSense A010 ToF（UART 版）：读出距离
#
# 依赖 a010_frame.py（纯解析，已被 t_tof.py 测红过）。本文件只负责**硬件那层**：
# FPIOA 配引脚 -> UART -> 发 AT 命令 -> 收字节流喂给 a010_frame。
#
# ## 接线（A010 4pin 1.0mm 座 -> K230 庐山派 **GH1.25 座 2**）
#
# 2026-10-10 实测走的是**座 2**（丝印 "2"，两个侧按按钮中间那个），线序 V/R/T/G：
#
#     座 2 V -> A010 5V      （A010 供电 5V / 0.5A）
#     座 2 R -> A010 **TX**  （K230 收，= GPIO12 / UART2_RXD）
#     座 2 T -> A010 **RX**  （K230 发，= GPIO11 / UART2_TXD）
#     座 2 G -> A010 GND
#
# ⚠️ 三个实测坑：
#   1. **TX/RX 必须交叉**（A010 TX 进座子的 R）。接成"同名相接"的症状是
#      **屏亮着（供电没问题）但 K230 一个字节都收不到** —— 2026-10-10 就卡在这。
#   2. 庐山派有 **三个** GH1.25-4P 座（丝印 2/3/0）：座 2 = UART2(GPIO11/12)、
#      座 3 = UART3(GPIO50/51 标准版 / GPIO32/33 Lite 版)、座 0 = UART0(系统占用)。
#      接线前先看座子旁边的丝印数字。
#   3. 排针上另有 UART2（GPIO5/6 = 排针 11/13），**和座 2 是同一个 UART2，二选一**，
#      别同时接。改接线方式时记得同步改下面 TX_PIN/RX_PIN。
#
#   （电平：庐山派资料明确"UART 只能 3.3V"。A010 实测接上能正常通信，
#     接之前最好还是量一下 A010 TX 的空闲电平。）
#
# ## 为什么先这么写（而不是直接进 app.py）
#
# 现在只求"最小闭环：REPL 里能 print 出距离"。等标度标定完、稳定跑过几分钟，
# 再谈接进 app.py 的 10Hz 节拍。参数暂时放在本文件顶部，集成时再搬去 config.py。
#
# ## 吞吐现实（先知道，别被 1 fps 吓到）
#
# 100x100 一帧 = 10022 字节；115200 8N1 下 ≈ **0.87 s/帧（~1.1 fps）**。
# 想要更快：AT+BAUD 提到 921600/3000000（K230 侧同步改 BAUD），或 AT+BINN 降分辨率。

import sys

sys.path.insert(0, "/sdcard/k230vision")   # exec 本文件时也能 import 同目录模块

from machine import UART, FPIOA
import time

import a010_frame

# ---- 参数（唯一要改的地方）----
# 接的是 **GH1.25 座 2**（丝印 2）：座 2 的 T = GPIO11(TXD)、R = GPIO12(RXD)。
# （排针那组是 GPIO5/6；两套是同一个 UART2，二选一 —— 换了线就改这里。）
UART_ID = UART.UART2
TX_PIN = 11             # GPIO11 = UART2_TXD = 座 2 的 T
RX_PIN = 12             # GPIO12 = UART2_RXD = 座 2 的 R
BAUD = 115200

# 一个深度计数代表多少毫米。由 AT+UNIT 决定，**当前是照着官方例程猜的，必须实测标定**
# （判据：把平板放在已知距离，读出来的值要对得上）。UNIT=4 时量程上限只有 ~1.0m
# （255*4mm）；要覆盖到 2.5m 得把 UNIT 调到 10。
TOF_UNIT = 4

READ_CHUNK = 512
INIT_WAIT_MS = 1500     # 上电后等模块稳定再搭话


class A010(object):
    """一个 A010 会话。用完记得 close()。"""

    def __init__(self, uart_id=UART_ID, tx=TX_PIN, rx=RX_PIN, baud=BAUD):
        fpioa = FPIOA()
        # 常量在不同固件里有的挂在类上、有的挂在实例上，两种都试一下
        tx_func = getattr(FPIOA, "UART2_TXD", None) or getattr(fpioa, "UART2_TXD")
        rx_func = getattr(FPIOA, "UART2_RXD", None) or getattr(fpioa, "UART2_RXD")
        fpioa.set_function(tx, tx_func)
        fpioa.set_function(rx, rx_func)

        self.uart = UART(uart_id,
                         baudrate=baud,
                         bits=UART.EIGHTBITS,
                         parity=UART.PARITY_NONE,
                         stop=UART.STOPBITS_ONE)
        self._buf = b""
        self.frames = 0

    # ---- 底层收发 ----
    def close(self):
        try:
            self.uart.deinit()
        except Exception as e:
            print("TOF uart.deinit EXC", e)

    def _drain(self, quiet_ms=40, max_ms=600):
        """把 UART 上现有的字节读走，直到静默 quiet_ms 或超时。返回读到的字节。"""
        got = b""
        t0 = time.ticks_ms()
        last = t0
        while time.ticks_diff(time.ticks_ms(), t0) < max_ms:
            d = self.uart.read(READ_CHUNK)
            if d:
                got += d
                last = time.ticks_ms()
            elif time.ticks_diff(time.ticks_ms(), last) >= quiet_ms:
                break
            else:
                time.sleep_ms(5)
        return got

    def cmd(self, text, timeout_ms=700):
        """发一条 AT 命令（自动补 \\r），读到 OK/ERR 或超时就停，返回回显。

        **刻意不长 drain**：像 AT+DISP=3 这种命令，设备会在回完 OK 的下一拍
        立刻开始吐深度帧。若再多读一秒（按静默时间收），那一整帧 10KB 就被
        当垃圾吞掉了 —— 这是仿真里先抓到的坑。
        """
        # 发之前**先清掉上一条命令留下的异步日志** —— A010 会吐
        # "Dragonfly ISP stop/start tranfer" 这类**异步**消息；不清的话它会被
        # 当成这一条的响应，整条 init 时序错位一拍（2026-10-10 实测：
        # AT+UNIT=4 读到的是 AT+COEFF? 的回答，后面全乱）。
        self._drain(quiet_ms=30, max_ms=150)
        self.uart.write(text + "\r")
        got = b""
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            d = self.uart.read(READ_CHUNK)
            if d:
                got += d
                if b"OK" in got or b"ERR" in got:
                    break
            else:
                time.sleep_ms(2)
        print("TOF cmd %-12s -> %s" % (text, _show(got)))
        return got

    # ---- 初始化时序（照抄官方 ros2 main.cc，顺序有意义）----
    def init(self, unit=TOF_UNIT):
        time.sleep_ms(INIT_WAIT_MS)
        # 板子可能正卡在"上一次会话没关、还在猛吐数据"的状态。这时光靠 drain
        # 清不完（数据是**持续**的，永远等不到静默）。所以先叫停出图，再清。
        self.uart.write("AT+ISP=0\r")
        time.sleep_ms(120)
        self._drain(quiet_ms=60, max_ms=600)   # 把在途的帧数据一并清掉

        self.cmd("AT+ISP=0")
        self.cmd("AT+DISP=1")
        self.cmd("AT+ISP=1")

        r = self.cmd("AT")
        if b"OK" not in r:
            print("TOF WARN: AT 没回 OK —— 可能线接错/波特率不对/模块没上电")
            return False

        self.cmd("AT+COEFF?")            # 内参 json，现在不用，读了清掉
        self.cmd("AT+UNIT=%d" % unit)
        # AT+DISP 是**位掩码**：OFF=0 / LCD=1 / USB=2 / UART=4（从 COMTool 逆向的包里拿到）。
        # ⚠️ 官方 ROS 例程写的 3 = LCD|USB —— 那是走 **USB CDC** 的。
        # 我们走**真 UART**，不含 UART 位(4) 就一个字节都收不到（2026-10-10 卡在这）。
        # 5 = LCD|UART：保留屏显，同时把深度帧吐到串口。
        r = self.cmd("AT+DISP=%d" % (1 | 4))
        # OK 之后紧跟的数据流**别丢**：塞回缓冲，交给 read_frame 去解
        i = r.find(b"OK")
        if i >= 0:
            tail = r[i + 2:].lstrip(b"\r\n")
            if tail:
                self._buf += tail
        return True

    # ---- 取一帧 ----
    def read_frame(self, timeout_ms=2000):
        """等到一帧完整、校验通过的深度帧；超时返回 None。"""
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            d = self.uart.read(READ_CHUNK)
            if d:
                self._buf += d
                fr, self._buf = a010_frame.feed(self._buf)
                if fr is not None:
                    self.frames += 1
                    return fr
            else:
                time.sleep_ms(2)
        return None


def _show(b, limit=48):
    """把回显打成可读的一行（不可见字符转义）。"""
    if not b:
        return "<none>"
    b = bytes(b)
    s = ""
    for c in b[:limit]:
        if 32 <= c < 127:
            s += chr(c)
        else:
            s += "\\x%02x" % c
    if len(b) > limit:
        s += "...(%d)" % len(b)
    return s


def demo(unit=TOF_UNIT, seconds=0):
    """连上就循环打印中心距离。seconds=0 表示一直跑（Ctrl-C 退出）。"""
    dev = A010()
    if not dev.init(unit=unit):
        print("TOF INIT FAILED —— 先查接线/电平/供电")
        dev.close()
        return
    print("TOF READY unit=%d mm/count  量程上限≈%dmm" % (unit, 255 * unit))

    start = time.ticks_ms()
    t0 = start
    n0 = 0
    while True:
        fr = dev.read_frame(timeout_ms=3000)
        if fr is None:
            print("TOF TIMEOUT 等帧超时（%d 帧后）" % dev.frames)
            continue
        mm = a010_frame.center_mm(fr, unit)
        now = time.ticks_ms()
        dt = time.ticks_diff(now, t0)
        fps = 0.0
        if dt >= 1000:
            fps = (dev.frames - n0) * 1000.0 / dt
            t0, n0 = now, dev.frames
        print("TOF id=%-4d %dx%d %s center=%smm  %.1ffps"
              % (fr["frame_id"], fr["rows"], fr["cols"],
                 "depth+ir" if fr["output_mode"] else "depth",
                 mm if mm is not None else "None", fps))
        if seconds and time.ticks_diff(now, start) >= seconds * 1000:
            break
    dev.close()
