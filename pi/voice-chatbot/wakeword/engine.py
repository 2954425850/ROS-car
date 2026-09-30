"""唤醒词检测 —— 硬件串口模块。

协议：5 字节定长帧 `AA 55 <CMD> 00 FB`，CMD 取值 0x01~0x06（对应不同唤醒词）。
模块直接接 USB 转串口，插上后一般是 /dev/ttyUSB0；也可以用 udev 规则做软链接
（config 默认的 /dev/myspeech 就是这个思路）。

本引擎**不负责并发**。它只管「读字节 → 解析帧 → 调回调」这一条链，
「回调里做什么、能不能阻塞、要不要防重入」全是调用方的事 ——
引擎自身永远不该被某个回调拖住。回调契约见 start() 的 Args。

与 09.AI_Big_Model/mic_serial.py 那份参考实现相比，这里修了四个坑：
  1. 原版逐字节 read() 后 sleep(0.1)，一个 5 字节帧平白多出 ~500ms 延迟。
     改成按 in_waiting 批量读，延迟降到 UART 帧时间。
  2. 原版字节不匹配时不复位状态机，一个噪声字节就能让解析器永久失步。
     这里任何不匹配都回到 SYNC1，且 0xAA 可以从任意状态重新起帧。
  3. 原版串口打不开就静默空转，用户完全不知道。这里 start() 直接抛错并给出可操作提示。
  4. 原版（以及本文件 2026-09-18 之前的版本）一次读失败就**永久停止监听**。
     那是当天生产事故的根因：USB 重枚举把唤醒模块从 ttyUSB0 挤成 ttyUSB1，
     进程攥着已删除节点的旧 fd，read 抛「device reports readiness to read but
     returned no data」。旧版在那个异常上直接 break，监听线程当场死亡 —— 而主进程
     还活着，systemd 的 `Restart=on-failure` 根本看不见，于是表现成最难查的那种故障：
     **`systemctl status` 是绿的，但喊它没反应**。
     现在改成「关掉坏句柄 → 按**路径**重开」，见 `_listen_loop` 与 `_reconnect`。
"""

import threading
import time
from typing import Callable

import serial
from loguru import logger

from utils.config import Config

_SYNC1, _SYNC2, _CMD, _RESERVED, _TAIL = range(5)

_HEADER_1 = 0xAA
_HEADER_2 = 0x55
_TAIL_BYTE = 0xFB
_VALID_CMDS = frozenset(range(0x01, 0x07))

# 重连退避的默认值。都能被 config 里的 wakeword.reconnect_* 覆盖
# （测试要把它们压到毫秒级，否则每个重连用例都得真等半秒）。
#
# 首次也要等：`open()` 成功但 `read()` 立刻失败（例如「端口被别的进程占用」那一支）
# 会退化成热循环 —— 每圈都开→读→炸。给个下限就不可能有热循环。
# 0.5s 不嫌长：USB 重枚举实测要 1~2 秒，首次 0.5s 约等于白等一小会儿。
_RECONNECT_INITIAL_SEC = 0.5
_RECONNECT_MAX_SEC = 5.0


class WakeWordEngine:
    """从串口读取硬件唤醒模块的唤醒帧，检测到就回调。

    Usage:
        engine = WakeWordEngine(config)
        engine.start(on_detected=lambda: print("Wake!"))
        # ... later ...
        engine.stop()
    """

    def __init__(self, config: Config):
        self._port = config.get("wakeword.port", "/dev/myspeech")
        self._baudrate = config.get("wakeword.baudrate", 115200)
        self._timeout = config.get("wakeword.serial_timeout_sec", 0.1)
        self._cooldown_sec = config.get("wakeword.cooldown_sec", 2.0)
        self._reconnect_initial_sec = config.get(
            "wakeword.reconnect_initial_sec", _RECONNECT_INITIAL_SEC
        )
        self._reconnect_max_sec = config.get(
            "wakeword.reconnect_max_sec", _RECONNECT_MAX_SEC
        )

        self._running = False
        self._thread: threading.Thread | None = None
        self._on_detected: Callable[[], None] | None = None
        self._last_detection_time = 0.0

        self._ser: serial.Serial | None = None
        self._step = _SYNC1
        self._pending_cmd = 0

        # 唤醒退避等待用的。stop() 会 set 它 —— 所以退避中的监听线程能**立刻**
        # 醒来退出。用 time.sleep() 的话，退避到 5 秒时 stop() 的 join(2.0)
        # 会白等超时，然后留下一条还活着的线程。
        self._stop_event = threading.Event()

    def start(self, on_detected: Callable[[], None]) -> None:
        """打开串口并启动监听线程。

        Args:
            on_detected: 检测到唤醒词时调用。**必须立刻返回**（几百毫秒以内）——
                回调是在监听线程上内联跑的，回调返回前监听循环不会继续读串口，
                所以任何长操作（录音、ASR、跑模型、TTS……）都会让唤醒词在它执行期间
                **失效**；长活儿请自己丢到后台线程去。
                **防重入由调用方自己负责**（例如「上一轮流水线还没跑完就不再起新一轮」），
                引擎不提供这层保护。
                「流水线跑着的时候唤醒词仍然存活」这件事**也不归本引擎保证** ——
                它由调用方侧的 `core/wake_handoff.WakeHandoff` 负责
                （见 tests/test_wake_handoff.py）。要修那个问题，去那边修，
                **不要给这个引擎加锁**：内联回调就是本引擎的设计，
                加锁会把「喊唤醒词 = 急停」这条路径一起堵死。

        Raises:
            serial.SerialException: 串口打不开（设备不存在、被占用、权限不足）。
                **这是有意为之**（见模块 docstring 第 3 条）：开机时模块还没枚举出来
                就抛错 → 服务退出 → systemd 10 秒后重启 → 自愈。不要把它改成
                「静默空转等设备出现」—— 那正是这份实现当初要修掉的毛病。
        """
        if self._running:
            logger.warning("WakeWord: already running")
            return

        self._on_detected = on_detected

        try:
            self._ser = serial.Serial(self._port, self._baudrate, timeout=self._timeout)
        except Exception as e:
            raise serial.SerialException(
                f"唤醒模块串口 {self._port} 打开失败：{e}。"
                f"请检查：① 模块是否插好（ls /dev/ttyUSB*）；"
                f"② 当前用户是否在 dialout 组（sudo usermod -aG dialout $USER，需重新登录）；"
                f"③ config.yaml 里 wakeword.port 是否与实际设备一致。"
            ) from e

        self._running = True
        self._stop_event.clear()          # 支持 stop() 之后再 start()
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        try:
            self._thread.start()
        except Exception:
            # 极端情况（起不了线程）：句柄归监听线程所有，它没跑起来就没人会关，
            # 所以这里得兜一下，否则串口一直占着。
            self._running = False
            self._close_serial()
            raise
        logger.info(f"WakeWord: listening on {self._port} @ {self._baudrate}")

    def stop(self) -> None:
        """停止监听并关闭串口。

        串口句柄的生命周期**归监听线程**：这里只负责「让它停」—— 置 `_running`、
        唤醒可能正在退避等待的它、join。真正的 `close()` 在线程退出时的 `finally`
        里（见 `_listen_loop`）。谁开谁关是同一个线程，就不会出现「stop() 刚关掉
        句柄、监听线程下一微秒又把它打开」那种竞态。

        Args 无。超时不是错误：只可能卡在 `serial.Serial()` 的 open 上（读和退避
        等待都是可中断的），那种情况记一条 warning 就算了 —— 线程是 daemon，
        进程退得掉。
        """
        self._running = False
        self._stop_event.set()            # 把可能正在退避等待的监听线程叫醒
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                logger.warning("WakeWord: 监听线程 2 秒内没退出，不再等它")
            self._thread = None
        logger.info("WakeWord: stopped")

    @property
    def is_ready(self) -> bool:
        """串口当前是否可用。掉线重连期间是 False（没有句柄），恢复后自动变回 True。

        注意这不是「监听线程还活着」——线程在重连期间一直是活的。要判断「喊一声
        有没有用」，看 `is_ready`；要看「引擎有没有在努力恢复」，看线程。
        """
        return self._ser is not None and self._ser.is_open

    # ---- 内部 ----

    def _listen_loop(self) -> None:
        """读字节 → 解析帧 → （内联）调回调。

        ★ 一次读失败**不是**终局。旧版在这里 `break` 出去，于是 USB 抖一下
          （换号 / 掉线 / 重枚举）就永久聋掉，而且因为进程还活着，systemd 的
          `Restart=on-failure` 永远触发不了 —— 见模块 docstring 第 4 条。
          现在的做法：关掉坏句柄（`_ser = None`），下一圈走 `_reconnect()`。
          循环条件只看 `_running`，所以「一直重连不上」也不会让线程退出。

        `finally` 里关句柄：无论怎么退出，串口都由这个线程自己收干净。
        """
        logged_disconnect = False      # 一次故障只报一条 warning，见下
        try:
            while self._running:
                if self._ser is None:
                    if not self._reconnect():
                        break          # 收到 stop() 了
                    continue

                try:
                    waiting = self._ser.in_waiting
                    data = self._ser.read(waiting if waiting else 1)
                except Exception as e:
                    if not self._running:
                        break          # 是 stop() 关的串口，不是故障
                    # 一条故障只报一条 warning：模块被拔掉时后面每 5 秒都会再来一次，
                    # 全是 warning 会把 journal 刷满。剩下的降级 debug。
                    if logged_disconnect:
                        logger.debug(f"WakeWord: 串口仍然不可用: {e}")
                    else:
                        logger.warning(f"WakeWord: 串口读失败，进入重连: {e}")
                        logged_disconnect = True
                    self._close_serial()
                    continue

                # ★ 复位点放在「读成功之后」，不是「重开成功之后」：
                #   「open() 成功但 read() 立刻抛」那条路径如果重开就复位，
                #   会退化成每 0.5 秒一条 warning —— 正是上面要避免的。
                logged_disconnect = False

                for byte in data:
                    cmd = self._feed(byte)
                    if cmd is not None:
                        self._handle_detection(cmd)
                        break  # 同一批里剩下的字节视为陈旧数据，交给 _drain 丢掉
        finally:
            self._close_serial()

    def _reconnect(self) -> bool:
        """重开串口，失败就退避重试。

        **按路径重开**（`self._port`，默认 `/dev/myspeech`）而不是按 fd —— 这正是
        本修复成立的关键：`/dev/myspeech` 是 udev 软链，`open()` 每次都会重新解析，
        所以模块从 ttyUSB0 换成 ttyUSB1 也能自动跟上。写死成 `/dev/ttyUSB0` 的话，
        重连会连到错的设备上（或者压根连不上），修了等于没修。

        退避：先等 `reconnect_initial_sec` 再开，之后每次开失败翻倍，封顶
        `reconnect_max_sec`。等待用 `_stop_event.wait()` 而非 `time.sleep()` ——
        理由见 `stop()` 的注释。

        Returns:
            True = 已连上（`self._ser` 可用）；False = 期间收到了 `stop()`，别再试了。
        """
        delay = self._reconnect_initial_sec
        attempt = 0
        while self._running:
            # 先等再开，不在失败路径上忙转。
            if self._stop_event.wait(delay):
                return False
            attempt += 1
            try:
                ser = serial.Serial(self._port, self._baudrate, timeout=self._timeout)
            except Exception as e:
                logger.debug(
                    f"WakeWord: 重开 {self._port} 失败（第 {attempt} 次）: {e}"
                )
                delay = min(delay * 2, self._reconnect_max_sec)
                continue

            if not self._running:
                # 开成功的同时收到了 stop()。句柄还没发布出去，谁也不会替我们关，
                # 所以就地关掉 —— 不然就是一个漏出去的 fd。
                self._shutdown_handle(ser)
                return False

            self._ser = ser
            self._step = _SYNC1   # 别继承掉线前的半帧状态（见 §4：这行是表意，
                                  # 不假装有测试覆盖 —— _feed 本来就能靠 0xAA 重新对齐）
            logger.info(
                f"WakeWord: 串口已重连 {self._port} @ {self._baudrate}"
                f"（第 {attempt} 次尝试）"
            )
            return True
        return False

    def _close_serial(self) -> None:
        """关掉当前句柄并置 None。幂等。

        关闭失败也**照样**置 None：那个句柄已经不可用了，留着只会让 `is_ready`
        谎报 True。只由监听线程调用（见 `stop()` 的注释）。
        """
        ser, self._ser = self._ser, None
        if ser is not None:
            self._shutdown_handle(ser)

    @staticmethod
    def _shutdown_handle(ser: serial.Serial) -> None:
        """关一个还没（或已经）发布的句柄 —— 尽力而为，绝不抛。"""
        try:
            if ser.is_open:
                ser.close()
        except Exception as e:
            logger.debug(f"WakeWord: 关闭串口失败: {e}")

    def _feed(self, byte: int) -> int | None:
        """喂一个字节给状态机。

        Returns:
            完整匹配一帧时返回帧里的 CMD，否则返回 None。
        """
        # 0xAA 可以从任何状态重新起帧 —— 这样即使前面被噪声带偏，也能立刻重新同步
        if byte == _HEADER_1:
            self._step = _SYNC2
            return None

        if self._step == _SYNC2:
            self._step = _CMD if byte == _HEADER_2 else _SYNC1
        elif self._step == _CMD:
            if byte in _VALID_CMDS:
                self._pending_cmd = byte
                self._step = _RESERVED
            else:
                self._step = _SYNC1
        elif self._step == _RESERVED:
            self._step = _TAIL if byte == 0x00 else _SYNC1
        elif self._step == _TAIL:
            self._step = _SYNC1
            if byte == _TAIL_BYTE:
                return self._pending_cmd
        else:
            self._step = _SYNC1

        return None

    def _handle_detection(self, cmd: int) -> None:
        now = time.monotonic()
        if now - self._last_detection_time < self._cooldown_sec:
            logger.debug(f"WakeWord: 冷却期内，忽略 cmd=0x{cmd:02X}")
            return
        self._last_detection_time = now
        logger.info(f"WakeWord: detected (cmd=0x{cmd:02X})")

        try:
            if self._on_detected is not None:
                self._on_detected()
        finally:
            # 清掉「回调执行期间」堆积的陈旧帧（例如急停那几百毫秒里用户又喊了一声）。
            # **这是有意丢帧，不是 bug**：不改成排队处理，否则一次唤醒结束后会被
            # 缓冲里的旧帧立刻再触发一次。用户想再唤醒，重新喊一声即可。
            self._step = _SYNC1
            self._drain()

    def _drain(self) -> None:
        try:
            if self._ser is not None and self._ser.is_open:
                self._ser.reset_input_buffer()
        except Exception as e:
            logger.debug(f"WakeWord: 清空串口缓冲失败: {e}")
