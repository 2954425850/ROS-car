"""测试替身。

FakeClock 让「按时间推进」的闭环逻辑在测试里瞬间跑完且完全确定 ——
不用真的 sleep，也不会因为机器慢而 flaky。

FakeSerial 让唤醒引擎的串口监听线程在测试里可控 —— 可以预置字节、也可以从别的线程
并发喂帧，而不用真插一块串口唤醒模块（见 test_wakeword_engine.py）。它还能模拟
**掉线**和**设备不在**，用来测引擎的重连行为（2026-09-18 加的）。
"""

from __future__ import annotations

import threading

import serial

from car.types import RobotState


class FakeClock:
    """手动推进的时钟。sleep() 只把时间往前拨，不真的等。"""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += seconds


class FakeBridge:
    """按「速度 × 时间」积分的假 ROS 桥。

    不做物理仿真 —— 只要能驱动闭环逻辑就够了。
    """

    def __init__(
        self,
        clock: FakeClock,
        *,
        speed_scale: float = 1.0,
        voltage: float | None = 12.0,
        fault: int | None = 0,
        online: bool = True,
        dead: bool = False,
    ) -> None:
        self.velocity_calls: list[tuple[float, float]] = []
        self.servo_calls: list[list[int]] = []
        self.closed = False
        self._clock = clock
        self._speed_scale = speed_scale
        self._voltage = voltage
        self._fault = fault
        self._online = online
        self._dead = dead              # True = 永远收不到上行（模拟驱动没跑）
        self._traveled = 0.0
        self._yaw = 0.0
        self._last_t = clock()

    def publish_velocity(self, vx: float, wz: float) -> None:
        self.velocity_calls.append((vx, wz))
        now = self._clock()
        dt = max(0.0, now - self._last_t)
        self._last_t = now
        self._traveled += abs(vx) * self._speed_scale * dt
        self._yaw += wz * dt

    def publish_servo(self, us: list[int]) -> None:
        self.servo_calls.append(list(us))

    def state(self) -> RobotState:
        if self._dead:
            return RobotState(voltage=None, fault=None, wheel_speeds=None,
                              yaw_rate=None, traveled=0.0, online=False)
        # yaw_rate 如实回报「上一帧指令」，让 controller 的积分跑得起来
        last_wz = self.velocity_calls[-1][1] if self.velocity_calls else 0.0
        return RobotState(
            voltage=self._voltage,
            fault=self._fault,
            wheel_speeds=(0.0, 0.0, 0.0, 0.0),
            yaw_rate=last_wz,
            traveled=self._traveled,
            online=self._online,
        )

    def close(self) -> None:
        self.closed = True


class FakeSerial:
    """假串口 —— 可预置、也可从别的线程并发喂字节，用来驱动唤醒引擎的监听循环。

    构造签名与 pyserial.Serial 对齐（`Serial(port, baudrate, timeout=...)`），
    所以测试里 monkeypatch 掉 `wakeword.engine.serial.Serial` 之后，
    `WakeWordEngine.start()` 建出来的就是这个假串口。

    行为刻意做得够真：
      * `read()` 没数据时返回 b""，并且**最多阻塞 self.timeout 秒**（喂帧会立刻把它
        唤醒）—— 少了这个阻塞，`_listen_loop` 会忙转成 100% CPU；真串口也不该这样。
      * `read(n)` 凑不满 n 个字节时有多少给多少（真串口 read 超时后就是这个行为），
        于是监听循环会遇到「一次只读到半个帧」，正是我们想让假串口暴露的那种情况。
      * `in_waiting` 只报「还没被 read() 取走」的字节数。
      * `reset_input_buffer()` **真的**丢掉缓冲里的字节，并把丢掉的**内容**记进
        `discard_calls`（唤醒引擎的 `_drain()` 就是靠它丢帧的 —— 每次调用都记一条，
        即使那条是 b""，这样测试既能断言「丢了」也能断言「丢的是什么」）。

    与真串口的两点已知差异（都是为了让测试快而稳）：
      * 凑不满 n 个字节时不会去等满 timeout，立刻返回手头这些；
      * `close()` 之后再 `read()` 不抛异常，只返回 b""。

    ---- 掉线模拟（2026-09-18 加，为唤醒引擎的重连测试）----

    真机事故的形态是「USB 重枚举 → 模块换设备号 → 旧 fd 失效」。这里用两个类属性
    和一个实例方法把它复现出来：

      * `disconnect()`  —— 让**这一个**句柄开始抛异常（旧 fd 死了）；
      * `open_error`    —— 让**之后所有**构造都抛异常（设备不在，连开都开不起来）；
      * `instances`     —— 每次构造都记一笔，测试靠它数「引擎到底重开了几次」。

    ★ `instances` 和 `open_error` 是**类属性**，跨用例会残留。用它们的测试文件必须
      有个 autouse fixture 清空（见 test_wakeword_engine.py 的 `_reset_fake_serial`），
      否则一个用例把 `open_error` 设上，后面所有用例的串口都开不起来了。
    """

    #: 每次构造都 append 一笔。`instances[-1]` 就是最近一次开出来的句柄。
    instances: list["FakeSerial"] = []

    #: 非 None 时 `__init__` 直接抛它 —— 模拟「设备现在不在」（被拔了 / 还没枚举）。
    open_error: Exception | None = None

    def __init__(self, port: str, baudrate: int, timeout: float = 0.1,
                 **kwargs: object) -> None:
        if FakeSerial.open_error is not None:
            raise FakeSerial.open_error
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        self.discard_calls: list[bytes] = []   # 每次 reset_input_buffer() 丢掉的字节
        self._buf = bytearray()
        self._cv = threading.Condition()
        self._read_error: Exception | None = None   # 非 None 时读操作一律抛它
        FakeSerial.instances.append(self)

    # ---- 设备侧（测试用）----

    def feed_frame(self, cmd: int) -> bytes:
        """按协议塞一帧 `AA 55 <cmd> 00 FB`。

        Returns:
            塞进去的那 5 个字节 —— 方便断言「它确实被丢掉了」。
        """
        frame = bytes([0xAA, 0x55, cmd, 0x00, 0xFB])
        self.feed_bytes(frame)
        return frame

    def feed_bytes(self, data: bytes) -> None:
        """塞任意字节，并唤醒正在 read() 里等的那一头。"""
        with self._cv:
            self._buf.extend(data)
            self._cv.notify_all()

    def disconnect(self, exc: Exception | None = None) -> None:
        """模拟「这个句柄背后的设备掉线了」：之后读操作一律抛异常。

        默认抛的是真机现场那条原文 —— pyserial 在「select 说可读、os.read 却返回
        空」时抛的就是它（设备被拔掉、重枚举换了设备号、或者端口被别的进程抢占）。
        **别改成别的文案**：测这个异常的用例，意义就在于它和事故现场一字不差。

        注意这**不会**影响新构造的句柄（那是 `open_error` 管的事）—— 真实情况正是
        如此：旧 fd 死了，重开 /dev/myspeech 能拿到一个健康的新句柄。
        """
        self._read_error = exc or serial.SerialException(
            "device reports readiness to read but returned no data "
            "(device disconnected or multiple access on port?)"
        )
        with self._cv:
            self._cv.notify_all()      # 把可能正卡在 read() 等待里的监听线程叫醒

    # ---- pyserial 侧 ----

    @property
    def in_waiting(self) -> int:
        # 真 pyserial 这里走 TIOCINQ ioctl，fd 死了同样会抛 —— 所以两个读入口都要抛，
        # 才能保证「引擎先碰哪一个」都不影响测试结论。
        if self._read_error is not None:
            raise self._read_error
        with self._cv:
            return len(self._buf)

    def read(self, size: int = 1) -> bytes:
        if self._read_error is not None:
            raise self._read_error
        if not self.is_open or size <= 0:
            return b""
        with self._cv:
            if not self._buf:
                self._cv.wait(self.timeout)     # 没数据就等，别忙转
            if not self._buf:
                return b""
            n = min(size, len(self._buf))
            data = bytes(self._buf[:n])
            del self._buf[:n]
            return data

    def reset_input_buffer(self) -> None:
        with self._cv:
            self.discard_calls.append(bytes(self._buf))
            self._buf.clear()

    def close(self) -> None:
        self.is_open = False
