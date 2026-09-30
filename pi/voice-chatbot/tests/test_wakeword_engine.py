"""唤醒引擎（`wakeword/engine.py`）的行为测试。

这个文件只测**属于引擎**的几件事：

  * `test_drain_discards_frames_accumulated_during_callback` —— `_drain()` 在回调之后
    丢弃「回调执行期间堆积的帧」是**有意行为**，不是 bug（设计文档 §7 坑 1）；
  * `test_cooldown_swallows_rapid_second_wake` —— 冷却期（默认 2.0s）内的第二次唤醒被忽略；
  * `test_read_error_reopens_serial_and_listening_survives`（2026-09-18 加）——
    串口读异常必须**重连**，而不是让监听线程死掉；
  * `test_reconnect_keeps_retrying_while_device_is_absent`（2026-09-18 加）——
    设备不在时一直重试，回来就自动接上；
  * `test_stop_interrupts_backoff_wait`（2026-09-18 加）—— stop() 能立刻打断退避等待。

**有意不测的东西**：回调是**内联**的，回调返回之前引擎不会再去读串口 —— 这是
`start()` 里写明的契约（回调必须立刻返回），不是缺陷。「流水线跑着的时候唤醒词仍然
存活」这件事**不归引擎保证**，由调用方侧的 `core/wake_handoff.WakeHandoff` 负责，
测试在 `tests/test_wake_handoff.py`。**想修那个问题就去那边修，不要给引擎加锁。**

2026-09-18 这里曾经有一个 `test_slow_callback_does_not_block_listening`，用户拍板删掉了。
记一笔免得有人再写一个：它实际测的只是「回调跑完 → `_drain()` 跑完 → 循环还在读」
（与上面第 1 个用例重复），并不能防「后人给引擎加锁」—— 因为「回调返回前不读串口」
**本来就是当前内联回调的行为**。留一个声称提供保护、实际不提供保护的测试，比没有更糟。

背景（设计文档 §3）：2026-09-18 的「唤醒握手重构」要解决的阻塞在**调用方** ——
`ConversationManager._on_wake_word` 把整条流水线（录音 / ASR / LLM / TTS，实测 4~10 秒）
跑在了 `on_detected` 回调里。引擎侧只改了契约文档，可执行代码零改动。

上面第 3~5 个用例来自**同一天的第二件事**：16:03 的生产事故（喊不应了，但
`systemctl status` 是绿的）。根因是 `_listen_loop` 在读异常上 `break`，监听线程死亡而
进程健在，systemd 的 `Restart=on-failure` 看不见。设计与因果链见
`docs/plans/2026-09-18-wakeword-serial-reconnect-design.md`。
"""

import threading
import time

import pytest
import serial

from tests.fakes import FakeSerial
from wakeword.engine import WakeWordEngine


class _FakeConfig:
    """引擎只用到 config.get()，给个字典就够了（照 test_car_controller.py 的写法）。"""

    def __init__(self, **values):
        self._values = values

    def get(self, key, default=None):
        return self._values.get(key, default)


@pytest.fixture(autouse=True)
def _reset_fake_serial():
    """清掉 FakeSerial 的**类属性**。

    `instances` / `open_error` 是类级的，不清的话会跨用例残留 —— 尤其是
    `open_error`：一个用例把它设上，后面所有用例的串口都开不起来了，而报错
    会出现在**别的**用例里，极难定位。
    """
    FakeSerial.instances.clear()
    FakeSerial.open_error = None
    yield
    FakeSerial.instances.clear()
    FakeSerial.open_error = None


def _start_engine(monkeypatch, on_detected, *, cooldown=0.0, timeout=0.05,
                  reconnect_initial=0.01, reconnect_max=0.05):
    """建引擎并 start()，返回 (engine, 引擎内部建出来的那个假串口)。

    `serial.Serial` 是**全局**替换的（monkeypatch 会在用例结束后还原：
    它替换的是 serial 模块上的 Serial 属性，而 wakeword.engine 引用的就是这个模块）。

    重连退避默认压到毫秒级 —— 生产默认值是 0.5s / 5s，不压的话每个重连用例都要
    真等半秒到五秒。要测退避本身（比如 stop() 能不能打断它）就显式传大值。
    """
    monkeypatch.setattr("wakeword.engine.serial.Serial", FakeSerial)
    config = _FakeConfig(**{
        "wakeword.port": "/dev/fake",
        "wakeword.baudrate": 115200,
        "wakeword.serial_timeout_sec": timeout,
        "wakeword.cooldown_sec": cooldown,
        "wakeword.reconnect_initial_sec": reconnect_initial,
        "wakeword.reconnect_max_sec": reconnect_max,
    })
    engine = WakeWordEngine(config)
    engine.start(on_detected)

    fake = engine._ser          # 引擎在 start() 里自己建的那一个
    assert isinstance(fake, FakeSerial), "serial.Serial 没被换成 FakeSerial？"
    return engine, fake


def _wait_until(predicate, timeout=2.0, interval=0.005):
    """轮询等条件成立。超时上限都压得很短 —— 出问题是「断言失败」，不是「测试挂死」。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return bool(predicate())


def test_drain_discards_frames_accumulated_during_callback(monkeypatch):
    """回调执行期间堆积的帧会被 `reset_input_buffer()` 丢掉 —— 有意行为，不是 bug。

    为什么必须丢：回调返回之后，缓冲里如果还留着用户在这几百毫秒里多喊的那一声，
    监听循环会**立刻**拿它再触发一次唤醒。这一轮刚跑完，马上又被旧帧唤起 ——
    比丢帧糟得多。所以「丢」是设计（设计文档 §7 坑 1），不是缺陷。

    这里同时钉住「丢了之后不会又被翻出来」：丢掉之后静静等一会儿，计数必须还是 1。
    """
    detections: list[int] = []
    first_callback_started = threading.Event()

    def on_detected():
        detections.append(len(detections))
        first_callback_started.set()
        time.sleep(0.3)                 # 假装在跑急停 / 流水线

    engine, fake = _start_engine(monkeypatch, on_detected, cooldown=0.0)
    try:
        fake.feed_frame(0x01)
        assert first_callback_started.wait(2.0), "第一次回调压根没跑起来"

        # ★ 这一帧落在**回调执行期间**：监听循环此刻正卡在回调里，没人去读串口，
        #   所以它只会躺在 OS 缓冲里等着被 _drain() 清掉。
        stale = fake.feed_frame(0x02)

        assert _wait_until(lambda: fake.discard_calls, 2.0), \
            "回调返回后 _drain() 没有执行"
        assert stale in fake.discard_calls, (
            "回调期间堆积的帧没有被丢掉 —— _drain() 的行为变了？"
            "（丢帧是设计，改它之前请先读 docstring）"
        )

        time.sleep(0.4)                 # 给「万一被翻出来」留足时间
        assert len(detections) == 1, "被丢掉的陈旧帧居然又触发了一次唤醒"
    finally:
        engine.stop()


class _FakeTime:
    """只提供 monotonic() 的假 time 模块 —— 用来把冷却期瞬间推过去，不用真等 2 秒。

    替换的是 `wakeword.engine` 模块里那个 `time` 名字（不是真的 time 模块），
    所以不会影响 pytest / logging 自己的计时。
    """

    def __init__(self, start=1000.0):
        self.t = start                  # 起点必须远离 0：_last_detection_time 初值就是 0.0

    def monotonic(self):
        return self.t


def test_cooldown_swallows_rapid_second_wake(monkeypatch):
    """冷却期（默认 2.0s）内的第二次唤醒被忽略，过了冷却期又能正常唤醒。

    冷却是引擎**自己的**防抖，跟调用方的防重入是两件事（后者管的是「上一轮流水线还在跑」）。
    两个不同的唤醒词（cmd 0x01 / 0x02）共用同一个冷却 —— 冷却按引擎算，不按 cmd 算。

    后半段（推进假时钟后必须醒得过来）不是凑数：只测「第二次被忽略」的话，监听线程
    万一已经死掉，那个用例照样会绿。
    """
    clock = _FakeTime()
    monkeypatch.setattr("wakeword.engine.time", clock)

    detections: list[int] = []
    engine, fake = _start_engine(monkeypatch, lambda: detections.append(1),
                                 cooldown=2.0)     # 就是 config 里的默认值
    try:
        fake.feed_frame(0x01)
        assert _wait_until(lambda: len(detections) == 1, 2.0), "第一次唤醒没反应"

        fake.feed_frame(0x02)                     # 冷却期内的第二声（<2s）
        time.sleep(0.4)
        assert len(detections) == 1, "冷却期内的第二次唤醒没被忽略"

        clock.t += 3.0                            # 冷却期过去了
        fake.feed_frame(0x03)
        assert _wait_until(lambda: len(detections) == 2, 2.0), \
            "冷却期过后仍然醒不过来 —— 引擎是不是已经停了？"
    finally:
        engine.stop()


# ---------------------------------------------------------------- 掉线重连
# 以下三个用例对应 2026-09-18 的生产事故。设计见
# docs/plans/2026-09-18-wakeword-serial-reconnect-design.md。


def test_read_error_reopens_serial_and_listening_survives(monkeypatch):
    """串口读异常必须**重连**，而不是让监听线程死掉 —— 事故的回归测试。

    2026-09-18 16:03 的现场：USB 重枚举把唤醒模块从 ttyUSB0 挤成 ttyUSB1，进程攥着
    已删除节点的旧 fd，read 抛「device reports readiness to read but returned no
    data」。修复前 `_listen_loop` 在这个异常上直接 `break`：监听线程死掉，而主进程
    健在 —— 于是 `systemctl status` 一直显示绿色 `active (running)`，用户看到的却是
    「喊它没反应」。systemd 的 `Restart=on-failure` 只在进程退出时触发，帮不上忙。

    这个用例钉两件事，缺一不可：
      * 引擎**重开了**串口（`FakeSerial.instances` 多了一个）；
      * 重开之后**真的在听** —— 在新句柄上喊一声必须能唤醒。

    只断言前者的话，「重连了但监听循环已经退出」这种半吊子实现照样绿。
    """
    detections: list[int] = []
    engine, fake1 = _start_engine(monkeypatch, lambda: detections.append(1))
    try:
        # 先证明它本来是活的 —— 不然最后「没反应」分不清是重连坏了还是一开始就没跑
        fake1.feed_frame(0x01)
        assert _wait_until(lambda: len(detections) == 1, 2.0), "初始状态就没在工作"

        fake1.disconnect()      # ★ 模拟 USB 重枚举：旧 fd 上的读操作开始抛异常

        assert _wait_until(lambda: len(FakeSerial.instances) >= 2, 3.0), (
            "读异常之后引擎没有重开串口 —— 监听线程是不是又 break 出去了？"
        )
        fake2 = FakeSerial.instances[-1]

        fake2.feed_frame(0x02)
        assert _wait_until(lambda: len(detections) == 2, 3.0), (
            "重连之后唤醒词失效 —— 串口是开了，但监听循环已经不在了"
        )
    finally:
        engine.stop()


def test_reconnect_keeps_retrying_while_device_is_absent(monkeypatch):
    """模块被拔掉期间引擎必须**一直**在重试，设备回来就自动接上。

    这比「换设备号」更狠一档：`serial.Serial()` 直接开不起来。引擎这时应该待在
    `_reconnect()` 的退避循环里不出来，既不放弃也不退出线程。

    `is_ready` 在断开期间必须是 False —— 它是「现在喊一声有没有用」的唯一诚实答案。
    （`is_ready` 反映的是**串口**，不是「线程还活着」：重连期间线程一直活着。）
    """
    detections: list[int] = []
    engine, fake1 = _start_engine(monkeypatch, lambda: detections.append(1))
    try:
        fake1.feed_frame(0x01)
        assert _wait_until(lambda: len(detections) == 1, 2.0), "初始状态就没在工作"

        # ★ 顺序要紧：先设 open_error 再 disconnect()。
        #   反过来的话，引擎可能在两步之间抢先重开成功（窗口很小但真实存在），
        #   那个新句柄是健康的，于是 is_ready 一直 True，用例就 flaky 了。
        FakeSerial.open_error = serial.SerialException("no such device")
        fake1.disconnect()

        assert _wait_until(lambda: not engine.is_ready, 2.0), (
            "断开之后 is_ready 还报 True —— 它会骗调用方说还能喊"
        )
        assert engine._thread.is_alive(), "监听线程退出了 —— 重试没有兜住「设备不在」"

        # 插回来：必须自动接上，不用重启任何东西
        FakeSerial.open_error = None
        assert _wait_until(lambda: len(FakeSerial.instances) >= 2, 3.0), \
            "设备回来了却没有重连上"
        fake2 = FakeSerial.instances[-1]

        fake2.feed_frame(0x02)
        assert _wait_until(lambda: len(detections) == 2, 3.0), "重连之后唤醒词失效"
    finally:
        engine.stop()


def test_stop_interrupts_backoff_wait(monkeypatch):
    """stop() 必须能**立刻**打断退避等待，而不是干等它睡完。

    退避等待用的是 `_stop_event.wait()`，不是 `time.sleep()`。换成 sleep 的话：
    设备拔掉时退避会涨到 `reconnect_max_sec`（生产默认 5 秒），而 `stop()` 里是
    `join(timeout=2.0)` —— 于是 stop() 白等 2 秒后超时返回，还留下一条活着的线程。
    退出/重配路径上这种「卡两秒」很难查，因为日志上只有一条 warning。

    这里把退避设成 10 秒、断言 stop() 在 1 秒内返回。sleep 实现要耗满 2 秒
    （join 超时），差距给足，不会 flaky。

    **真正起区分作用的是那条计时断言** —— 另外两条（`_thread` 清空、`is_ready` False）
    在 sleep 实现下也成立，它们只是顺带钉住 stop() 的收尾语义，不假装能发现这个 bug。
    """
    engine, fake1 = _start_engine(monkeypatch, lambda: None,
                                  reconnect_initial=10.0, reconnect_max=30.0)
    fake1.disconnect()
    FakeSerial.open_error = serial.SerialException("no such device")

    # 等它确实进到退避等待里，此刻 stop() 才有意义
    assert _wait_until(lambda: not engine.is_ready, 2.0), "没进入重连状态"

    t0 = time.monotonic()
    engine.stop()
    elapsed = time.monotonic() - t0

    assert elapsed < 1.0, (
        f"stop() 花了 {elapsed:.2f}s —— 退避等待没用 _stop_event.wait() 打断？"
        f"（生产里这一下会卡满 join 的 2 秒）"
    )
    assert engine._thread is None, "stop() 之后 _thread 没清掉"
    assert not engine.is_ready, "stop() 之后串口还开着"
