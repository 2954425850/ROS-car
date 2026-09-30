"""T7：`MusicController` 的 pause / resume。

依据：`docs/plans/2026-09-18-wake-handoff-and-vision-design.md` §9 ——
- §9.2 的不变量：**打断默认是「暂停、可恢复」，不是销毁**；只有用户明说
  「停下 / 不听了 / 别放了」才真正终止。
- §9.4 的实现要点：位置记**输入帧的 PTS**（`frame.pts * stream.time_base`）；
  `resume()` 用 `container.seek(...)`，**接受关键帧偏差**。
- §9.5 的坑：「暂停中」和「待播的新歌」是两个不同的槽；`stop_music` 只清后者的话，
  用户说了「别放了」之后一进 IDLE，音乐会**自己又响起来**。第 5 条测试专门钉它。

## 为什么这里用真解码器

位置逻辑最容易出的错是「记了个看起来对、其实和播放对不齐的数」。所以本文件用标准库
`wave` 现造一段本地 WAV，让 **PyAV 真去解**，只把 `Speaker` 换成假的（记录收到的
样本数）。**不联网、不依赖真喇叭。**

## 怎么把「暂停」卡在确定的位置

假 Speaker 写满 `block_after(pause_at)` 秒之后**阻塞**住，并设 `blocked` 事件；
同时它盯着真播放线程的 `_stop` Event —— `pause()` 一置位它就放行。
真 `Speaker.write()` 差不多也是这个行为（写完当前那块 ~64ms 就返回），
所以测试可以在主线程**同步**调 `pause()`，不用另起线程、也不用 sleep 猜时序。

（block 事件还有个 5s 超时自释放，防止某个用例失败时把播放线程永久挂住。）
"""

from __future__ import annotations

import array
import json
import math
import sys
import threading
import time
import wave

import pytest

from tools.music import MusicController

TOTAL_SEC = 6.0  # 造出来的 WAV 多长
PAUSE_AT_SEC = 2.0  # 在第几秒按下暂停


# --------------------------------------------------------------------- 工具


def write_wav(path, seconds: float, *, rate: int = 16000, freq: float = 441.0) -> float:
    """造一段 `seconds` 秒的 16kHz/mono/int16 正弦波 WAV，返回实际秒数。"""
    n = int(seconds * rate)
    buf = array.array(
        "h", (int(6000 * math.sin(2 * math.pi * freq * i / rate)) for i in range(n))
    )
    if sys.byteorder != "little":  # WAV 是 little-endian
        buf.byteswap()
    with wave.open(str(path), "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(rate)
        f.writeframes(buf.tobytes())
    return n / rate


def wait_until(pred, timeout: float = 5.0, interval: float = 0.01) -> bool:
    """轮询等条件成立（测试里不用裸 sleep 猜时序）。返回最终是否成立。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(interval)
    return pred()


# --------------------------------------------------------------------- 假件


class FakeConfig:
    """只实现 MusicController 用到的那一个方法。"""

    def __init__(self, values: dict | None = None) -> None:
        self._values = dict(values or {})

    def get(self, key: str, default=None):
        return self._values.get(key, default)


class FakeMCP:
    """假 QQ音乐 MCP server：搜歌固定返回一首，取链接固定返回给定的本地文件。"""

    def __init__(self, url: str) -> None:
        self._url = url
        self.calls: list[tuple[str, dict]] = []

    def call_tool(self, name: str, args: dict) -> str:
        self.calls.append((name, dict(args)))
        if name == "mcp__qqmusic__search_music":
            return json.dumps(
                {"songs": [{"mid": "m1", "name": "测试曲", "singers": "测试歌手"}]}
            )
        if name == "mcp__qqmusic__get_song_url":
            return json.dumps({"url": self._url})
        raise AssertionError(f"没预料到的工具调用：{name}")


class FakeSpeaker:
    """假 Speaker：只记「收到了多少样本」，不出声。"""

    def __init__(self, rate: int = 16000) -> None:
        self.rate = rate
        self._lock = threading.Lock()
        self._samples = 0
        self.writes = 0
        # 写满这么多样本之后阻塞一次（None = 从不阻塞）
        self.block_at_samples: int | None = None
        self._gated = False
        # 初始是 **clear** 的：一阻塞就等着被放行
        self.gate = threading.Event()
        self.blocked = threading.Event()
        # 真播放线程的 _stop Event（由 fixture 接上）
        self.stop_watch: threading.Event | None = None

    # -- 观测

    @property
    def samples(self) -> int:
        with self._lock:
            return self._samples

    @property
    def seconds(self) -> float:
        return self.samples / self.rate

    def reset(self) -> None:
        with self._lock:
            self._samples = 0

    # -- 控制

    def block_after(self, seconds: float) -> None:
        self.block_at_samples = int(seconds * self.rate)

    def release(self) -> None:
        self.gate.set()

    # -- Speaker 接口

    def write(self, data: bytes) -> None:
        n = len(data) // 2  # s16 / mono
        with self._lock:
            self._samples += n
            self.writes += 1
            total = self._samples

        if self._gated or self.block_at_samples is None or total < self.block_at_samples:
            return
        self._gated = True
        self.blocked.set()

        deadline = time.monotonic() + 5.0
        while not self.gate.is_set():
            if self.stop_watch is not None and self.stop_watch.is_set():
                return  # 真 Speaker.write() 也就是写完当前这块（~64ms）就返回
            if time.monotonic() > deadline:
                return  # 自释放，别把播放线程永久挂住
            time.sleep(0.005)


# --------------------------------------------------------------------- 夹具


@pytest.fixture
def rig(tmp_path):
    """(controller, speaker, 音频总秒数)。音频本地造，不联网。"""
    wav = tmp_path / "song.wav"
    total = write_wav(wav, TOTAL_SEC)
    speaker = FakeSpeaker()
    ctrl = MusicController(
        speaker=speaker,
        mcp_host=FakeMCP(str(wav)),
        config=FakeConfig(),
    )
    speaker.stop_watch = ctrl._stop
    return ctrl, speaker, total


def play_and_pause(ctrl: MusicController, speaker: FakeSpeaker, *, at: float = PAUSE_AT_SEC):
    """开播待播的歌，写到 `at` 秒时暂停。返回暂停位 (url, label, 秒数)。"""
    speaker.block_after(at)
    ctrl.play("测试")  # 立 pending 槽（play() 只是准备好链接，不立刻开播）
    ctrl.start_pending()  # 进 IDLE 时才会调它
    assert speaker.blocked.wait(5.0), "播放线程没能在 5s 内写到暂停点"
    assert ctrl.is_playing is True, "暂停前应该确实在放"
    assert ctrl.pause() is True
    assert ctrl._paused is not None, "pause() 之后应该有暂停位"
    return ctrl._paused


# --------------------------------------------------------------------- 测试


def test_pause_marks_paused_and_lets_playback_thread_exit(rig):
    """1. pause() 后 is_paused 为 True，且 _run 线程**已退出**（不是还挂着）。"""
    ctrl, speaker, _ = rig
    play_and_pause(ctrl, speaker)

    assert ctrl.is_paused is True
    assert ctrl.is_playing is False, "暂停后播放线程必须已经退出（不能只是不出声还挂着）"
    thread = ctrl._thread
    assert thread is None or not thread.is_alive()


def test_resume_continues_from_recorded_position(rig):
    """2. resume() 从记录位置继续：续播样本数 ≈ 总时长 − 暂停位置（允许关键帧偏差）。"""
    ctrl, speaker, total = rig
    _, _, paused_at = play_and_pause(ctrl, speaker)
    before = speaker.samples

    assert ctrl.resume() is True
    assert ctrl.is_paused is False, "续播之后暂停位应该被清掉（位置由播放线程接管）"
    assert wait_until(lambda: not ctrl.is_playing, 5.0), "续播应该会自己放完"

    resumed = (speaker.samples - before) / speaker.rate
    remaining = total - paused_at
    assert remaining - 1.0 <= resumed <= remaining + 1.0, (
        f"续播了 {resumed:.2f}s，应该约等于 总长 {total:.2f}s − 暂停位置 {paused_at:.2f}s "
        f"= {remaining:.2f}s（±1s 容差覆盖关键帧回退）"
    )
    assert resumed < total - 1.0, "续播听起来是从头重放了（没有 seek 到暂停位置）"


def test_resume_or_start_pending_resumes_paused_song_not_the_new_one(rig, tmp_path):
    """3. resume_or_start_pending() 在「暂停中」时续播（不是开新歌）。"""
    ctrl, speaker, total = rig
    _, _, paused_at = play_and_pause(ctrl, speaker)

    # 「暂停位」和「待播新歌」同时存在只能手工构造：公开 API 里 play() 会 stop(),
    # 而 stop() 是销毁性的（会清暂停位）。这里把 pending 槽直接塞进去，
    # 用来验「暂停中的音乐 > 待播的新歌」这条优先级。
    other = tmp_path / "other.wav"
    other_total = write_wav(other, 1.0, freq=880.0)
    with ctrl._state_lock:
        ctrl._pending = (str(other), "新歌")

    before = speaker.samples
    ctrl.resume_or_start_pending()

    assert ctrl.is_paused is False
    assert wait_until(lambda: not ctrl.is_playing, 5.0)
    got = (speaker.samples - before) / speaker.rate
    remaining = total - paused_at
    assert remaining - 1.0 <= got <= remaining + 1.0, (
        f"放出来 {got:.2f}s，应该续播那首暂停的（剩余约 {remaining:.2f}s），"
        f"而不是开那首 {other_total:.2f}s 的新歌"
    )
    assert abs(got - other_total) > 1.0, "放的是待播的新歌，不是暂停中的那首"


def test_resume_or_start_pending_starts_pending_when_nothing_paused(rig):
    """4. 没有暂停位、有待播新歌 → 开新歌。"""
    ctrl, speaker, total = rig
    ctrl.play("测试")  # 只立 pending，不立刻开播
    assert ctrl.is_playing is False
    assert speaker.samples == 0

    # 进 IDLE 之前可能先被喊了一声唤醒词（T8 的快动作会调 pause()）——
    # pause() 在「本来就没在放」时不该造出一个假的暂停位，把待播的歌顶掉。
    assert ctrl.pause() is True
    assert ctrl.is_paused is False

    ctrl.resume_or_start_pending()

    assert wait_until(lambda: not ctrl.is_playing, 5.0), "待播的歌应该开起来了"
    assert ctrl.is_paused is False
    assert speaker.seconds == pytest.approx(total, abs=0.5), "放的应该是整首 6s 的歌"


def test_stop_destroys_pause_slot_so_idle_plays_nothing(rig):
    """5. ★ stop() 之后 is_paused 为 False，且再调 resume_or_start_pending() 什么都不播。

    这条钉的就是设计 §9.5 那个坑：「说了别放，进 IDLE 又自己响起来」。
    """
    ctrl, speaker, _ = rig
    play_and_pause(ctrl, speaker)
    assert ctrl.is_paused is True

    assert ctrl.stop() is True
    assert ctrl.is_paused is False, "★ stop() 是销毁性的：暂停位必须一起清掉"

    before = speaker.samples
    ctrl.resume_or_start_pending()  # 进 IDLE 时 T8 会调它

    assert wait_until(lambda: ctrl.is_playing, 0.5) is False, "进了 IDLE 音乐又自己响起来了"
    time.sleep(0.2)
    assert speaker.samples == before, "★ 说了「别放了」，进 IDLE 之后又出声了（设计 §9.5）"
    assert ctrl.resume() is False


def test_stop_music_tool_clears_pause_slot(rig):
    """5b. ★ 同一条坑的**真实路径**：用户说「别放了」→ stop_music 工具 handler。

    如果 `stop_and_describe()` 的 had_anything 只算 pending/playing，
    「对着一首**暂停中**的音乐说别放了」会被当成「本来就没放」直接返回，
    暂停位没清 → 进 IDLE 又响起来。
    """
    ctrl, speaker, _ = rig
    play_and_pause(ctrl, speaker)

    text = ctrl.stop_and_describe()
    assert "没有在放音乐" not in text, f"暂停中的音乐被当成了「本来就没放」：{text!r}"
    assert ctrl.is_paused is False

    before = speaker.samples
    ctrl.resume_or_start_pending()
    assert wait_until(lambda: ctrl.is_playing, 0.5) is False
    time.sleep(0.2)
    assert speaker.samples == before, "★ 「别放了」之后进 IDLE 又出声了（设计 §9.5）"


def test_resume_without_pause_slot_returns_false_and_stays_silent(rig):
    """6. resume() 在没有暂停位时返回 False、不出声。"""
    ctrl, speaker, _ = rig

    assert ctrl.resume() is False
    assert ctrl.is_playing is False
    assert speaker.samples == 0

    # 放完一整首（自然结束）之后也没有暂停位
    ctrl.play("测试")
    ctrl.start_pending()
    assert wait_until(lambda: not ctrl.is_playing, 5.0)
    assert speaker.samples > 0
    before = speaker.samples

    assert ctrl.resume() is False
    time.sleep(0.2)
    assert speaker.samples == before, "resume() 在没有暂停位时不该出声"


def test_pause_records_the_position(rig):
    """7. 暂停位置确实被记录了（在 ~2s 停的，记下的秒数要落在合理区间）。"""
    ctrl, speaker, total = rig
    url, label, paused_at = play_and_pause(ctrl, speaker)

    assert str(url).endswith("song.wav"), f"暂停位里得记住 url，实际是 {url!r}"
    assert label == "测试曲 - 测试歌手", f"暂停位里得记住 label，实际是 {label!r}"
    assert 1.0 <= paused_at <= 3.0, f"记录的位置是 {paused_at:.2f}s，应该在 2.0s 附近"
    assert paused_at < total, "记录的位置不该超过总时长"


def test_pause_on_idle_is_true_but_creates_no_slot(rig):
    """补：pause() 幂等 —— 本来就没在放也返回 True，但不该凭空造出暂停位。"""
    ctrl, speaker, _ = rig

    assert ctrl.pause() is True
    assert ctrl.is_paused is False
    assert ctrl.is_playing is False
    assert ctrl.resume() is False

    # 已经暂停着再 pause()：幂等，暂停位保住
    play_and_pause(ctrl, speaker)
    first = ctrl._paused
    assert ctrl.pause() is True
    assert ctrl._paused == first, "重复 pause() 不该把暂停位弄丢或改坏"
    assert ctrl.is_playing is False
