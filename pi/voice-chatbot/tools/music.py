"""放歌。

## 怎么停（设计里最关键的一问）

播放跑在**独立线程**上，不是唤醒线程 —— 这一点是必须的：唤醒词回调本身就在唤醒
线程上，如果播放把唤醒线程占住，用户喊唤醒词时根本没人处理，只能等整首歌放完。
所以播放必须让出唤醒线程。

停止链路（「喊一声唤醒词，歌就停」）：

    用户喊唤醒词 → WakeWordEngine 线程 → ConversationManager._on_wake_word()
      → self._music.pause()            # 置 Event，播放循环下一帧就退出
      → （内部 join）等它真正放开 Speaker
      → 才 transition(LISTENING)        # 保证 TTS/录音不会和音乐抢同一条音频流

停完之后接着说什么都行：「别放了」「换一首」「刚才那首是谁唱的」。

另有两个自动停：歌放完了（自然结束）、超过 `music.max_minutes`（兜底）。

## 停 / 暂停 / 续播：三个动作不能混（T7）

| 动作 | 语义 | 保留什么 |
|---|---|---|
| `pause()`  | 暂停，**可恢复**（打断的默认语义） | url / label / **已播位置** |
| `resume()` | 从暂停位续播 | —— 起一条新播放线程，seek 到位置 |
| `stop()`   | **销毁性**：真的不听了 | 连暂停位一起清 |

判据来自用户定的通用原则（设计 §9.2）：**打断默认是「暂停、可恢复」，不是销毁**；
只有用户**明说**「停下 / 不听了 / 别放了」才走 `stop()`。

★ 因为 `stop()` 是销毁性的，`play()` 里那句「一次只放一首」的 `self.stop()`
会**顺带清掉暂停位** —— 这正是要的：用户说「换一首」，旧的暂停位不该再复活。

⚠️ 续播是**按关键帧**定位的（`container.seek`）：会落到暂停位置**之前**最近的
一个音频关键帧，最多差几百毫秒。听感是「接上了，但少了一两个词」，
**不是无缝续播**。这是设计明确接受的偏差，不要试图做精确 seek。

## 为什么用 PyAV 而不是系统自带的 GStreamer

实测本机 GStreamer **没有 AAC 解码器**（`faad` / `avdec_aac` 都不存在），只能解 MP3。
QQ音乐 默认给的是 m4a(AAC)，那样就只能挑特定音质。PyAV 的 pip 轮子自带整套 FFmpeg
（libavcodec 62.x，含 AAC），而且**不需要 sudo** —— apt 装 ffmpeg 是要 root 的。
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any

import av
from loguru import logger

from tools.registry import Tool, ToolRegistry

_PCM_RATE = 16000  # 与 Speaker / TTS 一致
_SEARCH_TOOL = "mcp__qqmusic__search_music"
_URL_TOOL = "mcp__qqmusic__get_song_url"
_JOIN_TIMEOUT = 3.0  # 等播放线程退出的上限（stop / pause 共用）


class MusicController:
    """搜歌 → 取播放链接 → 解码播放。线程安全（内部有状态锁）。"""

    def __init__(
        self,
        *,
        speaker: Any,
        mcp_host: Any,
        config: Any,
    ) -> None:
        self._speaker = speaker
        self._mcp = mcp_host
        self._config = config

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        # 已经找好链接、等 TTS 说完话再开播的曲目。见 start_pending()。
        self._pending: tuple[str, str] | None = None
        # 暂停位：(url, label, 已播秒数)。见 pause() / resume()。
        # 与 `_pending` 是**两个不同的槽** —— 混起来就是设计 §9.5 那个坑。
        self._paused: tuple[str, str, float] | None = None
        self._label = ""
        self._url = ""
        # 当前播放位置（秒）。由播放线程在解码循环里更新（见 _record_position），
        # 供 pause() 快照。初值 / 续播起点由 _start() 写。
        self._position = 0.0
        # 「这一轮已经因为缺 PTS 退过兜底了」——只用来压日志，不参与逻辑
        self._pts_missing_logged = False
        self._error: str | None = None
        self._state_lock = threading.Lock()

    # ------------------------------------------------------------ 对外

    @property
    def is_playing(self) -> bool:
        with self._state_lock:
            return self._thread is not None and self._thread.is_alive()

    @property
    def is_paused(self) -> bool:
        """是否有**暂停位**（= 有东西可以 resume）。

        注意它问的不是「现在没在出声」：什么都没放过、也没暂停过时，
        `is_paused` 是 False，而 `pause()` 仍然返回 True（见其 docstring）。
        """
        with self._state_lock:
            return self._paused is not None

    def play(self, query: str) -> str:
        """搜歌并开始播放。返回一句可以直接念给用户的话。"""
        query = (query or "").strip()
        if not query:
            return "没听清要放什么歌，再说一次歌名？"

        # 一次只放一首：先停掉上一首，避免两条流叠着写 Speaker。
        # ★ 这里顺带清掉暂停位 —— 用户说「换一首」时，被暂停的旧歌不该再复活。
        self.stop()

        song = self._search_first(query)
        if song is None:
            return f"没搜到「{query}」，换个说法再试试？"
        mid, name, singers = song

        url, quality = self._resolve_url(mid)
        if not url:
            logger.warning(f"音乐：《{name}》所有音质都拿不到播放链接")
            return (
                f"找到《{name}》（{singers}）了，但拿不到播放链接 —— "
                f"QQ音乐要登录凭证才行，得配 QQ_MUSIC_COOKIE。"
            )

        logger.info(f"音乐：{name} - {singers} | 音质 {quality}")
        # **不立刻开播**：等 TTS 把回话说完了再放（见 start_pending）。
        # 工具返回后 LLM 还会再生成一句「好，给你放…」，那句要走 TTS 出声；
        # 若此刻音乐已经开播，两路会同时写同一条音频流，糊成一团。
        with self._state_lock:
            self._pending = (url, f"{name} - {singers}")
        return f"好，给你放《{name}》，{singers}。要停就喊唤醒词，然后说「别放了」。"
        # 也不声明 requires_followup —— 播歌是「说完就完事」，
        # 自动重开麦克风的话音乐刚开始就被自己的录音打断。

    def start_pending(self) -> None:
        """开播上一首待播的曲子。**由 ConversationManager 在进入 IDLE 时调用** ——
        也就是 TTS 已经说完之后。这样人声和音乐永远不会同时占着 Speaker。

        （T8 起 IDLE 的入口换成了 `resume_or_start_pending()`，它优先续播暂停中的音乐。）
        """
        with self._state_lock:
            pending = self._pending
            self._pending = None
        if pending is None:
            return
        url, label = pending
        self._start(url, label)

    # ------------------------------------------------------------ 暂停 / 续播

    def pause(self) -> bool:
        """暂停播放，**保留 url / label / 已播位置**，等 `resume()` 接着放。

        这是「喊唤醒词打断」的默认动作（设计 §9.2）：用户不是不想听了，
        只是想插一句话 —— 插完了从差不多的地方接着听。

        幂等：本来就没在放（包括已经暂停着）也返回 True，并**保留**已有的暂停位。
        （返回的是「现在没在出声」这个事实，不是「刚刚停了什么」；
        要问「有没有得续」用 `is_paused`。）

        保证和 `stop()` 一样：返回时 `_stop` 已置位、播放线程已 join，
        播音线程不会再往 Speaker 写一个字节 —— 所以调用方可以放心 transition(LISTENING)。
        """
        with self._state_lock:
            thread = self._thread
            running = thread is not None and thread.is_alive()
            if running:
                self._stop.set()
            else:
                self._thread = None

        if running:
            thread.join(timeout=_JOIN_TIMEOUT)
            if thread.is_alive():
                # 和 stop() 同样的兜底：即使线程卡在 av.open() 的网络超时里没退出来，
                # `_stop` 已置位 → 它不会再出声，抢不到 Speaker。
                logger.warning("音乐：暂停时播放线程未在超时内退出（但已不会再出声）")
            with self._state_lock:
                # ★ 位置在 join **之后**快照：此时解码循环已经停了，位置不会再动。
                #   记的是「最后解码到的那一帧的 PTS」，最多领先扬声器一帧（~64ms）。
                self._paused = (self._url, self._label, self._position)
            logger.info(f"音乐：《{self._label}》已暂停在 {self._position:.1f}s")
        return True

    def resume(self) -> bool:
        """从暂停位续播。**没有暂停位**（或已经在放）返回 False，且不出声。

        续播起来之后暂停位就被清掉 —— 它已经在放了，位置由播放线程接管。
        """
        with self._state_lock:
            paused = self._paused
            if paused is None:
                return False
            if self._thread is not None and self._thread.is_alive():
                return False
            self._paused = None
            self._thread = None
        url, label, position = paused
        logger.info(f"音乐：续播《{label}》，从 {position:.1f}s 开始")
        self._start(url, label, start_sec=position)
        return True

    def resume_or_start_pending(self) -> None:
        """**进 IDLE 时的唯一入口**（T8 的 ConversationManager 调它）。

        优先级：**暂停中的音乐 > 待播的新歌**。
        - 有暂停位 → `resume()`（成功就清掉暂停位，它已经在放了）
        - 没有暂停位 → `start_pending()`（没有待播的就什么都不做）

        为什么必须定优先级：暂停位和待播槽是**两个不同的槽**。
        只顾 pending 的话，用户「喊唤醒词打断 → 助手说完 → 进 IDLE」时，
        被暂停的那首歌就再也不会响了；反过来，`stop()` 只清 pending 不清暂停位的话，
        用户说了「别放了」之后进 IDLE，音乐**会自己又响起来**（设计 §9.5，
        实车上极难查 —— 用户只会觉得「我明明让它停了」）。
        """
        if self.is_playing:
            return  # 已经在放了，不插一脚（正常流程走不到这里）
        if self.resume():
            return
        self.start_pending()

    def stop_and_describe(self) -> str:
        """给 stop_music 工具用：停掉并回报一句结果。"""
        label = self._label
        with self._state_lock:
            # ★ 「有东西」必须把**暂停位**算进去（设计 §9.4 最后一条）：
            #   只算 pending/playing 的话，用户对着暂停中的音乐说「别放了」会被
            #   当成「本来就没放」直接返回，暂停位没清 → 进 IDLE 又自己响起来（§9.5）。
            paused = self._paused is not None
            had_anything = (
                paused
                or self._pending is not None
                or (self._thread is not None and self._thread.is_alive())
            )
        if not had_anything:
            return "现在没有在放音乐。"
        stopped = self.stop()  # 销毁性：连暂停位一起清
        if not stopped:
            return "已经让它停了，但播放线程没及时退出。"
        return f"已停止播放《{label}》。" if label else "音乐已停止。"

    def stop(self, wait: bool = True, timeout: float = 3.0) -> bool:
        """**销毁性**停止播放。返回 True 表示已停（或本来就没在放）。

        和 `pause()` 的区别在**语义**上：这里是「真的不听了」——
        `_paused` 一并清掉，之后谁也别想把它续起来（设计 §9.2 / §9.5）。
        调用方：`play()`（换一首）、`stop_and_describe()`（用户明说别放）、`close()`。

        `_stop` 一置位，`_write()` 就再也不会往 Speaker 写东西 —— 所以即使播放线程
        卡在 `av.open()` 的网络超时里没能及时 join 掉，也不会和 TTS 抢音频流。
        """
        with self._state_lock:
            self._pending = None  # 还没开播的也一并取消
            self._paused = None  # ★ 暂停位也是「不听了」的一部分，一起销毁
            thread = self._thread
            if thread is None or not thread.is_alive():
                self._thread = None
                return True
            self._stop.set()

        if wait:
            thread.join(timeout=timeout)
            if thread.is_alive():
                logger.warning("音乐：播放线程未在超时内退出（但已不会再出声）")
                return False
        return True

    def close(self) -> None:
        self.stop()

    # ------------------------------------------------------------ 内部：搜索与取链接

    def _search_first(self, query: str) -> tuple[str, str, str] | None:
        raw = self._mcp.call_tool(_SEARCH_TOOL, {"keyword": query})
        data = _loads(raw)
        if data is None:
            logger.warning(f"音乐：搜索结果不是 JSON —— {str(raw)[:120]}")
            return None
        songs = data.get("songs") or []
        if not songs:
            return None
        hit = songs[0]
        return (
            str(hit.get("mid") or ""),
            str(hit.get("name") or query),
            str(hit.get("singers") or ""),
        )

    def _resolve_url(self, mid: str) -> tuple[str, str]:
        """按配置里的音质顺序逐个尝试，用第一个拿得到链接的。

        无 VIP / 未登录时高码率会返回空 url，所以必须能降级 —— 实测匿名状态下
        **所有**音质都返回空，那时的报错要能让用户看懂是缺 Cookie 而不是代码错。
        """
        qualities = self._config.get("music.qualities", ["320", "128", "m4a"]) or ["320", "128", "m4a"]
        for quality in qualities:
            raw = self._mcp.call_tool(_URL_TOOL, {"song_mid": mid, "quality": str(quality)})
            data = _loads(raw)
            if data is None:
                continue
            url = str(data.get("url") or "").strip()
            if url:
                return url, str(quality)
            logger.info(f"音乐：音质 {quality} 没拿到链接，继续往下降")
        return "", ""

    # ------------------------------------------------------------ 内部：播放

    def _start(self, url: str, label: str, start_sec: float = 0.0) -> None:
        """起一条播放线程。`start_sec > 0` 表示从该位置续播（见 resume）。"""
        self._stop.clear()
        self._error = None
        self._pts_missing_logged = False
        self._label = label
        with self._state_lock:
            self._url = url
            self._position = start_sec
            self._thread = threading.Thread(
                target=self._run,
                args=(url, label, start_sec),
                name="music-playback",
                daemon=True,
            )
            self._thread.start()

    def _run(self, url: str, label: str, start_sec: float = 0.0) -> None:
        started = time.monotonic()
        max_seconds = float(self._config.get("music.max_minutes", 10) or 0) * 60

        try:
            with av.open(url, timeout=15.0) as container:
                stream = next((s for s in container.streams if s.type == "audio"), None)
                if stream is None:
                    logger.error(f"音乐：《{label}》这个流里没有音轨")
                    return

                if start_sec > 0:
                    self._seek(container, stream, start_sec)

                # 直接重采样成 Speaker 要的格式（s16 / mono / 16k），
                # 免得再写一条「解码 → 重采样 → 写流」的手工链路。
                resampler = av.AudioResampler(format="s16", layout="mono", rate=_PCM_RATE)

                # 兜底基线：只在 frame.pts 缺失时用（见 _record_position）
                position = start_sec

                for frame in container.decode(stream):
                    if self._should_stop(started, max_seconds):
                        break
                    position = self._record_position(frame, stream, position)
                    self._write(resampler.resample(frame))
                else:
                    # for-else：只有正常放完才 flush 尾部残留；被打断就不必了
                    if not self._stop.is_set():
                        self._write(resampler.resample(None))
        except Exception as exc:  # noqa: BLE001 —— 播放失败不该影响对话
            self._error = str(exc)
            logger.error(f"音乐：播放《{label}》失败 —— {exc}")
        finally:
            logger.info(f"音乐：《{label}》结束，历时 {time.monotonic() - started:.1f}s")

    def _seek(self, container: Any, stream: Any, start_sec: float) -> None:
        """把解码位置定位到 `start_sec` 附近（续播用）。

        ★ 单位换算：PyAV 的 `container.seek(offset, stream=...)` 里 **offset 的单位是
        该流的 time_base**（不是秒，也不是 av.time_base）——

            offset = start_sec / stream.time_base      # time_base 是 Fraction，如 1/16000

        所以 2.0 秒在 16kHz 的流上就是 `int(2.0 / (1/16000)) == 32000`。
        实测（PyAV 18.1.0，16k mono WAV）：target=2.0s → offset=32000 →
        解出来的第一帧 PTS 正好是 2.0s，总时长 6.0-2.0=4.0s。

        ⚠️ seek 是**按关键帧**定位的（backward=True），会落到目标之前最近的一个音频
        关键帧上 —— 最多差几百毫秒，这就是设计接受的「接上了，但少了一两个词」。
        不去做精确 seek。

        seek 失败不致命（比如某些流不支持）：记一条 warning，从头放。
        """
        time_base = stream.time_base
        if time_base is None:
            logger.warning(f"音乐：这个流没有 time_base，无法定位到 {start_sec:.1f}s，从头放")
            return
        offset = int(start_sec / float(time_base))  # ← 秒 → 流 time_base 单位
        try:
            container.seek(offset, stream=stream)
        except Exception as exc:  # noqa: BLE001 —— 定位失败就退化成从头放，别把歌吞了
            logger.warning(f"音乐：定位到 {start_sec:.1f}s 失败（{exc}），从头放")
            return
        logger.info(
            f"音乐：续播定位 {start_sec:.1f}s → seek offset={offset} "
            f"(time_base={time_base})"
        )

    def _record_position(self, frame: Any, stream: Any, fallback_sec: float) -> float:
        """记下「已经播到哪一秒」，返回本次算出的位置（同时作为下次的兜底基线）。

        ★ 记的是**输入帧的 PTS**（`frame.pts * stream.time_base`），设计 §9.4 明确要求。
        为什么不用重采样后的样本数：重采样输出和 Speaker 的缓冲对不齐，
        累计样本数会越播越偏；PTS 是容器给的真时间戳，和扬声器无关。

        ⚠️ **兜底**：有些流的 `frame.pts` 是 None（没写时间戳的裸流），
        那时退回「上一帧位置 + 本帧样本数 / 输入采样率」累计。精度差一点
        （不体现容器里的空洞/跳变），但比没有位置强。走这条分支会记一条 warning
        （**只记一次**，避免每帧刷屏），别让它静默发生。
        """
        seconds = None
        if frame.pts is not None and stream.time_base is not None:
            seconds = float(frame.pts * stream.time_base)

        if seconds is None:
            if not self._pts_missing_logged:
                self._pts_missing_logged = True
                logger.warning("音乐：这个流的帧没有 PTS，位置改用累计样本数估算（续播会略偏）")
            rate = float(getattr(stream, "rate", None) or _PCM_RATE)
            seconds = fallback_sec + frame.samples / rate

        with self._state_lock:
            self._position = seconds
        return seconds

    def _should_stop(self, started: float, max_seconds: float) -> bool:
        if self._stop.is_set():
            return True
        if max_seconds > 0 and time.monotonic() - started > max_seconds:
            logger.warning(f"音乐：超过 {max_seconds / 60:.0f} 分钟上限，自动停止")
            self._stop.set()
            return True
        return False

    def _write(self, frames: Any) -> None:
        """把重采样后的帧写进 Speaker。

        每一帧前后都查 `_stop`：停止信号一置位就立刻收手，绝不和 TTS 抢同一条流。
        """
        for frame in frames or ():
            if self._stop.is_set():
                return
            payload = frame.to_ndarray().tobytes()
            if not payload:
                continue
            if self._stop.is_set():
                return
            # Speaker.write() 自带写锁，会和 TTS 串行化
            self._speaker.write(payload)


def register_music_tools(registry: ToolRegistry, controller: MusicController) -> None:
    """把放歌能力登记成原生工具。

    为什么是**原生工具**而不是 MCP：搜歌和取链接确实来自 QQ音乐 MCP server，
    但「在树莓派的喇叭上出声」是本机硬件能力，不是 MCP 该管的事。
    服务器给链接、宿主负责放 —— 这个分工才是对的。
    """
    registry.register(
        Tool(
            name="play_music",
            description=(
                "播放音乐。用户说「放首歌」「来一首…」「我想听…」「放周杰伦的歌」时调用。"
                "参数可以是歌名、歌手名，或「歌手 歌名」。查到了会立刻开始播放。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "要播放的歌，比如「晴天」「周杰伦」「周杰伦 晴天」",
                    }
                },
                "required": ["query"],
            },
            handler=lambda query: controller.play(query),
        )
    )

    registry.register(
        Tool(
            name="stop_music",
            description="停止正在播放的音乐。用户说「别放了」「停」「关掉音乐」时调用。",
            parameters={"type": "object", "properties": {}, "required": []},
            handler=lambda: controller.stop_and_describe(),
        )
    )


def _loads(raw: Any) -> dict[str, Any] | None:
    """MCP 工具返回的是 JSON 字符串；解析失败返回 None（调用方负责兜底）。"""
    if isinstance(raw, dict):
        return raw
    try:
        parsed = json.loads(str(raw))
    except (TypeError, ValueError):
        return None
    return parsed if isinstance(parsed, dict) else None
