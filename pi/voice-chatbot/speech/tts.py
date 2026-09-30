"""TTS client — Alibaba Cloud DashScope cosyvoice-v3-flash (streaming).

官方文档: https://help.aliyun.com/zh/model-studio/cosyvoice-tts-python-sdk

关键点:
  - 模块路径: dashscope.audio.tts_v2 (NOT tts)
  - SpeechSynthesizer 是 WebSocket 驱动的 — 音频通过 callback.on_data(data) 回调到达
  - streaming_call(text) 发送文本，streaming_complete() 通知结束
  - 默认输出 MP3 22050Hz — 通过 format=AudioFormat.PCM_16000HZ_MONO_16BIT 切换到 PCM
  - 每次调用需新建 SpeechSynthesizer 实例

架构: 用 queue.Queue + daemon thread 把回调式 API 适配为 generator
"""

import os
import queue
import threading
import time
from typing import Iterator

from dashscope.audio.tts_v2 import SpeechSynthesizer, ResultCallback, AudioFormat
from loguru import logger

from utils.config import Config


class _StreamCallback(ResultCallback):
    """Bridges DashScope push-based callback to a thread-safe Queue."""

    def __init__(self):
        self.queue: queue.Queue[bytes] = queue.Queue()
        self.done = threading.Event()
        self.error: str | None = None

    def on_open(self) -> None:
        logger.debug("TTS: WebSocket connected")

    def on_data(self, data: bytes) -> None:
        """Called from SDK thread when an audio chunk arrives."""
        self.queue.put(data)

    def on_complete(self) -> None:
        """Called when all audio has been synthesized."""
        logger.debug("TTS: server completed synthesis")
        self.done.set()

    def on_error(self, message: str) -> None:
        logger.error(f"TTS server error: {message}")
        self.error = message
        self.done.set()

    def on_close(self) -> None:
        self.done.set()


class TTSClient:
    """Text-to-speech via DashScope CosyVoice.

    Usage:
        client = TTSClient(config)
        for chunk in client.synthesize_stream("你好"):
            speaker.write(chunk)
    """

    def __init__(self, config: Config):
        self._model = config.get("tts.model", "cosyvoice-v3-flash")
        self._voice = config.get("tts.voice", "longanyang")
        self._api_key = config.get("tts.api_key", "")

        fmt_name = config.get("tts.format", "pcm")
        if fmt_name == "pcm":
            self._audio_format = AudioFormat.PCM_16000HZ_MONO_16BIT
        elif fmt_name == "mp3":
            self._audio_format = AudioFormat.MP3_22050HZ_MONO_256KBPS
        elif fmt_name == "wav":
            self._audio_format = AudioFormat.WAV_16000HZ_MONO_16BIT
        else:
            self._audio_format = AudioFormat.PCM_16000HZ_MONO_16BIT

    def synthesize(self, text: str) -> bytes:
        """One-shot synthesis. Collects all streaming chunks into a single buffer."""
        logger.info(f"TTS: synthesizing {len(text)} chars")
        return b"".join(self.synthesize_stream(text))

    def synthesize_stream(self, text: str) -> Iterator[bytes]:
        """Streaming synthesis with retry on WebSocket timeout.

        Retries up to 3 times with increasing delay if the initial
        WebSocket connection times out.
        """
        logger.info(f"TTS (stream): synthesizing {len(text)} chars")

        # DashScope SDK reads from dashscope.api_key (cached at import time)
        import dashscope
        if not dashscope.api_key:
            dashscope.api_key = self._api_key

        last_error = None
        for attempt in range(3):
            try:
                yield from self._synthesize_once(text)
                return  # success
            except TimeoutError as e:
                last_error = e
                if attempt < 2:
                    wait = (attempt + 1) * 2  # 2s, 4s backoff
                    logger.warning(
                        f"TTS: WebSocket timeout (attempt {attempt + 1}/3), "
                        f"retrying in {wait}s..."
                    )
                    time.sleep(wait)
                else:
                    logger.error(f"TTS: all 3 attempts failed: {e}")

        raise last_error or RuntimeError("TTS failed after retries")

    def _synthesize_once(self, text: str) -> Iterator[bytes]:
        """Single attempt — creates WebSocket, streams audio, tears down."""

        callback = _StreamCallback()
        synthesizer = SpeechSynthesizer(
            model=self._model,
            voice=self._voice,
            format=self._audio_format,
            callback=callback,
        )

        # Send the full text
        synthesizer.streaming_call(text)

        # streaming_complete() blocks → run in daemon thread
        def _finish():
            try:
                synthesizer.streaming_complete()
            except Exception as e:
                callback.error = str(e)
            finally:
                callback.done.set()

        t = threading.Thread(target=_finish, daemon=True)
        t.start()

        # Drain queue — yields chunks as they arrive
        while not callback.done.is_set() or not callback.queue.empty():
            try:
                chunk = callback.queue.get(timeout=0.05)
                yield chunk
            except queue.Empty:
                continue

        if callback.error:
            logger.error(f"TTS stream error: {callback.error}")

        logger.info("TTS (stream): complete")
