"""Voice Activity Detection recorder.
Uses webrtcvad to detect speech boundaries — records until sustained silence.
"""

import io
import time
import wave

import webrtcvad
from loguru import logger

from audio.mic import Microphone
from utils.config import Config


class VADRecorder:
    """Record speech from microphone, auto-stopping on silence.

    Usage:
        recorder = VADRecorder(mic, config)
        wav_bytes = recorder.record()  # blocks until user stops speaking
    """

    def __init__(self, mic: Microphone, config: Config):
        self._mic = mic
        self._vad = webrtcvad.Vad(config.get("vad.mode", 2))
        self._sample_rate = config.get("audio.sample_rate", 16000)
        self._silence_ms = config.get("vad.silence_threshold_ms", 800)
        self._max_sec = config.get("vad.max_duration_sec", 60)
        self._no_speech_timeout_sec = config.get("vad.no_speech_timeout_sec", 5)
        # 需要连续这么多帧判为语音才算「开口」。单帧太敏感：提示音尾音、
        # 扬声器残余、鼠标点击都会被误判成说话，导致还没开口录音就结束了。
        self._speech_start_frames = config.get("vad.speech_start_frames", 3)
        # A 30ms frame at 16kHz = 480 samples = 960 bytes
        self._frame_duration_ms = 30
        self._chunk_size = 480  # samples per frame for webrtcvad
        self._preroll_frames = max(
            0, config.get("vad.preroll_ms", 150) // self._frame_duration_ms
        )

    def record(self) -> bytes:
        """Record until the user stops speaking.

        Returns:
            Complete WAV file as bytes (16kHz, mono, int16),
            or `b""` if no speech started within `vad.no_speech_timeout_sec`.
        """
        frames: list[bytes] = []
        silence_frames = 0
        consecutive_speech = 0
        no_speech_count = 0
        max_silence_frames = self._silence_ms // self._frame_duration_ms
        max_frames = int(self._max_sec * 1000 / self._frame_duration_ms)
        no_speech_frames = int(self._no_speech_timeout_sec * 1000 / self._frame_duration_ms)
        has_speech = False
        first_speech_frame: int | None = None
        start_time = time.monotonic()

        logger.info("VAD: recording started")

        for _ in range(max_frames):
            chunk = self._mic.read_chunk()
            frames.append(chunk)

            is_speech = self._vad.is_speech(chunk, self._sample_rate)

            if is_speech:
                consecutive_speech += 1
                if not has_speech and consecutive_speech >= self._speech_start_frames:
                    has_speech = True
                    # 记下连续语音的**第一帧**，而不是触发的那一帧，避免把起始音吃掉
                    first_speech_frame = len(frames) - consecutive_speech
                    logger.info(
                        f"VAD: speech started at frame {first_speech_frame} "
                        f"({first_speech_frame * self._frame_duration_ms}ms)"
                    )
                if has_speech:
                    silence_frames = 0
            else:
                consecutive_speech = 0
                if has_speech:
                    silence_frames += 1
                else:
                    # 唤醒词误触发 / 用户没开口：别干录满 max_duration_sec，尽早放弃。
                    # 返回 b"" 而不是一段静音 WAV —— 静音 WAV 体积够大，会骗过上层
                    # `len(wav_bytes) < 1000` 的检查，白白送去 ASR 跑一趟。
                    no_speech_count += 1
                    if no_speech_count >= no_speech_frames:
                        logger.info(
                            f"VAD: no speech within {self._no_speech_timeout_sec:.1f}s, giving up"
                        )
                        return b""

            # Stop only after we've seen speech AND sustained silence
            if has_speech and silence_frames >= max_silence_frames:
                logger.info(
                    f"VAD: silence detected ({silence_frames * self._frame_duration_ms}ms), "
                    f"total {len(frames)} frames = {len(frames) * self._frame_duration_ms}ms"
                )
                break

        elapsed = time.monotonic() - start_time
        logger.info(f"VAD: recording stopped after {elapsed:.1f}s, {len(frames)} frames captured")

        # 裁掉开头的静音，只保留一小段预卷（pre-roll），避免把第一个字咬掉。
        # 尾部静音不裁：它已经被 silence_threshold_ms 限制住，且对 ASR 有帮助。
        if first_speech_frame is not None and first_speech_frame > self._preroll_frames:
            frames = frames[first_speech_frame - self._preroll_frames :]

        return self._frames_to_wav(frames)

    def close(self) -> None:
        """Release the VAD instance (mic is owned by caller)."""
        pass  # webrtcvad.Vad has no explicit close, but maintaining the interface

    def _frames_to_wav(self, frames: list[bytes]) -> bytes:
        """Pack raw PCM frames into a WAV container.

        Args:
            frames: List of raw PCM chunks (int16 mono).

        Returns:
            Complete WAV file as bytes.
        """
        buf = io.BytesIO()
        with wave.open(buf, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # int16 = 2 bytes
            wf.setframerate(self._sample_rate)
            wf.writeframes(b"".join(frames))
        return buf.getvalue()
