"""Speaker output via PyAudio. 16kHz mono int16."""

import threading

import pyaudio

from audio.devices import resolve_device_index
from utils.config import Config


class Speaker:
    """Play PCM audio to the default or configured output device.

    Usage:
        with Speaker(config) as spk:
            spk.write(pcm_bytes)  # blocking — waits for audio to finish playing
    """

    def __init__(self, config: Config):
        self._sample_rate = config.get("audio.sample_rate", 16000)
        self._channels = config.get("audio.channels", 1)
        self._device_index = resolve_device_index(config.get("audio.device_index", None))

        # 现在有两个写入方：TTS(唤醒线程) 和音乐播放(独立线程)。
        # 正常情况下音乐先被停掉再进 TTS，但万一没停干净，这把锁保证不会
        # 两条流交错写进同一个缓冲区（那会变成刺耳的噪音，很难查）。
        self._write_lock = threading.Lock()

        self._pa = pyaudio.PyAudio()
        self._stream = self._pa.open(
            format=pyaudio.paInt16,
            channels=self._channels,
            rate=self._sample_rate,
            output=True,
            output_device_index=self._device_index,
            frames_per_buffer=config.get("audio.chunk_size", 480),
        )

    def write(self, data: bytes) -> None:
        """Write PCM data and block until it finishes playing.

        线程安全：并发写入方会被串行化（见 _write_lock）。

        Args:
            data: Raw PCM bytes (int16, mono).
        """
        with self._write_lock:
            self._stream.write(data)

    def close(self) -> None:
        """Stop and close the audio stream and PyAudio instance."""
        if self._stream is not None:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None
        if self._pa is not None:
            self._pa.terminate()
            self._pa = None

    def __enter__(self) -> "Speaker":
        return self

    def __exit__(self, *args) -> None:
        self.close()
