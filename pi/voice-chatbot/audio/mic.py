"""Microphone input via PyAudio. 16kHz mono int16."""

import pyaudio

from audio.devices import resolve_device_index
from utils.config import Config


class Microphone:
    """Capture audio from the default or configured input device.

    Usage:
        with Microphone(config) as mic:
            while True:
                chunk = mic.read_chunk()  # blocks until 30ms of audio ready
    """

    def __init__(self, config: Config):
        self._sample_rate = config.get("audio.sample_rate", 16000)
        self._channels = config.get("audio.channels", 1)
        self._chunk_size = config.get("audio.chunk_size", 480)
        self._device_index = resolve_device_index(config.get("audio.device_index", None))

        self._pa = pyaudio.PyAudio()
        self._stream = self._pa.open(
            format=pyaudio.paInt16,
            channels=self._channels,
            rate=self._sample_rate,
            input=True,
            input_device_index=self._device_index,
            frames_per_buffer=self._chunk_size,
        )

    def read_chunk(self) -> bytes:
        """Read one chunk of audio. Blocks until data is available.

        Returns:
            Raw PCM bytes (int16, mono). Length = chunk_size * 2 bytes.
        """
        return self._stream.read(self._chunk_size, exception_on_overflow=False)

    def close(self) -> None:
        """Stop and close the audio stream and PyAudio instance."""
        if self._stream is not None:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None
        if self._pa is not None:
            self._pa.terminate()
            self._pa = None

    def __enter__(self) -> "Microphone":
        return self

    def __exit__(self, *args) -> None:
        self.close()
