"""ASR client — Alibaba Cloud DashScope paraformer-realtime-v2.

官方文档: https://help.aliyun.com/zh/model-studio/paraformer-real-time-speech-recognition-python-sdk

非流式 call() 用于一句话识别:
  - get_sentence() 返回 List[Dict], 每个 dict 通过 sentence['text'] 取文本
  - call() 的 file 参数必须是文件路径(str), 不能直接传 bytes
  - Recognition 通过环境变量 DASHSCOPE_API_KEY 获取 API Key
"""

import os
import tempfile
from http import HTTPStatus
from pathlib import Path

from dashscope.audio.asr import Recognition
from loguru import logger

from utils.config import Config


class ASRClient:
    """One-shot speech recognition via DashScope.

    Usage:
        client = ASRClient(config)
        text = client.recognize(wav_bytes)
    """

    def __init__(self, config: Config):
        self._api_key = config.get("asr.api_key")
        self._model = config.get("asr.model", "paraformer-realtime-v2")
        self._sample_rate = config.get("asr.sample_rate", 16000)

    def recognize(self, audio_wav: bytes) -> str:
        """Recognize speech from WAV audio bytes.

        Args:
            audio_wav: Complete WAV file bytes (16kHz, mono, int16).

        Returns:
            Recognized text, or empty string if nothing was understood.
        """
        logger.info("ASR: sending audio for recognition")

        # DashScope SDK reads from dashscope.api_key (cached at import time)
        import dashscope
        if not dashscope.api_key:
            dashscope.api_key = self._api_key

        # call() expects a file path, not raw bytes — write to temp file
        tmp_path = Path(tempfile.gettempdir()) / "_jarvis_asr.wav"
        tmp_path.write_bytes(audio_wav)

        recognition = Recognition(
            model=self._model,
            format="wav",
            sample_rate=self._sample_rate,
            callback=None,  # 非流式 call() 不需要回调
        )
        result = recognition.call(str(tmp_path))

        # cleanup temp file
        try:
            tmp_path.unlink()
        except OSError:
            pass

        if result.status_code != HTTPStatus.OK:
            logger.error(f"ASR: API error — status={result.status_code}, message={result.message}")
            return ""

        sentences = result.get_sentence()  # 非流式返回 List[Dict]
        if not sentences:
            logger.warning("ASR: no speech detected")
            return ""

        text = sentences[0]["text"]  # dict 取值，不是 .text 属性
        logger.info(f"ASR: recognized → '{text}'")
        return text
