"""Loguru logger setup with console + file output."""

import sys
from pathlib import Path

from loguru import logger


def setup_logger(level: str = "INFO", log_dir: str = "logs") -> None:
    """Configure loguru.

    - Colored console output (stderr)
    - Rotating file log in `log_dir/voice_chatbot_{time}.log`
    - Weekly rotation, 30 days retention
    """
    logger.remove()  # clear default handler

    logger.add(
        sys.stderr,
        format="<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | <level>{message}</level>",
        level=level,
        colorize=True,
    )

    log_path = Path(log_dir)
    log_path.mkdir(parents=True, exist_ok=True)
    logger.add(
        log_path / "voice_chatbot_{time:YYYY-MM-DD}.log",
        format="{time:YYYY-MM-DD HH:mm:ss.SSS} | {level: <8} | {name}:{function}:{line} | {message}",
        level="DEBUG",
        rotation="50 MB",
        retention="30 days",
        encoding="utf-8",
    )
