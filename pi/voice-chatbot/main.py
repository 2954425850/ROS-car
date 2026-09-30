#!/usr/bin/env python3
"""JARVIS Voice Chatbot — main entry point.

Usage:
    python main.py                    # Run with default config.yaml
    python main.py --config my.yaml   # Run with custom config
"""

import argparse
import signal
import sys
from pathlib import Path

from loguru import logger

from utils.config import Config
from utils.logger import setup_logger
from core.conversation import ConversationManager


def main() -> None:
    parser = argparse.ArgumentParser(description="JARVIS Voice Chatbot")
    parser.add_argument(
        "--config",
        default="config.yaml",
        help="Path to configuration file (default: config.yaml)",
    )
    args = parser.parse_args()

    # Load config
    config_path = Path(args.config)
    if not config_path.exists():
        print(f"Error: config file not found: {config_path}", file=sys.stderr)
        sys.exit(1)

    config = Config(str(config_path))

    # Setup logging
    setup_logger(level="INFO")

    # Create and run manager
    manager = ConversationManager(config)

    # Handle graceful shutdown on Ctrl+C
    def _signal_handler(signum, frame):
        manager.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        manager.run()
    except Exception as e:
        # 启动期最常见的失败是唤醒模块串口打不开（没插好 / 权限不足 / 端口名不对）。
        # 换掉原始 traceback，直接给一句能照着做的提示。
        logger.error(f"启动失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
