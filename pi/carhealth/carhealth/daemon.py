#!/usr/bin/env python3
"""carhealthd：常驻健康守护。一期只观察、只记录。"""

from __future__ import annotations

import argparse
import signal
import sys
import threading

from .engine import default_journal_dir, default_state_dir
from .registry import build_engine


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="整车健康守护（一期：只观察）")
    p.add_argument("--state-dir", default=str(default_state_dir()))
    p.add_argument("--journal-dir", default=None)
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--once", action="store_true", help="跑一轮就退出（验收用）")
    p.add_argument("--no-ros", action="store_true", help="不启动 rclpy 后端")
    return p.parse_args(argv)


def _loop(engine, interval_s: float, stop) -> None:
    while not stop.is_set():
        engine.run_once()
        if stop.wait(interval_s):
            break


def _default_factory(args):
    return build_engine(state_dir=args.state_dir,
                        journal_dir=args.journal_dir or str(default_journal_dir()),
                        ros=not args.no_ros,
                        interval_s=args.interval)


def main(argv=None, engine_factory=None) -> int:
    args = parse_args(argv)
    engine = (engine_factory or _default_factory)(args)

    stop = threading.Event()

    def _stop(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        if args.once:
            engine.run_once()
            return 0
        _loop(engine, args.interval, stop)
        return 0
    finally:
        engine.close()


if __name__ == "__main__":
    sys.exit(main())
