#!/usr/bin/env python3
"""carhealthctl：status / doctor / history / why。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .engine import default_journal_dir, default_state_dir
from .journal import Journal

STATE_STALE_S = 10.0


def parse_since(text: str) -> int:
    unit = text[-1]
    value = int(text[:-1])
    return value * {"m": 60, "h": 3600, "d": 86400}[unit]


def _load_state(state_dir: Path) -> dict | None:
    try:
        return json.loads((Path(state_dir) / "state.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def cmd_status(args) -> int:
    payload = _load_state(args.state_dir)
    now = float(args.now) if args.now else time.time()
    if payload is None:
        print("守护未运行：找不到 %s/state.json" % args.state_dir)
        return 2
    age = now - float(payload.get("wall_ts", 0))
    if age > STATE_STALE_S:
        print("守护未运行：状态文件已过期 %.0fs（%s）" % (age, args.state_dir))
        return 2
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0

    busy = payload.get("busy", {})
    print("carhealthd pid=%s  wall=%s  状态年龄=%.0fs"
          % (payload.get("pid"), payload.get("wall"), age))
    print("互锁: 臂=%s 底盘=%s 抓取=%s 语音=%s"
          % (busy.get("arm"), busy.get("chassis"), busy.get("grasp"), busy.get("voice")))
    print()
    print("%-12s %-9s %-6s %-14s %s" % ("组件", "状态", "持续", "根因", "原因"))
    for name, c in payload.get("components", {}).items():
        print("%-12s %-9s %-6ss %-14s %s" % (
            name, c.get("verdict"), c.get("since_s", 0),
            c.get("root_cause") or "-", c.get("reason", "")))
        for d in c.get("diagnoses") or []:
            print("    → [%s] %s：%s" % (d.get("action_class"), d.get("title"),
                                        d.get("hint")))
    globals_ = payload.get("diagnoses_global") or []
    if globals_:
        print()
        print("全局结论:")
        for d in globals_:
            print("  → [%s] %s：%s" % (d.get("action_class"), d.get("title"),
                                      d.get("hint")))
    return 0


def cmd_history(args) -> int:
    journal = Journal(args.journal_dir)
    if args.since in (None, "all"):
        events = journal.read()
    else:
        events = journal.read_since(float(parse_since(args.since)))
    if not events:
        print("没有记录")
        return 0
    for e in events:
        print("%s  %-16s %-12s %s->%s  %s" % (
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(e.get("ts", 0))),
            e.get("event", "?"), e.get("component", "-"),
            e.get("from", "-"), e.get("to", "-"),
            e.get("reason", e.get("root_cause", ""))))
    return 0


def cmd_why(args) -> int:
    journal = Journal(args.journal_dir)
    events = [e for e in journal.read()
              if e.get("component") == args.component]
    if not events:
        print("没有记录：%s" % args.component)
        return 1
    for e in events[-5:]:
        print(json.dumps(e, ensure_ascii=False, indent=2))
    return 0


def _level(verdict) -> tuple[str, int]:
    value = getattr(verdict, "value", verdict)
    if value == "OK":
        return "PASS", 0
    if value in ("DEGRADED", "UNKNOWN"):
        return "WARN", 1
    return "FAIL", 2


def _read_fresh_state(state_dir):
    payload = _load_state(state_dir)
    if payload is None:
        return None
    if time.time() - float(payload.get("wall_ts", 0)) > STATE_STALE_S:
        return None
    return payload


def _print_daemon_state(payload) -> int:
    print("（使用运行中的 carhealthd 状态，未自建 ROS 节点）")
    worst = 0
    print("%-12s %-6s %s" % ("组件", "结果", "说明"))
    for name, c in payload.get("components", {}).items():
        level, code = _level(c.get("verdict"))
        worst = max(worst, code)
        print("%-12s %-6s %s%s" % (
            name, level, c.get("reason", ""),
            "" if not c.get("root_cause") else "（根因 %s）" % c["root_cause"]))
        for d in c.get("diagnoses") or []:
            print("    → [%s] %s：%s" % (d.get("action_class"),
                                        d.get("title"), d.get("hint")))
    for d in payload.get("diagnoses_global") or []:
        print("    → [%s] %s：%s" % (d.get("action_class"),
                                    d.get("title"), d.get("hint")))
    print()
    print({0: "总体: PASS", 1: "总体: WARN", 2: "总体: FAIL"}[worst])
    return worst


def cmd_doctor(args, engine_factory=None, sleeper=time.sleep) -> int:
    import tempfile

    from .registry import build_engine

    # 守护在跑且状态新鲜：直接读它的结果。FastDDS 在这台 Pi 上发现慢，
    # 自己起节点等几秒容易误判 STALE。
    if not getattr(args, "no_daemon", False):
        payload = _read_fresh_state(args.state_dir)
        if payload is not None:
            return _print_daemon_state(payload)

    # 自建探针时用临时目录，**绝不能覆盖常驻守护的 state.json / journal**。
    with tempfile.TemporaryDirectory(prefix="carhealth-doctor-") as td:
        args.state_dir = td
        args.journal_dir = td

        if engine_factory is None:
            def engine_factory(a):
                return build_engine(state_dir=a.state_dir, journal_dir=a.journal_dir,
                                    ros=not a.no_ros, board_host=a.board_host,
                                    fake_throttled=a.fake_throttled)

        engine = engine_factory(args)
        try:
            if not args.no_ros:
                sleeper(float(getattr(args, "warmup", 8.0)))
            states = engine.run_once()
        finally:
            engine.close()

        worst = 0
        print("%-12s %-6s %s" % ("组件", "结果", "说明"))
        for name, st in states.items():
            level, code = _level(st.verdict)
            worst = max(worst, code)
            print("%-12s %-6s %s%s" % (
                name, level, st.reason,
                "" if st.root_cause is None else "（根因 %s）" % st.root_cause))
            for d in st.diagnoses:
                print("    → [%s] %s：%s" % (d.action_class, d.title, d.hint))
        for d in getattr(engine, "global_diagnoses", ()):
            print("    → [%s] %s：%s" % (d.action_class, d.title, d.hint))
        print()
        print({0: "总体: PASS", 1: "总体: WARN", 2: "总体: FAIL"}[worst])
        return worst


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="carhealthctl")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("status", help="表格 / JSON 查看当前健康")
    s.add_argument("--state-dir", default=str(default_state_dir()))
    s.add_argument("--json", action="store_true")
    s.add_argument("--now", default=None, help="测试用：覆盖当前时间戳")
    s.set_defaults(func=cmd_status)

    h = sub.add_parser("history", help="查看故障时间线")
    h.add_argument("--journal-dir", default=str(default_journal_dir()))
    h.add_argument("--since", default="1h", help="1h / 30m / 2d / all")
    h.set_defaults(func=cmd_history)

    w = sub.add_parser("why", help="某组件最近一次异常的完整证据")
    w.add_argument("component")
    w.add_argument("--journal-dir", default=str(default_journal_dir()))
    w.set_defaults(func=cmd_why)

    d = sub.add_parser("doctor", help="出车体检")
    d.add_argument("--state-dir", default=str(default_state_dir()))
    d.add_argument("--no-daemon", action="store_true",
                   help="不读运行中守护的状态，强制自建探针")
    d.add_argument("--warmup", type=float, default=8.0,
                   help="自建 ROS 节点时等发现的时间（秒）")
    d.add_argument("--no-ros", action="store_true")
    d.add_argument("--board-host", default=None)
    d.add_argument("--fake-throttled", default=None,
                   help="测试/验收用：伪造 vcgencmd 输出，如 0x50000")
    d.set_defaults(func=cmd_doctor)

    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
