"""机械臂语音动作工具：arm_act / arm_state / arm_home（设计 §6 定稿形状 B）。

schema **直接复用** `tools/arm_schema.py` 的 `SCHEMAS_B`（Task 1 评测定稿的那一份，不另写），
参数 → spec 用同文件的 `to_spec`。执行交给 `car/arm_client.py`（→ Pi 的 /arm/motion 等）。

- client 为 None 或没 enabled：工具照注册，调用时回统一降级文案（同 car/grasp 的做法，
  不让模型看到「没有这个工具」）。
- 每次调用追加一行 JSON 到 `logs/arm_motion.jsonl`（ts/tool/args/result/dur_s），给 P3
  "档位按反馈调 / 高频动作升级为预设"攒数据。**写日志失败不影响返回**。`logs/` 已 gitignore。
- 「何时调用」全在 description（arm_schema.py 里），**不改 system_prompt**。
"""

from __future__ import annotations

import copy
import json
import time
from pathlib import Path

from loguru import logger

from tools.arm_schema import SCHEMAS_B, STEP_FIELDS, to_spec
from tools.registry import Tool, ToolRegistry

DEFAULT_LOG_PATH = Path(__file__).resolve().parents[1] / "logs" / "arm_motion.jsonl"

UNAVAILABLE_PREFIX = "机械臂现在用不了"

_KNOWN_ACT_ARGS = set(STEP_FIELDS) | {"steps", "speed", "repeat"}


def unavailable_text(client) -> str:
    why = getattr(client, "reason", None) if client is not None else None
    return f"{UNAVAILABLE_PREFIX} —— {why or '客户端没起来（ROS 环境或 arm-grasp 服务不在）'}"


def _append_log(path: Path, record: dict) -> None:
    """追加一行；任何失败只记 warning，绝不外抛。"""
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
    except Exception as e:  # noqa: BLE001
        logger.warning(f"臂动作日志写不进去（{path}）：{e!r}")


def register_arm_tools(registry: ToolRegistry, client, log_path: str | Path | None = None) -> None:
    """登记 arm_act / arm_state / arm_home（group="arm"）。client 可以是 None。"""
    log_file = Path(log_path) if log_path is not None else DEFAULT_LOG_PATH
    if client is None or not getattr(client, "enabled", False):
        logger.warning(f"机械臂：客户端不可用，仍注册工具（调用时说明原因）—— "
                       f"{getattr(client, 'reason', None) if client is not None else 'None'}")

    def _ready() -> bool:
        return client is not None and bool(getattr(client, "enabled", False))

    def _act(args: dict) -> str:
        unknown = sorted(set(args) - _KNOWN_ACT_ARGS)
        try:
            spec = to_spec("arm_act", args)
        except ValueError as e:
            return f"参数不对，没动：{e}"
        if not spec.get("steps"):
            extra = f"（不认识的参数：{'、'.join(unknown)}）" if unknown else ""
            return ("没动：没说要怎么动" + extra + "。至少填 toward/turn/shift/approach/gripper/"
                    "oscillate 之一，或者用 steps。")
        if not _ready():
            return unavailable_text(client)
        return client.act(spec)

    def _state(args: dict) -> str:
        if not _ready():
            return unavailable_text(client)
        return client.describe()

    def _home(args: dict) -> str:
        if not _ready():
            return unavailable_text(client)
        return client.home()

    impl = {"arm_act": _act, "arm_state": _state, "arm_home": _home}

    def _make_handler(name: str):
        def _handler(**kwargs):
            t0 = time.monotonic()
            try:
                result = impl[name](dict(kwargs))
            except Exception as e:  # noqa: BLE001 —— 日志照写，再交给 dispatch 兜底
                _append_log(log_file, {"ts": time.time(), "tool": name, "args": kwargs,
                                       "error": repr(e),
                                       "dur_s": round(time.monotonic() - t0, 3)})
                raise
            _append_log(log_file, {"ts": time.time(), "tool": name, "args": kwargs,
                                   "result": result, "dur_s": round(time.monotonic() - t0, 3)})
            return result

        return _handler

    for schema in SCHEMAS_B:
        fn = copy.deepcopy(schema["function"])
        registry.register(Tool(
            name=fn["name"],
            group="arm",
            description=fn["description"],
            parameters=fn["parameters"],
            handler=_make_handler(fn["name"]),
        ))
