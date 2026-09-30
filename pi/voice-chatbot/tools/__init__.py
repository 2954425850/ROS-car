"""LLM 工具调用层。

registry.py —— 工具注册表与白名单分发
builtin.py  —— 内置工具（ask_user / get_current_time）
car.py      —— 小车控制工具（car_move / car_turn / car_stop / car_camera / car_status）
"""

from tools.registry import Tool, ToolError, ToolNotFound, ToolOutcome, ToolRegistry
from tools.builtin import build_default_registry

__all__ = [
    "Tool",
    "ToolError",
    "ToolNotFound",
    "ToolOutcome",
    "ToolRegistry",
    "build_default_registry",
]
