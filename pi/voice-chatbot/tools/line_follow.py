"""循迹语音入口：line_follow(enable) → /line_follow/enable（std_srvs/SetBool）。

定位：**一句话的开关**。它不启动循迹节点、不碰运动学、不发 cmd_vel。
「何时调用」全在 description（同 tools/car.py 的原则：不往 system_prompt 里加能力提示）。
client 不可用时工具照注册，调用时回一句原因，模型不用猜。

删除成本：删本文件 + `conversation.py` 里两行注册 + `car/line_follow_client.py`。
循迹节点（`pi/scripts/line_follow.py`）不受影响，仍可手动 `--drive` 用。
"""

from __future__ import annotations

from loguru import logger

from tools.registry import Tool, ToolRegistry

UNAVAILABLE_PREFIX = "循迹现在用不了"


def register_line_follow_tools(registry: ToolRegistry, client) -> None:
    """登记 line_follow（group="car"）。client 可以是 None。"""
    if client is None or not getattr(client, "enabled", False):
        logger.warning(
            "循迹：客户端不可用，仍注册工具（调用时说明原因）—— "
            f"{getattr(client, 'reason', None) if client is not None else 'None'}")

    def _handler(**kwargs) -> str:
        enable = kwargs.get("enable")
        if not isinstance(enable, bool):
            return "参数不对：enable 必须是 true/false（开始传 true，停止传 false）"
        if client is None or not getattr(client, "enabled", False):
            why = getattr(client, "reason", None) if client is not None else "客户端没起来"
            return f"{UNAVAILABLE_PREFIX} —— {why}"
        return client.set_enabled(enable)

    registry.register(Tool(
        name="line_follow",
        description=(
            "开始或停止小车沿黑线自动行驶（循迹）。"
            "用户说「开始循迹」「沿着线走」「跟着线跑」时传 enable=true；"
            "说「停止循迹」「别沿着线走了」「结束循迹」时传 enable=false。"
            "开始前车必须已经放在线上；开始后如果用户又要走动/转向，"
            "先传 false 停掉循迹，再用车的移动工具。"),
        parameters={
            "type": "object",
            "properties": {
                "enable": {
                    "type": "boolean",
                    "description": "true=开始循迹，false=停止",
                }
            },
            "required": ["enable"],
        },
        handler=_handler,
        group="car",
    ))
