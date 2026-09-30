"""内置工具。

- ask_user         —— 取代旧的 {"continue": true} 标记：模型调它 = 问完这句继续听
- get_current_time —— 真实可离线验证的工具，用来端到端打通 function calling 链路

后续接小车控制（跟随/巡线/音乐等）时，在这里再写一个 `register_car_tools(registry)`，
handler 直接调用 09.AI_Big_Model 里的 Car_base_control / Car_music_api 等函数即可。
"""

from datetime import datetime

from loguru import logger

from tools.registry import Tool, ToolOutcome, ToolRegistry

_WEEKDAYS = "一二三四五六日"


def ask_user(question: str) -> ToolOutcome:
    """交互型工具：本身不做任何事，只声明「念这句话，然后继续听」。"""
    logger.info(f"ask_user: {question}")
    return ToolOutcome(
        result=f"已向用户提问「{question}」，等待用户回答。",
        speak_text=question,
        requires_followup=True,
    )


def get_current_time() -> str:
    """返回当前本地时间，交给模型组织成口语回答。"""
    now = datetime.now()
    return f"{now:%Y-%m-%d %H:%M:%S} 星期{_WEEKDAYS[now.weekday()]}"


def build_default_registry() -> ToolRegistry:
    """构建默认工具集。"""
    registry = ToolRegistry()

    registry.register(
        Tool(
            name="ask_user",
            description=(
                "向用户追问、确认或澄清信息时调用。"
                "只要还需要用户补充信息才能完成任务，就必须调用本工具提问，"
                "不要只在回复文字里问 —— 只有调用本工具，麦克风才会自动重新打开等待回答。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "question": {
                        "type": "string",
                        "description": "要问用户的问题，口语化中文，一句话说完。",
                    }
                },
                "required": ["question"],
            },
            handler=ask_user,
            speaks=True,
        )
    )

    registry.register(
        Tool(
            name="get_current_time",
            description="获取当前日期、时间和星期。用户问现在几点、今天几号、今天星期几时调用。",
            parameters={"type": "object", "properties": {}, "required": []},
            handler=get_current_time,
        )
    )

    return registry
