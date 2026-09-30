"""对话上下文管理。

按「原子轮次」裁剪历史。一轮 = 一条 user 消息 + 其后所有 assistant / tool 消息
（可能包含 assistant.tool_calls → tool 结果 → 最终 assistant 的多步结构）。

为什么要按轮而不是按条裁剪：OpenAI 兼容接口要求 assistant 的 tool_calls 消息后面
必须紧跟对应的 tool 结果消息。如果按条数截断，可能把 tool 消息和它的父消息拆散，
接口会直接报错。按轮裁剪保证每一轮内部的消息序始终完整。
"""

import json
from typing import Any

from utils.config import Config


class ConversationContext:
    """维护对话历史，按轮次保留最近 N 轮。

    Usage:
        ctx = ConversationContext(config)
        ctx.add_user_message("你好")
        ctx.add_assistant_message("你好！")
        messages = ctx.get_messages()  # 可直接用于 API 调用
    """

    def __init__(self, config: Config):
        self._system_prompt = config.get("llm.system_prompt", "").strip()
        self._max_rounds = config.get("conversation.max_history_rounds", 20)
        self._history: list[dict[str, Any]] = []

    # ---- 写入 ----

    def add_user_message(self, text: str) -> None:
        """追加一条用户消息。"""
        self._history.append({"role": "user", "content": text})
        self._trim()

    def add_assistant_message(
        self, text: str, tool_calls: list[Any] | None = None
    ) -> None:
        """追加一条助手消息。

        Args:
            text: 助手回复的文字内容，工具调用轮可能为空字符串。
            tool_calls: ToolCall 列表。非空时会一并写入 OpenAI 的 tool_calls 字段。
        """
        message: dict[str, Any] = {"role": "assistant", "content": text}
        if tool_calls:
            message["tool_calls"] = [
                {
                    "id": call.id,
                    "type": "function",
                    "function": {
                        "name": call.name,
                        "arguments": _dumps_args(call.arguments),
                    },
                }
                for call in tool_calls
            ]
        self._history.append(message)
        self._trim()

    def add_tool_result(self, tool_call_id: str, content: str) -> None:
        """追加一条工具执行结果，必须紧跟在对应的 assistant.tool_calls 之后。"""
        self._history.append(
            {"role": "tool", "tool_call_id": tool_call_id, "content": content}
        )
        self._trim()

    # ---- 读取 ----

    def get_messages(self) -> list[dict[str, Any]]:
        """返回可直接发给 LLM 的消息列表：system + 最近 N 轮完整历史。"""
        messages: list[dict[str, Any]] = [
            {"role": "system", "content": self._system_prompt}
        ]
        for turn in self._turns():
            messages.extend(turn)
        return messages

    def reset(self) -> None:
        """清空全部对话历史。"""
        self._history.clear()

    # ---- 内部 ----

    def _turns(self) -> list[list[dict[str, Any]]]:
        """把扁平历史切成轮次，每轮以一条 user 消息开头。"""
        turns: list[list[dict[str, Any]]] = []
        for message in self._history:
            if message["role"] == "user":
                turns.append([message])
            elif turns:
                turns[-1].append(message)
            # 历史开头的孤立 assistant/tool 消息直接丢弃
        return turns

    def _trim(self) -> None:
        """超出窗口时从最旧的轮次整轮丢弃。"""
        turns = self._turns()
        if len(turns) <= self._max_rounds:
            return
        kept = turns[-self._max_rounds :]
        self._history = [message for turn in kept for message in turn]


def _dumps_args(args: dict[str, Any]) -> str:
    """把工具参数序列化成 API 要求的 JSON 字符串。"""
    return json.dumps(args, ensure_ascii=False)
