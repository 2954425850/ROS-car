"""LLM 客户端 —— DeepSeek API（OpenAI 兼容接口），支持流式 function calling。

流式工具调用的关键：开启 `stream=True` 后，模型返回的 tool_calls 是**分片**到达的 ——
`delta.tool_calls[i]` 只带一个片段，其中 `function.arguments` 是一段**未闭合的 JSON 字符串**。
必须按 `index` 归并到同一个槽位、把 arguments 拼起来，等流结束后再整体 json.loads。
中途对分片做 json.loads 一定会失败，这是最常见的踩坑点。

参数拼接用「双缓冲」：同时记录增量拼接结果和最后一片快照。标准 OpenAI 协议是增量模式
（用前者即可），但少数网关是累积重发模式（此时前者永远解析不了，退回到后者）。两个都留，
哪个能解析用哪个 —— 比用启发式猜模式更稳。
"""

import json
from dataclasses import dataclass
from typing import Any, Callable, Iterator, Literal

from loguru import logger
from openai import BadRequestError, OpenAI

from llm.context import ConversationContext
from tools.registry import ToolRegistry
from utils.config import Config


@dataclass(frozen=True)
class ToolCall:
    """模型请求调用的一次工具。"""

    id: str
    name: str
    arguments: dict[str, Any]


@dataclass(frozen=True)
class LLMEvent:
    """流式输出的一个事件。

    type == "text"      → text 是本轮新增的文字片段
    type == "tool_call" → tool_call 是一个已解析完成的工具调用
    type == "done"      → 本轮结束，不再有事件
    """

    type: Literal["text", "tool_call", "done"]
    text: str = ""
    tool_call: ToolCall | None = None


class LLMClient:
    """带工具能力的流式对话客户端。

    Usage:
        for event in client.chat("现在几点了"):
            if event.type == "text":
                ...
            elif event.type == "tool_call":
                ...
    """

    def __init__(
        self,
        config: Config,
        context: ConversationContext,
        registry: ToolRegistry | None = None,
        extra_schemas: Callable[[], list[dict[str, Any]]] | None = None,
    ):
        self._openai = OpenAI(
            api_key=config.get("llm.api_key"),
            base_url=config.get("llm.base_url", "https://api.deepseek.com"),
        )
        self._model = config.get("llm.model", "deepseek-v4-flash")
        self._max_tokens = config.get("llm.max_tokens", 2048)
        self._temperature = config.get("llm.temperature", 0.7)
        self._context = context
        self._registry = registry if registry is not None else ToolRegistry()
        self._tools_enabled = config.get("llm.tools_enabled", True)
        # MCP 工具不塞进 ToolRegistry（会丢掉 server 归属和 annotations），
        # 而是由调用方提供一个返回「已投影 schema」的可调用对象。见 mcp_host/。
        self._extra_schemas = extra_schemas

    def chat(self, text: str) -> Iterator[LLMEvent]:
        """发送一条用户消息并流式返回本轮结果。"""
        self._context.add_user_message(text)
        yield from self.chat_turn()

    def chat_turn(self) -> Iterator[LLMEvent]:
        """不追加用户消息，直接基于当前上下文请求一轮（用于工具执行后的续跑）。"""
        messages = self._context.get_messages()
        tools = self._collect_tools()
        use_tools = self._tools_enabled and len(tools) > 0
        logger.info(
            f"LLM: sending {len(messages)} messages to {self._model} "
            f"(tools={len(tools) if use_tools else 0})"
        )

        kwargs: dict[str, Any] = {
            "model": self._model,
            "messages": messages,
            "max_tokens": self._max_tokens,
            "temperature": self._temperature,
            "stream": True,
        }
        if use_tools:
            kwargs["tools"] = tools

        full_text = ""
        pending: dict[int, dict[str, str]] = {}

        try:
            response = self._openai.chat.completions.create(**kwargs)
            for chunk in response:
                if not chunk.choices:
                    continue  # 只带 usage 的收尾 chunk
                delta = chunk.choices[0].delta

                if delta.content:
                    full_text += delta.content
                    yield LLMEvent(type="text", text=delta.content)

                for fragment in delta.tool_calls or []:
                    function = fragment.function
                    # 某些网关会在结尾补一个空壳 chunk，跳过它避免造出幽灵工具调用
                    if not fragment.id and function is None:
                        continue
                    slot = pending.setdefault(
                        fragment.index,
                        {"id": "", "name": "", "append": "", "last": ""},
                    )
                    if fragment.id:
                        slot["id"] = fragment.id
                    if function is not None:
                        if function.name:
                            slot["name"] = function.name
                        if function.arguments is not None:
                            slot["append"] += function.arguments
                            slot["last"] = function.arguments
        except BadRequestError as e:
            if use_tools and _mentions_tools(e):
                logger.error(
                    f"LLM: 模型 {self._model} 似乎不支持 function calling —— "
                    f"请换一个支持工具的模型，或把 llm.tools_enabled 设为 false。"
                    f"原始错误: {e}"
                )
            else:
                logger.error(f"LLM: API error — {e}")
            yield LLMEvent(type="done")
            return
        except Exception as e:
            logger.error(f"LLM: API error — {e}")
            yield LLMEvent(type="done")
            return

        tool_calls = self._parse_tool_calls(pending)

        # 空回复（既没文字也没工具调用）不要写进历史，否则会污染后续请求
        if full_text.strip() or tool_calls:
            self._context.add_assistant_message(full_text, tool_calls=tool_calls or None)

        for call in tool_calls:
            yield LLMEvent(type="tool_call", tool_call=call)
        yield LLMEvent(type="done")

    def _collect_tools(self) -> list[dict[str, Any]]:
        """原生工具 + MCP 工具。两者的 schema 形状本来就一致，直接拼。"""
        tools = self._registry.schemas()
        if self._extra_schemas is not None:
            try:
                tools.extend(self._extra_schemas())
            except Exception as e:  # noqa: BLE001 —— 工具表拿不到不该拖垮整个对话
                logger.warning(f"LLM: 获取 MCP 工具 schema 失败 —— {e}")
        return tools

    def complete(
        self,
        messages: list[dict[str, Any]],
        system_prompt: str | None = None,
        max_tokens: int | None = None,
    ) -> str:
        """一次性（非流式）补全，**不带工具**。

        专给 MCP sampling 用：服务器借用宿主的 LLM。刻意不碰 self._context ——
        sampling 是在一次 LLM 回合**中途**被触发的（重入），共用主上下文会把
        宿主的对话历史污染掉。也因此这里不注册工具，避免无限递归。
        """
        payload: list[dict[str, Any]] = []
        if system_prompt:
            payload.append({"role": "system", "content": system_prompt})
        payload.extend(messages)

        response = self._openai.chat.completions.create(
            model=self._model,
            messages=payload,  # type: ignore[arg-type]
            max_tokens=max_tokens or self._max_tokens,
            temperature=self._temperature,
            stream=False,
        )
        return (response.choices[0].message.content or "").strip()

    @staticmethod
    def _parse_tool_calls(pending: dict[int, dict[str, str]]) -> list[ToolCall]:
        """把累积好的分片解析成 ToolCall。坏 JSON 直接丢弃并记日志，不往上抛。"""
        calls: list[ToolCall] = []
        for index in sorted(pending):
            slot = pending[index]
            name = slot["name"]
            if not name:
                logger.error(f"LLM: 收到无名工具调用，已丢弃: {slot!r}")
                continue

            arguments = _parse_arguments(name, slot["append"], slot["last"])
            if arguments is None:
                continue

            calls.append(
                ToolCall(
                    id=slot["id"] or f"call_{index}",
                    name=name,
                    arguments=arguments,
                )
            )
        return calls


def _parse_arguments(name: str, incremental: str, cumulative: str) -> dict[str, Any] | None:
    """解析工具参数。先按增量模式解，失败再按累积重发模式解。"""
    for label, raw in (("增量", incremental), ("累积", cumulative)):
        text = raw.strip()
        if not text:
            continue
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            return parsed
        logger.error(f"LLM: 工具 {name} 的参数不是对象（{label}模式），已丢弃: {parsed!r}")
        return None

    logger.error(
        f"LLM: 工具 {name} 的参数无法解析为 JSON，已丢弃; "
        f"incremental={incremental!r} cumulative={cumulative!r}"
    )
    return None


def _mentions_tools(error: Exception) -> bool:
    """判断 400 是否与工具能力相关，用于给出可操作的提示。"""
    text = str(error).lower()
    return "tool" in text or "function" in text
