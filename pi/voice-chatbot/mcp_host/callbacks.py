"""MCP host 的四个回调 —— 协议里 client→server 的那一半。

MCP 不是「远程函数库」，服务器可以反过来找客户端：
    elicitation/create        向用户提问
    sampling/createMessage    借用客户端的 LLM
    roots/list                询问操作边界
    notifications/message     推日志

这四件事助手里本来就有对应物，这里把它们接上：
    elicitation → ask_user 的「念一句 → 重开麦克风」机制
    sampling    → 现有 LLMClient（但用独立 context，见 host 侧说明）
    roots       → 项目根 + ~/Music
    logging     → loguru

**能力的诚实声明是自然结果**：ClientSession 从「你传了哪些回调」推导
ClientCapabilities，没传的就不会被声明。所以没接 elicitation 就别传它，
而不是声明了却在回调里假装拒绝。
"""

from __future__ import annotations

import concurrent.futures
import queue
from dataclasses import dataclass, field
from typing import Any, Callable

from loguru import logger
from mcp import types

# MCP 的 8 档日志级别 → loguru 的级别名
_LOG_LEVELS = {
    "debug": "DEBUG",
    "info": "INFO",
    "notice": "INFO",
    "warning": "WARNING",
    "error": "ERROR",
    "critical": "CRITICAL",
    "alert": "CRITICAL",
    "emergency": "CRITICAL",
}


@dataclass
class HostBridge:
    """host 与助手之间的接线。

    每个字段为 None 表示「不支持该项能力」—— 对应的回调不会被传给
    ClientSession，能力也就不会被声明。这是刻意的：宁可不声明，也不要
    声明了却做不到。
    """

    elicit: Callable[[str, dict[str, Any]], dict[str, Any]] | None = None
    """语音追问。签名 (message, requested_schema) -> {"action","content"}。

    **必须由主线程实现**（要念话 + 开麦录音），因此不能直接被事件循环线程调用 ——
    见 ElicitationRelay。
    """

    sample: Callable[[list[dict[str, Any]], str | None, int], str] | None = None
    """借 LLM。签名 (messages, system_prompt, max_tokens) -> 文本。"""

    roots: Callable[[], list[tuple[str, str]]] | None = None
    """返回 [(绝对路径, 显示名)]。"""

    sample_model: str = "unknown"
    """sampling 结果里回报的 model 名 —— 协议要求这个字段。"""


class ElicitationRelay:
    """把「事件循环线程上的 elicitation 请求」转交主线程，并阻塞等答复。

    这是 elicitation 死锁问题的正面解法。死锁长这样：

        主线程 → _execute_tool_calls → portal.call(call_tool)   # 主线程在此阻塞
          └─ 事件循环线程发起 tools/call，服务器在调用中途发 elicitation/create
               └─ 回调跑在事件循环线程上
                    └─ 但「念话 + 开麦录音」必须回主线程（麦克风和 StateMachine 绑定）
                         └─ 主线程正阻塞等 call_tool 返回 → **互等**

    解法：主线程在等 call_tool 的同时**保持可服务**（pump）。事件循环线程把请求
    投进队列后阻塞在 Future 上，主线程取出、用语音问出答案、set_result 回灌。
    同一时刻只有一个 MCP 调用在飞，助手的串行设计不破。
    """

    def __init__(self, handler: Callable[[str, dict[str, Any]], dict[str, Any]]) -> None:
        self._handler = handler
        self._queue: queue.Queue = queue.Queue()

    # ---- 事件循环线程调用 ----

    def request(self, message: str, schema: dict[str, Any], timeout: float) -> dict[str, Any]:
        """阻塞直到主线程给出答复。超时则当作取消。"""
        future: concurrent.futures.Future = concurrent.futures.Future()
        self._queue.put((future, message, schema))
        try:
            return future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            logger.warning(f"Elicitation: 等待用户答复超时（{timeout:.0f}s），按取消处理")
            return {"action": "cancel", "content": None}

    # ---- 主线程调用 ----

    def pump_once(self, timeout: float) -> bool:
        """主线程在等 MCP 工具返回时反复调用本方法。处理掉一个请求返回 True。"""
        try:
            item = self._queue.get(timeout=timeout)
        except queue.Empty:
            return False

        future, message, schema = item
        try:
            future.set_result(self._handler(message, schema))
        except BaseException as exc:  # noqa: BLE001 —— 必须回灌，否则事件循环线程永久挂住
            logger.error(f"Elicitation: 语音追问失败 —— {exc}")
            future.set_exception(exc)
        return True


# ---------------------------------------------------------------- 回调工厂


def make_session_callbacks(
    bridge: HostBridge,
    server_name: str,
    relay: ElicitationRelay | None,
) -> dict[str, Any]:
    """按 bridge 实际支持的能力，组装传给 ClientSession 的回调。

    返回的 dict 直接展开给 ClientSession(...)。只包含真正实现了的能力 ——
    ClientSession 会据此推导 ClientCapabilities。
    """
    callbacks: dict[str, Any] = {"logging_callback": _make_logging(server_name)}

    if relay is not None:
        callbacks["elicitation_callback"] = _make_elicitation(server_name, relay)
    if bridge.sample is not None:
        callbacks["sampling_callback"] = _make_sampling(server_name, bridge)
    if bridge.roots is not None:
        callbacks["list_roots_callback"] = _make_roots(server_name, bridge.roots)

    return callbacks


def _make_elicitation(server_name: str, relay: ElicitationRelay):
    """全在事件循环线程上执行 —— 真正干活的是 relay（转交主线程）。"""

    async def callback(_context: Any, params: types.ElicitRequestParams) -> Any:
        mode = getattr(params, "mode", "form")

        if mode == "url":
            # URL 模式（让用户去浏览器打开链接）在纯语音场景下没有意义。
            # 明确拒绝，好过假装支持然后卡住用户。
            logger.info(
                f"MCP[{server_name}]: 收到 URL 模式 elicitation，"
                f"语音场景不支持，已拒绝（{getattr(params, 'url', '?')}）"
            )
            return types.ElicitResult(action="decline")

        message = getattr(params, "message", "") or ""
        schema = getattr(params, "requestedSchema", None) or {}
        logger.info(f"MCP[{server_name}]: 发起 elicitation —— {message}")

        # 超时给得比助手自己的无语音超时宽一点，让助手先放弃
        answer = relay.request(message, schema, timeout=120.0)
        action = answer.get("action", "decline")
        content = answer.get("content")
        logger.info(f"MCP[{server_name}]: elicitation 答复 action={action} content={content}")
        return types.ElicitResult(action=action, content=content)

    return callback


def _make_sampling(server_name: str, bridge: HostBridge):
    """服务器借宿主的 LLM。

    注意：这条路径会在一次 LLM 回合**中途**回到 LLM —— 重入。bridge.sample 的实现
    必须用独立的 ConversationContext，绝不能碰主对话历史，否则上下文会被污染。
    """

    async def callback(_context: Any, params: types.CreateMessageRequestParams) -> Any:
        messages = [
            {"role": m.role, "content": _sampling_content_to_text(m.content)}
            for m in (getattr(params, "messages", None) or [])
        ]
        system_prompt = getattr(params, "systemPrompt", None)
        max_tokens = getattr(params, "maxTokens", None) or 512

        logger.info(f"MCP[{server_name}]: 请求 sampling（{len(messages)} 条消息, max_tokens={max_tokens}）")
        try:
            text = bridge.sample(messages, system_prompt, max_tokens)  # type: ignore[misc]
        except Exception as exc:  # noqa: BLE001
            logger.error(f"MCP[{server_name}]: sampling 失败 —— {exc}")
            return types.ErrorData(code=types.INTERNAL_ERROR, message=f"sampling 失败: {exc}")

        return types.CreateMessageResult(
            role="assistant",
            content=types.TextContent(type="text", text=text or ""),
            model=bridge.sample_model,
            stopReason="endTurn",
        )

    return callback


def _make_roots(server_name: str, provider: Callable[[], list[tuple[str, str]]]):
    async def callback(_context: Any) -> Any:
        roots = []
        for path, name in provider():
            try:
                roots.append(types.Root(uri=f"file://{path}", name=name))  # type: ignore[arg-type]
            except Exception as exc:  # noqa: BLE001
                logger.warning(f"MCP[{server_name}]: root {path} 无效，跳过 —— {exc}")
        logger.debug(f"MCP[{server_name}]: 上报 {len(roots)} 个 roots")
        return types.ListRootsResult(roots=roots)

    return callback


def _make_logging(server_name: str):
    """服务器推来的日志转进 loguru。这条不声明任何能力（logging 是服务器侧能力）。"""

    async def callback(params: types.LoggingMessageNotificationParams) -> None:
        level = _LOG_LEVELS.get(getattr(params, "level", "info"), "INFO")
        data = getattr(params, "data", "")
        origin = getattr(params, "logger", None)
        prefix = f"MCP[{server_name}]" + (f".{origin}" if origin else "")
        logger.log(level, f"{prefix}: {data}")

    return callback


def _sampling_content_to_text(content: Any) -> str:
    """SamplingMessage.content 可能是单块，也可能是数组，还可能带图片/工具块。"""
    if isinstance(content, list):
        return "\n".join(filter(None, (_sampling_content_to_text(c) for c in content)))
    text = getattr(content, "text", None)
    if text:
        return str(text)
    kind = getattr(content, "type", None) or type(content).__name__
    return f"[{kind}]"
