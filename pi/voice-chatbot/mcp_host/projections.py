"""MCP ↔ 助手 的投影层 —— 整个集成里唯一知道 "function calling" 存在的地方。

为什么要单独一层：MCP 是双向协议，不是一个「远程函数库」。把 MCP Tool 直接塞进
助手的 `Tool` dataclass（tools/registry.py）会在投影中丢掉一大半协议：

    resources / prompts / notifications(progress, logging) / tool annotations
    / outputSchema，以及整条 client→server 方向(elicitation / sampling / roots)。

所以 MCP 自己的类型（Tool / Annotations / Resource / Prompt）在 `mcp_host` 内部
保持原样，只在要把工具交给 LLM 的那一刻，由本模块做一次薄投影。别的地方不要
import 本模块 —— 哪天换了不靠 function calling 的模型，只需要改这里。

命名空间用 mcp__<server>__<tool>，与原生工具(get_current_time 等)零冲突，
而且从名字就能看出工具属于哪个 server。
"""

from __future__ import annotations

from typing import Any

PREFIX = "mcp__"
_SEP = "__"


def qualify(server: str, tool_name: str) -> str:
    """组装全局唯一的工具名。"""
    return f"{PREFIX}{server}{_SEP}{tool_name}"


def parse_qualified(name: str) -> tuple[str, str] | None:
    """把 mcp__<server>__<tool> 拆回 (server, tool)；不是 MCP 工具则返回 None。

    server 名和 tool 名都允许包含单个下划线（MCP 工具名本来就常见
    get_song_url 这种），所以用第一个 "__" 做分隔符。
    """
    if not name.startswith(PREFIX):
        return None
    server, sep, tool = name[len(PREFIX):].partition(_SEP)
    if not (sep and server and tool):
        return None
    return server, tool


def tool_to_schema(server: str, tool: Any) -> dict[str, Any]:
    """MCP Tool → OpenAI 兼容的 function schema。

    description 前缀 [server] 是为了让模型在多服务器场景下知道该找谁。
    inputSchema 的形状本来就是 type/properties/required，直接就是 function 的
    parameters，不需要转换。
    """
    description = (tool.description or "").strip()
    return {
        "type": "function",
        "function": {
            "name": qualify(server, tool.name),
            "description": f"[{server}] {description}" if description else f"[{server}] {tool.name}",
            "parameters": tool.inputSchema or {"type": "object", "properties": {}},
        },
    }


def is_destructive(tool: Any) -> bool:
    """MCP 的 destructiveHint。没有 annotations 时保守地当作「不危险」。

    annotations 是 MCP 独有的概念，助手的 Tool 里没有对应物 —— 这也是不能把
    MCP 工具压进 ToolRegistry 的原因之一。
    """
    annotations = getattr(tool, "annotations", None)
    if annotations is None:
        return False
    return bool(getattr(annotations, "destructiveHint", False))


def call_result_to_text(result: Any) -> str:
    """把 CallToolResult.content 拍平成一段文本。

    **不抛异常**：`isError=True` 也只把文本返回去，让模型自己兜底 ——
    与 `ToolRegistry.dispatch()` 的约定一致（见 tools/registry.py 的文档）。
    """
    chunks: list[str] = []
    for item in getattr(result, "content", None) or []:
        kind = getattr(item, "type", None)
        if kind == "text":
            chunks.append(getattr(item, "text", "") or "")
        elif kind == "resource":
            embedded = getattr(item, "resource", None)
            text = getattr(embedded, "text", None)
            chunks.append(text if text else f"[嵌入资源 {getattr(embedded, 'uri', '?')}]")
        else:
            # image / audio 之类在语音场景下没有意义，留个占位即可
            chunks.append(f"[{kind or '未知'} 内容]")

    text = "\n".join(c for c in chunks if c).strip()
    if not text:
        return "(工具返回空结果)"
    if getattr(result, "isError", False):
        return f"工具报错：{text}"
    return text


def resource_result_to_text(result: Any) -> str:
    """把 ReadResourceResult.contents 拍平成文本。"""
    chunks: list[str] = []
    for item in getattr(result, "contents", None) or []:
        text = getattr(item, "text", None)
        chunks.append(text if text else f"[二进制内容 {getattr(item, 'uri', '?')}]")
    return "\n".join(chunks).strip() or "(空资源)"


def prompt_result_to_text(result: Any) -> str:
    """把 GetPromptResult 拍平成文本。"""
    chunks: list[str] = []
    description = getattr(result, "description", None)
    if description:
        chunks.append(str(description))
    for message in getattr(result, "messages", None) or []:
        content = getattr(message, "content", None)
        text = getattr(content, "text", None)
        role = getattr(message, "role", "?")
        if text:
            chunks.append(f"{role}: {text}")
    return "\n".join(chunks).strip() or "(空提示)"
