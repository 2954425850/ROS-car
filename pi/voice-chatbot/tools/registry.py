"""工具注册表 —— function calling 的白名单分发层。

设计要点：
  - 每个工具显式登记「名字 + JSON Schema + 处理函数」，模型只能调用登记过的名字。
    这是用来取代 09.AI_Big_Model 里 `eval(模型输出)` 的做法 —— 那条路径等于远程代码执行。
  - 处理函数返回 `str` 或 `ToolOutcome`。`ToolOutcome` 让工具能额外声明两件事：
      speak_text        —— 除了模型正文之外，还要念给用户听的话
      requires_followup —— 执行完要把话筒交还用户（自动重开麦克风）
    这样 `ask_user` 这类交互工具就不需要调用方去猜它参数叫什么名字。
  - `dispatch()` 永不抛异常：任何失败都变成一条错误字符串回灌给模型，
    让模型自己决定怎么兜底。调用方不必为工具错误写分支。
"""

from dataclasses import dataclass
from typing import Any, Callable, Iterable


class ToolError(Exception):
    """工具分发失败。"""


class ToolNotFound(ToolError):
    """模型调用了未登记的工具名（通常是幻觉）。"""


@dataclass(frozen=True)
class ToolOutcome:
    """一次工具执行的结果。

    Attributes:
        result: 写进 role="tool" 消息的内容，模型会看到。
        speak_text: 需要朗读给用户的文本（如追问的问题），没有则 None。
        requires_followup: True 表示执行后应自动重开麦克风等用户回答。
    """

    result: str
    speak_text: str | None = None
    requires_followup: bool = False

    @classmethod
    def coerce(cls, value: Any) -> "ToolOutcome":
        """把处理函数的返回值统一成 ToolOutcome。"""
        if isinstance(value, ToolOutcome):
            return value
        return cls(result=str(value))


@dataclass(frozen=True)
class Tool:
    """一个可供 LLM 调用的工具。

    Attributes:
        name: 工具名，全局唯一。
        description: 给模型看的说明，决定模型何时会调用它 —— 写得越具体越好。
        parameters: 参数的 JSON Schema（type/properties/required）。
        handler: 实际执行函数，接收关键字参数，返回 str 或 ToolOutcome。
        group: 能力分组。现在只有 "core" / "car"，全部都会发给模型；
               只是**预留**——将来工具过 20 个、开始稀释模型注意力时，
               按组过滤即可（见 schemas() 的 groups 参数），不用改数据结构。
        speaks: True = 这个工具自己会念话（通过 ToolOutcome.speak_text 带出来）。
                调用方据此避免在它之前再念开场白 —— 否则同一句话会被念两遍。
    """

    name: str
    description: str
    parameters: dict[str, Any]
    handler: Callable[..., Any]
    group: str = "core"
    speaks: bool = False

    def schema(self) -> dict[str, Any]:
        """转换成 OpenAI 兼容的 tools 数组元素。"""
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": self.parameters,
            },
        }


class ToolRegistry:
    """工具注册表。

    Usage:
        registry = ToolRegistry()
        registry.register(Tool(name="foo", ...))
        registry.dispatch("foo", {"bar": 1})
    """

    def __init__(self) -> None:
        self._tools: dict[str, Tool] = {}

    def register(self, tool: Tool) -> None:
        """登记一个工具。重名直接报错，避免静默覆盖。"""
        if tool.name in self._tools:
            raise ValueError(f"工具重复登记: {tool.name}")
        self._tools[tool.name] = tool

    def get(self, name: str) -> Tool:
        """按名字取工具，不存在则抛 ToolNotFound。"""
        try:
            return self._tools[name]
        except KeyError:
            raise ToolNotFound(f"未登记的工具: {name}") from None

    def schemas(self, groups: Iterable[str] | None = None) -> list[dict[str, Any]]:
        """返回可直接塞进 `chat.completions.create(tools=...)` 的列表。

        Args:
            groups: 只要这些分组的工具。
                **None = 全都要；空集合 = 一个都不要。** 两者语义不同，
                别拿空 list 当「没配置」传进来 —— 那会静默发出空的 tools 数组。
        """
        tools = self._tools.values()
        if groups is not None:
            wanted = set(groups)
            tools = [t for t in tools if t.group in wanted]
        return [tool.schema() for tool in tools]

    def dispatch(self, name: str, args: dict[str, Any]) -> ToolOutcome:
        """按名字执行工具。**永不抛异常**，失败信息作为 result 返回给模型。"""
        tool = self._tools.get(name)
        if tool is None:
            return ToolOutcome(result=f"工具调用失败：不存在名为 {name} 的工具。")

        try:
            return ToolOutcome.coerce(tool.handler(**args))
        except TypeError as e:
            return ToolOutcome(result=f"工具 {name} 参数不匹配：{e}")
        except Exception as e:
            return ToolOutcome(result=f"工具 {name} 执行失败：{e}")

    def __contains__(self, name: object) -> bool:
        return name in self._tools

    def __len__(self) -> int:
        return len(self._tools)
