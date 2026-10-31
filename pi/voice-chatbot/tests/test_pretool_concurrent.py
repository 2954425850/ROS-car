"""2026-10-31：开场白（「稍等」）对车/臂/抓取工具改成和动作并行，其它工具保持先念后做。"""

from __future__ import annotations

import threading
import time
import types

from llm.client import ToolCall
from tools.registry import Tool, ToolRegistry
from tests.test_arm_tools import conversation_mod  # noqa: F401  （复用 fixture）


class _LLM:
    """第 0 轮：带开场白 + 一批工具；第 1 轮：最终答复。"""

    def __init__(self, calls):
        self.calls = calls
        self.n = 0

    def _ev(self, **kw):
        return types.SimpleNamespace(tool_call=None, text="", **{k: v for k, v in kw.items()})

    def _stream(self):
        self.n += 1
        if self.n == 1:
            yield types.SimpleNamespace(type="text", text="好的", tool_call=None)
            for c in self.calls:
                yield types.SimpleNamespace(type="tool_call", text="", tool_call=c)
        else:
            yield types.SimpleNamespace(type="text", text="做好了", tool_call=None)

    def chat(self, _t):
        return self._stream()

    def chat_turn(self):
        return self._stream()


def _manager(conv, group):
    log = []
    reg = ToolRegistry()

    def handler(**_):
        log.append(("tool_start", time.monotonic()))
        time.sleep(0.05)
        return "ok"

    reg.register(Tool(name="t1", description="d", group=group,
                      parameters={"type": "object", "properties": {}}, handler=handler))
    cm = object.__new__(conv.ConversationManager)
    cm._registry = reg
    cm._llm = _LLM([ToolCall(id="1", name="t1", arguments={})])
    cm._max_tool_iterations = 3
    cm._pre_tool_filler = "稍等"
    cm._pending_response = ""
    cm._should_listen = False

    def speak(text):
        log.append(("say_start", time.monotonic()))
        time.sleep(0.3)
        log.append(("say_end", time.monotonic()))

    cm._speak = speak

    def execute(calls):
        for c in calls:
            reg.get(c.name).handler()
        log.append(("tools_done", time.monotonic()))
        return [], False

    cm._execute_tool_calls = execute
    return cm, log


def _t(log, k):
    return next(t for name, t in log if name == k)


def test_motion_tools_start_while_preamble_is_spoken(conversation_mod):
    for group in ("arm", "car", "grasp"):
        cm, log = _manager(conversation_mod, group)
        cm._run_agent_loop("低头")
        assert _t(log, "tool_start") < _t(log, "say_end"), group      # 没等念完就开始动
        # 往下走（下一轮 LLM / 最终答复）之前开场白必须已经念完
        assert cm._pending_response == "做好了"
        assert [n for n, _ in log].count("say_end") == 1


def test_other_tools_keep_say_then_do(conversation_mod):
    cm, log = _manager(conversation_mod, "core")
    cm._run_agent_loop("放首歌")
    assert _t(log, "say_end") <= _t(log, "tool_start")


def test_preamble_finished_before_loop_continues(conversation_mod):
    cm, log = _manager(conversation_mod, "arm")
    orig = cm._llm.chat_turn
    seen = {}

    def chat_turn():
        seen["say_done"] = any(n == "say_end" for n, _ in log)
        return orig()

    cm._llm.chat_turn = chat_turn
    cm._run_agent_loop("低头")
    assert seen["say_done"]
