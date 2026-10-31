"""2026-10-31 实车：工具调用达到上限后，历史里留下没有 tool 结果的 assistant.tool_calls，
之后每次请求都 400 ⇒ 语音助手"卡死"。get_messages() 必须把它补齐。"""

from dataclasses import dataclass

from llm.context import UNANSWERED_TOOL_RESULT, ConversationContext


@dataclass
class _Call:
    id: str
    name: str
    arguments: dict


class _Cfg:
    def get(self, key, default=None):
        return {"llm.system_prompt": "sys"}.get(key, default)


def _ctx():
    return ConversationContext(_Cfg())


def _check_api_invariant(msgs):
    """OpenAI 兼容接口的约束：每个 tool_call id 都要在紧随其后的 tool 消息里被回应。"""
    i = 0
    while i < len(msgs):
        calls = msgs[i].get("tool_calls")
        i += 1
        if not calls:
            continue
        got = set()
        while i < len(msgs) and msgs[i]["role"] == "tool":
            got.add(msgs[i]["tool_call_id"])
            i += 1
        assert {c["id"] for c in calls} <= got


def test_cap_reached_leaves_no_dangling_tool_calls():
    c = _ctx()
    c.add_user_message("转一下然后看地上")
    c.add_assistant_message("", tool_calls=[_Call("a", "car_turn", {"angle_deg": 30})])
    c.add_tool_result("a", "ok")
    c.add_assistant_message("", tool_calls=[_Call("b", "arm_act", {}), _Call("c", "look", {})])
    # 达到上限：b、c 没执行
    c.add_user_message("往回转")
    msgs = c.get_messages()
    _check_api_invariant(msgs)
    stubs = [m for m in msgs if m.get("content") == UNANSWERED_TOOL_RESULT]
    assert [m["tool_call_id"] for m in stubs] == ["b", "c"]
    assert msgs[-1] == {"role": "user", "content": "往回转"}


def test_partially_answered_batch_gets_only_missing_stub():
    c = _ctx()
    c.add_user_message("x")
    c.add_assistant_message("", tool_calls=[_Call("a", "t", {}), _Call("b", "t", {})])
    c.add_tool_result("a", "ok")
    msgs = c.get_messages()
    _check_api_invariant(msgs)
    assert sum(1 for m in msgs if m["role"] == "tool") == 2


def test_complete_history_is_unchanged():
    c = _ctx()
    c.add_user_message("x")
    c.add_assistant_message("", tool_calls=[_Call("a", "t", {})])
    c.add_tool_result("a", "ok")
    c.add_assistant_message("好了")
    msgs = c.get_messages()
    assert not any(m.get("content") == UNANSWERED_TOOL_RESULT for m in msgs)
    assert len(msgs) == 5
