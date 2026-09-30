"""`follow_person` / `follow_stop` 工具注册测试 —— 只查 schema 形状与接线。

target 用替身：要么回一句话，要么抛 `TargetError`。
真实客户端（跑子进程）的测试在 `tests/test_k230_follow.py`。
"""

import pytest

from tools.follow import register_follow_tools
from tools.registry import ToolRegistry
from vision.k230_follow import TargetError

OK_MSG = "ok"
# 技术口吻的失败消息，照 vision/k230_follow.py 里真实会抛的样子写。
NO_REPLY = "板子 15 秒没回话（k230ctl follow）"
CTL_FAIL = "k230ctl follow 失败：TARGET FAILED: 连不上 K230 192.168.1.112:8557"
BOARD_ERR = "k230ctl follow 失败：err: who must be [A-Za-z0-9_-]"

_UNSET = object()


class _StubConfig:
    """只实现 `Config.get(dotted_key, default)` 的形状（照 test_vision_tools）。"""

    def __init__(self, data=None):
        self._data = data or {}

    def get(self, key, default=None):
        node = self._data
        for part in key.split("."):
            if isinstance(node, dict) and part in node:
                node = node[part]
            else:
                return default
        return node


class _StubTarget:
    def __init__(self, answer=OK_MSG, error=None):
        self.calls = []
        self._answer = answer
        self._error = error

    def follow(self, name):
        self.calls.append(("follow", name))
        if self._error is not None:
            raise self._error
        return self._answer

    def stop(self):
        self.calls.append(("stop",))
        if self._error is not None:
            raise self._error
        return self._answer


def _registry(target=_UNSET, config=None):
    registry = ToolRegistry()
    register_follow_tools(registry,
                          _StubTarget() if target is _UNSET else target,
                          config=config)
    return registry


def _schema(registry, name):
    for s in registry.schemas():
        if s["function"]["name"] == name:
            return s["function"]
    raise AssertionError(f"{name} 没注册")


# ---------------------------------------------------------------- 形状

def test_follow_person_registered_with_expected_shape():
    f = _schema(_registry(), "follow_person")
    assert f["parameters"]["required"] == ["name"]
    assert f["parameters"]["properties"]["name"]["type"] == "string"


def test_follow_stop_registered_with_no_params():
    f = _schema(_registry(), "follow_stop")
    assert f["parameters"]["properties"] == {}
    assert f["parameters"]["required"] == []


def test_both_tools_are_in_follow_group():
    r = _registry()
    assert {s["function"]["name"] for s in r.schemas()} == {"follow_person",
                                                            "follow_stop"}


# ---------------------------------------------------------------- 描述守则

def test_description_separates_follow_from_look_and_camera():
    """★ 「何时使用」只能由 description 承担（不动 system_prompt）——
    而且必须把三个容易混的工具划清界限，别把它写没了。"""
    desc = _schema(_registry(), "follow_person")["description"]
    assert "look" in desc and "car_camera" in desc


def test_description_warns_about_waiting_for_face():
    """★ 这个动作要等人露正脸才锁得上 —— 不说清楚，模型会以为它即时生效。"""
    desc = _schema(_registry(), "follow_person")["description"]
    assert "正对镜头" in desc or "露脸" in desc or "露正脸" in desc


def test_stop_description_distinguishes_from_car_stop():
    desc = _schema(_registry(), "follow_stop")["description"]
    assert "car_stop" in desc


# ---------------------------------------------------------------- 接线

def test_follow_passes_name_through():
    t = _StubTarget()
    r = _registry(target=t)
    out = r.dispatch("follow_person", {"name": "张三"})
    assert t.calls == [("follow", "张三")]
    assert out.result == OK_MSG


def test_stop_calls_stop():
    t = _StubTarget()
    r = _registry(target=t)
    out = r.dispatch("follow_stop", {})
    assert t.calls == [("stop",)]
    assert out.result == OK_MSG


# ---------------------------------------------------------------- 失败 → 人话

@pytest.mark.parametrize("msg", [NO_REPLY, CTL_FAIL, BOARD_ERR])
def test_target_error_becomes_human_text(msg):
    """客户端抛的是技术口吻；给模型念的必须是人话，且**不许出现异常名/Traceback**。"""
    r = _registry(target=_StubTarget(error=TargetError(msg)))
    out = r.dispatch("follow_person", {"name": "张三"})
    assert out.result
    assert "TargetError" not in out.result
    assert "Traceback" not in out.result


def test_no_reply_mentions_possible_offline():
    r = _registry(target=_StubTarget(error=TargetError(NO_REPLY)))
    out = r.dispatch("follow_person", {"name": "张三"})
    assert "回话" in out.result or "没连上" in out.result


def test_unavailable_target_says_why_not_missing_tool():
    """target=None 时仍注册，调用回一句解释 —— 别让模型看到「没有这个工具」。"""
    r = _registry(target=None)
    out = r.dispatch("follow_person", {"name": "张三"})
    assert "不可用" in out.result or "跟不了" in out.result


def test_missing_name_goes_through_dispatch_as_param_error():
    """参数漏传 → 交给 registry.dispatch 报「参数不匹配」，别在这里硬翻。"""
    out = _registry().dispatch("follow_person", {})
    assert "参数不匹配" in out.result


def test_real_client_accepts_the_tool_param_name():
    """★ 工具的 `name` 和**真客户端**的参数名必须一致。

    `_guard` 是把 kwargs **直通**给客户端的（`getattr(target, m)(**kwargs)`），
    名字对不上就变成"参数不匹配"—— 这个 bug 真的发生过一次（客户端原来叫 who）。

    这条必须用**真客户端**：上面那些用替身的测试抓不到它，
    因为替身的签名是我们自己按工具的形状写的，永远"对得上"。
    （变异自检验证过：把真客户端改回 who，只有这一条会红。）
    """
    import inspect

    from vision.k230_follow import K230Target

    for tool_name, method in (("follow_person", "follow"),
                              ("follow_stop", "stop")):
        tool = _schema(_registry(), tool_name)
        params = set(tool["parameters"]["properties"])
        sig = set(inspect.signature(getattr(K230Target, method)).parameters) - {"self"}
        assert params == sig, (
            f"{tool_name} 的参数 {params} 和 K230Target.{method} 的 {sig} 对不上 —— "
            "_guard 是 kwargs 直通，名字不一致会变成'参数不匹配'")


# ---------------------------------------------------------------- 开关

def test_disabled_config_does_not_register():
    r = _registry(config=_StubConfig({"follow": {"enabled": False}}))
    assert r.schemas() == []


def test_enabled_config_registers():
    r = _registry(config=_StubConfig({"follow": {"enabled": True}}))
    assert len(r.schemas()) == 2
