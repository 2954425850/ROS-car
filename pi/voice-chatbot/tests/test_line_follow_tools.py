"""循迹语音工具（tools/line_follow.py + car/line_follow_client.py）。

不联网、不需要 rclpy：客户端用假服务注入。覆盖：
  - 注册：只有 line_follow 一个工具，group="car"，required=["enable"]；
  - 分发：true/false 原样传给 client；非 bool 拒绝；client 不可用统一降级文案；
  - 客户端：服务不在/没回话/成功 的三种回话；请求里 data 正确。
"""

from __future__ import annotations

import types

from tools.line_follow import UNAVAILABLE_PREFIX, register_line_follow_tools
from tools.registry import ToolRegistry
from car.line_follow_client import LineFollowClient


class FakeClient:
    def __init__(self, enabled=True, reason="ok"):
        self.enabled = enabled
        self.reason = reason
        self.calls = []

    def set_enabled(self, enable):
        self.calls.append(enable)
        return "已开始循迹" if enable else "已停止循迹"


def _reg(client):
    reg = ToolRegistry()
    register_line_follow_tools(reg, client)
    return reg


# --------------------------------------------------------------------------- 注册 / 分发

def test_registers_one_tool_in_car_group():
    reg = _reg(FakeClient())
    assert len(reg) == 1
    s = reg.schemas()[0]["function"]
    assert s["name"] == "line_follow"
    assert s["parameters"]["required"] == ["enable"]
    assert reg.get("line_follow").group == "car"


def test_dispatches_true_and_false():
    c = FakeClient()
    reg = _reg(c)
    assert reg.dispatch("line_follow", {"enable": True}).result == "已开始循迹"
    assert reg.dispatch("line_follow", {"enable": False}).result == "已停止循迹"
    assert c.calls == [True, False]


def test_rejects_non_bool_and_does_not_call_client():
    c = FakeClient()
    reg = _reg(c)
    out = reg.dispatch("line_follow", {"enable": "true"}).result
    assert "true/false" in out
    assert c.calls == []


def test_unavailable_client_still_registers():
    reg = _reg(None)
    out = reg.dispatch("line_follow", {"enable": True}).result
    assert out.startswith(UNAVAILABLE_PREFIX)


# --------------------------------------------------------------------------- 客户端（假服务）

class _Fut:
    def __init__(self, result):
        self._r = result

    def done(self) -> bool:
        return True

    def result(self):
        return self._r


class _FakeSetBool:
    class Request:
        def __init__(self):
            self.data = None


class _FakeCli:
    def __init__(self, available=True, success=True, message="已开始循迹"):
        self.available = available
        self.success = success
        self.message = message
        self.reqs = []

    def wait_for_service(self, timeout_sec=None):
        return self.available

    def call_async(self, req):
        self.reqs.append(req)
        return _Fut(types.SimpleNamespace(success=self.success, message=self.message))


def _client_with(cli):
    c = LineFollowClient()
    c.enabled = True
    c._SetBool = _FakeSetBool
    c._cli = cli
    return c


def test_client_returns_service_message_and_sends_bool():
    cli = _FakeCli(message="马上开始（车要在线上才会走）")
    c = _client_with(cli)
    assert c.set_enabled(True) == "马上开始（车要在线上才会走）"
    assert cli.reqs and cli.reqs[0].data is True


def test_client_missing_service_says_so():
    c = _client_with(_FakeCli(available=False))
    out = c.set_enabled(True)
    assert "没在跑" in out


def test_client_success_with_empty_message_falls_back():
    c = _client_with(_FakeCli(message=""))
    assert c.set_enabled(False) == "已停止循迹"
    assert c.set_enabled(True) == "已开始循迹"


def test_client_failure_message_is_prefixed():
    c = _client_with(_FakeCli(success=False, message="急停在按着"))
    assert c.set_enabled(True) == "没做成：急停在按着"


def test_client_not_started_degrades():
    c = LineFollowClient()
    assert "用不了" in c.set_enabled(True)
