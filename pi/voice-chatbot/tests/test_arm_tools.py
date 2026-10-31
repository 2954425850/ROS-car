"""机械臂语音工具（Task 5）：tools/arm.py + car/arm_client.py + conversation 唤醒路径。

不联网、不需要 rclpy（ArmClient 的 ROS 部分用假对象注入）。覆盖：
  - 注册：用的就是 arm_schema.SCHEMAS_B（不是另写的一份），group="arm"；
  - 参数 → spec：平铺 / steps / 平铺+steps / oscillate+repeat+speed；
  - 降级：client 为 None / 未 enabled 的统一文案；
  - 日志：每次调用一行 JSON；日志写失败不影响返回；
  - ArmClient：缺 rclpy 时 start() 返回 False 不抛；结果文案只用服务端回读；cancel 能打断 act、绝不抛；
  - conversation._wake_fast_action：调了臂 cancel，cancel 抛异常也不外泄、急停照做。
"""

from __future__ import annotations

import json
import sys
import threading
import time
import types

import pytest

from tools.arm import UNAVAILABLE_PREFIX, register_arm_tools
from tools.arm_schema import SCHEMAS_B
from tools.registry import ToolRegistry

STATE = {"text": "视线朝左 20°、向下 35°；爪尖在车前 16cm、高 8cm；夹爪张开", "fields": [1, 2, 3, 4, 5, 6]}
STATE_JSON = json.dumps(STATE, ensure_ascii=False)


# --------------------------------------------------------------------------- 假 client

class FakeClient:
    def __init__(self, enabled=True, reason="ok"):
        self.enabled = enabled
        self.reason = reason
        self.calls = []

    def act(self, spec):
        self.calls.append(("act", spec))
        return "已完成：动作。现在：" + STATE["text"] + "。"

    def describe(self):
        self.calls.append(("describe",))
        return "机械臂现在：" + STATE["text"] + "。"

    def home(self):
        self.calls.append(("home",))
        return "已回到复位位姿（夹爪保持）。"


def _reg(client, tmp_path):
    reg = ToolRegistry()
    log = tmp_path / "logs" / "arm_motion.jsonl"
    register_arm_tools(reg, client, log_path=log)
    return reg, log


# --------------------------------------------------------------------------- 注册

def test_registers_the_final_schemas_b_in_group_arm(tmp_path):
    reg, _ = _reg(FakeClient(), tmp_path)
    assert len(reg) == 3
    got = {s["function"]["name"]: s for s in reg.schemas(groups={"arm"})}
    want = {s["function"]["name"]: s for s in SCHEMAS_B}
    assert got == want                         # 逐字就是定稿那份 schema
    assert {reg.get(n).group for n in got} == {"arm"}


def test_registers_even_when_client_is_none(tmp_path):
    reg, _ = _reg(None, tmp_path)
    assert {"arm_act", "arm_state", "arm_home"} <= {s["function"]["name"] for s in reg.schemas()}


# --------------------------------------------------------------------------- 参数 → spec

def _act_spec(tmp_path, args):
    c = FakeClient()
    reg, _ = _reg(c, tmp_path)
    out = reg.dispatch("arm_act", args).result
    assert [k[0] for k in c.calls] == ["act"], out
    return c.calls[0][1]


def test_flat_fields_become_one_step(tmp_path):
    spec = _act_spec(tmp_path, {"turn": [{"dir": "下", "amount": "一点"}], "approach": {"dir": "靠近"}})
    assert spec == {"steps": [{"turn": [{"dir": "下", "amount": "一点"}], "approach": {"dir": "靠近"}}]}


def test_steps_keep_order_and_speed_passes_through(tmp_path):
    spec = _act_spec(tmp_path, {"steps": [{"turn": [{"dir": "下", "amount": "一些"}]}, {"gripper": "张开"}],
                                "speed": "慢"})
    assert spec == {"speed": "慢",
                    "steps": [{"turn": [{"dir": "下", "amount": "一些"}]}, {"gripper": "张开"}]}


def test_flat_fields_go_before_steps(tmp_path):
    spec = _act_spec(tmp_path, {"toward": "左边", "steps": [{"gripper": "合上"}]})
    assert spec["steps"] == [{"toward": "左边"}, {"gripper": "合上"}]


def test_oscillate_with_repeat_and_speed(tmp_path):
    spec = _act_spec(tmp_path, {"oscillate": {"axis": "上下", "amount": "一些", "times": 3},
                                "repeat": 2, "speed": "快"})
    assert spec == {"speed": "快", "repeat": 2,
                    "steps": [{"oscillate": {"axis": "上下", "amount": "一些", "times": 3}}]}


def test_empty_or_flattened_args_do_not_call_the_client(tmp_path):
    c = FakeClient()
    reg, _ = _reg(c, tmp_path)
    out = reg.dispatch("arm_act", {"dir": "下", "amount": "一点"}).result
    assert c.calls == []
    assert out.startswith("没动") and "dir" in out


def test_state_and_home_call_the_client(tmp_path):
    c = FakeClient()
    reg, _ = _reg(c, tmp_path)
    assert STATE["text"] in reg.dispatch("arm_state", {}).result
    assert "复位" in reg.dispatch("arm_home", {}).result
    assert c.calls == [("describe",), ("home",)]


# --------------------------------------------------------------------------- 降级

@pytest.mark.parametrize("client", [None, FakeClient(enabled=False, reason="缺依赖（No module named 'rclpy'）")])
@pytest.mark.parametrize("tool,args", [("arm_act", {"gripper": "张开"}), ("arm_state", {}), ("arm_home", {})])
def test_unavailable_text_when_client_missing_or_disabled(tmp_path, client, tool, args):
    reg, _ = _reg(client, tmp_path)
    out = reg.dispatch(tool, args).result
    assert out.startswith(UNAVAILABLE_PREFIX)
    if client is not None:
        assert "rclpy" in out                  # 带上真实原因
        assert client.calls == []


# --------------------------------------------------------------------------- 日志

def test_each_call_appends_one_json_line(tmp_path):
    reg, log = _reg(FakeClient(), tmp_path)
    r1 = reg.dispatch("arm_act", {"gripper": "张开"}).result
    r2 = reg.dispatch("arm_state", {}).result
    lines = log.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    a, b = (json.loads(x) for x in lines)
    assert a["tool"] == "arm_act" and a["args"] == {"gripper": "张开"} and a["result"] == r1
    assert b["tool"] == "arm_state" and b["result"] == r2
    for rec in (a, b):
        assert isinstance(rec["ts"], float) and rec["dur_s"] >= 0


def test_log_failure_does_not_break_the_tool(tmp_path):
    reg = ToolRegistry()
    blocker = tmp_path / "logs"
    blocker.write_text("我是个文件，不是目录")    # logs/arm_motion.jsonl 无法创建
    register_arm_tools(reg, FakeClient(), log_path=blocker / "arm_motion.jsonl")
    out = reg.dispatch("arm_act", {"gripper": "张开"}).result
    assert out.startswith("已完成")


# --------------------------------------------------------------------------- ArmClient

from car import arm_client as AC  # noqa: E402


def test_arm_client_start_without_rclpy_returns_false(monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", None)          # import rclpy ⇒ ImportError
    c = AC.ArmClient()
    assert c.start() is False
    assert c.enabled is False and "缺依赖" in c.reason
    assert c.act({"steps": [{"gripper": "张开"}]}).startswith(UNAVAILABLE_PREFIX)
    assert c.describe().startswith(UNAVAILABLE_PREFIX)
    assert c.home().startswith(UNAVAILABLE_PREFIX)
    c.cancel()                                                # 没 goal 也不抛
    c.close()


def test_format_result_ok_uses_server_reason_and_real_state():
    out = AC.format_result("OK", "已完成：低头一点", STATE_JSON)
    assert out == "已完成：低头一点。现在：" + STATE["text"] + "。"


def test_format_result_without_state_has_no_readout():
    for sj in ("", "not json", json.dumps({"no_text": 1})):
        out = AC.format_result("PARTIAL", "做了：左转很多。底座到限位，还差 10°", sj)
        assert "现在" not in out and out.startswith("做了：左转很多")


@pytest.mark.parametrize("code,needle", [
    ("BUSY", "机械臂正在忙（抓取中或在复位）"),
    ("ABORTED_BY_OPERATOR", "你动了手柄，交给你了"),
    ("CANCELED", "打断"),
    ("ESTOP", "急停"),
])
def test_format_result_maps_codes(code, needle):
    out = AC.format_result(code, "服务端原话", STATE_JSON)
    assert needle in out and "服务端原话" not in out and STATE["text"] in out


def test_format_result_error_codes_keep_server_reason():
    out = AC.format_result("NO_FEEDBACK", "读不到可信的机械臂回读，没动", "")
    assert "读不到可信的机械臂回读" in out


# ---- 假 ROS 对象 ----

class Fut:
    def __init__(self):
        self._v = None
        self._done = threading.Event()

    def set(self, v):
        self._v = v
        self._done.set()

    def done(self):
        return self._done.is_set()

    def result(self):
        return self._v


class FakeGH:
    def __init__(self, on_cancel=None, accepted=True):
        self.accepted = accepted
        self.res_fut = Fut()
        self.cancels = 0
        self._on_cancel = on_cancel

    def get_result_async(self):
        return self.res_fut

    def cancel_goal_async(self):
        self.cancels += 1
        if self._on_cancel:
            self._on_cancel(self)


def _res(code, reason="", state_json=""):
    r = types.SimpleNamespace(code=code, reason=reason, state_json=state_json, ok=code in ("OK", "PARTIAL"))
    return types.SimpleNamespace(result=r)


class FakeAction:
    def __init__(self, gh, server=True):
        self.gh = gh
        self.server = server
        self.goals = []

    def wait_for_server(self, timeout_sec=0):
        return self.server

    def send_goal_async(self, goal):
        self.goals.append(goal)
        f = Fut()
        f.set(self.gh)
        return f


class FakeTrigger:
    def __init__(self, success=True, message=STATE_JSON, exists=True):
        self.success, self.message, self.exists = success, message, exists
        self.n = 0

    def wait_for_service(self, timeout_sec=0):
        return self.exists

    def call_async(self, req):
        self.n += 1
        f = Fut()
        f.set(types.SimpleNamespace(success=self.success, message=self.message))
        return f


def _ready_client(gh, describe=None, home=None, **kw):
    c = AC.ArmClient(**kw)
    c.enabled, c.reason = True, "ok"
    c._ArmMotion = types.SimpleNamespace(Goal=lambda: types.SimpleNamespace(spec_json="", dry_run=True))
    c._Trigger = types.SimpleNamespace(Request=lambda: object())
    c._action = FakeAction(gh)
    c._describe_cli = describe or FakeTrigger()
    c._home_cli = home or FakeTrigger(message="已回到复位位姿（夹爪保持）")
    return c


def _run_act_in_thread(c, spec, timeout=None):
    box = {}
    t = threading.Thread(target=lambda: box.setdefault("out", c.act(spec, timeout=timeout)), daemon=True)
    t.start()
    return t, box


def test_act_sends_spec_json_not_dry_run_and_returns_server_state():
    gh = FakeGH()
    gh.res_fut.set(_res("OK", "已完成：张开夹爪", STATE_JSON))
    c = _ready_client(gh)
    out = c.act({"steps": [{"gripper": "张开"}]})
    g = c._action.goals[0]
    assert json.loads(g.spec_json) == {"steps": [{"gripper": "张开"}]} and g.dry_run is False
    assert out == "已完成：张开夹爪。现在：" + STATE["text"] + "。"


def test_act_rejected_goal_is_busy():
    c = _ready_client(FakeGH(accepted=False))
    assert "机械臂正在忙" in c.act({"steps": [{"gripper": "张开"}]})


def test_act_server_missing_is_reported():
    c = _ready_client(FakeGH())
    c._action.server = False
    assert "/arm/motion 不在" in c.act({"steps": [{"gripper": "张开"}]})


def test_cancel_interrupts_act_and_returns_server_freeze_state():
    frozen = json.dumps({"text": "冻结后的真实回读"}, ensure_ascii=False)
    gh = FakeGH(on_cancel=lambda g: threading.Timer(
        0.05, lambda: g.res_fut.set(_res("CANCELED", "取消了（低头），臂停在当前位置", frozen))).start())
    c = _ready_client(gh)
    t, box = _run_act_in_thread(c, {"steps": [{"turn": [{"dir": "下"}]}]}, timeout=30)
    time.sleep(0.15)
    t0 = time.monotonic()
    c.cancel()
    t.join(2.0)
    assert not t.is_alive() and time.monotonic() - t0 < 1.0
    assert gh.cancels >= 1
    assert "打断" in box["out"] and "冻结后的真实回读" in box["out"]


def test_cancel_returns_within_grace_even_if_server_never_answers():
    gh = FakeGH()                                   # 永远不出结果
    desc = FakeTrigger(message=json.dumps({"text": "describe 的真实回读"}, ensure_ascii=False))
    c = _ready_client(gh, describe=desc, cancel_grace=0.2)
    t, box = _run_act_in_thread(c, {"steps": [{"gripper": "合上"}]}, timeout=30)
    time.sleep(0.1)
    c.cancel()
    t.join(2.0)
    assert not t.is_alive()
    assert "打断" in box["out"] and "describe 的真实回读" in box["out"] and desc.n == 1


def test_cancel_during_send_is_applied_once_goal_is_accepted():
    gh = FakeGH(on_cancel=lambda g: g.res_fut.set(_res("CANCELED", "", "")))
    c = _ready_client(gh)
    orig_send = c._action.send_goal_async

    def send_and_wake(goal):                        # 唤醒词恰好在发送途中
        c.cancel()
        return orig_send(goal)

    c._action.send_goal_async = send_and_wake
    out = c.act({"steps": [{"gripper": "合上"}]}, timeout=5)
    assert gh.cancels == 1 and "打断" in out


def test_previous_wake_does_not_cancel_the_next_act():
    gh = FakeGH()
    gh.res_fut.set(_res("OK", "已完成：抬头", STATE_JSON))
    c = _ready_client(gh)
    c.cancel()                                      # 上一句被唤醒词打断过
    out = c.act({"steps": [{"turn": [{"dir": "上"}]}]})
    assert out.startswith("已完成：抬头") and gh.cancels == 0


def test_timeout_cancels_and_reports_timeout():
    gh = FakeGH(on_cancel=lambda g: g.res_fut.set(_res("CANCELED", "", STATE_JSON)))
    c = _ready_client(gh)
    out = c.act({"steps": [{"gripper": "合上"}]}, timeout=0.1)
    assert gh.cancels == 1 and "超时" in out and STATE["text"] in out


def test_cancel_never_raises_even_if_ros_does():
    def boom(_g):
        raise RuntimeError("rclpy 炸了")
    c = _ready_client(FakeGH(on_cancel=boom))
    c._gh = c._action.gh
    c.cancel()                                      # 不抛


def test_describe_and_home_use_real_readout():
    c = _ready_client(FakeGH())
    assert c.describe() == "机械臂现在：" + STATE["text"] + "。"
    assert c.home() == "已回到复位位姿（夹爪保持）。现在：" + STATE["text"] + "。"
    c._describe_cli = FakeTrigger(success=False, message="读不到可信的机械臂回读")
    assert "读不到" in c.describe()
    assert "现在" not in c.home()                    # 读不到就不说读数
    c._home_cli = FakeTrigger(success=False, message="抓取进行中，先等它结束或取消")
    assert c.home().startswith("没能复位") and "抓取进行中" in c.home()


# --------------------------------------------------------------------------- conversation 唤醒路径

@pytest.fixture
def conversation_mod(monkeypatch):
    """导入 core.conversation。Windows 缺 av/mcp ⇒ 只在导入失败时给 tools.music / mcp_host 打桩。"""
    try:
        import core.conversation as conv
    except ImportError:
        conv = None
    if conv is not None:
        yield conv
        return
        pass
    m = types.ModuleType("tools.music")
    m.MusicController = object
    m.register_music_tools = lambda *a, **k: None
    h = types.ModuleType("mcp_host")
    h.HostBridge = object
    h.MCPHost = object
    monkeypatch.setitem(sys.modules, "tools.music", m)
    monkeypatch.setitem(sys.modules, "mcp_host", h)
    monkeypatch.delitem(sys.modules, "core.conversation", raising=False)
    import core.conversation as conv
    yield conv
    sys.modules.pop("core.conversation", None)


class _Rec:
    def __init__(self, log, name, exc=None):
        self.log, self.name, self.exc = log, name, exc

    def __getattr__(self, attr):
        def f(*a, **k):
            self.log.append(f"{self.name}.{attr}")
            if self.exc is not None and attr == "cancel":
                raise self.exc
        return f


def _bare_manager(conv, arm):
    log = []
    cm = object.__new__(conv.ConversationManager)
    cm._music = _Rec(log, "music")
    cm._car = _Rec(log, "car")
    cm._arm = arm(log) if arm else None
    cm._handoff = types.SimpleNamespace(is_busy=True)
    cm._interrupt = threading.Event()
    return cm, log


def test_wake_fast_action_cancels_the_arm_after_car_stop(conversation_mod):
    cm, log = _bare_manager(conversation_mod, lambda log: _Rec(log, "arm"))
    cm._wake_fast_action()
    assert log == ["music.pause", "car.stop_all", "arm.cancel"]
    assert cm._interrupt.is_set()


def test_wake_fast_action_swallows_arm_cancel_errors(conversation_mod):
    cm, log = _bare_manager(conversation_mod, lambda log: _Rec(log, "arm", exc=RuntimeError("boom")))
    cm._wake_fast_action()                         # 不抛
    assert "car.stop_all" in log and "arm.cancel" in log
    assert cm._interrupt.is_set()                  # 后面的打断置位照做


def test_wake_fast_action_without_arm(conversation_mod):
    cm, log = _bare_manager(conversation_mod, None)
    cm._wake_fast_action()
    assert log == ["music.pause", "car.stop_all"]
