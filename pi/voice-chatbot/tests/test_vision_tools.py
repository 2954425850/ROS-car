"""`look` 工具注册测试 —— 只查 schema 形状与接线，不碰摄像头、不发网络请求。

vision 用替身（`_StubVision`）：要么回一句话，要么抛 `LookError`。
真实客户端（跑子进程 / 发 HTTP）的测试在 `tests/test_k230_look.py`。
"""

import pytest

from tools.registry import ToolRegistry
from tools.vision import register_vision_tools
from vision.k230_look import LookError

ANSWER = "一个红黄相间的苹果放在白色窗台上。"

# T4b 的「可疑照片」保留语。★ 这是**成功路径**，不是失败路径 —— 必须原样透传。
SUSPICIOUS = "（照片可能没拍全，看不太准）画面里像是一张白纸。"

# 技术口吻的失败消息，照 vision/k230_look.py 里真实会抛的样子写。
SNAP_FAILURE = "snap 失败（退出码 1）—— SNAP FAILED: rtsp timeout"
SNAP_TIMEOUT = "拍照超时（超过 25 秒，K230 可能没连上）"
VIEW_FAILURE = "视觉请求失败 —— APITimeoutError: request timed out"
VIEW_EMPTY = "视觉模型只回了思考过程、正文是空的（max_tokens=1500 可能被 reasoning 吃光）"
# ★ 真机实测（2026-09-18）：k230ctl 自己崩掉时 stderr 的首行**就是**这句 ——
# 也就是说这个 LookError 消息里真的会带 "Traceback"。
SNAP_CRASH = "snap 失败（退出码 1）—— Traceback (most recent call last):"

_UNSET = object()


class _StubVision:
    """只实现 `look(question) -> str` / 抛 LookError 的替身。"""

    def __init__(self, answer=ANSWER, error=None):
        self.calls = []
        self._answer = answer
        self._error = error

    def look(self, question):
        self.calls.append(question)
        if self._error is not None:
            raise self._error
        return self._answer


class _StubConfig:
    """只实现 `Config.get(dotted_key, default)` 的形状（不读文件）。"""

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


def _registry(vision=_UNSET, config=None, **vision_kwargs):
    """建一个只登记了 look 的注册表。vision 不传 = 一个正常的替身。"""
    registry = ToolRegistry()
    if vision is _UNSET:
        vision = _StubVision(**vision_kwargs)
    register_vision_tools(registry, vision, config)
    return registry


def _schema(registry, name="look"):
    return {s["function"]["name"]: s for s in registry.schemas()}[name]


# ---------------- 注册与 schema 形状 ----------------


def test_registers_exactly_the_look_tool():
    registry = _registry()
    assert set(registry._tools) == {"look"}
    assert registry.get("look").group == "vision"


def test_look_schema_is_openai_shaped():
    schema = _schema(_registry())
    assert schema["type"] == "function"
    assert schema["function"]["name"] == "look"
    assert schema["function"]["description"]
    assert schema["function"]["parameters"]["type"] == "object"


def test_question_is_a_required_string():
    params = _schema(_registry())["function"]["parameters"]
    assert params["properties"]["question"]["type"] == "string"
    assert params["properties"]["question"]["description"]
    assert params["required"] == ["question"]


def test_description_carries_when_to_use():
    """「何时使用」只能由 description 承担（不动 system_prompt）—— 别把它写没了。"""
    description = _schema(_registry())["function"]["description"]
    # 钉住那句「判据」—— 它是整个 description 存在的理由，不是随手可删的润色。
    assert "需要看画面才能回答" in description
    assert "question" in description      # 告诉模型把原话写进参数
    assert "秒" in description            # 延迟预告，免得用户以为它死了


# ---------------- 分发：正常路径 ----------------


def test_dispatch_returns_what_vision_said():
    vision = _StubVision()
    outcome = _registry(vision).dispatch("look", {"question": "这是什么"})
    assert outcome.result == ANSWER
    assert vision.calls == ["这是什么"]        # 原话原样传下去


def test_dispatch_passes_the_raw_question():
    vision = _StubVision()
    _registry(vision).dispatch("look", {"question": "上面写的什么"})
    assert vision.calls == ["上面写的什么"]


def test_suspicious_photo_note_is_passed_through_verbatim():
    """★ 可疑照片是**成功路径**：文本前面带一句保留语，要原样透传，不许当错误处理。"""
    outcome = _registry(_StubVision(answer=SUSPICIOUS)).dispatch(
        "look", {"question": "上面写的什么"}
    )
    assert outcome.result == SUSPICIOUS


# ---------------- 分发：失败路径 → 人话 ----------------


@pytest.mark.parametrize(
    "message, expected",
    [
        (SNAP_FAILURE, "没拍成"),
        (SNAP_TIMEOUT, "没拍成"),
    ],
)
def test_snap_failure_is_translated_to_human_text(message, expected):
    outcome = _registry(_StubVision(error=LookError(message))).dispatch(
        "look", {"question": "前面有什么"}
    )
    assert expected in outcome.result          # 人话
    assert "摄像头" in outcome.result
    assert message in outcome.result           # 技术细节留给排查


@pytest.mark.parametrize(
    "message, expected",
    [
        (VIEW_FAILURE, "没看清楚"),
        (VIEW_EMPTY, "没看清楚"),
    ],
)
def test_view_failure_is_translated_to_human_text(message, expected):
    outcome = _registry(_StubVision(error=LookError(message))).dispatch(
        "look", {"question": "前面有什么"}
    )
    assert expected in outcome.result
    assert message in outcome.result


def test_unknown_failure_still_says_something_human():
    outcome = _registry(_StubVision(error=LookError("something odd"))).dispatch(
        "look", {"question": "前面有什么"}
    )
    assert "没成功" in outcome.result


@pytest.mark.parametrize(
    "error",
    [LookError(SNAP_FAILURE), LookError(VIEW_FAILURE), LookError(SNAP_TIMEOUT)],
)
def test_failures_never_leak_technical_words(error):
    result = _registry(_StubVision(error=error)).dispatch("look", {"question": "x"}).result
    for tech in ("Traceback", "Exception"):
        assert tech not in result
    assert result.strip()


def test_handler_swallows_look_error_and_does_not_raise():
    """★ 直接调 handler（不经过 dispatch）：LookError 必须在这里就被吃掉。

    走 dispatch 断言「不抛」是句废话 —— dispatch 本来就永不抛异常，所以「没抛」
    证明不了任何事。这里直接调 handler：LookError 一旦漏出去，就会变成 dispatch 的
    「工具 look 执行失败：…」，那是给开发者看的，不是给模型念的。
    """
    handler = _registry(_StubVision(error=LookError(SNAP_FAILURE))).get("look").handler
    result = handler(question="前面有什么")
    assert "没拍成" in result                    # 人话是这一层加的
    assert "工具 look 执行失败" not in result     # 不是 dispatch 兜底的产物


def test_real_snap_crash_does_not_leak_a_traceback_header():
    """真机实测的首行就是 Traceback 头 —— 那行零信息，不能原样递给模型。"""
    result = _registry(_StubVision(error=LookError(SNAP_CRASH))).dispatch(
        "look", {"question": "前面有什么"}
    ).result
    assert "没拍成" in result
    assert "Traceback" not in result


def test_unexpected_exception_is_also_humanized():
    class _Boom:
        def look(self, question):
            raise RuntimeError("视觉请求失败 —— 网络断了")

    outcome = _registry(_Boom()).dispatch("look", {"question": "前面有什么"})
    assert "没看清楚" in outcome.result
    assert "工具 look 执行失败" not in outcome.result


# ---------------- 开关与不可用 ----------------


def test_disabled_in_config_means_no_tool_at_all():
    registry = _registry(config=_StubConfig({"vision": {"enabled": False}}))
    assert len(registry) == 0
    assert "look" not in registry


def test_enabled_true_registers():
    registry = _registry(config=_StubConfig({"vision": {"enabled": True}}))
    assert set(registry._tools) == {"look"}


def test_enabled_defaults_to_true_when_absent():
    """配置里没有 vision 段（老 config.yaml）= 默认开。"""
    assert set(_registry(config=_StubConfig({}))._tools) == {"look"}


def test_real_config_class_reads_the_vision_switch(tmp_path):
    """用真的 `utils.config.Config` 读一遍，钉住点号路径 `vision.enabled` 没写错。"""
    from utils.config import Config

    path = tmp_path / "config.yaml"
    path.write_text("vision:\n  enabled: false\n", encoding="utf-8")
    assert len(_registry(config=Config(str(path)))) == 0


def test_dispatch_of_an_unregistered_tool_does_not_silently_succeed():
    """关掉之后模型若还调 look，必须得到一条明确的失败文本（registry 既有保证）。"""
    outcome = _registry(config=_StubConfig({"vision": {"enabled": False}})).dispatch("look", {})
    assert "不存在" in outcome.result


def test_registers_even_when_vision_is_none():
    """K230 起不来也要注册 —— 让模型调了以后得到一句解释，而不是「没有这个工具」。"""
    assert set(_registry(vision=None)._tools) == {"look"}


def test_look_says_so_when_vision_is_none():
    outcome = _registry(vision=None).dispatch("look", {"question": "前面有什么"})
    assert "不可用" in outcome.result
    assert "Traceback" not in outcome.result
    assert outcome.result.strip()


def test_missing_question_is_reported_as_argument_mismatch():
    """模型漏传 question 时走 registry 的参数校验分支，而不是被翻成一句含糊的人话。"""
    outcome = _registry().dispatch("look", {})
    assert "参数不匹配" in outcome.result
