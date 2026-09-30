"""`K230Vision` 的行为测试 —— 假 runner + 假 client，不插 K230、不联网。

风格照 `tests/test_car_controller.py`：`_FakeConfig` 小替身，不用真 `Config` + 临时 yaml。
外部依赖全从构造函数注入（照 `car/ros_bridge.py` 的做法），所以这一整份测试
在没插板子、断网的机器上也必须全绿。

T4b 之后 `snap()` 多了「可疑照片重试一次」这层语义：
「可疑」= 退出码 0 但 stderr 里有 `WARN`（k230ctl 攒的帧太少、未必跨过 IDR）。
坏画面会让视觉模型**自信地描述一个不存在的东西**，所以这条路径的测试
（`# ---- 可疑照片 ----` 那一段）是本文件里最重要的部分。
"""

import base64
import contextlib
import subprocess
import time
import types

import httpx
import pytest
from loguru import logger
from openai import APIConnectionError, APITimeoutError

from vision.k230_look import K230Vision, LookError

_JPEG_BYTES = b"\xff\xd8\xff\xe0" + b"fake-jpeg-body" * 4
# k230ctl 攒的帧不够时往 stderr 写的那一行（成功退出码也是 0）
_WARN_STDERR = "WARN 只攒到 12 帧（< 30），未必跨过 IDR，画面可能不可靠\n"

# 「单次子进程调用」的上限，必须**严格大于** k230ctl 自己的 --timeout 默认值（12s），
# 这样内层才先干净地退出（T6 把它从 12 调到 15，见 vision/k230_look.py）
_PER_CALL_CAP = 15.0


class _FakeConfig:
    """只要有个 get(key, default) 就行 —— 和 utils.config.Config 同形状。"""

    def __init__(self, **overrides):
        self._overrides = overrides

    def get(self, key, default=None):
        return self._overrides.get(key, default)


@contextlib.contextmanager
def _captured_logs(level="WARNING"):
    """抓这一段里 loguru 打出来的日志（用来断言「记了 warning/error」这类契约）。"""
    messages: list[str] = []
    sink = logger.add(lambda m: messages.append(str(m)), level=level, format="{message}")
    try:
        yield messages
    finally:
        logger.remove(sink)


def _ok(stdout, stderr=""):
    return subprocess.CompletedProcess(["k230ctl"], 0, stdout, stderr)


def _bad(stderr="SNAP FAILED: 攒帧超时\n"):
    return subprocess.CompletedProcess(["k230ctl"], 1, "", stderr)


class _FakeRunner:
    """假 subprocess.run：记下参数，每次调用返回同一个可预置的结果（或抛同一个异常）。"""

    def __init__(self, *, stdout="", stderr="", returncode=0, exc=None):
        self.calls = []
        self._stdout = stdout
        self._stderr = stderr
        self._returncode = returncode
        self._exc = exc

    def __call__(self, argv, **kwargs):
        self.calls.append((argv, kwargs))
        if self._exc is not None:
            raise self._exc
        return subprocess.CompletedProcess(argv, self._returncode, self._stdout, self._stderr)


class _SequenceRunner:
    """按次序返回预置结果（或抛预置异常）；用完之后一直重复最后一个。

    重试相关的测试靠它造「第一次怎样、第二次怎样」。
    """

    def __init__(self, *outcomes):
        self.calls = []
        self._outcomes = list(outcomes)

    def __call__(self, argv, **kwargs):
        self.calls.append((argv, kwargs))
        outcome = self._outcomes[min(len(self.calls), len(self._outcomes)) - 1]
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome


class _HangingRunner:
    """模拟「子进程挂住、被我们掐掉」：睡得比自己的 timeout 还久，再抛 TimeoutExpired。

    用来验**总预算**：第一次就把预算烧光之后，第二次尝试不该被发起。
    """

    def __init__(self):
        self.calls = []

    def __call__(self, argv, **kwargs):
        self.calls.append((argv, kwargs))
        time.sleep(kwargs["timeout"] + 0.2)
        raise subprocess.TimeoutExpired(cmd=argv, timeout=kwargs["timeout"])


class _FakeClient:
    """假 OpenAI 客户端：记下每次 create 的 kwargs，返回可预置的 response。

    默认 response 里**故意带上 `reasoning_content`** —— 真身 `deepseek-flash` 就是
    先吐它、再吐 content，把这条形状留在替身里，免得后人以为正文是唯一字段。
    """

    def __init__(self, *, content="一个红苹果放在窗台上", choices=None, exc=None):
        self.calls = []
        self._content = content
        self._choices = choices
        self._exc = exc
        self.chat = types.SimpleNamespace(
            completions=types.SimpleNamespace(create=self._create)
        )

    def _create(self, **kwargs):
        self.calls.append(kwargs)
        if self._exc is not None:
            raise self._exc
        if self._choices is not None:
            return types.SimpleNamespace(choices=self._choices)
        message = types.SimpleNamespace(
            content=self._content, reasoning_content="（推理模型的思考过程）"
        )
        return types.SimpleNamespace(choices=[types.SimpleNamespace(message=message)])


def _request():
    """openai 的 APIConnectionError / APITimeoutError 都要一个 httpx.Request。"""
    return httpx.Request("POST", "https://api.deepseek.com/chat/completions")


def _shot(tmp_path, name="shot.jpg", data=_JPEG_BYTES):
    path = tmp_path / name
    path.write_bytes(data)
    return path


def _image_url_of(client) -> str:
    return client.calls[0]["messages"][0]["content"][1]["image_url"]["url"]


# ---------------------------------------------------------------- snap

def test_snap_returns_the_path_and_strips_trailing_blanks(tmp_path):
    """`k230ctl snap` 成功时 stdout 只有一行路径，但可能带尾随空格/空行。

    调用方是 `IMG=$(k230ctl snap)` 这种用法 —— 尾随空格混进路径就找不到文件了。
    顺带钉住两点：命令按 `shlex.split` 拆成 argv（不是丢给 shell），
    以及单次调用的超时是 `min(15, 剩余预算)`（默认预算 25 → 取 15）。
    """
    shot = _shot(tmp_path)
    runner = _FakeRunner(stdout=f"\n  {shot}  \n\n")
    vision = K230Vision(_FakeConfig(), runner=runner)

    assert vision.snap() == str(shot)

    argv, kwargs = runner.calls[0]
    assert argv == ["/home/cy/k230-vision/pi/k230ctl", "snap"]
    assert kwargs["timeout"] == _PER_CALL_CAP
    assert kwargs["capture_output"] is True


def test_snap_nonzero_exit_raises_with_the_stderr_line(tmp_path):
    """`k230ctl` 失败时往 stderr 写 `SNAP FAILED: <原因>` —— 那句话必须带出来，
    否则模型只能收到一句「拍照失败」，没法告诉用户为什么。"""
    runner = _FakeRunner(stdout="", stderr="SNAP FAILED: 攒帧超时（只等到 2 帧）\n后续细节\n", returncode=1)
    vision = K230Vision(_FakeConfig(), runner=runner)

    with pytest.raises(LookError) as excinfo:
        vision.snap()
    assert "SNAP FAILED: 攒帧超时（只等到 2 帧）" in str(excinfo.value)


def test_snap_timeout_raises_look_error(tmp_path):
    """超时必须以 LookError 出来（工具层只认它）。"""
    runner = _FakeRunner(exc=subprocess.TimeoutExpired(cmd="k230ctl", timeout=12))
    vision = K230Vision(_FakeConfig(), runner=runner)

    with pytest.raises(LookError) as excinfo:
        vision.snap()
    assert "超时" in str(excinfo.value)


def test_snap_missing_file_raises_look_error(tmp_path):
    """snap 报了路径，文件却不在（被删/写失败）—— 当成功就会在读文件时才炸。"""
    runner = _FakeRunner(stdout=str(tmp_path / "nope.jpg") + "\n")
    vision = K230Vision(_FakeConfig(), runner=runner)

    with pytest.raises(LookError) as excinfo:
        vision.snap()
    assert "不存在" in str(excinfo.value)


def test_snap_zero_byte_file_raises_look_error(tmp_path):
    """★ 0 字节是 snap 的**已知中间态**（`pi/README.md`：文件先创建、再写内容）。

    退出码是 0、路径也存在 —— 只查「文件在不在」的话这里会绿，然后把一张
    空图发给模型，换回一句莫名其妙的回答，而日志里什么都看不出来。
    """
    empty = _shot(tmp_path, name="empty.jpg", data=b"")
    runner = _FakeRunner(stdout=str(empty) + "\n")
    vision = K230Vision(_FakeConfig(), runner=runner)

    with pytest.raises(LookError) as excinfo:
        vision.snap()
    assert "0 字节" in str(excinfo.value)


def test_snap_missing_command_raises_look_error(tmp_path):
    """T6 验收会「拔掉 K230 或把 snap_cmd 指向不存在的东西」—— 那一步抛的是
    FileNotFoundError（OSError）。不包住的话它就不是 LookError，工具层兜不住。"""
    runner = _FakeRunner(exc=FileNotFoundError(2, "No such file or directory"))
    vision = K230Vision(_FakeConfig(), runner=runner)

    with pytest.raises(LookError):
        vision.snap()


# ---------------------------------------------------------------- 可疑照片（T4b）

def test_suspect_first_attempt_retries_and_returns_clean_text(tmp_path):
    """★ 第一次可疑、第二次干净 → 返回**干净**文本（不带保留），拍了 2 次。

    没跨过 IDR 的画面可能真是坏的，但**重试一次拿到的多半是好照片** ——
    这时候要是还加保留，就是误报：用户会以为相机坏了，而画面其实一点问题没有。
    """
    shot = _shot(tmp_path)
    runner = _SequenceRunner(
        _ok(str(shot) + "\n", stderr=_WARN_STDERR),
        _ok(str(shot) + "\n"),
    )
    client = _FakeClient(content="一只猫")
    vision = K230Vision(_FakeConfig(), runner=runner, client=client)

    assert vision.look("前面有什么") == "一只猫"
    assert len(runner.calls) == 2, "可疑照片必须重试一次"


def test_suspect_twice_returns_text_with_a_caveat_and_logs_a_warning(tmp_path):
    """★★ T4b 的核心，别删。

    两次都可疑 → 照片照给、模型照看，但回答**前面必须带一句保留**，并记 warning。

    这里既不能抛 `LookError`（照片是拿到了的，直接说「没拍成」是撒谎），
    也不能默默返回 —— 坏画面 + 不加保留 = 助手**自信地描述一个不存在的东西**，
    比「说看不清」糟得多，而且事后极难发现（日志里一切正常）。
    """
    shot = _shot(tmp_path)
    runner = _SequenceRunner(
        _ok(str(shot) + "\n", stderr=_WARN_STDERR),
        _ok(str(shot) + "\n", stderr=_WARN_STDERR),
    )
    vision = K230Vision(_FakeConfig(), runner=runner, client=_FakeClient(content="前面有一只猫"))

    with _captured_logs() as warnings:
        text = vision.look("前面有什么")

    assert len(runner.calls) == 2
    # 不写死保留的措辞（那是我定的文案，不是契约），只钉「提前面加了一句」+
    # 「模型说的话原样在后面」—— 顺序反了或者吞掉了正文，都算没做到。
    assert text.endswith("前面有一只猫"), text
    assert text != "前面有一只猫", f"可疑照片却没加保留：{text!r}"
    assert any("仍可疑" in m for m in warnings), f"没记 warning：{warnings}"


def test_hard_failure_then_success_returns_normal_text(tmp_path):
    """★ 新行为：**硬失败也会重试一次**（T4b 改掉了「硬失败只试一次」）。

    K230 抓帧偶发快速失败（pi/README.md），一次不成不代表没得看。
    """
    shot = _shot(tmp_path)
    runner = _SequenceRunner(
        _bad("SNAP FAILED: 攒帧超时\n"),
        _ok(str(shot) + "\n"),
    )
    vision = K230Vision(_FakeConfig(), runner=runner, client=_FakeClient(content="一只猫"))

    assert vision.look("前面有什么") == "一只猫"
    assert len(runner.calls) == 2


def test_hard_failure_twice_raises_look_error(tmp_path):
    """★ 两次都硬失败 → 照旧 `LookError`（工具层 T5 会转成「没拍成」）。

    没有照片就不能硬编一句话出来 —— 那才是真的「自信地说错」。
    """
    runner = _SequenceRunner(_bad(), _bad(stderr="SNAP FAILED: 没有视频流\n"))
    vision = K230Vision(_FakeConfig(), runner=runner, client=_FakeClient())

    with pytest.raises(LookError) as excinfo:
        vision.look("前面有什么")
    assert len(runner.calls) == 2
    assert "SNAP FAILED: 没有视频流" in str(excinfo.value)


def test_budget_exhausted_stops_before_a_second_attempt(tmp_path):
    """★ `snap_timeout_sec` 是**整个 snap()（含重试）的总预算**，不是单次超时。

    用真 sleep 烧预算：第一次调用就睡掉 0.6 秒（比它自己的 0.4 秒上限还久，
    模拟子进程挂住被掐），剩余预算 ≤ 0 → 第二次尝试**根本不该被发起**。
    断言是「只调用了 1 次」—— 没有预算检查的实现会老老实实再调一次。
    """
    runner = _HangingRunner()
    vision = K230Vision(_FakeConfig(**{"vision.snap_timeout_sec": 0.4}), runner=runner)

    with pytest.raises(LookError):
        vision.snap()
    assert len(runner.calls) == 1, (
        f"总预算 0.4 秒、第一次就烧掉 0.6 秒，却发起了 {len(runner.calls)} 次 —— 预算没被当回事"
    )


def test_zero_budget_never_runs_the_command(tmp_path):
    """预算是 0 时连第一次都不该发起（「剩余预算 ≤ 0 就直接收工」）。"""
    runner = _FakeRunner(stdout="x\n")
    vision = K230Vision(_FakeConfig(**{"vision.snap_timeout_sec": 0}), runner=runner)

    with pytest.raises(LookError):
        vision.snap()
    assert runner.calls == []


def test_per_call_timeout_is_min_of_the_cap_and_the_remaining_budget(tmp_path):
    """★ 每次子进程调用的上限 = `min(15, 剩余预算)`。

    这个 `min` 的两半都承重：写死 15 → 预算只有 5 秒时会超预算 3 倍；
    直接用预算 → 单次挂死的子进程一口吃掉全部重试机会。
    """
    shot = _shot(tmp_path)

    runner = _FakeRunner(stdout=str(shot) + "\n")
    K230Vision(_FakeConfig(), runner=runner).snap()
    assert runner.calls[0][1]["timeout"] == _PER_CALL_CAP  # 默认预算 25 > 15 → 取 15

    runner2 = _FakeRunner(stdout=str(shot) + "\n")
    K230Vision(_FakeConfig(**{"vision.snap_timeout_sec": 5}), runner=runner2).snap()
    per_call = runner2.calls[0][1]["timeout"]
    assert 0 < per_call <= 5, f"单次上限 {per_call} 秒 —— 超过了 5 秒的总预算"


def test_clean_snap_never_retries(tmp_path):
    """stderr 干净（没有 WARN）时**不许**重试 —— 正常情况多拍一张就是白等 3~8 秒。"""
    shot = _shot(tmp_path)
    runner = _FakeRunner(stdout=str(shot) + "\n", stderr="")
    vision = K230Vision(_FakeConfig(), runner=runner, client=_FakeClient(content="一只猫"))

    assert vision.look("前面有什么") == "一只猫"
    assert len(runner.calls) == 1


# ---------------------------------------------------------------- describe

def test_describe_returns_the_model_text(tmp_path):
    """正常路径：把模型正文原样（去掉首尾空白）返回。"""
    shot = _shot(tmp_path)
    client = _FakeClient(content="  一个红苹果放在窗台上  \n")
    vision = K230Vision(_FakeConfig(), client=client)

    assert vision.describe("这是什么", str(shot)) == "一个红苹果放在窗台上"


def test_describe_empty_content_raises_instead_of_returning_blank(tmp_path):
    """★★ 本任务最容易踩的坑，别删。

    `deepseek-flash` 是**推理模型**：先吐 `reasoning_content` 再吐 `content`，
    `max_tokens` 给小了（实测 200）正文就是空的。返回空串之后的表现是
    「工具坏了」，而真因是预算被思考过程吃光 —— 所以这里必须抛错，
    **并且**在日志里留下「max_tokens 可能被 reasoning 吃光」这句话。
    """
    vision = K230Vision(_FakeConfig(), client=_FakeClient(content=""))

    with _captured_logs("ERROR") as errors:
        with pytest.raises(LookError):
            vision.describe("这是什么", str(_shot(tmp_path)))

    assert any("max_tokens" in m for m in errors), (
        f"没有记下「预算被 reasoning 吃光」的 error 日志：{errors}"
    )


def test_describe_without_choices_raises_look_error(tmp_path):
    """网关偶尔回一个空壳 response（没有 choices）—— 那也是 LookError，不是 IndexError。"""
    vision = K230Vision(_FakeConfig(), client=_FakeClient(choices=[]))

    with pytest.raises(LookError):
        vision.describe("这是什么", str(_shot(tmp_path)))


def test_describe_api_error_raises_look_error(tmp_path):
    """HTTP/网络错误必须是 LookError —— 它会一路上到工具层变成人话。"""
    vision = K230Vision(_FakeConfig(), client=_FakeClient(exc=APIConnectionError(request=_request())))

    with pytest.raises(LookError):
        vision.describe("这是什么", str(_shot(tmp_path)))


def test_describe_api_timeout_raises_look_error(tmp_path):
    """超时同理。顺带钉住 `request_timeout_sec` 真被传给了 SDK 调用。"""
    client = _FakeClient(exc=APITimeoutError(request=_request()))
    vision = K230Vision(_FakeConfig(**{"vision.request_timeout_sec": 30}), client=client)

    with pytest.raises(LookError):
        vision.describe("这是什么", str(_shot(tmp_path)))
    assert client.calls[0]["timeout"] == 30


def test_describe_payload_carries_the_configured_knobs(tmp_path):
    """★ 发出去的 payload 本身。

    `max_tokens` 必须是 config 的那个值、**不能写死** —— 2026-09-18 那次
    「工具返回空串」的事故就是 200 写死（或没传）造成的（设计 §4.3）。
    另外 `detail=low` 决定成本（一张 720p ≈ 180 token），`stream=False`
    是因为这里要一次性拿完整正文、不做流式。
    """
    shot = _shot(tmp_path)
    client = _FakeClient()
    vision = K230Vision(
        _FakeConfig(
            **{
                "vision.model": "deepseek-flash",
                "vision.max_tokens": 1500,
                "vision.detail": "low",
            }
        ),
        client=client,
    )

    vision.describe("这是什么", str(shot))

    payload = client.calls[0]
    assert payload["model"] == "deepseek-flash"
    assert payload["max_tokens"] == 1500
    assert payload["stream"] is False

    content = payload["messages"][0]["content"]
    assert content[0]["type"] == "text"
    assert "这是什么" in content[0]["text"]          # 用户的问题原话要带上
    image_url = content[1]["image_url"]
    assert image_url["detail"] == "low"
    assert image_url["url"].startswith("data:image/jpeg;base64,")
    # 不只是前缀对 —— 解出来必须就是那个文件的字节
    assert base64.b64decode(image_url["url"].split(",", 1)[1]) == _JPEG_BYTES


def test_describe_rejects_a_zero_byte_file(tmp_path):
    """describe 也能被喂一个现成的路径（冒烟测试就是），所以这条防线它自己也要有。"""
    empty = _shot(tmp_path, name="empty.jpg", data=b"")
    vision = K230Vision(_FakeConfig(), client=_FakeClient())

    with pytest.raises(LookError):
        vision.describe("这是什么", str(empty))


def test_describe_with_an_explicit_path_never_adds_the_caveat(tmp_path):
    """给现成路径时**不**加保留 —— 我们不知道那张图是怎么来的，
    凭空加一句「可能没拍全」就是假警报。"""
    shot = _shot(tmp_path)
    runner = _SequenceRunner(_ok(str(shot) + "\n", stderr=_WARN_STDERR),
                             _ok(str(shot) + "\n", stderr=_WARN_STDERR))
    vision = K230Vision(_FakeConfig(), runner=runner, client=_FakeClient(content="一只猫"))

    vision.snap()  # ← 这一次拍照是「可疑」的，内部状态已经置位
    assert vision.describe("这是什么", str(shot)) == "一只猫"


# ---------------------------------------------------------------- look

def test_look_snaps_only_once(tmp_path):
    """`look` = snap + describe，**只准拍一张**。

    多拍一张就是白等 3~8 秒 —— snap 正是整条链路里最慢的一跳（设计 §4.4）。
    所以这里既数 snap 次数，也验发出去的那张图确实是刚拍的那个文件。
    """
    shot = _shot(tmp_path)
    runner = _FakeRunner(stdout=str(shot) + "\n")
    client = _FakeClient(content="一只猫")
    vision = K230Vision(_FakeConfig(), runner=runner, client=client)

    assert vision.look("前面有什么") == "一只猫"

    assert len(runner.calls) == 1, f"拍了 {len(runner.calls)} 张 —— 一次 look 只该拍一张"
    assert base64.b64decode(_image_url_of(client).split(",", 1)[1]) == _JPEG_BYTES


def test_defaults_work_without_a_vision_section_in_config_yaml(tmp_path):
    """★ `config.yaml` 的 `vision:` 段是 T5 才加的 —— 在那之前本模块必须照常工作。

    所以默认值只能写在代码里，且必须和设计 §5 一致。这里**一点 vision 配置都不给**，
    直接看真正跑出去的命令和真正发出去的 payload。
    （单次超时是 `min(15, 剩余预算)`：默认预算 25 秒 > 15，所以取 15。）
    """
    shot = _shot(tmp_path)
    runner = _FakeRunner(stdout=str(shot) + "\n")
    client = _FakeClient()
    vision = K230Vision(_FakeConfig(), runner=runner, client=client)

    assert vision.look("前面有什么") == "一个红苹果放在窗台上"

    argv, snap_kwargs = runner.calls[0]
    assert argv == ["/home/cy/k230-vision/pi/k230ctl", "snap"]
    assert snap_kwargs["timeout"] == _PER_CALL_CAP

    payload = client.calls[0]
    assert payload["model"] == "deepseek-flash"
    assert payload["max_tokens"] == 1500
    assert payload["timeout"] == 30
    assert payload["messages"][0]["content"][1]["image_url"]["detail"] == "low"
    assert payload["messages"][0]["content"][0]["text"].startswith(
        "用一到两句口语化中文回答，直接说结论，不要罗列。"
    )
