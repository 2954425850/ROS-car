"""「看一眼」工具 —— K230 拍照 → 云端视觉模型 → 一句人话。

## 为什么是工具，为什么叫 look

按 car-tools-design 已经定死的那条线：**意图 → 工具，原语 → 不是工具**。
`k230ctl snap` 是**原语**；「看一眼那是什么」才是**意图**。而「该不该看一眼」只有模型
能判断（用户说「前面有什么」要拍，「现在几点」不要拍），所以工具名取**意图级**的
`look`，不叫 `take_photo`。

## 描述怎么写

「何时该调用」**全部由 description 承担**，**不要往 system_prompt 里加针对视觉的提示行**
—— 在 persona 里点名一批能力，会给那批 token 加权，小模型于是主动推销
（2026-09-13 踩过两次，见记忆 prompt-no-capability-emphasis）。

## 这一层只做一件事：把技术口吻的失败翻成人话

客户端（`vision/k230_look.py`）**失败一律抛 `LookError`**，消息是技术口吻
（「snap 失败（退出码 1）—— …」「视觉请求失败 —— …」）—— 这是刻意的，那一层要保持
纯粹、好单测。转人话是**本层**的责任：模型拿到的是「一句得体的口语 + 一点技术细节
供排查」，而不是一串英文异常名。措辞照 `tools/car.py` 的 `_UNAVAILABLE`。

注意区分**成功路径**：T4b 的「可疑照片」处理在照片可疑时**照常返回文本**，
只是在前面带一句保留（如「（照片可能没拍全，看不太准）」）。那是成功不是失败 ——
本层**原样透传**，不许当错误处理。
"""

from __future__ import annotations

from loguru import logger

from tools.registry import Tool, ToolRegistry
from vision.k230_look import LookError

_UNAVAILABLE = (
    "看不了 —— 视觉模块当前不可用（K230 没连上，或者启动时没起来）。"
    "请检查服务日志。"
)

# ---------------------------------------------------------------- 失败 → 人话
# 判据只能是**消息文本**：客户端的 LookError 不带 stage 字段（见 vision/k230_look.py），
# 所以按关键词把失败归到「哪一跳」上：
#   snap —— 拍照那一跳（含读文件）：摄像头 / 链路的问题
#   view —— 上云那一跳：拍到了，但没看成
# 顺序要紧：**先判上云**。snap 的 stderr 里偶尔会带「请求」这类词，反过来判会误归。
_VIEW_HINTS = ("视觉", "模型", "分析", "请求", "API")
_SNAP_HINTS = ("snap", "拍照", "照片", "路径", "字节")

# ★ 真机实测（2026-09-18）：`k230ctl` 自己崩掉时，stderr 的**首行就是**这句。
# 它零信息，而且正是「返回给模型的文本里不该出现 Traceback」那种词。原始堆栈客户端
# 已经 `logger.warning` 留了档，这里砍掉不丢排查线索。
_CRASH_MARKER = "Traceback (most recent call last)"


def _classify(message: str) -> str:
    """把一条 LookError 消息归到 snap / view / other。"""
    if any(hint in message for hint in _VIEW_HINTS):
        return "view"
    if any(hint in message for hint in _SNAP_HINTS):
        return "snap"
    return "other"


def _humanize(exc: BaseException) -> str:
    """技术口吻的异常 → 模型能照着说的一句口语（技术细节附在后面供排查）。

    只在这里加人话，客户端的消息**不改**（它是给日志和单测看的）。
    """
    # rstrip("。.")：客户端的技术消息常以「.」或「。」收尾（openai 的
    # "Connection error." 就是），不去掉的话会拼出「error.。」这种双句号。
    detail = str(exc).strip().rstrip("。.") or "（没有更多信息）"
    if _CRASH_MARKER in detail:
        prefix = detail.split(_CRASH_MARKER, 1)[0].strip().rstrip("—").strip()
        detail = (
            f"{prefix} 命令自己崩了（堆栈见服务日志）"
            if prefix
            else "命令自己崩了（堆栈见服务日志）"
        )
    kind = _classify(detail)
    if kind == "snap":
        head = "没拍成，可能是摄像头没连上"
    elif kind == "view":
        head = "拍到了，但没看清楚"
    else:
        head = "看这一眼没成功"
    return f"{head} —— {detail}。请检查服务日志。"


def _guard(vision):
    """vision 为 None 时统一回一句解释，而不是让模型看到「没有这个工具」（照 car.py）。"""

    def _wrap(method_name: str):
        def _handler(**kwargs):
            if vision is None:
                return _UNAVAILABLE
            logger.info(f"视觉：调用 {method_name}（question={kwargs.get('question')!r}）")
            try:
                return getattr(vision, method_name)(**kwargs)
            except LookError as exc:
                logger.warning(f"视觉：{method_name} 失败 —— {exc}")
                return _humanize(exc)
            except TypeError:
                # 参数形状不对（模型漏传 question）—— 原样抛给 registry.dispatch，
                # 它报的「工具 look 参数不匹配」比在这里硬翻一句人话更准。
                raise
            except Exception as exc:  # noqa: BLE001 —— 客户端没料到的错误
                # 仍给一句人话：dispatch 的兜底是「工具 look 执行失败：…」，
                # 那是给开发者排查用的，不是给模型念的。
                logger.exception(f"视觉：{method_name} 出现了预期外的错误")
                return _humanize(exc)

        return _handler

    return _wrap


def register_vision_tools(registry: ToolRegistry, vision, config=None) -> None:
    """把「看一眼」登记成原生工具。

    Args:
        registry: 工具注册表。
        vision: `K230Vision` 实例，**可以是 None**（K230 没连上 / 起不来时）——
            照 car 的做法仍然注册，调用时回一句解释，而不是让模型看到「没有这个工具」。
        config: `utils.config.Config` 那个形状（有 `get(key, default)`）。
            **T6 必须传**：不传的话 `vision.enabled=false` 不生效（见下）。
            传 None = 不做开关判断、总是注册（单测里只关心工具形状时的用法）。

    `vision.enabled` 为 false → **不注册**（照 `llm.tools_enabled` 的做法，
    读 `config.get("vision.enabled", True)`）。放在注册这一层而不是调用那一层，
    是为了让关掉的能力**根本不进**发给模型的 `tools=` 数组 —— 那样模型连
    「有这么个工具但我不能用」都不知道。
    """
    if config is not None and not config.get("vision.enabled", True):
        logger.info("视觉：config 里 vision.enabled=false，不注册 look 工具")
        return

    if vision is None:
        logger.warning("视觉：客户端不可用，仍注册 look 工具（调用时会说明原因）")

    registry.register(
        Tool(
            name="look",
            group="vision",
            description=(
                "让 K230 摄像头拍一张照片，交给云端视觉模型看，返回看到的内容。"
                "用户问「前面有什么」「这是什么」「看看那边」「帮我看看」"
                "「上面写的什么」这类**需要看画面才能回答**的问题时调用。"
                "把用户的问题原话写进 question。"
                "只是想转动云台、不需要知道画面里有什么（「往左看看」）用 car_camera；"
                "需要知道那边有什么、要一个答案时才用本工具。"
                "注意：这个操作要花几秒到十几秒，一次就够，同一个问题不要连着调好几次。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "question": {
                        "type": "string",
                        "description": "用户想问画面里的什么，用他的原话最好。",
                    },
                },
                "required": ["question"],
            },
            handler=_guard(vision)("look"),
        )
    )
