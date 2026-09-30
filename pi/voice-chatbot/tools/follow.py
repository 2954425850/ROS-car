"""「跟着谁」工具 —— 让助手能说「跟着张三」「别跟了」。

## 为什么是工具，为什么叫 follow_person

照 car-tools-design 定死的那条线：**意图 → 工具，原语 → 不是工具**。
`k230ctl follow id1` 是**原语**；「跟着张三」是**意图**。而"该不该跟"只有模型能判断
（用户说「跟着他」要跟，说「他在哪」不要跟），所以名字取意图级的 `follow_person`。

## 和另外两个工具的分界（写进 description，别写进 system_prompt）

- `look`：**看一眼、要个答案**（"前面有什么"）
- `car_camera`：**转云台看看那边**（"往左看看"）
- `follow_person`：**让镜头一直盯着某个人**（"跟着张三"）

## 描述怎么写

「何时该调用」**全部由 description 承担**，不要往 system_prompt 里加提示行 ——
在 persona 里点名一批能力会给那批 token 加权，小模型于是主动推销
（2026-09-13 踩过两次，见记忆 prompt-no-capability-emphasis）。

## 本层只做一件事：把技术口吻的失败翻成人话

客户端（`vision/k230_follow.py`）失败一律抛 `TargetError`，消息是技术口吻
（「板子 15 秒没回话」「k230ctl follow 失败：…」）—— 那是刻意的，那一层要保持
纯粹、好单测。转人话是**本层**的责任，措辞照 `tools/car.py` 的 `_UNAVAILABLE`。
"""

from __future__ import annotations

from loguru import logger

from tools.registry import Tool, ToolRegistry
from vision.k230_follow import TargetError

_UNAVAILABLE = (
    "跟不了 —— 视觉模块当前不可用（K230 没连上，或者启动时没起来）。"
    "请检查服务日志。"
)


def _humanize(exc: Exception) -> str:
    """技术口吻 → 一句得体的话 + 一点排查用的细节。"""
    msg = str(exc)
    if "没回话" in msg or "timeout" in msg.lower():
        return "跟丢了 —— 板子迟迟没回话，可能没连上或者正忙。等会儿再试。"
    if "连不上" in msg or "跑不起" in msg:
        return "跟不了 —— 联系不上摄像头那块板子。请检查它是否在线。"
    if "err" in msg or "失败" in msg:
        # 板子的 err 是给开发者看的英文短句，原样带上，模型能据此措辞。
        return "没跟成 —— %s" % msg
    return "跟不了 —— %s" % msg


def _guard(target):
    """target 为 None 时统一回一句解释，而不是让模型看到「没有这个工具」（照 car.py）。"""

    def _wrap(method_name: str, **log_extra):
        def _handler(**kwargs):
            if target is None:
                return _UNAVAILABLE
            logger.info(f"跟随：调用 {method_name}（{kwargs}）")
            try:
                return getattr(target, method_name)(**kwargs)
            except TargetError as exc:
                logger.warning(f"跟随：{method_name} 失败 —— {exc}")
                return _humanize(exc)
            except TypeError:
                # 参数形状不对 —— 原样抛给 registry.dispatch，
                # 它报的「参数不匹配」比在这里硬翻一句人话更准。
                raise
            except Exception as exc:  # noqa: BLE001
                logger.exception(f"跟随：{method_name} 出现了预期外的错误")
                return _humanize(exc)

        return _handler

    return _wrap


def register_follow_tools(registry: ToolRegistry, target, config=None) -> None:
    """把「跟着谁」登记成原生工具。

    Args:
        registry: 工具注册表。
        target: `K230Target` 实例，**可以是 None**（K230 连不上时）—— 照 car/vision
            的做法仍然注册，调用时回一句解释。
        config: 有 `get(key, default)` 的形状。`follow.enabled=false` → **不注册**，
            那样模型连"有这么个工具但我不能用"都不知道（照 vision.enabled 的做法）。
            传 None = 不做开关判断、总是注册（单测里只关心工具形状时的用法）。
    """
    if config is not None and not config.get("follow.enabled", True):
        logger.info("跟随：config 里 follow.enabled=false，不注册 follow 工具")
        return

    if target is None:
        logger.warning("跟随：客户端不可用，仍注册工具（调用时会说明原因）")

    wrap = _guard(target)

    registry.register(
        Tool(
            name="follow_person",
            group="follow",
            description=(
                "让摄像头**一直盯着某个人**。用户说「跟着张三」「跟着他」"
                "「别让他跑了」「盯住那个人」这类**要持续跟随**的话时调用。"
                "name 填用户说的那个名字（原话最好）。"
                "⚠️ 这个动作**要等对方正对镜头露出脸**才能锁上，可能等几秒；"
                "人不在画面里、或者一直没露正脸，会失败。"
                "只是想知道「他在哪」「那是谁」用 look；只是想转过去看看用 car_camera。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "name": {"type": "string",
                             "description": "要跟的人的名字，用用户的原话。"},
                },
                "required": ["name"],
            },
            handler=wrap("follow"),
        )
    )

    registry.register(
        Tool(
            name="follow_stop",
            group="follow",
            description=(
                "**别再跟着他了**。用户说「别跟了」「不用跟了」「停下别跟」时调用。"
                "注意：让**车**停下是 car_stop，这个是停「跟随」这件事，两者不一样。"
            ),
            parameters={"type": "object", "properties": {}, "required": []},
            handler=wrap("stop"),
        )
    )
