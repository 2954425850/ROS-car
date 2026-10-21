"""「盯住那个 X」：视觉模型画框 → 交给板子的 NanoTrack 跟住。

只管**镜头跟住**（K230 跟踪器锁定）；抓取另有工具（等抓取 action 化后接）。
停止跟踪复用 follow_stop（"别盯了""别跟了"）。
"""

from __future__ import annotations

import time

from loguru import logger

from tools.registry import Tool, ToolRegistry


def _wait_track(state, timeout: float = 2.5) -> bool:
    """等结果流里出现跟踪器框（src=track）。"""
    if state is None:
        return True                # 没有结果流就没法验，按板子回的 ok 算
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if any(o.src == "track" for o in state.window.objects(time.monotonic())):
            return True
        time.sleep(0.1)
    return False


def register_track_tools(registry: ToolRegistry, locator, target, state) -> None:
    if locator is None or target is None:
        return

    def track_object(target_desc: str) -> str:
        loc = locator.locate(target_desc)
        logger.info(f"盯住：{target_desc!r} → {loc}")
        if not loc.ok:
            return f"没能找到「{target_desc}」：{loc.why}"
        l, t, r, b = loc.box
        try:
            target.box(l, t, r, b)
        except Exception as e:  # noqa: BLE001
            return f"找到了{loc.label}，但交给跟踪器失败：{e}"
        if not _wait_track(state):
            return f"找到了{loc.label}并交给了跟踪器，但它没能锁住（可能目标纹理太少）。"
        return f"已经盯住{loc.label}了。"

    registry.register(Tool(
        name="track_object",
        group="vision",
        description=(
            "让摄像头**盯住（持续跟踪）一个物体**。用户说「盯住那个杯子」「跟住那个黄色的」"
            "「看着那个方块别丢」时调用。target_desc 用用户的原话描述那个东西"
            "（带上颜色、位置等，比如「左边那个黄色的辣椒」）。"
            "跟人请用 follow_person；只是想知道画面里有什么用 look。"
        ),
        parameters={
            "type": "object",
            "properties": {"target_desc": {"type": "string", "description": "要盯住的东西，用原话。"}},
            "required": ["target_desc"],
        },
        handler=track_object,
    ))
