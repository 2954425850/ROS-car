"""「抓那个 X」工具：视觉模型画框 → 调 /arm/grasp 抓取。

和 `track_object` 一样先定位（Qwen3-VL 画框），然后把框交给抓取 action。
抓取服务负责：锁框 → 对准 → 下扎 → 合爪 → 抬起 → 回复位位姿（保持夹爪）。
全程同步，用户说"抓 X"后要等它抓完才会答话。
"""

from __future__ import annotations

from loguru import logger

from tools.registry import Tool, ToolRegistry


def register_grasp_tools(registry: ToolRegistry, locator, grasp) -> None:
    if locator is None or grasp is None:
        return

    def grasp_object(target_desc: str) -> str:
        loc = locator.locate(target_desc)
        logger.info(f"抓取：{target_desc!r} → {loc}")
        if not loc.ok:
            return f"没能找到「{target_desc}」：{loc.why}"
        logger.info(f"抓取：框 {[round(v, 3) for v in loc.box]} → /arm/grasp")
        return grasp.grasp(loc.box)

    registry.register(Tool(
        name="grasp_object",
        group="grasp",
        description=(
            "让机械臂**把某个物体抓起来**。用户说「抓那个辣椒」「把杯子拿起来」「捡起那个方块」"
            "这类**要动手抓取**的话时调用。target_desc 用用户的原话描述那个东西"
            "（带上颜色、位置，比如「那个黄色的辣椒」）。"
            "⚠️ 抓完机械臂会自动回到初始位置、一直夹着那东西，不会放下。"
            "只是想「看着/跟着」不动手，用 track_object，不是本工具。"
        ),
        parameters={
            "type": "object",
            "properties": {"target_desc": {"type": "string", "description": "要抓的东西，用原话。"}},
            "required": ["target_desc"],
        },
        handler=grasp_object,
    ))
