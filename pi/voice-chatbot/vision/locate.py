"""「那个 X 在画面哪儿」→ 一个归一化框（l,t,r,b，1280x720 推流系）。

**目标指定一律让视觉模型画框**（用户定，2026-10-21）。原计划先用 YOLO 按类别匹配，
实测 YOLO 只认 80 类、还把青苹果认成 sports ball（0.41），"认错类却当真"比慢 2 秒糟；
YOLO 只留在板子上做每帧的事（人体框给跟人、障碍物、跟踪器纠偏、上位机显示）。

模型实测（2026-10-21 桌面场景）：Qwen3-VL IoU 0.90~0.99、~2s、两轮一致；
DeepSeek 对辣椒两次 IoU 0（框到空地上）。所以默认 Qwen3-VL，DeepSeek 可切（locate.*）。

⚠️ 画框要 ~2s，期间车和臂**不能动**，否则框和画面对不上 —— 调用方负责（语音
   调这个时车本来就是停着的）。
"""

from __future__ import annotations

import base64
import re
from dataclasses import dataclass

from loguru import logger


_PROMPT = ("在这张 1280x720 的图里找到「{q}」。只输出 JSON："
           '{{"found": true/false, "box": [x1, y1, x2, y2]}}，坐标是 0~1000 的整数'
           "（相对图像宽高归一化后乘 1000），框要紧贴目标；找不到就 found=false。")


@dataclass
class Located:
    ok: bool
    box: tuple[float, float, float, float] | None = None
    src: str = ""              # vlm
    label: str = ""
    why: str = ""


def parse_box(text: str) -> tuple[float, float, float, float] | None:
    """模型输出 → 归一化框；found=false / 没框 / 退化框 ⇒ None。"""
    if re.search(r'"found"\s*:\s*false', text or ""):
        return None
    m = re.search(r"\[\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*\]", text or "")
    if not m:
        return None
    l, t, r, b = (min(1.0, max(0.0, float(v) / 1000.0)) for v in m.groups())
    if r - l < 0.005 or b - t < 0.005:
        return None
    return (l, t, r, b)


class VlmBoxer:
    """OpenAI 兼容接口的"画框"客户端（Qwen3-VL / DeepSeek 都走这个）。"""

    def __init__(self, config):
        get = config.get
        self.model = get("locate.model", "qwen3-vl-plus")
        self.base_url = get("locate.base_url", "https://dashscope.aliyuncs.com/compatible-mode/v1")
        # 默认用 ASR 的 DashScope key（同一个百炼账号）
        self.api_key = get("locate.api_key") or get("asr.api_key")
        self.timeout = float(get("locate.timeout_sec", 20))
        self._client = None

    def box(self, image_path: str, query: str):
        if self._client is None:
            from openai import OpenAI
            self._client = OpenAI(api_key=self.api_key, base_url=self.base_url,
                                  timeout=self.timeout)
        with open(image_path, "rb") as f:
            img = base64.b64encode(f.read()).decode()
        r = self._client.chat.completions.create(
            model=self.model, max_tokens=4000,
            messages=[{"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + img}},
                {"type": "text", "text": _PROMPT.format(q=query)}]}])
        out = r.choices[0].message.content or ""
        return parse_box(out), out


class Locator:

    def __init__(self, snapper, boxer):
        self._snapper = snapper    # 有 snap() -> 图片路径（K230Vision）
        self._boxer = boxer        # VlmBoxer

    def locate(self, target: str) -> Located:
        if self._snapper is None or self._boxer is None:
            return Located(False, why="视觉模型不可用")
        try:
            path = self._snapper.snap()
        except Exception as e:  # noqa: BLE001
            return Located(False, why=f"拍不到照片：{e}")
        try:
            box, raw = self._boxer.box(path, target)
        except Exception as e:  # noqa: BLE001
            logger.warning(f"定位：视觉模型出错 —— {e}")
            return Located(False, why=f"视觉模型出错：{e}")
        if box is None:
            logger.info(f"定位：视觉模型没找到「{target}」—— {raw[:120]!r}")
            return Located(False, why="画面里没找到")
        return Located(True, box, "vlm", target, "视觉模型画的框")
