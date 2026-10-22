"""「现在跟我说话的是谁」—— 从唤醒前后几秒的人脸里挑一个（纯逻辑）。

## 原则：宁可不认，不要认错
叫错名字比不叫名字糟糕得多。所以只在证据充分时给出 `known`：

1. 只看**质量门通过**（`ok`）且**活体**过关的脸；
2. 每一帧只取**面积最大**的那张脸（离得最近 ≈ 在跟我说话的人）；
3. 同一个 id 至少出现 `min_hits` 帧、且占这些帧的多数，相似度（取中位数）≥ `accept`；
4. 窗口里同时出现过多张脸的帧太多 ⇒ `crowd`（多人在场，不猜是谁在说话 ——
   声源定位/唇动属于以后的独立课题）。

## 结果
- `known`     认出来了，`pid` 有值
- `stranger`  有一张清楚的活人脸、但库里没有 ⇒ 可以自动建档
- `unsure`    有脸，但分数落在模糊区间 / 几个 id 打架 ⇒ 不叫名也不建档
  （模糊区间的人如果被当成新人录进去，同一个人会变成两个身份）
- `crowd`     多人在场
- `none`      没看到可用的脸（背对、太远、板子没数据）
"""

from __future__ import annotations

import statistics
from collections import Counter
from dataclasses import dataclass, field

from vision.k230_state import FaceSighting


@dataclass
class SpeakerGuess:
    status: str                       # known | stranger | unsure | crowd | none
    pid: str | None = None
    score: float | None = None        # known 时的相似度中位数
    frames: int = 0                   # 参与判断的帧数
    detail: str = ""
    sightings: list[FaceSighting] = field(default_factory=list, repr=False)


@dataclass
class SpeakerConfig:
    accept: float = 0.85              # 叫名字的门槛（板子尺度 score；板子自己的门限是 0.78）
    live_min: float = 0.5             # 活体分下限
    min_hits: int = 2                 # 同一 id 至少几帧
    min_frames_stranger: int = 3      # 判"陌生人"至少要看清几帧
    crowd_frac: float = 0.3           # 多脸帧超过这个比例 ⇒ crowd


def _best_match(sightings: list[FaceSighting]) -> FaceSighting | None:
    """窗口里相似度最高的那张脸（板子对未知脸也会给 match，只是 who=None）。"""
    scored = [s for s in sightings if s.match is not None]
    return max(scored, key=lambda s: s.match) if scored else None


def _none_detail(sightings: list[FaceSighting], cfg: SpeakerConfig) -> str:
    """`none` 的病因必须能分开 —— 没脸、质量门、活体、分数不够是四种排查方向。

    以前这里一律写「N 张脸都不可用（质量/活体）」，连「一张脸都没检测到」
    也写成「0 张脸都不可用」，等于把四个病因压成一句话，没法据此调站位/门限。
    """
    if not sightings:
        return "窗口内没有检测到人脸（板子在线，但没有报脸）"
    q_bad = sum(1 for s in sightings if not s.ok)
    live_bad = sum(1 for s in sightings
                   if s.ok and s.live is not None and s.live < cfg.live_min)
    bits = []
    if q_bad:
        bits.append(f"质量门未过 {q_bad} 张")
    if live_bad:
        bits.append(f"活体分低 {live_bad} 张")
    best = _best_match(sightings)
    extra = ""
    if best is not None:
        extra = f"，最高相似度 {best.match:.2f}" + (
            f"（{best.who}）" if best.who else "（库里没有）")
    return f"{len(sightings)} 张脸都不可用：" + ("、".join(bits) or "原因未知") + extra


def guess_speaker(sightings: list[FaceSighting], cfg: SpeakerConfig | None = None) -> SpeakerGuess:
    cfg = cfg or SpeakerConfig()
    good = [s for s in sightings
            if s.ok and (s.live is None or s.live >= cfg.live_min)]
    if not good:
        return SpeakerGuess("none", frames=len(sightings),
                            detail=_none_detail(sightings, cfg), sightings=sightings)

    # 每帧（同一拍摄时刻）只留最大的那张
    by_frame: dict[float, FaceSighting] = {}
    for s in good:
        cur = by_frame.get(s.t)
        if cur is None or s.area > cur.area:
            by_frame[s.t] = s
    frames = sorted(by_frame.values(), key=lambda s: s.t)
    n = len(frames)

    multi = sum(1 for s in frames if s.n_faces > 1)
    if multi / n > cfg.crowd_frac:
        return SpeakerGuess("crowd", frames=n, detail=f"{multi}/{n} 帧里不止一张脸")

    ids = Counter(s.who for s in frames if s.who)
    if ids:
        pid, hits = ids.most_common(1)[0]
        scores = [s.match for s in frames if s.who == pid and s.match is not None]
        med = statistics.median(scores) if scores else None
        if (hits >= cfg.min_hits and hits * 2 > n
                and med is not None and med >= cfg.accept):
            return SpeakerGuess("known", pid=pid, score=med, frames=n,
                                detail=f"{pid} {hits}/{n} 帧，相似度中位 {med:.2f}",
                                sightings=frames)
        return SpeakerGuess("unsure", frames=n,
                            detail=f"候选 {dict(ids)}，最高 {pid} {hits}/{n} 帧、"
                                   f"中位 {med if med is None else round(med, 2)}",
                            sightings=frames)

    if n >= cfg.min_frames_stranger:
        best = _best_match(frames)
        extra = (f"，最高相似度 {best.match:.2f}（低于板子门限）"
                 if best is not None else "")
        return SpeakerGuess("stranger", frames=n,
                            detail=f"{n} 帧清楚的单人脸、库里没有{extra}",
                            sightings=frames)
    return SpeakerGuess("none", frames=n, detail=f"只看清 {n} 帧，不够判断")
