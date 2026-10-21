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


def guess_speaker(sightings: list[FaceSighting], cfg: SpeakerConfig | None = None) -> SpeakerGuess:
    cfg = cfg or SpeakerConfig()
    good = [s for s in sightings
            if s.ok and (s.live is None or s.live >= cfg.live_min)]
    if not good:
        return SpeakerGuess("none", detail=f"{len(sightings)} 张脸都不可用（质量/活体）")

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
        return SpeakerGuess("stranger", frames=n, detail=f"{n} 帧清楚的单人脸、库里没有",
                            sightings=frames)
    return SpeakerGuess("none", frames=n, detail=f"只看清 {n} 帧，不够判断")
