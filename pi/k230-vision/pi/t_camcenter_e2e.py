# /home/cy/k230-vision/pi/t_camcenter_e2e.py —— 云台居中的**端到端**验收
#
# 前面那些测试都是"假云台 + 假结果流"。这一个接真的：
#   读真的 K230 结果流 → 真的 CarController → 真的云台转动
#
# ## 挑哪个目标
#
# ⚠️ 第一版挑"最大的框"，结果挑到的正好是**本来就锁得住**的那个 —— 0 步就
# `centered`，等于什么都没测。改成**优先挑"锁不住"的框**（那才是这个模块存在的理由）；
# 全都锁得住时才退回最大的那个，并明说"没活干"。
#
# ## 怎么跨帧跟住它
#
# 检测器的框**没有 ID**（`track_id` 只给 `src:"track"`）。所以用"**同类框里
# 离上一次位置最近的那个**"来跟 —— 云台每步只转几度，框不会跳太远。
#
# 用法（在树莓派上）：
#     source /opt/ros/jazzy/setup.bash
#     cd /home/cy/k230-vision/pi && python3 -u t_camcenter_e2e.py

import json
import sys
import time

sys.path.insert(0, "/home/cy/k230-vision/pi")

from camcenter import CamCenterer, CenterConfig, describe, CENTERED   # noqa: E402
from geom import box_complete                                        # noqa: E402

RESULT = "/tmp/k230/latest-result.json"
MARGIN = 0.05


def _load():
    try:
        with open(RESULT) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _area(b):
    return (b[2] - b[0]) * (b[3] - b[1])


def _ctr(b):
    return ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)


def pick_target():
    """优先挑"锁不住"的框（面积最大的那个）；都没有就退回最大的。"""
    d = _load()
    if not d or not (d.get("objs") or []):
        return None
    need, any_ok = [], []
    for o in d["objs"]:
        (need if not box_complete(o["box"], MARGIN) else any_ok).append(o)
    pool, why = (need, "锁不住") if need else (any_ok, "本来就锁得住")
    best = max(pool, key=lambda o: _area(o["box"]))
    return best, why, d.get("frame")


def make_reader(cls, start_box):
    """同类框里离上一次最近的 → 跟住同一个目标（检测框没有 ID）。"""
    state = {"c": _ctr(start_box)}

    def read():
        d = _load()
        if not d:
            return None
        cands = [o["box"] for o in (d.get("objs") or []) if o["cls"] == cls]
        if not cands:
            return None
        cx, cy = state["c"]
        best = min(cands, key=lambda b: (_ctr(b)[0] - cx) ** 2 + (_ctr(b)[1] - cy) ** 2)
        state["c"] = _ctr(best)
        return (best, d.get("frame"))

    return read


def make_nudge():
    sys.path.insert(0, "/home/cy/voice-chatbot")
    from car.controller import CarController

    class _Cfg:
        def get(self, key, default=None):
            return default

    ctrl = CarController(_Cfg())
    ctrl.start()
    return ctrl.camera_nudge


def main():
    print("=" * 72)
    got = pick_target()
    if got is None:
        print("结果流里没有 objs —— 画面里没东西可测")
        return 1
    obj, why, frame = got
    box = obj["box"]
    print("目标：%s   box=[%.3f, %.3f, %.3f, %.3f]  frame=%s"
          % (obj["cls"], box[0], box[1], box[2], box[3], frame))
    print("为什么挑它：%s" % why)
    if why == "本来就锁得住":
        print("⚠️ 画面里没有锁不住的目标 —— 这次测不出东西（挪个东西到画面边上去）")
        return 2

    nudge = make_nudge()
    print("CarController 就绪，开始居中…")
    t0 = time.time()

    c = CamCenterer(nudge, make_reader(obj["cls"], box), CenterConfig())
    res = c.center()

    print()
    print("耗时 %.1fs  步数 %d" % (time.time() - t0, res.steps))
    print("转动序列：%s" % [(d, round(g, 1)) for d, g in res.pulses])
    if res.box is not None:
        print("收尾目标：box=[%.3f, %.3f, %.3f, %.3f]  锁得住=%s"
              % (res.box[0], res.box[1], res.box[2], res.box[3],
                 box_complete(res.box, MARGIN)))
    print("结果：%s（%s）" % (res.outcome, res.note))
    print(describe(res))
    print("E2E_DONE")
    return 0 if res.outcome == CENTERED else 1


if __name__ == "__main__":
    sys.exit(main())
