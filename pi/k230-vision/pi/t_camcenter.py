# /home/cy/k230-vision/pi/t_camcenter.py —— camcenter.py 的纯逻辑测试
#
# **不接 ROS、不接云台、不碰 socket** —— 假云台 + 假结果流，秒级跑完。
# 每次跑完出 PASS/FAIL；有 FAIL 时退出码 1。
#
# 本测试**能红**（见文件末尾的用法）：故意改错 camcenter 的方向，第 1 组必须失败。

import sys

from camcenter import (CamCenterer, CenterConfig, CENTERED, HIT_LIMIT,
                       NO_PROGRESS, NO_TARGET, STALE, TIMEOUT, BOX_TOO_BIG,
                       describe)
from geom import box_complete

FAILS = []


def chk(cond, label):
    print(("  ok    " if cond else "  FAIL  ") + label)
    if not cond:
        FAILS.append(label)


# ---------------- 假的云台 + 结果流 ----------------
#
# 物理：画面里目标的位置 = 目标在世界里的位置 − 云台的偏角。
#   相机往右转(pan+)，目标在画面里往左移。
#   「left」→ pan 减小。

# 2026-09-19 真车实测的每度位移（t_camcalib.py 标出来的）。
# **两轴差 2.4 倍** —— 测试里必须照实写，否则验不出"按轴分开的增益"这件事。
DEG_TO_NORM = {"pan": 0.013047, "tilt": 0.031389}


class FakeGimbal:
    def __init__(self, invert=False, limit=None):
        self.pan = 0.0
        self.tilt = 0.0
        self.invert = invert
        self.limit = limit          # pan 的活动范围 ±limit（None=不限制）
        self.calls = []
        self.hits = 0

    def __call__(self, direction, degrees):
        axis = "pan" if direction in ("left", "right") else "tilt"
        step = DEG_TO_NORM[axis] * abs(float(degrees))
        if self.invert:
            step = -step
        hit = False
        if direction == "left":
            self.pan -= step
        elif direction == "right":
            self.pan += step
        elif direction == "up":
            self.tilt -= step
        elif direction == "down":
            self.tilt += step
        else:
            raise KeyError(direction)
        if self.limit is not None and abs(self.pan) > self.limit:
            self.pan = self.limit if self.pan > 0 else -self.limit
            hit = True
            self.hits += 1
        self.calls.append((direction, degrees))
        return (1500, hit)          # (脉宽, 是否撞限位)


class FakeStream:
    """结果流。auto_tick=False 时帧号不动 —— 用来复现「结果流卡住」。"""

    def __init__(self, gimbal, target_uv, box_wh, auto_tick=True):
        self.g = gimbal
        self.tu, self.tv = target_uv
        self.w, self.h = box_wh
        self.frame = 0
        self.auto_tick = auto_tick

    def __call__(self):
        cx = self.tu - self.g.pan
        cy = self.tv - self.g.tilt
        box = [cx - self.w / 2, cy - self.h / 2, cx + self.w / 2, cy + self.h / 2]
        if self.auto_tick:
            self.frame += 1
        return (box, self.frame)


class NoTarget:
    def __call__(self):
        return None


def run(rotator, reader, **kw):
    cfg = CenterConfig(**kw)
    # 时钟/sleep 都换成假的 —— 测试瞬间跑完
    t = [0.0]

    def clock():
        return t[0]

    def sleeper(s):
        t[0] += s

    c = CamCenterer(rotator, reader, cfg, clock=clock, sleeper=sleeper)
    return c.center()


print("=" * 72)
print("1) 目标在画面外（贴在右边）—— 应该转回来并判 CENTERED")
g = FakeGimbal()
s = FakeStream(g, (1.15, 0.5), (0.10, 0.20))
r = run(g, s)
chk(r.outcome == CENTERED, "结果 = CENTERED（实得 %s / %s）" % (r.outcome, r.note))
chk(g.pan > 0, "云台往右转了（pan=%.3f）" % g.pan)
chk(all(c[0] == "right" for c in g.calls),
    "每一动都是 right（实得 %s）" % [c[0] for c in g.calls])
chk(r.box is not None and box_complete(r.box, 0.05),
    "收尾时框确实完整可见 %s" % (r.box,))
chk(1 <= r.steps <= 6, "步数合理（%d 步）" % r.steps)

print()
print("2) 方向反了 —— 必须判 NO_PROGRESS，不能傻转到超时")
g = FakeGimbal(invert=True)
s = FakeStream(g, (1.15, 0.5), (0.10, 0.20))
r = run(g, s)
chk(r.outcome == NO_PROGRESS, "结果 = NO_PROGRESS（实得 %s）" % r.outcome)
chk(r.steps < CenterConfig().max_steps,
    "没转满就放弃（%d < %d 步）" % (r.steps, CenterConfig().max_steps))

print()
print("3) 云台撞行程限位 —— 必须判 HIT_LIMIT 而不是一直转")
g = FakeGimbal(limit=0.05)
s = FakeStream(g, (1.15, 0.5), (0.10, 0.20))
r = run(g, s)
chk(r.outcome == HIT_LIMIT, "结果 = HIT_LIMIT（实得 %s）" % r.outcome)
chk(g.hits > 0, "假云台确实报过限位")

print()
print("4) 画面里没目标 —— 必须判 NO_TARGET，且一步都不转")
g = FakeGimbal()
r = run(g, NoTarget())
chk(r.outcome == NO_TARGET, "结果 = NO_TARGET（实得 %s）" % r.outcome)
chk(len(g.calls) == 0, "没有转过云台（%d 次）" % len(g.calls))

print()
print("5) 结果流卡住不更新 —— 必须判 STALE，而不是拿旧图反复转")
g = FakeGimbal()
s = FakeStream(g, (1.15, 0.5), (0.10, 0.20), auto_tick=False)
r = run(g, s)
chk(r.outcome == STALE, "结果 = STALE（实得 %s）" % r.outcome)
chk(len(g.calls) <= 1, "最多转过一次就发现流不动了（%d 次）" % len(g.calls))

print()
print("6) 目标本来就在安全区 —— 直接 CENTERED，一步都不转")
g = FakeGimbal()
s = FakeStream(g, (0.5, 0.5), (0.10, 0.20))
r = run(g, s)
chk(r.outcome == CENTERED, "结果 = CENTERED（实得 %s）" % r.outcome)
chk(r.steps == 0 and len(g.calls) == 0, "零步零转动（steps=%d calls=%d）"
    % (r.steps, len(g.calls)))

print()
print("6b) 俯仰轴：增益和步长上限是**另一套**（实测差 2.4 倍），别串了")
g = FakeGimbal()
s = FakeStream(g, (0.5, 1.15), (0.10, 0.20))
r = run(g, s)
chk(r.outcome == CENTERED, "结果 = CENTERED（实得 %s / %s）" % (r.outcome, r.note))
chk(all(c[0] == "down" for c in g.calls),
    "每一动都是 down（实得 %s）" % [c[0] for c in g.calls])
chk(all(c[1] <= 4.5 + 1e-6 for c in g.calls),
    "俯仰每步不超过 4.5°（实得 %s）" % [round(c[1], 2) for c in g.calls])
chk(r.box is not None and box_complete(r.box, 0.05),
    "收尾时框确实完整可见 %s" % (r.box,))

print()
print("6c) 目标框比安全区还大 —— 必须当场判 BOX_TOO_BIG，不能傻转到超时")
g = FakeGimbal()
s = FakeStream(g, (0.5, 0.5), (0.98, 0.98))
r = run(g, s)
chk(r.outcome == BOX_TOO_BIG, "结果 = BOX_TOO_BIG（实得 %s）" % r.outcome)
chk(len(g.calls) == 0, "一步都不转（%d 次）" % len(g.calls))

print()
print("7) 每一档结果都得有一句人话（describe 不能漏）")
for oc in (CENTERED, HIT_LIMIT, NO_PROGRESS, TIMEOUT, NO_TARGET, STALE,
           BOX_TOO_BIG):
    class _R:
        pass
    _r = _R()
    _r.ok = (oc == CENTERED)
    _r.outcome = oc
    _r.steps = 3
    chk(bool(describe(_r)), "%s -> 有话可说" % oc)

print()
print("=" * 72)
if FAILS:
    print("结果: %d 项失败" % len(FAILS))
    for f in FAILS:
        print("   -", f)
    sys.exit(1)
print("结果: 全部通过")
sys.exit(0)
