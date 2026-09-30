# /home/cy/k230-vision/pi/t_gimbal_track.py —— gimbal_track.py 的纯逻辑测试
#
# **不接 ROS、不接云台、不碰 socket、不读 /tmp/k230** —— 假云台 + 假结果流，秒级跑完。
#
#     cd /home/cy/k230-vision/pi && python3 t_gimbal_track.py
#
# 最后一行是结束标记：全过 = GIMBAL_TRACK_TESTS_DONE，有 FAIL = GIMBAL_TRACK_TESTS_FAILED
# （退出码 1）。
#
# 测的是**控制律本身**：每帧一步、方向对不对、度数对不对，以及
# 同帧 / 丢目标 / 过期 / 死区 / 撞限位 / 两轴同时动 这几条路径走没走到。
# 不是端到端的闭环收敛仿真（那要真车）。
#
# ⚠️ 这里**故意不模拟"真车每度转多少像素"**：绝大多数断言查的是控制律的输出
# （方向 + 度数），和被控对象的绝对灵敏度无关。只有第 11 组的假对象用了
# 17 px/度，而那组只断言"误差单调变小、最后停在死区里"，同样只依赖灵敏度是正数。
#
# 第 14 组起是**三道守卫**（框有效性 / 无进展 / 数据陈旧）的用例，其中第 14 组是
# 2026-09-21 那次跑飞的**回归样本**（真实的 du 和框都原样抄在用例里）。
#
# 本测试**能红**：变异自检记录见文件末尾注释。

import sys

from geom import box_complete, safe_center_range
from gimbal_track import (GimbalTracker, TrackConfig, describe,
                          HOLD, TRACKED, NO_TARGET, NO_NEW_FRAME, STALE,
                          HIT_LIMIT, BAD_BOX, BOX_CLIPPED, BOX_TOO_BIG,
                          NO_PROGRESS)

FAILS = []


def chk(cond, label):
    print(("  ok    " if cond else "  FAIL  ") + label)
    if not cond:
        FAILS.append(label)


def nearly(a, b, tol=1e-9):
    return abs(a - b) <= tol


def call_eq(got, direction, deg, tol=1e-9):
    """云台收到的这条指令是不是 (direction, deg)。度数**必须用容差比** ——
    16 * 0.2 = 3.1999999999999993 这种二进制浮点误差不该让测试变红。"""
    return (got is not None and got[0] == direction
            and abs(got[1] - deg) <= tol)


# ---------------- 假的云台 + 假的结果流 ----------------
#
# 符号约定（照 t_camcenter.py）：画面里目标的位置 = 目标在世界里的位置 − 云台的偏角。
#   相机往右转(pan+)，目标在画面里往左移。「left」→ pan 减小。

# 实测：两轴像素灵敏度都是 ~17 px/度。**只在第 11 组的假对象里用**。
PX_PER_DEG = 17.0
W, H = 1280, 720
NORM_PER_DEG = {"pan": PX_PER_DEG / W, "tilt": PX_PER_DEG / H}


def box_at(cx, cy, w=0.10, h=0.20):
    """中心在 (cx, cy) 的归一化框。"""
    return [cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0]


class FakeGimbal:
    """假云台。`limit_axes` 里的轴**永远**报撞限位（而且纹丝不动）—— 用来验
    "撞了限位那个方向就不再推，但另一轴照常"。"""

    def __init__(self, limit_axes=(), invert=False):
        self.pan = 0.0
        self.tilt = 0.0
        self.calls = []                    # [(direction, degrees)]
        self.limit_axes = set(limit_axes)
        self.invert = invert               # True = 云台反着走（验方向用）
        self.hits = 0

    def __call__(self, direction, degrees):
        if direction not in ("left", "right", "up", "down"):
            raise KeyError(direction)
        axis = "pan" if direction in ("left", "right") else "tilt"
        deg = abs(float(degrees))
        self.calls.append((direction, float(degrees)))
        if axis in self.limit_axes:
            self.hits += 1
            return (1500, True)            # (脉宽, 撞限位) —— 云台没动
        step = NORM_PER_DEG[axis] * deg * (-1.0 if self.invert else 1.0)
        if direction == "left":
            self.pan -= step
        elif direction == "right":
            self.pan += step
        elif direction == "up":
            self.tilt -= step
        else:
            self.tilt += step
        return (1500, False)

    def axes_called(self):
        return [("pan" if d in ("left", "right") else "tilt") for d, _ in self.calls]

    def n_calls(self, axis):
        return sum(1 for d, _ in self.calls
                   if (d in ("left", "right")) == (axis == "pan"))

    def last(self, axis):
        got = [(d, g) for d, g in self.calls
               if (d in ("left", "right")) == (axis == "pan")]
        return got[-1] if got else None


class BoxStream:
    """吐一个**固定**的框；帧号由测试手动 tick() 控制。单帧断言用这个最清楚。"""

    def __init__(self, box, frame=1):
        self.box = box
        self.frame = frame

    def tick(self):
        self.frame += 1
        return self

    def __call__(self):
        return (list(self.box), self.frame)


class ScriptedStream:
    """按剧本吐框：第 i 次调用吐 boxes[i]（越界就一直重复最后一条），帧号每次 +1。

    用来验"误差冻住 / 缓慢改善 / 中途改善"这几条**跟时间序列有关**的守卫 ——
    单帧/固定框用 BoxStream 手动 tick()，序列就用这个。
    """

    def __init__(self, boxes):
        self.boxes = [list(b) for b in boxes]
        self.frame = 0

    def __call__(self):
        self.frame += 1
        i = min(self.frame - 1, len(self.boxes) - 1)
        return (list(self.boxes[i]), self.frame)


class MovingStream:
    """会动的流：目标在画面里的位置 = 世界位置 − 云台偏角。第 11 组用。"""

    def __init__(self, gimbal, target_uv, box_wh=(0.10, 0.20)):
        self.g = gimbal
        self.tu, self.tv = target_uv
        self.w, self.h = box_wh
        self.frame = 0

    def __call__(self):
        self.frame += 1
        return (box_at(self.tu - self.g.pan, self.tv - self.g.tilt, self.w, self.h),
                self.frame)


class NoData:
    def __call__(self):
        return None


class Switch:
    """中途换掉 read_target（验"跟到一半数据源断了"）。"""

    def __init__(self, reader):
        self.reader = reader

    def __call__(self):
        return self.reader()


def mk(rotator, reader, cfg=None, **kw):
    """造一个 tracker，时钟 / sleep 都换成假的 —— 时间由测试自己推。"""
    cfg = cfg or TrackConfig(**kw)
    t = [0.0]

    def clock():
        return t[0]

    def sleeper(s):
        t[0] += s

    return GimbalTracker(rotator, reader, cfg,
                         clock=clock, sleeper=sleeper), t


C = TrackConfig()
print("=" * 72)
print("控制律参数（增益 / 符号取自 camcenter.CenterConfig，实测）：")
print("  k        pan=%.1f  tilt=%.1f   度 / 归一化偏差" % (C.k["pan"], C.k["tilt"]))
print("  max_step pan=%.1f  tilt=%.1f   度 / 帧" % (C.max_step["pan"], C.max_step["tilt"]))
print("  sign     pan=%+d  tilt=%+d" % (C.pan_sign, C.tilt_sign))
print("  deadband pan=%.3f  tilt=%.3f  stale_s=%.1f  poll_s=%.2f"
      % (C.deadband["pan"], C.deadband["tilt"], C.stale_s, C.poll_s))
print("=" * 72)

print()
print("1) 目标就在画面中心 —— HOLD，一步都不动")
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.5, 0.5)))
r = tr.update(now=0.0)
chk(r.outcome == HOLD, "结果 = HOLD（实得 %s / %s）" % (r.outcome, r.note))
chk(len(g.calls) == 0, "一次都没转（%d 次）" % len(g.calls))
chk(r.ok and not r.moved, "ok=True 且 moved=False")

print()
print("1b) 偏差在死区内的小抖动 —— 也不动（防抖，别在中心附近来回蹭）")
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.515, 0.5)))     # |du| = 0.015 < deadband 0.03
r = tr.update(now=0.0)
chk(r.outcome == HOLD, "结果 = HOLD（实得 %s）" % r.outcome)
chk(len(g.calls) == 0, "一次都没转（%d 次）" % len(g.calls))
chk(r.axes["pan"].skipped == "deadband", "pan 的 skip 原因是 deadband（实得 %s）"
    % r.axes["pan"].skipped)

print()
print("2) 目标偏右 —— pan 往 right 转，度数 = k_pan × |d|")
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.75, 0.5)))      # du = -0.25
r = tr.update(now=0.0)
chk(r.outcome == TRACKED, "结果 = TRACKED（实得 %s / %s）" % (r.outcome, r.note))
chk(call_eq(g.last("pan"), "right", C.k["pan"] * 0.25),
    "方向/度数都对：实得 %s，应得 ('right', %.2f)" % (g.last("pan"), C.k["pan"] * 0.25))
chk(nearly(r.axes["pan"].deg, C.k["pan"] * 0.25), "axes[pan].deg = %.2f" % r.axes["pan"].deg)
chk(r.axes["tilt"].skipped == "deadband", "tilt 偏 0 所以在死区内（实得 %s）"
    % r.axes["tilt"].skipped)

print()
print("2b) 目标偏左 —— 对称地往 left 转")
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.25, 0.5)))      # du = +0.25
r = tr.update(now=0.0)
chk(call_eq(g.last("pan"), "left", C.k["pan"] * 0.25),
    "方向/度数都对：实得 %s，应得 ('left', %.2f)" % (g.last("pan"), C.k["pan"] * 0.25))
chk(r.outcome == TRACKED, "结果 = TRACKED（实得 %s）" % r.outcome)

print()
print("2c) 目标偏上 —— tilt 往 **up** 转，且用的是 tilt 自己那套增益")
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.5, 0.30)))      # dv = +0.20
r = tr.update(now=0.0)
chk(call_eq(g.last("tilt"), "up", C.k["tilt"] * 0.20),
    "方向/度数都对：实得 %s，应得 ('up', %.2f)" % (g.last("tilt"), C.k["tilt"] * 0.20))
chk(nearly(r.axes["tilt"].deg, C.k["tilt"] * 0.20), "axes[tilt].deg = %.2f"
    % r.axes["tilt"].deg)

print()
print("2d) 目标偏下 —— tilt 往 down 转")
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.5, 0.70)))      # dv = -0.20
r = tr.update(now=0.0)
chk(call_eq(g.last("tilt"), "down", C.k["tilt"] * 0.20),
    "方向/度数都对：实得 %s，应得 ('down', %.2f)" % (g.last("tilt"), C.k["tilt"] * 0.20))

print()
print("3) 偏差很大 —— 被 max_step **按轴分别**截住（不是共用一个上限）")
# ⚠️ 2026-09-21：这组原来用 box_at(1.20, 1.20)（框整个在画面外）造 |d|=0.70。
# 那种框现在被**守卫 ①** 拦下了（框被切 → 反馈不可信 → 一条指令都不发，见第 14 组），
# 所以改成用**完整落在画面内**的框把每轴压到上限。
# 顺带一个结论：pan 的 k=38 而画面内的框最多只能给 |du|≈0.45（38×0.45=17.1 < 20）——
# **画面内的目标永远压不到 pan 的上限**，这道夹取实际只在框被切/坏时才起作用；
# 要验它只能像下面这样把 k_pan 调大。
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.50, 0.20)))     # dv = +0.30 → 16×0.30 = 4.8 > 4.5
r = tr.update(now=0.0)
chk(nearly(r.axes["tilt"].deg, C.max_step["tilt"]),
    "tilt 被截到 %.1f°（实得 %.2f°）" % (C.max_step["tilt"], r.axes["tilt"].deg))
chk(call_eq(g.last("tilt"), "up", C.max_step["tilt"]), "截住的是度数，方向不变")
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.12, 0.50)), k_pan=100.0)   # 100×0.38 = 38 > 20
r = tr.update(now=0.0)
chk(nearly(r.axes["pan"].deg, C.max_step["pan"]),
    "pan 被截到 %.1f°（实得 %.2f°）" % (C.max_step["pan"], r.axes["pan"].deg))
chk(call_eq(g.last("pan"), "left", C.max_step["pan"]), "方向不变")
chk(C.max_step["tilt"] < C.max_step["pan"], "两轴上限确实不是同一个数")

print()
print("4) 同一帧不重复动 —— 帧号没变就一步都不发")
g = FakeGimbal()
s = BoxStream(box_at(0.75, 0.5), frame=7)
tr, t = mk(g, s)
r1 = tr.update(now=0.00)
r2 = tr.update(now=0.02)
chk(r1.outcome == TRACKED, "第一拍 TRACKED（实得 %s）" % r1.outcome)
chk(r2.outcome == NO_NEW_FRAME, "第二拍 NO_NEW_FRAME（实得 %s）" % r2.outcome)
chk(len(g.calls) == 1, "只发过 1 次指令（实得 %d 次）" % len(g.calls))
chk(r2.pulses == [], "第二拍 pulses 为空（%s）" % r2.pulses)
s.tick()
r3 = tr.update(now=0.04)
chk(r3.outcome == TRACKED and len(g.calls) == 2,
    "换新帧后又动了（%s / %d 次）" % (r3.outcome, len(g.calls)))

print()
print("5) 跟到一半目标丢了 —— 原地保持，什么都不发")
g = FakeGimbal()
sw = Switch(BoxStream(box_at(0.75, 0.5), frame=9))
tr, t = mk(g, sw)
r1 = tr.update(now=0.0)
n1 = len(g.calls)
sw.reader = NoData()                       # 流还活着（0.2s < stale_s），只是没目标
r2 = tr.update(now=0.2)
chk(r1.outcome == TRACKED, "第一拍还在跟（%s）" % r1.outcome)
chk(r2.outcome == NO_TARGET, "第二拍 NO_TARGET（实得 %s / %s）" % (r2.outcome, r2.note))
chk(len(g.calls) == n1, "没有多发指令（%d -> %d）" % (n1, len(g.calls)))

print()
print("6) 数据过期 —— 保持，什么都不发")
g = FakeGimbal()
tr, t = mk(g, NoData())                    # 从没见过任何数据
r0 = tr.update(now=0.0)
chk(r0.outcome == STALE, "从没拿到过数据 -> STALE（实得 %s）" % r0.outcome)
chk(len(g.calls) == 0, "一步都没转（%d 次）" % len(g.calls))

g = FakeGimbal()
sw = Switch(BoxStream(box_at(0.75, 0.5), frame=3))
tr, t = mk(g, sw)
tr.update(now=0.0)
n1 = len(g.calls)
sw.reader = NoData()
r1 = tr.update(now=0.5)
chk(r1.outcome == NO_TARGET, "断了 0.5s（< stale_s=1.0）仍算 NO_TARGET（实得 %s）"
    % r1.outcome)
r2 = tr.update(now=2.0)
chk(r2.outcome == STALE, "断了 2.0s（>= stale_s）-> STALE（实得 %s / %s）"
    % (r2.outcome, r2.note))
chk(len(g.calls) == n1, "整段都没再发指令（%d -> %d）" % (n1, len(g.calls)))

print()
print("7) 撞行程限位 —— 那个方向不再推，**另一轴照常**")
g = FakeGimbal(limit_axes=("pan",))
s = BoxStream(box_at(0.75, 0.70), frame=1)      # 两轴都偏
tr, t = mk(g, s)
r1 = tr.update(now=0.0)
chk(r1.outcome == HIT_LIMIT, "结果 = HIT_LIMIT（实得 %s / %s）" % (r1.outcome, r1.note))
chk(r1.axes["pan"].hit_limit, "pan 那一步确实撞了限位")
chk(call_eq(g.last("tilt"), "down", C.k["tilt"] * 0.20),
    "**另一轴照常发了指令**：%s" % (g.last("tilt"),))
n_pan = g.n_calls("pan")
n_tilt = g.n_calls("tilt")

s.tick()
r2 = tr.update(now=0.1)
chk(g.n_calls("pan") == n_pan, "同方向不再往 pan 推（%d -> %d）" % (n_pan, g.n_calls("pan")))
chk(g.n_calls("tilt") == n_tilt + 1, "tilt 照常继续跟（%d -> %d）"
    % (n_tilt, g.n_calls("tilt")))
chk(r2.axes["pan"].blocked and r2.axes["pan"].skipped == "blocked",
    "pan 被 latch 标成 blocked")
chk(r2.axes["pan"].direction is None and r2.axes["pan"].wanted == "right",
    "被拦下的 pan 不算「发过指令」（direction=None, wanted=right）")
chk(r2.outcome == TRACKED and len(r2.pulses) == 1,
    "有轴在动就算 TRACKED（实得 %s, %d 条脉冲）" % (r2.outcome, len(r2.pulses)))

print()
print("7b) 只有 pan 偏、而且它已顶在限位上 —— HIT_LIMIT，且一次都不发")
g = FakeGimbal(limit_axes=("pan",))
s = BoxStream(box_at(0.75, 0.5), frame=1)
tr, t = mk(g, s)
tr.update(now=0.0)
n1 = len(g.calls)
s.tick()
r = tr.update(now=0.1)
chk(r.outcome == HIT_LIMIT, "结果 = HIT_LIMIT（实得 %s / %s）" % (r.outcome, r.note))
chk(len(g.calls) == n1, "没有继续往限位方向推（%d -> %d）" % (n1, len(g.calls)))
chk(tr._blocked["pan"] == "right", "pan 的 latch 记在 right 上（实得 %s）"
    % tr._blocked["pan"])

print()
print("7c) 目标转到**另一边** —— 被限位的那个方向要放行")
print("    （否则一次瞬时的限位会把这一轴永久废掉）")
s.box = box_at(0.25, 0.5)          # du = +0.25 -> 该往 left 了
s.tick()
r = tr.update(now=0.2)
chk(r.axes["pan"].direction == "left", "pan 方向翻成 left（实得 %s）"
    % r.axes["pan"].direction)
chk(call_eq(g.last("pan"), "left", C.k["pan"] * 0.25),
    "真的发出了 left 指令（实得 %s）" % (g.last("pan"),))

print()
print("8) 两轴同时偏 —— **同一次 update 里两轴都动**（不是像 camcenter 那样一次只动一轴）")
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.75, 0.70)))
r = tr.update(now=0.0)
chk(r.outcome == TRACKED, "结果 = TRACKED（实得 %s）" % r.outcome)
chk(len(r.pulses) == 2, "一拍发了 2 条脉冲（实得 %s）" % (r.pulses,))
chk(set(g.axes_called()) == {"pan", "tilt"}, "两轴都发了（%s）" % g.axes_called())
chk(len(g.calls) == 2 and call_eq(g.calls[0], "right", C.k["pan"] * 0.25)
    and call_eq(g.calls[1], "down", C.k["tilt"] * 0.20),
    "方向和度数都对（实得 %s）" % (g.calls,))

print()
print("9) 坏框不吃 —— 退化成点 / 不是 4 个数，都不许动")
g = FakeGimbal()
tr, t = mk(g, BoxStream([0.5, 0.5, 0.5, 0.5]))   # 零面积
r = tr.update(now=0.0)
chk(r.outcome == BAD_BOX, "零面积框 -> BAD_BOX（实得 %s / %s）" % (r.outcome, r.note))
chk(len(g.calls) == 0, "一步都没转（%d 次）" % len(g.calls))

g = FakeGimbal()
tr, t = mk(g, BoxStream([0.1, 0.2, 0.3]))        # 三个数
r = tr.update(now=0.0)
chk(r.outcome == BAD_BOX, "三个数 -> BAD_BOX（实得 %s）" % r.outcome)
chk(len(g.calls) == 0, "一步都没转（%d 次）" % len(g.calls))

print()
print("10) pan_sign / tilt_sign 确实被用上了（配反了方向就得反过来）")
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.25, 0.5)), pan_sign=-1)
r = tr.update(now=0.0)
chk(r.axes["pan"].direction == "right",
    "pan_sign=-1 时，偏左的目标改成往 right 转（实得 %s）" % r.axes["pan"].direction)
g = FakeGimbal()
tr, t = mk(g, BoxStream(box_at(0.5, 0.30)), tilt_sign=-1)
r = tr.update(now=0.0)
chk(r.axes["tilt"].direction == "down",
    "tilt_sign=-1 时，偏上的目标改成往 down 转（实得 %s）" % r.axes["tilt"].direction)

print()
print("11) 多帧闭环：误差单调变小，最后停进死区（假对象，符号对就行）")
g = FakeGimbal()
# ⚠️ 起点从 (0.90, 0.80) 挪到 (0.88, 0.78)：0.90 那份的框右边正好是
#    0.90 + 0.05 = 0.9500000000000001 > 1 - margin = 0.95，被守卫 ① 判成
#    "框被边缘切掉"。这是浮点边界，不是逻辑问题 —— 但用例不该卡在边界上。
s = MovingStream(g, (0.88, 0.78))
tr, t = mk(g, s)
errs = []
outcomes = []
for i in range(40):
    r = tr.update(now=i * 0.05)
    outcomes.append(r.outcome)
    if r.outcome in (TRACKED, HOLD):
        l, tt, rr, b = r.box
        errs.append(abs(0.5 - (l + rr) / 2.0) + abs(0.5 - (tt + b) / 2.0))
    if r.outcome == HOLD:
        break
chk(outcomes[-1] == HOLD, "最后收敛到 HOLD（实得 %s，走了 %d 帧）"
    % (outcomes[-1], len(outcomes)))
chk(all(errs[i + 1] <= errs[i] + 1e-12 for i in range(len(errs) - 1)),
    "误差单调不增（%s）" % [round(e, 3) for e in errs])
chk(errs[-1] <= C.deadband["pan"] + C.deadband["tilt"] + 1e-9,
    "收尾残差在死区量级（%.3f）" % errs[-1])
chk(all(d == "right" for d, _ in g.calls if d in ("left", "right")),
    "pan 全程只往 right（%s）" % [d for d, _ in g.calls if d in ("left", "right")])
chk(all(d == "down" for d, _ in g.calls if d in ("up", "down")),
    "tilt 全程只往 down（%s）" % [d for d, _ in g.calls if d in ("up", "down")])
chk(len(errs) >= 3, "至少走了 3 帧才收敛，收敛不是因为一步到位（%d 帧）" % len(errs))

print()
print("12) run() 常驻循环：睡 poll_s、同一帧不重复发、stop() 能退出")
g = FakeGimbal()
s = BoxStream(box_at(0.75, 0.5), frame=1)        # 永远同一帧
tr, t = mk(g, s)
seen = []
counter = [0]


def stop():
    counter[0] += 1
    return counter[0] > 5


tr.run(stop=stop, on_result=seen.append)
chk(len(g.calls) == 1, "同一帧跑了 5 拍也只发 1 次指令（实得 %d）" % len(g.calls))
chk(len(seen) == 5, "回调收到 5 拍（实得 %d）" % len(seen))
chk(seen[0].outcome == TRACKED
    and all(x.outcome == NO_NEW_FRAME for x in seen[1:]),
    "第一拍跟、后面都是 NO_NEW_FRAME（%s）" % [x.outcome for x in seen])
chk(nearly(t[0], 5 * C.poll_s, 1e-9), "走的是注入的假时钟（t=%.3f，应 %.3f）"
    % (t[0], 5 * C.poll_s))

print()
print("13) 每一档结果都得有一句人话（describe 不能漏）")
for oc in (HOLD, TRACKED, NO_TARGET, NO_NEW_FRAME, STALE, HIT_LIMIT, BAD_BOX,
           BOX_CLIPPED, BOX_TOO_BIG, NO_PROGRESS):
    class _R:
        pass
    _r = _R()
    _r.outcome = oc
    _r.axes = {}
    _r.pulses = []
    _r.moved = False
    chk(bool(describe(_r)), "%s -> 有话可说" % oc)
chk("云台正在跟" in describe(
    type("R", (), {"outcome": TRACKED,
                   "axes": {"pan": type("S", (), {"direction": "right",
                                                  "deg": 9.5, "blocked": False})()},
                   "pulses": [("right", 9.5)], "moved": True})()),
    "TRACKED 的话里带方向和度数")

print()
print("14) 【2026-09-21 跑飞回归样本】框贴着画面左边缘被切 —— 一条指令都不许发")
# 实况：一个**完全静止**的目标（椅子）被画面左边缘切掉（l≈0.000），跟踪器照旧报
# du≈+0.41 而且**纹丝不动**，环拿这个废反馈继续全额叠指令：
#   误差 du:  +0.4114 +0.4086 +0.4070 +0.4077 +0.4083 +0.4075 +0.4045 +0.4036 ... +0.3916
#   目标框:  [0.001,0.346,0.176,0.949]  →  [0.008,0.188,0.208,0.796]   （l≈0.000）
#   实际指令: ch5 left 15.4° / 15.3° / 15.4° / 15.0° / 15.1°  ← 0.76 秒 5 条，累计 76°
#   结果:    pan 撞到行程限位 800µs 才停
DU_REAL = [0.4114, 0.4086, 0.4070, 0.4077, 0.4083, 0.4075, 0.4045, 0.4036, 0.3916]
BOX_REAL = [[0.001, 0.346, 0.176, 0.949], [0.008, 0.188, 0.208, 0.796]]
# 先证明样本抄的是真值：这两个框的中心确实复现了那串误差的首尾
chk(abs(0.5 - (BOX_REAL[0][0] + BOX_REAL[0][2]) / 2.0 - DU_REAL[0]) < 2e-3,
    "样本自洽：框 1 的中心算出 du=%.4f（记录 %.4f）"
    % (0.5 - (BOX_REAL[0][0] + BOX_REAL[0][2]) / 2.0, DU_REAL[0]))
chk(abs(0.5 - (BOX_REAL[1][0] + BOX_REAL[1][2]) / 2.0 - DU_REAL[-1]) < 2e-3,
    "样本自洽：框 2 的中心算出 du=%.4f（记录 %.4f）"
    % (0.5 - (BOX_REAL[1][0] + BOX_REAL[1][2]) / 2.0, DU_REAL[-1]))
g = FakeGimbal()
tr, t = mk(g, ScriptedStream([BOX_REAL[0], BOX_REAL[0], BOX_REAL[1],
                              BOX_REAL[1], BOX_REAL[0]]))
outs = [tr.update(now=i * 0.15) for i in range(5)]      # 0.76s 里那几拍的节奏
chk(len(g.calls) == 0, "**一条指令都没发**（实得 %d 条：%s）" % (len(g.calls), g.calls))
chk(all(o.outcome == BOX_CLIPPED for o in outs),
    "每一拍都是 BOX_CLIPPED（实得 %s）" % [o.outcome for o in outs])
chk(all(not o.moved for o in outs), "pulses 全空（%s）" % [o.pulses for o in outs])
# 对照：那 15° 是怎么来的 —— du=0.4114 时控制律本来就给这么多，守卫一拦就一条都发不出去
chk(15.0 <= min(C.k["pan"] * DU_REAL[0], C.max_step["pan"]) <= 16.0,
    "若不是守卫拦着，du=%.4f 会算出 %.2f°，正是实况里那 5 条 15.0~15.4° 的量级"
    % (DU_REAL[0], min(C.k["pan"] * DU_REAL[0], C.max_step["pan"])))

print()
print("15) 框完整落在画面内 —— 守卫不能把正常跟踪一起掐死，该发还得发")
g = FakeGimbal()
tr, t = mk(g, BoxStream([0.10, 0.40, 0.20, 0.60]))      # du=+0.35，dv=0
r = tr.update(now=0.0)
chk(box_complete([0.10, 0.40, 0.20, 0.60], C.margin), "这个框确实完整落在画面内")
chk(r.outcome == TRACKED, "结果 = TRACKED（实得 %s / %s）" % (r.outcome, r.note))
chk(call_eq(g.last("pan"), "left", C.k["pan"] * 0.35),
    "方向和度数都对（实得 %s）" % (g.last("pan"),))
chk(len(g.calls) == 1, "发了一条（实得 %d）" % len(g.calls))
# 正好卡在留边上（l 恰好 = margin）也要**放行** —— "贴着边"不等于"被切掉"
g = FakeGimbal()
tr, t = mk(g, BoxStream([0.05, 0.05, 0.45, 0.45]))      # l = t = 0.05 == margin
r = tr.update(now=0.0)
chk(r.outcome == TRACKED, "正好卡在留边上的框照常跟（实得 %s）" % r.outcome)
chk(len(g.calls) == 2, "两轴都发了（实得 %d）" % len(g.calls))

print()
print("16) 框大到连安全区都放不下 —— BOX_TOO_BIG，和 BOX_CLIPPED 分得开")
chk(BOX_TOO_BIG != BOX_CLIPPED, "两个码不是同一个")
for bad in ([0.0, 0.0, 1.0, 1.0],              # 比画面还大
            [0.02, 0.02, 0.98, 0.98],          # 0.96 x 0.96，居中都放不下
            [-0.3, 0.1, 0.9, 1.2]):            # 又大又出画
    umin, umax, vmin, vmax = safe_center_range(bad, C.margin)
    chk(umin > umax or vmin > vmax,
        "%s 的安全区确实是空的（u %.2f~%.2f / v %.2f~%.2f）"
        % ([round(v, 2) for v in bad], umin, umax, vmin, vmax))
    g = FakeGimbal()
    tr, t = mk(g, BoxStream(bad))
    r = tr.update(now=0.0)
    chk(r.outcome == BOX_TOO_BIG,
        "结果 = BOX_TOO_BIG（实得 %s / %s）" % (r.outcome, r.note))
    chk(len(g.calls) == 0, "一条指令都没发（实得 %d）" % len(g.calls))
# 对照：同样是"没整个落在画面内"，但框自己塞得进安全区 → 是 BOX_CLIPPED
g = FakeGimbal()
tr, t = mk(g, BoxStream([0.001, 0.346, 0.176, 0.949]))
r = tr.update(now=0.0)
chk(r.outcome == BOX_CLIPPED,
    "放得下的贴边框 -> BOX_CLIPPED 而不是 BOX_TOO_BIG（实得 %s）" % r.outcome)

print()
print("17) 误差**冻住不动** —— 连发 patience 帧后转 NO_PROGRESS，指令条数必须被封住")
g = FakeGimbal()
s = BoxStream(box_at(0.75, 0.5))        # du=-0.25 恒定：云台转了它也不动（废反馈）
tr, t = mk(g, s)
outs = []
for i in range(C.patience + 4):
    s.tick()
    outs.append(tr.update(now=i * 0.05))
chk(outs[C.patience - 1].outcome == TRACKED,
    "第 %d 帧还在发指令（实得 %s）" % (C.patience, outs[C.patience - 1].outcome))
chk(outs[C.patience].outcome == NO_PROGRESS,
    "第 %d 帧转 NO_PROGRESS（实得 %s / %s）"
    % (C.patience + 1, outs[C.patience].outcome, outs[C.patience].note))
chk(len(g.calls) == C.patience,
    "**指令条数被封住**：一共只发了 patience=%d 条（实得 %d 条：%s）"
    % (C.patience, len(g.calls), g.calls))
chk(all(d == "right" for d, _ in g.calls),
    "发的都是同方向的 right（%s）—— 没有反着乱推" % [d for d, _ in g.calls])
chk(not any(o.moved for o in outs[C.patience:]),
    "停手之后一条都不发（%s）" % [o.pulses for o in outs[C.patience:]])
chk(outs[-1].outcome == NO_PROGRESS,
    "最后一帧还是 NO_PROGRESS（实得 %s）" % outs[-1].outcome)
_st = outs[C.patience].axes["pan"]
chk(_st.direction is None and _st.wanted == "right" and _st.skipped == NO_PROGRESS,
    "被守卫拦下的 pan：direction=None, wanted=right, skip=%s" % _st.skipped)

print()
print("18) 误差**缓慢但持续改善** —— 不许触发 NO_PROGRESS（别误伤正常收敛）")
# 每帧只改善 0.04（刚过 min_progress=0.03），离收敛还远
ERRS = [0.38, 0.34, 0.30, 0.26, 0.22, 0.18, 0.14, 0.10, 0.06]
g = FakeGimbal()
tr, t = mk(g, ScriptedStream(
    [[0.5 - e - 0.05, 0.40, 0.5 - e + 0.05, 0.60] for e in ERRS]))
outs = [tr.update(now=i * 0.05) for i in range(len(ERRS))]
chk(all(o.outcome == TRACKED for o in outs),
    "每帧都在跟，没有一帧 NO_PROGRESS（实得 %s）" % [o.outcome for o in outs])
chk(len(g.calls) == len(ERRS),
    "每一帧都发了指令（实得 %d / %d）" % (len(g.calls), len(ERRS)))

print()
print("19) **恢复**：进了 NO_PROGRESS 之后误差真的改善了 —— 要能重新开始跟")
g = FakeGimbal()
FROZEN = [0.10, 0.40, 0.20, 0.60]       # du = +0.35
BETTER = [0.20, 0.40, 0.30, 0.60]       # du = +0.25，改善 0.10 > min_progress
tr, t = mk(g, ScriptedStream([FROZEN] * 6 + [BETTER] * 3))
outs = []
n_stopped = None       # **停手那一刻**已经发出去的条数（不能跑完 9 帧再数，那会把恢复后的算进来）
for i in range(9):
    outs.append(tr.update(now=i * 0.05))
    if n_stopped is None and outs[-1].outcome == NO_PROGRESS:
        n_stopped = len(g.calls)
chk(outs[C.patience - 1].outcome == TRACKED,
    "第 %d 帧还在发（实得 %s）" % (C.patience, outs[C.patience - 1].outcome))
chk(outs[C.patience].outcome == NO_PROGRESS,
    "第 %d 帧停手（实得 %s / %s）"
    % (C.patience + 1, outs[C.patience].outcome, outs[C.patience].note))
chk(n_stopped == C.patience, "停手时一共只发了 %d 条（实得 %s）"
    % (C.patience, n_stopped))
chk(all(o.outcome == NO_PROGRESS for o in outs[C.patience:6]),
    "冻住的这段时间一直停手（%s）" % [o.outcome for o in outs[C.patience:6]])
chk(outs[6].outcome == TRACKED,
    "误差改善之后**又跟起来了**（实得 %s / %s）" % (outs[6].outcome, outs[6].note))
chk(len(g.calls) > n_stopped,
    "恢复后真的又发了指令（%d -> %d）" % (n_stopped, len(g.calls)))
chk(call_eq(g.last("pan"), "left", C.k["pan"] * 0.25),
    "恢复后发的方向/度数也对（实得 %s）" % (g.last("pan"),))

print()
print("20) 边界：patience 次里**恰好有一次**改善够 min_progress —— 不该触发")
# err: 0.30 → 0.29 → **0.25（改善 0.05 > 0.03，计数清零）** → 0.24 → 0.23
# 若不清零，第 5 帧就成了"连着 3 帧没改善"，会误判成 NO_PROGRESS。
g = FakeGimbal()
ERRS2 = [0.30, 0.29, 0.25, 0.24, 0.23]
tr, t = mk(g, ScriptedStream(
    [[0.5 - e - 0.05, 0.40, 0.5 - e + 0.05, 0.60] for e in ERRS2]))
outs = [tr.update(now=i * 0.05) for i in range(len(ERRS2))]
chk(all(o.outcome == TRACKED for o in outs),
    "5 帧全在跟，没有 NO_PROGRESS（实得 %s）" % [o.outcome for o in outs])
chk(len(g.calls) == len(ERRS2),
    "每帧都发了（实得 %d / %d）" % (len(g.calls), len(ERRS2)))

print()
print("=" * 72)
if FAILS:
    print("结果: %d 项失败" % len(FAILS))
    for f in FAILS:
        print("   -", f)
    print("GIMBAL_TRACK_TESTS_FAILED")
    sys.exit(1)
print("结果: 全部通过")
print("GIMBAL_TRACK_TESTS_DONE")
sys.exit(0)

# ---------------------------------------------------------------------------
# 变异自检记录（2026-09-21，在树莓派上实跑）
#
# 本文件**必须能红**。做法：把 gimbal_track.py 的源码读进来，做一处字符串替换，
# 在内存里 exec 成模块（不落地第三个文件），再 exec 本文件 —— 看它是否 FAIL。
#
#   M1  pan_sign=_REF.pan_sign  ->  pan_sign=-_REF.pan_sign   （方向的符号取反）
#       -> 第 2、2b、8 组变红（方向断言），退出码 1
#   M2  if abs(d) < cfg.deadband[axis]:  ->  if False:        （死区失效）
#       -> 第 1、1b 组变红（中心/死区内乱动），退出码 1
#
# 2026-09-21（加回三道守卫那次）追加三处，办法同上（读源码 → 内存里 exec 成模块
# → 再 exec 本文件；脚本本体在 /tmp，**不落地到项目里**）：
#
#   M3_boxguard    `        if not box_complete(box, cfg.margin):` -> `        if False:`
#       -> 第 14、16 组变红（框被切/太大时照样发指令），退出码 1
#   M4_progress    `        if planned:` -> `        if False:`（无进展守卫整块失效）
#       -> 第 17、19 组变红（冻住的误差被无限叠指令），退出码 1
#   M5_patience1   `if self._no_progress >= cfg.patience:` -> `>= 1`（守卫过度敏感）
#       -> 第 17、19、20 组变红（正常收敛被误判成没进展），**另外还带红了第 4、7、7c 组** ——
#          那三组本来只验帧对齐 / 限位 latch，守卫一过敏就跟着塌（第 4 组那一条指令被吞、
#          7c 的"换向要放行"也发不出去）。也就是说：**守卫过敏的破坏面比守卫缺失更大。**
#
# M5 是**反向**变异：它证明第 17/19/20 组守的是"别误伤"，不是"别放过"。
# ---------------------------------------------------------------------------
