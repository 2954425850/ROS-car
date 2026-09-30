# /home/cy/k230-vision/pi/camcenter.py —— 云台居中闭环
#
# ## 它解决什么
#
# 板子的 8557 要求「目标框完整可见」，而实测（2026-09-19）目标框一开始出画，
# NanoTrack 立刻崩（框塌缩、ar_dev 跳、lost）—— 所以画面边缘那一圈**锁不上**。
# 软件补灰解决不了（目标是"真的有一半不在相机视野里"），唯一解法是
# **把云台转过去，让目标进画面**。
#
# ## 边界（说清楚，免得被当成本事更大）
#
# 这是**单次动作**：转到位就停，随后由别人去发 8557 锁定。
# **不是**"一边跟一边转"的持续伺服 —— 那要处理"画面在动时跟踪器怎么不丢"，
# 是另一件事（跟人走那一期再说）。
#
# ## 设计要点
#
# - **比例控制**：只看"目标偏了多少"，按比例给一个步长，转完再看。
#   ⚠️ 但**符号和增益必须实测**（`t_camcalib.py`）—— 方向搞反了不会报错，
#   只会越转越远。`CenterConfig` 里的默认值是 2026-09-19 在真车上标出来的。
# - **等新帧再动下一次**：结果流靠 `frame` 对齐（板子文档明写"做帧对齐只能用 frame"）。
#   不等新帧就会拿着同一张旧图反复转，转飞。
# - **一切都可注入**（`rotator` / `read_target` / 时钟 / sleep）：
#   单测用假的，不接 ROS、不接云台也能把失败路径全验一遍。
#   —— 照 `voice-chatbot/vision/k230_look.py` 的同一套路子。

import time

from geom import box_complete, box_size, safe_center_range

# ---- 结果码 ----
CENTERED = "centered"            # 目标已经进安全区，可以去锁了
HIT_LIMIT = "hit_limit"          # 云台撞行程限位（转不动了）
NO_PROGRESS = "no_progress"      # 连着几步没改善 —— 方向反了 / 云台没响应
TIMEOUT = "timeout"              # 转满了步数还没到
NO_TARGET = "no_target"          # 结果流里没有目标 / 结果太旧
STALE = "stale"                  # 结果一直没更新
BOX_TOO_BIG = "box_too_big"        # 目标框比安全区还大 —— 转云台也没用


class CenterConfig:
    """全部可调，都有默认值。

    ⚠️ 带「实测」注释的默认值来自 2026-09-19 在真车上的标定
    （`t_camcalib.py`，图像配准法，两轴内点都 >110、画面旋转 <1°）。
    **两轴灵敏度差 2.4 倍**，所以增益和步长上限都是**按轴分开**的 ——
    共用一个 k 会让其中一轴要么太慢、要么直接冲过头。

    未标定的车上重跑一遍 `t_camcalib.py`，把下面这几个数换掉。
    """

    def __init__(self, margin=0.05,
                 k_pan=38.0, k_tilt=16.0,             # 实测：度 / 归一化偏差
                 max_step_pan=20.0, max_step_tilt=4.5,  # 实测：一步最多转几度
                 min_step_deg=2.0,
                 max_steps=14, patience=3, min_progress=0.03,
                 frame_timeout_s=2.0, poll_s=0.05,
                 pan_sign=1, tilt_sign=1):            # 实测：都是 +1
        self.margin = margin
        self.min_step_deg = min_step_deg
        self.max_steps = max_steps
        self.patience = patience              # 连续几步没改善就放弃
        self.min_progress = min_progress      # "有改善"的最低幅度
        self.frame_timeout_s = frame_timeout_s
        self.poll_s = poll_s
        self.pan_sign = pan_sign
        self.tilt_sign = tilt_sign
        # 按轴分开查表（循环里只按 axis 取，不再到处 if）
        self.k = {"pan": k_pan, "tilt": k_tilt}
        self.max_step = {"pan": max_step_pan, "tilt": max_step_tilt}


class CenterResult:
    def __init__(self, outcome, steps=0, box=None, note="", pulses=None):
        self.outcome = outcome
        self.steps = steps
        self.box = box
        self.note = note
        self.pulses = pulses or []
        self.ok = outcome == CENTERED

    def __repr__(self):
        return "<CenterResult %s steps=%d %s>" % (self.outcome, self.steps, self.note)


class CamCenterer:
    """把目标从画面边缘转到「锁得住」的位置。

    rotator(direction, degrees) -> (目标脉宽 us, 是否撞限位)
        direction ∈ left/right/up/down。**必须给结构化返回值** ——
        `CarController.camera_move` 返回的是给人念的中文串，不能直接拿来当回路反馈。
    read_target() -> (box, frame) 或 None
        box = [l,t,r,b] 归一化；frame = 板子的帧号（用来判断"是不是新的一帧"）。
    """

    def __init__(self, rotator, read_target, cfg=None,
                 clock=time.monotonic, sleeper=time.sleep):
        self._rotator = rotator
        self._read_target = read_target
        self.cfg = cfg or CenterConfig()
        self._clock = clock
        self._sleep = sleeper

    # ---- 对外 ----
    def center(self):
        cfg = self.cfg
        last_frame = None
        best_err = None
        no_progress = 0
        steps = 0
        pulses = []
        last_box = None

        while steps < cfg.max_steps:
            got = self._read_target()
            if got is None:
                return CenterResult(NO_TARGET, steps, last_box,
                                    "结果流里没有目标", pulses)
            box, frame = got

            # 结果流没更新就等新帧 —— 否则会拿着同一张旧图反复转
            if frame == last_frame:
                got = self._wait_new_frame(frame)
                if got is None:
                    return CenterResult(STALE, steps, last_box,
                                        "结果流 %.1fs 没更新" % cfg.frame_timeout_s, pulses)
                box, frame = got
            last_frame = frame
            last_box = box

            if box_complete(box, cfg.margin):
                return CenterResult(CENTERED, steps, box, "已在安全区内", pulses)

            # ⚠️ 框自己就放不下时，**转云台永远也到不了** —— 当场说清楚，
            # 别让它转到超时（2026-09-19 端到端实测：一个 0.544x0.939 的检测框
            # 就是这么把回路拖进 no_progress 的）。
            _umin, _umax, _vmin, _vmax = safe_center_range(box, cfg.margin)
            if _umin > _umax or _vmin > _vmax:
                bw, bh = box_size(box)
                return CenterResult(
                    BOX_TOO_BIG, steps, box,
                    "目标框 %.3f x %.3f 比安全区还大，转云台也没用"
                    "（得换个更紧的框，或把目标离远一点）" % (bw, bh), pulses)

            # 偏离量：离画面中心多远（>0 表示偏那边）
            l, t, r, b = box
            du = 0.5 - (l + r) / 2.0
            dv = 0.5 - (t + b) / 2.0
            err = abs(du) + abs(dv)

            if best_err is None or err < best_err - cfg.min_progress:
                best_err = err
                no_progress = 0
            else:
                no_progress += 1
                if no_progress >= cfg.patience:
                    return CenterResult(
                        NO_PROGRESS, steps, box,
                        "连着 %d 步没改善（err=%.3f，最好 %.3f）—— 方向可能反了，"
                        "或云台没在动" % (cfg.patience, err, best_err), pulses)

            # 一次只动一个轴：偏离大的那个
            if abs(du) >= abs(dv):
                axis, d = "pan", du
            else:
                axis, d = "tilt", dv
            direction = self._direction(axis, d)
            deg = min(max(cfg.k[axis] * abs(d), cfg.min_step_deg),
                      cfg.max_step[axis])

            _us, hit_limit = self._rotator(direction, deg)
            pulses.append((direction, deg))
            steps += 1
            if hit_limit:
                return CenterResult(HIT_LIMIT, steps, box,
                                    "云台往 %s 转 %0.1f° 时撞到行程限位" % (direction, deg),
                                    pulses)

        return CenterResult(TIMEOUT, steps, last_box,
                            "转满 %d 步还没进安全区" % cfg.max_steps, pulses)

    # ---- 内部 ----
    def _direction(self, axis, d):
        """d>0 = 目标在中心**左边/上边**，要把画面内容往右/往下挪 → 相机往左/上转。"""
        sign = self.cfg.pan_sign if axis == "pan" else self.cfg.tilt_sign
        toward_pos = d > 0
        if sign < 0:
            toward_pos = not toward_pos
        if axis == "pan":
            return "left" if toward_pos else "right"
        return "up" if toward_pos else "down"

    def _wait_new_frame(self, frame):
        t0 = self._clock()
        while self._clock() - t0 < self.cfg.frame_timeout_s:
            self._sleep(self.cfg.poll_s)
            got = self._read_target()
            if got is not None and got[1] != frame:
                return got
        return None


def describe(res):
    """给人/给模型看的一句话。"""
    if res.ok:
        return "目标已经在画面中间，可以锁定了。"
    return {
        HIT_LIMIT: "云台已经转到头了，目标还是不在画面中间。",
        NO_PROGRESS: "转了云台但画面没变化 —— 可能方向不对或者云台没响应。",
        TIMEOUT: "转了 %d 步还没把目标转进画面中间。" % res.steps,
        NO_TARGET: "现在画面里看不到这个目标。",
        STALE: "拿不到新的画面数据。",
        BOX_TOO_BIG: "那个目标框太大了，转到哪儿都锁不住 —— 换个更紧的框吧。",
    }.get(res.outcome, "云台居中没成功（%s）。" % res.outcome)
