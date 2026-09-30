"""陀螺偏航速率闭环（纯 Python，无 ROS 依赖）。

【为什么需要它】

四驱滑移转向的车，转弯时四个轮子必须横向擦地滑移。实测这台车**只转到指令的
48~56%**，而且速度越快越不够：

    指令 (0.2, -0.3)  -> 陀螺实测 / 指令 = 56%
    指令 (0.3, -0.45) -> 陀螺实测 / 指令 = 48%

把短缺点折算成"有效轮距 T_eff"是 1.35 m 和 1.52 m 两个值 —— **不是常数**。
它的物理本质就是轮胎侧滑修正，随速度/载重/地面变。所以**开环写死任何一个 b
都不可能准**，调参是条死路。

本模块的做法：**在有传感器的环节闭环**。Pi 上有陀螺（上行帧里的 gz @50Hz，
是原始测量、可信），拿它当反馈去修正发给固件的 wz。打滑多少、地面怎么变都
无所谓，**T_eff / b 直接从问题里消失**。

【控制律】每个 tx tick（20Hz）：

    e      = wz_ref - wz_meas
    I     += Ki * e * dt          钳到 |I| <= I_CLAMP_FRAC * |wz_ref|
    wz_out = clamp(wz_ref + Kp*e + I, +-max_wz_out)
    wz_out = 再过一个速率限幅（|dwz_out/dt| <= out_rate_limit）

前馈 wz_ref + PI。稳态时积分项自动补掉未知的增益 k，使 wz_meas -> wz_ref，
**与 k 无关** —— 这正是要的效果。积分项的稳态值就是 wz_ref*(1/k - 1)，所以
`i_clamp_frac` 实际上是在声明"本环至少能补偿到多小的 k"：
I_CLAMP_FRAC=1.5 即 k_min = 1/(1+1.5) = 0.40。

【k 的设计点取 1.0，不是实测的 0.5】
侧滑只会让等效轮距**变大**，所以 k <= 1 是物理上界，实测的 48~56% 是"好走"
的地面。最危险的是侧滑最小的地面（干燥水泥/地毯），那里 k 逼近 1。增益按
k=1 定，才在worst case留得住相位裕度。

【安全】车会真的转，以下每条都对应一个具体的跑飞场景：

  · 陀螺没数据/过期/上位机未获准 -> 直通 wz_ref，状态清零
  · **陀螺卡死**（恒 0 或恒在某个非零值）—— 见 _check_stall()。判据是
    "输出动了但测量没动"，对两种卡死对称生效，且自带触发条件（输出得先
    动起来才可能触发），所以小指令下不会误判。
  · **符号判反** -> 正反馈。见 _check_wrong_sign()。**刻意不提供取反参数**：
    配反了就是朝反方向猛转，比没有闭环危险得多；符号只能靠现场验证，
    不能靠配置兜底。这条守卫是兜底，不是替代品。
  · 输出硬钳位 + 速率限幅：后者管住偏航**角加速度**，也把所有状态切换
    （投入、脱开、latch、过死区）从阶跃变成斜坡。
  · 死区带迟滞（进 0.05 / 出 0.02）：否则指令停在阈值附近会逐拍抖动，
    每次抖动都清一次积分再重新爬。**刻意不做航向保持**。
  · dt 由调用方用单调时钟实测，再钳到 [0.5, 1.5]x 标称 —— Pi 不是实时系统，
    一次长 dt 不该让积分跳一大步。

【已知残余风险（诚实记下）】
陀螺真的卡死时，从投入到 _check_stall() 判定成立约 0.8s，窗口内输出最大
(1 + Kp + I_CLAMP_FRAC) = 4.0 倍指令。**关键是它正比于指令而不是绝对值**：
操作员松杆，它立刻消失。0.3 rad/s 的指令对应约 1.2 rad/s、约 55 度误转。
所以 yaw_loop_enable 默认 false，且第一次上机必须先做符号验证。
"""

import math
from collections import deque
from dataclasses import dataclass


@dataclass
class YawLoopConfig:
    """闭环参数。由驱动节点从 ROS 参数填，每次 tx tick 重建（这样
    `ros2 param set` 能立刻生效，不用重启节点）。"""

    # PI 增益。
    #
    # ⚠️ 2026-09-20 实车修正：**最初的 1.5/2.5 是错的，基于错误的被控对象
    # 模型**（当初按 tau~0.1s 的一阶对象做稳定性估算，算出 54 度相位裕度）。
    # 实车一开就剧烈摆动（实测偏航率在指令的 ±100% 之间、约 2s 周期），
    # 因为真实对象完全不是那样：
    #
    #   · **有 ~0.8~1.2s 死区** —— 原地转要先克服四轮横向擦地的静摩擦
    #   · **响应是秒级**，不是 0.1s；8 秒都还没完全稳
    #   · **欠阻尼**：给一个恒定指令，车的偏航率自己就在 0.107~0.239
    #     之间摆（这跟环路无关，开环也这样）
    #
    # 所以增益得往下砍一个数量级。实测（弧线 vx=0.3 wz=-0.45，8s）：
    #   开环 61.6% -> 闭环 92.8%，输出自己从 0.45 长到 0.59 补掉打滑。
    #
    # 另外注意：**纯原地转不需要闭环**（开环就有 73~109%），亏速只出现在
    # 弧线（边走边转）—— 前进分量和差速要求互相较劲，擦地严重得多。
    kp: float = 0.2
    ki: float = 0.2

    # 积分钳位 = i_clamp_frac * |wz_ref|。声明的是 k_min = 1/(1+frac)。
    # 1.5 -> k_min 0.40（实测最差 0.48，留了余量）。
    i_clamp_frac: float = 1.5

    # 输出硬钳位 (rad/s)。
    # ⚠️ 与 max_wz 的关系：补偿 k=0.5 需要 2x 指令，所以**必须**
    # max_wz_out >= 2*max_wz，否则大指令下闭环没法到位。
    # 驱动节点启动时会检查并在不满足时告警。
    max_wz_out: float = 2.0

    # 输出速率限幅 (rad/s^2)。管的是偏航角加速度 —— 那才是让车失稳的量。
    # 正常跟踪时输出斜率在 1.4 (k=0.5) ~ 5 (k=1, 满幅阶跃) 之间，所以这个
    # 值对正常信号是透明的，只在故障和状态切换时起作用。
    out_rate_limit: float = 6.0

    # 死区，带迟滞：进 0.05 / 出 0.02。进阈值取得明显高，是因为它还要盖过
    # 行驶中的陀螺零偏（未实测，估计 0.02~0.05 rad/s 量级）。
    engage_thresh: float = 0.05
    disengage_thresh: float = 0.02

    # 陀螺一阶低通截止频率 (Hz)。<=0 表示不滤波。
    # 12Hz 而不是更低：5Hz 会在穿越频率处吃掉 19 度相位裕度，12Hz 只吃 8 度。
    # 它压的是电机振动带来的整流误差，不是传感器噪声 —— MEMS 陀螺噪声经 Kp
    # 放大后只有 0.002 rad/s 量级，可以忽略。
    gyro_fc_hz: float = 12.0

    # 陀螺超过这么久没更新就当作不可用（秒）。0.2s @50Hz = 连丢 10 帧。
    stale_timeout: float = 0.2

    # ---- 卡死检测：滑动窗口内"输出动了、测量没动" ----
    #
    # stall_settle 是这里最容易写错的一处。判据不能直接拿整个窗口的极差比：
    # **打杆反向的那一拍**输出就开始往回收，而测量还没来得及动，于是窗口里
    # 一边极差已经很大、另一边还是 0 —— 看起来和卡死一模一样，实测每 2s
    # 反向一次就会误触发一次。
    #
    # 所以把窗口劈开：**衡量"输出动过"只看窗口的前段**（截止到
    # stall_settle 之前），"测量有没有跟上"看整个窗口。这样输出一动就立刻
    # 判定的话，前段里它还没动，不会误armed；等 settle 过去，正常对象的
    # 测量早就跟上了，而卡死的陀螺仍然纹丝不动。
    #
    # stall_settle 取得比对象建立时间（tau + 延迟 ~0.5s）的一半略小：
    # 太大则检测迟钝，太小则正常滞后来不及跟上。
    stall_window: float = 1.0
    stall_settle: float = 0.4
    stall_min_fill: float = 0.4     # 前段至少要跨越这么久才下判定
    stall_out_span: float = 0.20    # 前段内 wz_out 的极差 (rad/s)
    stall_meas_span: float = 0.05   # 整个窗口内 wz_meas 的极差 (rad/s)

    # ⚠️ 角度判据必须用**长得多的**窗口，这是 2026-09-20 实车调完之后补的：
    # 极差判据要求"输出在窗口里动过 0.2"，而**低增益时输出爬得极慢** ——
    # yaw 环从 1.5/2.5 降到 0.2/0.2 之后，1 秒窗口里极差不到 0.2，判据压根
    # 不 armed，**"陀螺死了"这条保护直接失效**（单测抓到的）。
    # 角度口径积的是"输出-时间"，低增益只是爬得慢、不是积不出来，
    # 所以它在长窗口上照样能判出来。
    stall_angle_window: float = 2.5
    stall_angle_out: float = 0.5    # 长窗内 ∫|wz_out|dt (rad)
    stall_angle_meas: float = 0.05  # 而 ∫|wz_meas|dt 仍低于这个

    # ---- 符号判反守卫 ----
    # 判反 = 正反馈，指数发散，所以要早抓。难点在于**正常的反向打杆**也会
    # 短暂异号（车还在往旧方向转，那是物理事实，不是故障）。
    #
    # 靠"持续时间"区分是行不通的：正常反向的异号时长和对象的建立时间同
    # 量级（tau=0.15s 时约 0.3s），dwell 取小了误判、取大了正反馈要转
    # 好久才被拦住。
    #
    # 靠"幅度"区分才干净：符号正确时 |wz_meas| <= ~1.2*|wz_ref|（超调），
    # 符号反了则反馈把输出顶到钳位，|wz_meas| -> k*max_wz_out，是 |wz_ref|
    # 的好几倍，**而且在增长**。所以阈值取 max(ratio*|wz_ref|, abs) 就能
    # 把"正常反向"（0.5*|ref| 量级）和"正反馈"分开，dwell 也就能取得很短。
    #
    # abs 那一项是兜底：指令很小而车被外力转着的时候，光看比例会误判。
    wrong_sign_dwell: float = 0.2
    wrong_sign_ratio: float = 1.5
    wrong_sign_abs: float = 0.3

    # latch 解除：指令须持续回中位这么久（秒）。单拍过零不许解除 ——
    # 快速打杆穿越中位不是"操作员已回到安全状态"的证据。
    latch_clear_time: float = 0.5

    # 标称控制周期 (s)，用于 dt 钳位。
    dt_nominal: float = 0.05


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


class YawRateLoop:
    """偏航速率闭环。**不是线程安全的** —— 调用方自己加锁。

    典型用法（见 driver_node.py）：

        loop = YawRateLoop(YawLoopConfig())
        # 读数环 @50Hz：
        loop.update_gyro(up.gz, time.monotonic())
        # tx 定时器 @20Hz：
        wz_out = loop.step(wz_ref, time.monotonic(), allowed=...)
    """

    def __init__(self, cfg: YawLoopConfig = None):
        self.cfg = cfg or YawLoopConfig()
        self.reset()

    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """把控制器状态清干净。"""
        self._integral = 0.0
        self._prev_out = 0.0            # 速率限幅的起点
        self._gyro_filt = None          # None = 还没有陀螺数据
        self._gyro_stamp = None
        self._last_gyro_time = None
        self._last_step_time = None
        self._win = deque()             # (t, wz_out, wz_meas)
        self._neutral_since = None
        self._wrong_since = None
        self._latched = False
        self._engaged = False
        self.trip_reason = None         # None / "stall" / "wrong_sign"
        # 诊断（给 /yaw_loop 话题，见 driver_node._publish_yaw_loop）
        self.last_wz_ref = 0.0
        self.last_wz_out = 0.0

    def _disengage(self) -> None:
        """脱开：清积分、清卡死窗口、清符号守卫计时器。

        积分必须跟着清 —— 否则下次投入时会带着一个陈旧的积分项冲出去。
        _prev_out **不清**（清了输出会跳到 0，那本身就是个阶跃）。
        """
        self._integral = 0.0
        self._win.clear()
        self._wrong_since = None
        self._engaged = False

    # ------------------------------------------------------------------
    # 陀螺（读数环 @50Hz）
    # ------------------------------------------------------------------

    def update_gyro(self, gz: float, now: float) -> None:
        """喂入一帧陀螺角速度。在**读数线程**里按上行帧率（50Hz）调用。

        滤波放在 50Hz 而不是 20Hz：采样率高一个量级，同样的截止频率下相位
        滞后更小。这里只做一次乘加，保持极短 —— 同一个线程还要做里程计积分
        和若干次 DDS 发布，那些都可能阻塞几十毫秒，会直接变成陀螺的采样抖动。
        """
        if not math.isfinite(gz):
            return
        dt = 0.0 if self._last_gyro_time is None else now - self._last_gyro_time
        self._last_gyro_time = now
        self._gyro_stamp = now

        fc = self.cfg.gyro_fc_hz
        if fc <= 0.0 or self._gyro_filt is None:
            # 不滤波，或首帧：直接采用。首帧直接采用而不是从 0 慢慢爬 ——
            # 否则节点刚起来那几秒反馈会偏小，闭环会额外多给一点。
            self._gyro_filt = gz
            return

        tau = 1.0 / (2.0 * math.pi * fc)
        # dt 用实测的，丢帧时用标称值会让滤波器"以为"时间没过
        a = _clamp(dt / (tau + dt), 0.0, 1.0) if dt > 0.0 else 0.0
        self._gyro_filt = self._gyro_filt + a * (gz - self._gyro_filt)

    def gyro_age(self, now: float):
        """陀螺读数距今多久（秒）。从未收到过返回 None。"""
        if self._gyro_stamp is None:
            return None
        return now - self._gyro_stamp

    @property
    def wz_meas(self):
        """滤波后的陀螺角速度 (rad/s)。还没有数据时返回 None。"""
        return self._gyro_filt

    # ------------------------------------------------------------------
    # 控制（tx 定时器 @20Hz）
    # ------------------------------------------------------------------

    def step(self, wz_ref: float, now: float, allowed: bool = True) -> float:
        """算这一拍该发给固件的 wz。

        wz_ref  : cmd_vel 请求的偏航角速度 (rad/s)
        now     : 单调时钟 (time.monotonic())
        allowed : 上位机侧的外部允许位 —— 急停/启动预热帧/固件故障位/
                  电压过低/总开关，都折算进它。为 False 时直通。

        陀螺新鲜度由本模块自己判（它owns那些时间戳）。
        """
        if not math.isfinite(wz_ref):
            wz_ref = 0.0                # 收到 inf/nan 就当"别转了"

        dt = self._elapsed(now)
        target = self._desired(wz_ref, now, allowed, dt)
        out = self._slew(target, dt)
        self.last_wz_ref = wz_ref
        self.last_wz_out = out
        return out

    def _elapsed(self, now):
        """实测 dt，钳到 [0.5, 1.5]x 标称。None = 这一拍没有可用的 dt。"""
        if self._last_step_time is None:
            self._last_step_time = now
            return None
        raw = now - self._last_step_time
        self._last_step_time = now
        if not math.isfinite(raw) or raw <= 0.0:
            return None
        return _clamp(raw, self.cfg.dt_nominal * 0.5, self.cfg.dt_nominal * 1.5)

    def _slew(self, target: float, dt) -> float:
        """输出速率限幅。所有路径（含直通和 latch）都过这里，
        所以任何状态切换都是斜坡而不是阶跃。"""
        step = self.cfg.out_rate_limit * (dt if dt is not None else self.cfg.dt_nominal)
        out = _clamp(target, self._prev_out - step, self._prev_out + step)
        self._prev_out = out
        return out

    def _desired(self, wz_ref, now, allowed, dt):
        """算速率限幅**之前**的目标输出。"""
        w = self.wz_meas
        age = self.gyro_age(now)

        # 1) 不获准 / 陀螺不可用 / dt 不可用 -> 直通，状态清零。
        #    这里**不**把 wz_ref 置零：上位机没授权时该发什么还是发什么，
        #    紧急停车是别人的职责，本模块只负责"不添乱"。
        if (not allowed or dt is None or w is None
                or not math.isfinite(w) or age is None
                or age >= self.cfg.stale_timeout):
            self._disengage()
            return wz_ref

        # 2) 中位：带迟滞地脱开。latch 只在持续回中位后才解除。
        if abs(wz_ref) < self.cfg.disengage_thresh:
            self._disengage()
            if self._neutral_since is None:
                self._neutral_since = now
            elif now - self._neutral_since >= self.cfg.latch_clear_time:
                self._latched = False
                self.trip_reason = None
            return wz_ref
        self._neutral_since = None

        # 3) 还没到投入阈值（迟滞的上半段）：保持脱开，不启用
        if not self._engaged and abs(wz_ref) < self.cfg.engage_thresh:
            return wz_ref

        # 4) latch 中：一直直通，直到指令持续回中位（第 2 步解除）
        if self._latched:
            return wz_ref

        # 5) PI
        e = wz_ref - w
        i_lim = self.cfg.i_clamp_frac * abs(wz_ref)
        self._integral = _clamp(self._integral + self.cfg.ki * e * dt,
                                -i_lim, i_lim)
        out = _clamp(wz_ref + self.cfg.kp * e + self._integral,
                     -self.cfg.max_wz_out, self.cfg.max_wz_out)
        self._engaged = True

        # 6) 守卫。**任何一条触发都退回直通**（而不是置零）：退回的是已经
        #    验证过的开环行为；置零是个没验证过的新行为，更糟。
        trip = (self._check_wrong_sign(wz_ref, w, now)
                or self._check_stall(now, out, w))
        if trip:
            self._latched = True
            self.trip_reason = trip
            self._disengage()
            return wz_ref

        return out

    def _check_wrong_sign(self, wz_ref, w, now):
        """陀螺与指令异号、**且幅度大到正确符号不可能达到**、持续够久
        -> 符号判反（正反馈）。

        只看异号是不够的：正常反向打杆时车还在往旧方向转，会异号 0.2~0.3s，
        和正反馈在早期无法区分。加上幅度条件才能真正分开（见 cf. 注释）。
        """
        thresh = max(self.cfg.wrong_sign_ratio * abs(wz_ref),
                     self.cfg.wrong_sign_abs)
        if w * wz_ref < 0.0 and abs(w) > thresh:
            if self._wrong_since is None:
                self._wrong_since = now
            elif now - self._wrong_since >= self.cfg.wrong_sign_dwell:
                return "wrong_sign"
        else:
            self._wrong_since = None
        return None

    def _check_stall(self, now, wz_out, w):
        """滑动窗口内"输出动了、测量没动" -> 陀螺卡死。

        对**恒零**和**恒在非零值**两种卡死对称生效（只看极差，不看绝对
        大小），也对"车被顶住/架在架子上"生效 —— 后者是合法的触发，
        而退回直通同样是那里最安全的行为。

        自带触发条件：前段里 wz_out 的极差得先超过 stall_out_span，
        小指令下闭环的总修正量本来就没那么大，所以不会误判。
        """
        win = self._win
        win.append((now, wz_out, w))
        # 缓冲区按**最长**的那个窗口保留，两个判据各取所需
        keep = max(self.cfg.stall_window, self.cfg.stall_angle_window)
        cutoff = now - keep
        while win and win[0][0] < cutoff:
            win.popleft()

        # ---- 角度判据（长窗口）：低增益下唯一能抓到"陀螺死了"的一条 ----
        awin = [s for s in win if s[0] >= now - self.cfg.stall_angle_window]
        if len(awin) >= 8:
            aspan = awin[-1][0] - awin[0][0]
            if aspan >= self.cfg.stall_angle_window * 0.8:
                n = float(len(awin))
                io = sum(abs(s[1]) for s in awin) / n * aspan
                im = sum(abs(s[2]) for s in awin) / n * aspan
                if (io > self.cfg.stall_angle_out
                        and im < self.cfg.stall_angle_meas):
                    return "stall"

        # ---- 极差判据（短窗口）：抓快环，也抓"输出顶在钳位上不动"----
        # 衡量"输出动过"只看前段（详见 cf. 注释：打杆反向那一拍会瞬间
        # 造出一个假的极差）。"测量有没有跟上"看整个窗口。
        short = [s for s in win if s[0] >= now - self.cfg.stall_window]
        settle = now - self.cfg.stall_settle
        old = [s for s in short if s[0] <= settle]
        if len(old) < 4 or (old[-1][0] - old[0][0]) < self.cfg.stall_min_fill:
            return None
        if max(s[1] for s in old) - min(s[1] for s in old) <= \
                self.cfg.stall_out_span:
            return None
        meas_span = max(s[2] for s in short) - min(s[2] for s in short)
        return "stall" if meas_span < self.cfg.stall_meas_span else None

    # ------------------------------------------------------------------
    # 诊断
    # ------------------------------------------------------------------

    @property
    def engaged(self) -> bool:
        """上一拍闭环是否真的在起作用（还是直通）。"""
        return self._engaged

    @property
    def latched(self) -> bool:
        """是否已被守卫 latch（指令持续回中位后自动解除）。"""
        return self._latched

    @property
    def integral(self) -> float:
        return self._integral

    @property
    def out_unlimited(self) -> float:
        """速率限幅**之前**的目标输出。诊断用：看限幅有没有在削峰。"""
        return self._prev_out


# --------------------------------------------------------------------------
# 离线自检（完整用例见 test/test_yaw_loop.py）
# --------------------------------------------------------------------------

def _demo_plant(k, tau, wz_ref, seconds=5.0, dt=0.05):
    """一阶惯性被控对象，验证闭环确实把未知增益补掉了。"""
    loop = YawRateLoop(YawLoopConfig())
    wz, t = 0.0, 0.0
    for _ in range(int(seconds / dt)):
        loop.update_gyro(wz, t)
        out = loop.step(wz_ref, t)
        wz += (k * out - wz) * (dt / (tau + dt))
        t += dt
    return wz


if __name__ == "__main__":
    K_MIN = 1.0 / (1.0 + YawLoopConfig().i_clamp_frac)
    print(f"指令 0.3 rad/s，开环只能走到："
          + ", ".join(f"k={k}: {k * 0.3:.3f}" for k in (0.4, 0.5, 1.0)))

    # 在 i_clamp_frac 声明的补偿范围内，闭环必须精确收敛 —— 与 k 无关。
    for k in (0.4, 0.5, 0.7, 1.0, 1.5):
        got = _demo_plant(k, 0.1, 0.3)
        assert abs(got - 0.3) < 0.01, f"k={k} 未收敛：{got:.4f}"
        print(f"  闭环 k={k:<4} -> {got:+.4f} rad/s  (开环 {k * 0.3:.3f})")

    # k 低于 k_min 时**故意**不补满：这是 i_clamp_frac 在设计上声明的边界，
    # 不是 bug。补满需要无限大的积分，而积分越大、陀螺卡死时越危险。
    got = _demo_plant(0.3, 0.1, 0.3)
    assert got < 0.3, "k=0.3 竟然补满了，说明钳位没生效"
    print(f"  闭环 k=0.30 -> {got:+.4f} rad/s  "
          f"<- 低于声明的 k_min={K_MIN:.2f}，按设计只补到这里")

    print("yaw_loop.py 自检通过（完整用例：test/test_yaw_loop.py）")
