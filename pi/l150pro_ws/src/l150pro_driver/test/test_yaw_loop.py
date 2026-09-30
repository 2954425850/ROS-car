"""陀螺偏航闭环测试。

不依赖 ROS，可直接运行：
    python test/test_yaw_loop.py

也兼容 pytest：
    pytest test/

【本文件的组织方式：性质函数 + 变异自检】

每个被测性质写成一个**返回 bool** 的函数（prop_xxx），而不是直接 assert。
这样才能做变异自检：把某一处故意改坏，用**同一个函数**再算一遍，断言它这次
返回 False。用例能红才叫用例 —— 全绿不构成证据。

    check(prop_converges(),            "闭环收敛")
    _mutate(...)                       # 注入一个具体的 bug
    check(not prop_converges(),        "变异自检：注入这个 bug 时用例必须变红")

模拟的被控对象在**本文件**里，不在 yaw_loop.py 里 —— 模块保持零依赖，
跟 protocol.py 的定位一致。
"""

import math
import os
import random
import sys
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from l150pro_driver.yaw_loop import YawLoopConfig, YawRateLoop  # noqa: E402

_failures = []


def check(cond, label):
    if cond:
        print(f"  pass  {label}")
    else:
        print(f"  FAIL  {label}")
        _failures.append(label)


# --------------------------------------------------------------------------
# 被控对象与仿真骨架
# --------------------------------------------------------------------------

class Plant:
    """一阶惯性 + 未知增益 + 可选的纯传输延迟。

    实测这台车 k 在 0.5 附近（只走到指令的一半），但**设计点取 k=1.0** ——
    侧滑只会让等效轮距变大，所以 k<=1 是物理上界，0.5 是"好走"的地面。
    """

    def __init__(self, k=0.5, tau=0.1, delay_ticks=0):
        self.k = k
        self.tau = tau
        self.wz = 0.0
        self._q = deque([0.0] * max(1, delay_ticks), maxlen=max(1, delay_ticks))
        self._delayed = delay_ticks > 0

    def advance(self, wz_out, dt):
        """把这一拍的下行指令走一步，返回新的真实角速度。"""
        if self._delayed:
            self._q.append(wz_out)
            wz_out = self._q[0]
        target = self.k * wz_out
        self.wz += (target - self.wz) * (dt / (self.tau + dt))
        return self.wz


def simulate(cfg, plant, ref_fn, seconds=6.0, dt=0.05,
             allowed_fn=None, gyro_fn=None, delay_ticks=0):
    """跑闭环。返回 (loop, [(t, wz_ref, wz_out, wz_meas, allowed), ...])。

    时序刻意做成因果的：先读陀螺（此刻的**真实**角速度），再算这一拍该发
    什么，指令要下一拍才影响对象。反过来的话会凭空多出零延迟的反馈。
    """
    loop = YawRateLoop(cfg)
    if delay_ticks:
        plant = Plant(plant.k, plant.tau, delay_ticks)
    t, out, hist = 0.0, 0.0, []
    for _ in range(int(round(seconds / dt))):
        gz = gyro_fn(t, plant.wz) if gyro_fn else plant.wz
        loop.update_gyro(gz, t)
        ref = ref_fn(t)
        allowed = allowed_fn(t) if allowed_fn else True
        out = loop.step(ref, t, allowed=allowed)
        plant.advance(out, dt)
        hist.append((t, ref, out, gz, allowed))
        t += dt
    return loop, hist


def settled(hist, tail_s=2.0, dt=0.05):
    """取末段样本，用来判稳态。"""
    n = max(1, int(tail_s / dt))
    return hist[-n:]


def _mutate(obj, name, replacement):
    """临时把 obj.name 换掉，返回一个还原用的上下文管理器。"""
    class _Ctx:
        def __enter__(self):
            self.orig = getattr(obj, name)
            setattr(obj, name, replacement)

        def __exit__(self, *exc):
            setattr(obj, name, self.orig)
            return False
    return _Ctx()


def _cfg_mutated(**overrides):
    """临时替换本模块的 YawLoopConfig，让 simulate() 拿到被改坏的参数。

    注意补的是**测试文件**的全局名，不是 yaw_loop 模块里的那个：
    cfg 是实例属性（YawRateLoop.__init__ 里挂上去的），没法在类上打补丁。
    所有变异用例都显式构造 Tc()，所以换这个名字就够了。
    """
    class _Ctx:
        def __enter__(self):
            self._orig = globals()["YawLoopConfig"]

            def broken(*a, **kw):
                return self._orig(*a, **dict(kw, **overrides))

            globals()["YawLoopConfig"] = broken

        def __exit__(self, *exc):
            globals()["YawLoopConfig"] = self._orig
            return False
    return _Ctx()


CFG = YawLoopConfig()          # **真实默认**（实车调出来的那组）

# ⚠️ 单测里跑算法的用例要用**快增益**，不能用默认值。
# 默认值（0.2/0.2）是照**实车那个慢对象**调的（有 ~1s 死区、秒级响应），
# 而这里的仿真对象是快的（tau=0.1s）——用默认值跑，6 秒都收敛不了，
# 于是用例测的是"默认增益够不够快"而不是"算法对不对"。
# 2026-09-20 改默认值时这批用例集体变红，就是踩了这个耦合。
# 守卫是否在**真实默认增益**下有效，另见 test_guards_at_real_gains()。
FAST = dict(kp=1.5, ki=2.5)


def Tc(**kw):
    """测试用配置 = 快增益 + 覆盖项。

    内部照常走全局名 YawLoopConfig，所以 _cfg_mutated 的变异照旧生效。
    """
    return YawLoopConfig(**dict(FAST, **kw))




# --------------------------------------------------------------------------
# 1. 负反馈：闭环必须收敛
# --------------------------------------------------------------------------

def prop_converges(k=0.5, wz_ref=0.3, tau=0.1):
    _, hist = simulate(Tc(), Plant(k=k, tau=tau),
                       lambda t: wz_ref, seconds=6.0, delay_ticks=1)
    mean_meas = sum(s[3] for s in settled(hist)) / len(settled(hist))
    return abs(mean_meas - wz_ref) < 0.01


def test_convergence():
    print("\n== 1. 负反馈：闭环收敛 ==")
    for k in (0.4, 0.5, 0.7, 1.0, 1.5):
        check(prop_converges(k=k),
              f"k={k} 收敛到 0.3（开环只能到 {k * 0.3:.2f}）")

    # 反向指令也要收敛（不许有半边生效）
    check(prop_converges(k=0.5, wz_ref=-0.3), "k=0.5、指令 -0.3 收敛")

    print("  -- 变异自检 --")
    # 把反馈取反 = 正反馈。符号守卫会介入并退回直通，于是收不住。
    orig = YawRateLoop.wz_meas
    with _mutate(YawRateLoop, "wz_meas", property(lambda self: -orig.fget(self))):
        check(not prop_converges(k=0.5), "反馈取反 -> 用例变红")
    # 去掉积分：稳态只剩 w = k*(1+Kp)*ref/(1+k*Kp)，补不满
    with _cfg_mutated(ki=0.0):
        check(not prop_converges(k=0.5), "去掉积分 -> 用例变红")


# --------------------------------------------------------------------------
# 2. 与增益无关：这才是本模块存在的理由
# --------------------------------------------------------------------------

def test_gain_independence():
    print("\n== 2. 与未知增益 k 无关 ==")
    k_min = 1.0 / (1.0 + CFG.i_clamp_frac)
    for k in (0.4, 0.5, 0.8, 1.0):
        check(prop_converges(k=k), f"k={k} 精确收敛（补偿范围 k_min={k_min:.2f}）")

    # 低于声明的 k_min **故意**不补满，写清楚免得后人当 bug 修
    _, hist = simulate(Tc(), Plant(k=0.3), lambda t: 0.3,
                       seconds=6.0, delay_ticks=1)
    m = sum(s[3] for s in settled(hist)) / len(settled(hist))
    check(0.2 < m < 0.3, f"k=0.30 < k_min 时按设计只补到 {m:.3f}（不是 bug）")


# --------------------------------------------------------------------------
# 3. 防跑飞：陀螺卡死
# --------------------------------------------------------------------------

def prop_stall_bounded(wz_ref=0.3, seconds=6.0):
    """卡死时输出必须**正比于指令**（而不是顶到绝对钳位）。
    返回 (是否触发, 窗口内 |wz_out| 峰值，相对指令的倍数)。"""
    _, hist = simulate(Tc(), Plant(k=0.5), lambda t: wz_ref,
                       seconds=seconds, gyro_fn=lambda t, w: 0.0)
    loop, _ = simulate(Tc(), Plant(k=0.5), lambda t: wz_ref,
                       seconds=seconds, gyro_fn=lambda t, w: 0.0)
    peak = max(abs(s[2]) for s in hist) / wz_ref
    return loop.latched, peak


def test_stall_stuck_zero():
    print("\n== 3. 陀螺卡死在 0 ==")
    latched, peak = prop_stall_bounded()
    check(latched, "卡死被检测到并 latch")
    # 边界要用**本用例实际用的增益**（FAST），不是 CFG —— CFG 是实车默认值
    bound = 1.0 + FAST["kp"] + CFG.i_clamp_frac
    check(peak <= bound + 1e-9,
          f"峰值 {peak:.2f}x 指令，不超过 1+Kp+I_CLAMP_FRAC={bound:.2f}x")

    # 松杆即消失 —— 这是"正比于指令"最要紧的实际后果
    _, hist = simulate(Tc(), Plant(k=0.5),
                       lambda t: 0.3 if t < 3.0 else 0.0,
                       seconds=6.0, gyro_fn=lambda t, w: 0.0)
    check(hist[-1][2] == 0.0, "松杆后输出归零（不会自己转下去）")

    print("  -- 变异自检 --")
    # 积分钳位从"正比于指令"退化成"绝对上限"：峰值就不再正比于指令了
    with _cfg_mutated(i_clamp_frac=1e3):
        _, peak2 = prop_stall_bounded()
        check(peak2 > 1.0 + CFG.kp + CFG.i_clamp_frac,
              f"去掉积分钳位 -> 峰值涨到 {peak2:.1f}x，用例变红")


def test_stall_stuck_nonzero():
    print("\n== 4. 陀螺卡在非零值（比卡在 0 更阴）==")
    # 反馈恒 +0.5：符号是对的、CRC 是对的、幅度也像真的，只是永不变化
    def gyro(t, w):
        return 0.5

    loop, hist = simulate(Tc(), Plant(k=0.5),
                          lambda t: 0.3 if t < 2.0 else -0.3,
                          seconds=8.0, gyro_fn=gyro)
    check(loop.latched, "卡在 +0.5 也能被抓到（只看极差，不看绝对值）")


# --------------------------------------------------------------------------
# 5. 卡死检测的误报面
# --------------------------------------------------------------------------

def test_stall_false_positives():
    print("\n== 5. 卡死检测不许误报 ==")

    # 5a 小指令下的慢转：闭环总修正量本来就小，触发条件不该成立
    loop, _ = simulate(Tc(), Plant(k=0.5), lambda t: 0.03,
                       seconds=12.0, gyro_fn=lambda t, w: 0.1 * w)
    check(not loop.latched, "指令 0.03 rad/s 的慢转不误判")

    # 5b 被控对象迟钝（tau=0.3s）：响应滞后不等于卡死
    loop, _ = simulate(Tc(), Plant(k=1.0, tau=0.3), lambda t: 0.5,
                       seconds=12.0, delay_ticks=1)
    check(not loop.latched, "tau=0.3s 的迟钝对象不误判")

    # 5c 陀螺有噪声
    rnd = random.Random(7)
    loop, _ = simulate(Tc(), Plant(k=0.5), lambda t: 0.5,
                       seconds=12.0,
                       gyro_fn=lambda t, w: w + rnd.uniform(-0.05, 0.05),
                       delay_ticks=1)
    check(not loop.latched, "±0.05 rad/s 陀螺噪声不误判")

    print("  -- 变异自检 --")
    # 退回"只看当前拍"的瞬时判据 —— 这正是滑动窗口要解决的问题
    def naive_stall(self, now, wz_out, w):
        return ("stall" if (abs(w) < self.cfg.stall_meas_span
                            and abs(wz_out) > self.cfg.stall_out_span) else None)

    with _mutate(YawRateLoop, "_check_stall", naive_stall):
        loop2, _ = simulate(Tc(), Plant(k=1.0, tau=0.3),
                            lambda t: 0.5, seconds=12.0, delay_ticks=1)
        check(loop2.latched, "退回瞬时判据 -> 迟钝对象一上来就被误判，用例变红")


# --------------------------------------------------------------------------
# 6. 符号判反守卫
# --------------------------------------------------------------------------

def test_guards_at_real_gains():
    """⚠️ 守卫必须在**实车默认增益**下有效 —— 2026-09-20 补的回归用例。

    把默认增益从 1.5/2.5 降到 0.2/0.2（实车调出来的那组）之后，卡死守卫
    **静默失效**：极差判据要求"窗口内输出动过 0.2"，而低增益输出爬得极慢，
    1 秒窗口里根本不动那么多，判据压根不 armed；角度判据当时又只在 0.6 秒
    的子窗口上积，也够不到门槛。

    **是这套单测抓到的，不是实车** —— 但抓到的原因是当时所有用例都拿默认
    配置跑，所以默认值一变就集体变红。修法是：算法用例改显式快增益（见 Tc），
    守卫用例明确地用 CFG 盯着默认值。
    """
    print("\n== 9b. 守卫在【真实默认增益】下也必须有效 ==")
    print(f"   （默认 Kp={CFG.kp} Ki={CFG.ki} —— 实车调出来的那组）")

    loop, _ = simulate(YawLoopConfig(), Plant(k=0.5), lambda t: 0.4,
                       seconds=3.0, gyro_fn=lambda t, w: 0.0)
    check(loop.latched, "默认增益下陀螺卡死在 0 也必须被抓到")

    # ⚠️ 已知限制：**"陀螺卡在非零值"在低增益下检测不到。** 不假装能抓，
    # 断言的是"后果有界"而不是"能检测"：
    #   · 极差判据靠"输出动过 0.2"armed —— 低增益下不成立
    #   · 角度判据看不见它 —— 非零读数本身就是"测量有响应"
    #   · 也不能改用"测量冻住了" —— 固件的 gz 量化到 1 mrad/s，
    #     **真实稳转时读数本来就长时间不变**，那样会误判
    # 在实验增益（1.5/2.5）下这条是能抓到的（见 test_stall_stuck_nonzero）。
    loop2, hist2 = simulate(YawLoopConfig(), Plant(k=0.5), lambda t: 0.4,
                            seconds=6.0, gyro_fn=lambda t, w: 0.5)
    peak = max(abs(s[2]) for s in hist2)
    bound = 1.0 + CFG.kp + CFG.i_clamp_frac
    check(peak <= bound + 1e-9,
          f"卡在非零值：检测不到（已知限制），但幅度有界 {peak:.2f}x ≤ {bound:.2f}x")

    print("  -- 变异自检 --")
    # 角度判据的窗口缩回和极差判据一样短（≈当初失效的那个 0.6s 子窗口）
    with _cfg_mutated(stall_angle_window=0.6):
        loop3, _ = simulate(YawLoopConfig(), Plant(k=0.5), lambda t: 0.4,
                            seconds=3.0, gyro_fn=lambda t, w: 0.0)
        check(not loop3.latched,
              "角度窗口缩到 0.6s -> 低增益下守卫失效，用例变红")


def test_wrong_sign():
    print("\n== 6. 符号判反守卫（正反馈）==")
    loop, hist = simulate(Tc(), Plant(k=0.5), lambda t: 0.3,
                          seconds=6.0, gyro_fn=lambda t, w: -w)
    check(loop.latched, "反馈整体取反被守卫抓到")
    check(loop.trip_reason == "wrong_sign", f"归因是 {loop.trip_reason!r}")
    check(max(abs(s[2]) for s in hist) <= CFG.max_wz_out + 1e-9,
          "期间输出没突破硬钳位")

    # 正常的快速反向打杆**不许**误判：车还在往旧方向转是物理事实
    loop2, _ = simulate(Tc(), Plant(k=0.5, tau=0.15),
                        lambda t: 0.4 if (t // 2.0) % 2 == 0 else -0.4,
                        seconds=12.0, delay_ticks=1)
    check(not loop2.latched, "每 2s 反向打杆不误判（dwell > 建立时间）")

    print("  -- 变异自检 --")
    with _mutate(YawRateLoop, "_check_wrong_sign", lambda self, *a: None):
        loop3, _ = simulate(Tc(), Plant(k=0.5), lambda t: 0.3,
                            seconds=6.0, gyro_fn=lambda t, w: -w)
        check(not loop3.latched, "去掉守卫 -> 正反馈无人拦，用例变红")


# --------------------------------------------------------------------------
# 7. 状态切换必须无阶跃
# --------------------------------------------------------------------------

def prop_bumpless(rate=None):
    """全程 |Δwz_out| <= rate*dt（速率限幅不被突破）。"""
    cfg = Tc() if rate is None else Tc(out_rate_limit=rate)
    f = cfg.out_rate_limit
    dt = 0.05
    # 故意制造各种切换：投入、过死区、脱开、再投入
    def ref(t):
        if t < 1.0:
            return 0.0
        if t < 2.0:
            return 0.5                      # 阶跃投入
        if t < 2.5:
            return 0.03                     # 掉进死区
        if t < 4.0:
            return 0.5                      # 再投入
        return 0.0                          # 松杆
    _, hist = simulate(cfg, Plant(k=0.5), ref, seconds=6.0, delay_ticks=1)
    prev = 0.0
    for _, _, out, _, _ in hist:
        if abs(out - prev) > f * dt + 1e-9:
            return False
        prev = out
    return True


def test_bumpless():
    print("\n== 7. 状态切换无阶跃（速率限幅）==")
    check(prop_bumpless(), "投入/过死区/脱开/松杆全程不超过速率限幅")

    # 投入那一拍：没有限幅的话输出是 (1+Kp)*ref = 2.5 倍指令，是个明显的前冲
    _, hist = simulate(Tc(), Plant(k=0.5),
                       lambda t: 0.5 if t >= 1.0 else 0.0,
                       seconds=3.0, delay_ticks=1)
    i = next(j for j, s in enumerate(hist) if s[0] >= 1.0)
    check(abs(hist[i][2] - hist[i - 1][2]) <= CFG.out_rate_limit * 0.05 + 1e-9,
          "投入那一拍是斜坡，不是阶跃")

    print("  -- 变异自检 --")
    with _mutate(YawRateLoop, "_slew", lambda self, target, dt: target):
        check(not prop_bumpless(), "去掉速率限幅 -> 用例变红")


# --------------------------------------------------------------------------
# 8. 死区迟滞
# --------------------------------------------------------------------------

def _toggle_count(engage, disengage, dither=0.002, seconds=10.0):
    """围绕投入阈值抖动指令，数投入/脱开的切换次数。

    用**正常对象**（陀螺有真实反馈），所以卡死守卫不会介入，
    观察到的就纯粹是迟滞本身的行为。
    """
    loop = YawRateLoop(YawLoopConfig(engage_thresh=engage,
                                     disengage_thresh=disengage))
    plant = Plant(k=0.5)
    t, n, prev = 0.0, 0, None
    for i in range(int(seconds / 0.05)):
        loop.update_gyro(plant.wz, t)
        out = loop.step(engage + (dither if i % 2 else -dither), t)
        plant.advance(out, 0.05)
        if prev is not None and loop.engaged != prev:
            n += 1
        prev = loop.engaged
        t += 0.05
    return n


def prop_no_chatter(engage, disengage):
    return _toggle_count(engage, disengage) <= 1


def test_hysteresis():
    print("\n== 8. 死区迟滞 ==")
    check(prop_no_chatter(CFG.engage_thresh, CFG.disengage_thresh),
          f"带迟滞（进 {CFG.engage_thresh} / 出 {CFG.disengage_thresh}）："
          f"阈值附近抖 10s 内切换 <= 1 次")

    print("  -- 变异自检 --")
    check(not prop_no_chatter(0.05, 0.05),
          f"塌成单阈值 -> 切换 {_toggle_count(0.05, 0.05)} 次，用例变红")


# --------------------------------------------------------------------------
# 9. latch 只能靠回中位解除
# --------------------------------------------------------------------------

def test_latch_reset():
    print("\n== 9. latch 的解除语义 ==")

    def still_latched(neutral_s):
        """触发守卫 -> 回中位 neutral_s 秒 -> 给回指令，看它有没有重新投入。"""
        loop = YawRateLoop(Tc())
        t = 0.0
        for _ in range(int(3.0 / 0.05)):      # 反馈恒 0，轮询到守卫触发
            loop.update_gyro(0.0, t)
            loop.step(0.4, t)
            t += 0.05
        assert loop.latched, "前置条件：守卫本该已触发"
        for _ in range(int(round(neutral_s / 0.05))):
            loop.update_gyro(0.0, t)
            loop.step(0.0, t)
            t += 0.05
        for _ in range(10):                   # 给回指令，跑几拍
            loop.update_gyro(0.0, t)
            loop.step(0.4, t)
            t += 0.05
        return loop.latched and not loop.engaged

    check(still_latched(0.4), "回中位 0.4s（< 0.5s）仍保持 latch")
    check(not still_latched(0.7), "回中位 0.7s（>= 0.5s）解除并重新投入")

    print("  -- 变异自检 --")
    # 单拍过零就解除：一次快速打杆穿越中位就会清掉真故障。
    # 用**同一个** 0.4s 回中位输入做 A/B
    with _cfg_mutated(latch_clear_time=0.0):
        check(not still_latched(0.4),
              "改成单拍即解除 -> 同样的 0.4s 回中位就解除了，用例变红")


# --------------------------------------------------------------------------
# 10. allowed / 陀螺过期
# --------------------------------------------------------------------------

def test_gating():
    print("\n== 10. 允许位与新鲜度 ==")

    # allowed=False 必须直通，且状态清零（不许带着陈旧积分再投入）
    loop = YawRateLoop(Tc())
    t = 0.0
    for _ in range(10):
        loop.update_gyro(0.0, t)
        loop.step(0.3, t)
        t += 0.05
    check(loop.integral != 0.0, "前置条件：积分本该爬起来")
    out = loop.step(0.3, t, allowed=False)
    check(loop.integral == 0.0, "allowed=False 清积分")
    # "直通"不是一拍到位：输出仍走速率限幅，几拍内收到 wz_ref。
    # 要的是"不再放大"，不是一个瞬时的值。
    for _ in range(10):
        t += 0.05
        out = loop.step(0.3, t, allowed=False)
    check(out == 0.3, f"allowed=False 收敛到直通值（实得 {out!r}）")
    check(out <= 0.3 + 1e-9, "allowed=False 时输出不会**超过**指令")

    # 电压跌落 -> 固件拒绝运动 -> 对象冻结，此时闭环会顶着死对象积分爬。
    # allowed 掉了之后必须一直直通，不许再放大。
    _, hist = simulate(Tc(), Plant(k=0.5), lambda t: 0.3,
                       seconds=4.0,
                       allowed_fn=lambda t: t < 2.0,      # 2s 后欠压
                       gyro_fn=lambda t, w: w)
    check(hist[-1][2] == 0.3, "不获准期间一直直通")

    # 陀螺过期
    loop3 = YawRateLoop(Tc())
    loop3.update_gyro(0.0, 0.0)
    loop3.step(0.3, 0.0)
    out = loop3.step(0.3, 5.0)
    check(out == 0.3 and loop3.integral == 0.0, "陀螺过期时直通并清积分")
    check(loop3.gyro_age(0.0) == 0.0, "gyro_age 正确")
    check(YawRateLoop(Tc()).gyro_age(1.0) is None, "从未收到陀螺 -> None")


# --------------------------------------------------------------------------
# 11. 死区精确性与输入卫生
# --------------------------------------------------------------------------

def test_edge_inputs():
    print("\n== 11. 边界输入 ==")

    # 死区内必须是**位精确**的直通，不是"接近"。容差比较会掩盖整类 off-by-one
    loop = YawRateLoop(Tc())
    t = 0.0
    for _ in range(200):                      # 先跑到稳态
        loop.update_gyro(0.0, t)
        loop.step(0.0, t)
        t += 0.05
    out = loop.step(0.019, t)
    check(out == 0.019, f"死区内位精确直通（实得 {out!r}）")
    check(not loop.engaged, "死区内不投入")

    # dt 卫生：0 / 负 / 巨大 / nan / inf 都不许炸，也不许把输出搞成非有限
    loop = YawRateLoop(Tc())
    bad = [0.0, -1.0, 1e9, float("nan"), float("inf")]
    t, bad_i = 0.0, None
    for i in range(400):
        loop.update_gyro(0.2, t)
        ref = [0.3, -0.3, 0.0, 0.05][i % 4]
        out = loop.step(ref, t)
        if not (math.isfinite(out) and abs(out) <= CFG.max_wz_out + 1e-9):
            bad_i = (i, out)
            break
        t += bad[i % len(bad)] if i % 7 == 0 else 0.05
    check(bad_i is None,
          "异常 dt（0/负/巨大/nan/inf）下输出始终有限且在钳位内"
          + (f" —— 第 {bad_i[0]} 拍出了 {bad_i[1]!r}" if bad_i else ""))

    # 指令本身是 nan/inf 时不许传播
    loop = YawRateLoop(Tc())
    ok = True
    for r in (float("nan"), float("inf"), float("-inf"), None):
        try:
            o = loop.step(r if r is not None else float("nan"), 0.0)
            ok = ok and math.isfinite(o) and abs(o) <= CFG.max_wz_out + 1e-9
        except TypeError:
            pass
    check(ok, "非法指令值不传播成非有限的输出")


# --------------------------------------------------------------------------
# 12. 时间语义
# --------------------------------------------------------------------------

def test_timing():
    print("\n== 12. 时间语义 ==")

    def integral_at(dt, n):
        """跑 n 拍、每拍 dt，返回积分值。累计仿真时长 = n*dt。

        预热那一拍 dt 未知（_elapsed 首拍返回 None），不积分也不计时，
        否则两边差一拍会把"步长无关"测成假的。
        """
        loop = YawRateLoop(YawLoopConfig(dt_nominal=dt))
        t = 0.0
        loop.update_gyro(0.0, t)
        loop.step(0.4, t)
        for _ in range(n):
            t += dt
            loop.update_gyro(0.0, t)
            loop.step(0.4, t)
        return loop.integral

    # 同样的累计仿真时长、不同步长 -> 积分应当一样，说明 Ki 真的按 dt 在积。
    # 拍数刻意取小，避开积分钳位（i_lim = 1.5*0.4 = 0.6）—— 饱和之后
    # 两边都顶在钳位上，"相等"就变成平凡的了，掩盖真问题。
    a, b = integral_at(0.05, 5), integral_at(0.025, 10)
    check(abs(a - b) <= 0.01 * max(abs(a), 1e-9) + 1e-6,
          f"Ki 与步长无关：dt=0.05 -> {a:.4f}，dt=0.025 -> {b:.4f}")

    print("  -- 变异自检 --")
    # 写死 dt=0.05、无视传入的 dt
    with _mutate(YawRateLoop, "_elapsed", lambda self, now: 0.05):
        a2, b2 = integral_at(0.05, 5), integral_at(0.025, 10)
        check(abs(a2 - b2) > 0.01 * max(abs(a2), 1e-9) + 1e-6,
              f"写死 dt -> 两者分叉（{a2:.4f} vs {b2:.4f}），用例变红")


# --------------------------------------------------------------------------
# 13. 无隐藏状态
# --------------------------------------------------------------------------

def test_no_hidden_state():
    print("\n== 13. 无隐藏状态 ==")

    def run_once(seed):
        rnd = random.Random(seed)
        loop = YawRateLoop(Tc())
        t, acc = 0.0, []
        for _ in range(100):
            loop.update_gyro(rnd.uniform(-0.3, 0.3), t)
            acc.append(loop.step(rnd.uniform(-0.5, 0.5), t))
            t += 0.05
        return acc

    a, b = run_once(1), run_once(1)
    check(a == b, "同样输入序列 -> 完全相同的输出（无类级共享状态）")

    # reset() 必须精确回到初始状态
    loop = YawRateLoop(Tc())
    t = 0.0
    for _ in range(50):
        loop.update_gyro(0.2, t)
        loop.step(0.3, t)
        t += 0.05
    loop.reset()
    fresh = YawRateLoop(Tc())
    for attr in ("integral", "wz_meas", "engaged", "latched", "trip_reason"):
        check(getattr(loop, attr) == getattr(fresh, attr),
              f"reset() 后 {attr} 与新建实例一致")
    check(loop.gyro_age(0.0) is None, "reset() 后陀螺时间戳已清")


# --------------------------------------------------------------------------

def main():
    test_convergence()
    test_gain_independence()
    test_stall_stuck_zero()
    test_stall_stuck_nonzero()
    test_stall_false_positives()
    test_guards_at_real_gains()
    test_wrong_sign()
    test_bumpless()
    test_hysteresis()
    test_latch_reset()
    test_gating()
    test_edge_inputs()
    test_timing()
    test_no_hidden_state()
    print("\n" + "=" * 60)
    if _failures:
        print(f"{len(_failures)} 项失败：")
        for f in _failures:
            print(f"  - {f}")
        sys.exit(1)
    print("全部通过")


if __name__ == "__main__":
    main()
