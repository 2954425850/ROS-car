# test/test_grasp.py
import math
import pytest
from arm_grasp import geom, grasp
from arm_grasp.grasp import Refused

# 单测夹具点：**不是当前瓶盖位置**（瓶盖已被挪过，旧数据一律作废），只用来做几何单测。
# 2026-09-30 实测扫过一遍（z=0.016、α∈[−88°,−45°]、s=0 的末态）：
#     r ≤ 16cm → **refuse**（解不出姿态）；r=17cm → slack 14（warn）；r=18cm → 47；
#     r=20cm → 111；r=25cm → 163。
# 所以夹具取 **r≈20cm**，而且**故意离轴**（方位角 −60°）：在 −y 轴上时下扎轴的水平 x 分量
# 恒为 0（ux≈3e-17），单分量反号的变异抓不到（见 test_axis_point_direction_off_axis）。
CAP = (0.10, -0.175, 0.016)          # r = 20.2cm，方位角 −60.3°


def _cfg(**kw):
    d = dict(alpha_lo=-88.0, alpha_hi=-45.0, alpha_span=14.0, alpha_step=1.0,
             alpha0=-58.0, min_slack=40.0, warn_slack=10.0, slack_tie=5.0,
             gripper=240.0, wrist_roll=496.0,
             vis=grasp.Vis(0.08 * 320, 0.92 * 320, 0.12 * 180, 0.86 * 180))
    d.update(kw)
    return grasp.GraspConfig(**d)


def test_axis_point_is_s_above_O_along_the_gripper_axis():
    """s=0 ⇒ 就是 O 本身；s>0 ⇒ 沿下扎轴退 s，且退的方向与可见像素的方位一致。"""
    cfg = _cfg()
    assert grasp.axis_point(CAP, 0.0, -60.0) == pytest.approx(CAP, abs=1e-12)
    T = grasp.axis_point(CAP, 0.05, -60.0)
    assert math.dist(T, CAP) == pytest.approx(0.05, abs=1e-9)
    # 下扎轴指向下：z 分量必须为正（爪尖在目标**上方**）
    assert T[2] > CAP[2]


def test_pick_target_respects_min_slack():
    """选出来的姿态余量必须尽量大（这条对应 17:34 那次肘关节顶死的失败）。

    夹具点本身必须是"舒服"的点 ⇒ 直接断言 `tier()=='ok'`：夹具哪天飘到边缘，
    这条会**立刻红**并说明是夹具的问题，而不是悄悄退化成 warn（上一版就只差 1 count）。
    """
    cfg = _cfg()
    t = grasp.pick_target(CAP, cfg, 0.0, -58.0, cfg.vis)
    assert grasp.tier(t.slack, cfg) == 'ok'    # 夹具点必须余量充裕
    assert t.slack >= 40.0                     # ≥ min_slack，留出余量
    assert isinstance(t.fields, list) and len(t.fields) == 6
    assert all(125.0 <= v <= 875.0 for v in t.fields[2:5])


def test_pick_target_refuses_unreachable():
    with pytest.raises(Refused):
        grasp.pick_target((0.0, 0.0, 3.0), _cfg(), 0.0, -58.0, _cfg().vis)   # 3 米高


def test_visible_uses_project():
    """看得见 = 模型预测的像素落在有效区。用一个被裁到画面外的点验证它真的会红。"""
    from arm_grasp.arm_kin import from_fields
    from arm_grasp.servo import ik_open_m
    j = ik_open_m(CAP[0], CAP[1], CAP[2] + 0.05, -60.0)
    ok, px = grasp.visible(j, CAP, _cfg().vis)
    assert isinstance(ok, bool) and len(px) == 2
    far = (0.0, -0.9, 0.5)                      # 很远的点 → 一定出画
    ok2, _ = grasp.visible(j, far, _cfg().vis)
    assert ok2 is False


def test_tier_three_levels():
    cfg = _cfg()
    assert grasp.tier(60.0, cfg) == 'ok'
    assert grasp.tier(20.0, cfg) == 'warn'
    assert grasp.tier(3.0, cfg) == 'refuse'


def test_plan_verdict_tiers():
    lvl, tgt, msg = grasp.plan_verdict(CAP, _cfg(), _cfg().vis)
    assert lvl in ('ok', 'warn', 'refuse') and msg
    if lvl != 'refuse':
        assert tgt is not None
    lvl2, _, _ = grasp.plan_verdict((0.0, 0.0, 3.0), _cfg(), _cfg().vis)
    assert lvl2 == 'refuse'


# ---------------------------------------------------------------------------
# ★ 以下两条是**变异自检的兜底**：计划 Task 4 Step 4 自己写了
#   "如果测试仍绿…那就在测试里加一条"。上面 6 条是计划原文，一字未改。
# ---------------------------------------------------------------------------

def test_axis_point_direction_off_axis():
    """★ 兜底（对应变异 1：`O_m[0] - s_m * ux` → `O_m[0] + s_m * ux`）。

    为什么计划里那条抓不到（两条独立的理由，都算过）：
    1. 夹具点 CAP=(0,-0.17,·) 在 **−y 轴**上 ⇒ 下扎轴的水平 x 分量
       ux = cosα·cos(tb) ≈ 3.1e-17，x 项贡献只有 **1.5e-18 m**；
    2. 更根本："把某一个分量反号"**不改变 |T−O|** ⇒ 只断言 `math.dist` 的用例
       在数学上不可能抓到任何单分量反号。
    这里换一个**离轴**的点（方位角 −45°，ux ≈ 0.354），并把**完整方向向量**
    钉死 ⇒ 三个分量里任何一个反号都会红。
    """
    O = (0.12, -0.12, 0.016)
    s, alpha = 0.05, -60.0
    T = grasp.axis_point(O, s, alpha)
    assert T == pytest.approx((0.1023223304703363, -0.1023223304703363,
                               0.05930127018922193), abs=1e-9)
    d = tuple((T[i] - O[i]) / s for i in range(3))      # 单位下扎轴 −ẑ_T(α)
    assert d == pytest.approx((-0.3535533905932739, 0.3535533905932739,
                               0.8660254037844386), abs=1e-9)
    assert d[0] < 0.0 and d[1] > 0.0                    # 与 −ẑ_T 同向，不是镜像
    assert d[2] == pytest.approx(-math.sin(math.radians(alpha)))


def test_pick_target_breaks_slack_ties_toward_prefer_alpha():
    """★ 兜底（对应变异 3：`near.sort` 的 key 去掉 `abs(c.alpha - prefer_alpha)`）。

    为什么计划里那条抓不到：默认 `slack_tie=5.0` 时，夹具点上**并列档里只有
    一个候选**（s=0 时是 −72°，s=0.05 时是 −69°）⇒ 那条排序是死代码。
    把宽容放宽到能让 8 个候选全进并列档，排序才被真正覆盖 —— 这正好是那个
    "避免每拍换解"的连续性约束：并列时**宁可选余量只有 2.8 的 −65°，也要选
    离 prefer(−58°) 最近的那个**（余量最大的是 −69°/50.5）。
    """
    cfg = _cfg(slack_tie=60.0)          # 宽容放大到 60 → 并列档里真有多个候选
    t = grasp.pick_target(CAP, cfg, 0.05, -58.0, cfg.vis)
    # 实测并列档（本夹具、s=5cm、prefer=−58°）：
    #   α     : -70   -69   -68   -67   -66   -65   -64   -63   -62   -61
    #   slack : 18.5  31.4  43.6  55.2  66.4 *77.3* 68.5  54.4  40.7  27.4
    # 余量最大的是 −65°(77.3)；选中的是**离 prefer(−58°) 最近**的 −61°(27.4)
    # —— 为了"每拍不换解"的连续性主动放弃 50 点余量（故意的：那条排序规则就是这么用的）。
    assert t.alpha == pytest.approx(-61.0)
    assert t.slack == pytest.approx(27.406628, abs=1e-6)


# ---------------------------------------------------------------------------
# Task 5：流式主循环 + 估计器（M2）
# ---------------------------------------------------------------------------
from arm_grasp.arm_kin import from_fields, to_fields
from arm_grasp.servo import ik_open_m


class Plant:
    """一台"真"臂：关节按速度上限追指令。模型即真值（模型误差另用参数注入）。"""

    def __init__(self, j0, v_max_deg=20.0, tip_err_m=(0.0, 0.0, 0.0)):
        self.j = dict(j0)
        self.v = v_max_deg
        self.tip_err = tip_err_m

    def step(self, j_cmd, dt):
        for k in self.j:
            d = j_cmd[k] - self.j[k]
            m = self.v * dt
            self.j[k] += max(-m, min(m, d))


class FakeLink:
    """实现 Link 协议：假臂 + 用 geom 当"真"相机。"""

    def __init__(self, plant, O_true, hz=10.0, noise_px=0.0, cut_after=None,
                 seed=1):
        self.p = plant
        self.O = O_true
        self.hz = hz
        self.t = 0.0
        self.pub = None
        self.pub_n = 0
        self.pubs = []      # 每一拍发出去的 field 都记下来（变异自检要查**全部**，不是最后一条）
        self.noise = noise_px
        self.cut_after = cut_after
        self.rng = __import__('random').Random(seed)

    def now(self):
        return self.t

    def spin(self, dt):
        self.t += dt

    def fb(self):
        return to_fields(self.p.j, 240.0, 496.0)

    def obs(self):
        if self.cut_after is not None and self.pub_n > self.cut_after:
            return None
        try:
            u, v = geom.project(self.p.j, self.O)
        except ValueError:
            # ★ 相对计划原文补的一层：目标在**相机平面之后**时 `geom.project` 抛
            #   ValueError，计划原文会让它从 `run()` 里直接冒出来（而不是"这一拍没观测"）。
            #   真链路里"看不到"就是没有框 ⇒ 这里返回 None 才是对的语义。
            return None
        if self.noise:
            u += self.rng.gauss(0, self.noise)
            v += self.rng.gauss(0, self.noise)
        return grasp.Obs(self.t, u, v, self.pub_n, 'track', None)

    def publish(self, fields):
        self.pub = list(fields)
        self.pubs.append(list(fields))
        self.pub_n += 1
        self.p.j = from_fields(fields)                # 用 arm_kin 的，不要绕 grasp
        self.p.step(self.p.j, 1.0 / self.hz)


def _start_joints(s_m=0.10):
    """起步姿态：**用几何反推**（目标上方 s），别手写坐标——手写的很可能不可达。"""
    return grasp.pick_target(CAP, _cfg(), s_m, -58.0, _cfg().vis).joints


# 「够不着」用的假目标点：计划原文写的是 `(0.0, 0.0, 3.0)`（车体上方 3m），
# 但**这条和链路自相矛盾**，实测两点都对不上（2026-09-30 逐条跑出来）：
#   ① `geom.project` 对它在起步姿态抛 `点在相机平面之后（Z = -2.3934）` ——
#      相机是朝**下**看的，臂上方的点在光轴负侧 ⇒ 假相机根本生成不出这条观测；
#   ② 更深一层：`estimate_point` 是**射线∩z_plane_m 平面**求 O 的，"平面之上"的点
#      连 O 都产不出来（射线朝上 ⇒ 被 `d_z > -0.3` 挡掉），永远走不到 `pick_target` 的
#      refuse 分支。**"够不着"必须是一个相机看得见（在相机下方、射线朝下）但臂解不出的点。**
# 换成同一个平面 z=0.020 上、r=40cm 的点：project 正常（像素 (334, 12)）、
# 射线 d_z=-0.546 够陡、`pick_target` 实测 refuse（r≥40cm 超出臂展）。
UNREACHABLE = (0.0, -0.40, 0.020)


def test_loop_converges_to_the_object():
    """★ 物理判据（不是"循环自己说收敛了"）：**回读**关节算出的真爪尖到目标姿态点的距离。"""
    cfg = _cfg()._replace(hz=10.0, max_seconds=30.0, max_ticks=300)
    plant = Plant(_start_joints())
    link = FakeLink(plant, CAP)
    rep = grasp.run(cfg, link, phase='aim', log=lambda *a: None)
    assert rep['stopped'], rep['stopped']
    # ★ 相对计划原文加的一行（变异自检要求的）：**必须由"对准完成"这条判据退出**。
    #   只断言"爪尖恰好停在目标附近"是不够的——把收敛判据的 tol 改成 −1（变异 3）之后，
    #   循环会一直发到 stall/超时才退出，而**臂最后还是停在同一个地方** ⇒ 计划原文那几条
    #   断言全绿（实测：变异 3 下仍然 13 passed）。判据坏没坏，只有它能自己说出来。
    assert rep['ok'], rep['stopped']
    T_want = grasp.axis_point(CAP, cfg.s_pre_m, rep['alpha'])
    tip = grasp.tip_open_m(plant.j)
    assert math.dist(tip, T_want) < 0.01                           # ≤1cm
    assert grasp.standoff_along_axis(tip, CAP, rep['alpha']) == pytest.approx(
        cfg.s_pre_m, abs=0.01)                                     # 确实停在目标上方 s
    assert rep['max_step_deg_actual'] <= cfg.max_step_deg + 1e-6    # 限速真的生效


def test_loop_descends_and_stops_shallow():
    """下扎：实际余量必须压到 s_stop 以内（**按实际余量限速**压下去，不是按指令）。"""
    cfg = _cfg()._replace(hz=10.0, max_seconds=30.0, max_ticks=300)
    plant = Plant(_start_joints(s_m=0.05))
    link = FakeLink(plant, CAP)
    rep = grasp.run(cfg, link, phase='descend', log=lambda *a: None)
    assert rep['ok'], rep['stopped']
    # ★ 下界是相对计划原文加的：`s_ach` 是"爪尖**在目标上方**还有多远"，**必须是正的**
    #   （= `standoff_along_axis` 那道符号）。计划原文只钉了上界 ⇒ 符号反了的时候
    #   爪尖在目标**下方** 46mm 也能满足 `<= 0.004`，而且循环会在**第一拍**就报"扎到位"
    #   ——这条测试照样绿。实测过：把 `standoff_along_axis` 的负号去掉后，只有加上这个
    #   下界这条才会红。
    assert 0.0 <= rep['s_ach_m'] <= cfg.s_stop_m + 0.001
    assert grasp.tip_open_m(plant.j)[2] > CAP[2]        # 爪尖确实停在瓶盖**上方**
    assert rep['ticks'] >= 5                            # 是"压下去"的，不是一拍就宣布到位


def test_loop_holds_when_observation_dies():
    """观测断了：必须**明确报出来**并保持住估计，而不是崩/继续乱走。

    ★ `max_step_deg=0.3` 是相对计划原文加的一个覆盖，理由（实测出的，不是猜的）：
      计划原文用默认 1.5°/拍、起手姿态 `_start_joints()`（离目标只有 2cm）⇒ 循环在
      **第 5 拍**就"对准完成"退出，而观测正好在第 5 拍断掉、`obs_lost_s=1.0` 要 10 拍
      才到期 ⇒ **"观测丢失"这条分支在默认参数下根本不可达**（这条测试当时红着，
      `obs_lost=False`）。把接近速度放慢到 0.3°/拍（≈3°/s）后，走完这段要 ~25 拍，
      观测在第 5 拍断、丢失计时器在第 15 拍到期 ⇒ 分支真的被走到（留 10 拍余量）。
      物理上这也是更真实的一幕：**相机是在臂还在摆的中途掉的**，不是站定之后掉的。
    """
    cfg = _cfg()._replace(max_seconds=10.0, max_ticks=100, max_step_deg=0.3)
    plant = Plant(_start_joints())
    link = FakeLink(plant, CAP, cut_after=3)
    rep = grasp.run(cfg, link, phase='aim', log=lambda *a: None)
    assert rep['obs_lost'] is True                      # 明确报出来
    assert rep['O_last'] is not None                    # 保持住估计，不是崩
    assert '观测丢失' in rep['stopped']


def test_loop_refuses_when_target_unreachable():
    cfg = _cfg()._replace(max_seconds=5.0, max_ticks=50)
    link = FakeLink(Plant(_start_joints()), UNREACHABLE)
    rep = grasp.run(cfg, link, phase='aim', log=lambda *a: None)
    assert rep['ok'] is False and 'refuse' in rep['stopped']


def test_loop_rejects_noisy_observations():
    """★ Step 4 变异 2 需要的用例（计划自己在变异列表里要求"把 noise_px 加到 20 再断言
    `obs_bad > 0`"，但 Step 1 的代码块里**没有**这条测试）——门限必须真的在挡东西。

    判据是 `obs_bad > 0`：20px 的噪声折到平面上 ≈ 2~3cm，超过 `gate_m=0.03` 的
    抖动会被 `Est.update` 整条丢掉。把门限改成 `1e6` ⇒ 什么都不丢 ⇒ 这条红。
    """
    cfg = _cfg()._replace(hz=10.0, max_seconds=5.0, max_ticks=50)
    plant = Plant(_start_joints())
    link = FakeLink(plant, CAP, noise_px=20.0)
    rep = grasp.run(cfg, link, phase='aim', log=lambda *a: None)
    assert rep['obs_bad'] > 0, rep['obs_bad']
    assert rep['O_last'] is not None                    # 仍然锁在目标附近，不是崩
    assert math.dist(rep['O_last'], CAP) < 0.10


def test_loop_tolerates_start_below_field_floor():
    """★ 实机开场姿态**就在界外**：判据必须查"目标"，指令夹到界内。

    2026-09-30 在 Pi 上只读读到的开机姿态：
        feedback = [240, 497, 175, **121**, 409, -11]   ← p4=121 **在固件下限 125 之下**
    （肘关节歇在机械限位上）。这时第一拍的限速指令（p4≈123）必然也在界外 ——
    若判据查的是**这一拍的指令**，整条链会在第一拍就 `refuse：目标 field 出界`，**臂永远动不了**。
    假臂测试里的起手姿态永远是合法的，所以抓不到这件事；这条用例就是补这个缺口。
    判据改成查**目标**的 field + 指令夹进 [125,875] 之后：能正常起步、且发出去的值永远在界内。
    """
    # ⚠️ `max_step_deg=0.5` 是**必须的**，不是为了慢：肘关节离目标 27°，每拍 1.5°(6.25 count) 时
    #   第一拍 p4 就已经 127 > 125（界内）⇒ 夹取那条路根本走不到，**变异也不会红**
    #   （第一版就是 1.5，实测变异照绿）。用 0.5°/拍（2.08 count，与 T6 慢速真跑一致）
    #   第一拍 p4≈123.08 < 125 ⇒ 才真的覆盖到夹取。
    cfg = _cfg()._replace(hz=10.0, max_seconds=30.0, max_ticks=300, max_step_deg=0.5)
    FB_REST = [240.0, 497.0, 175.0, 121.0, 409.0, -11.0]   # 实机开机读数
    plant = Plant(from_fields(FB_REST))
    link = FakeLink(plant, CAP)
    rep = grasp.run(cfg, link, phase='aim', log=lambda *a: None)
    assert not (rep['stopped'] or '').startswith('refuse'), rep['stopped']
    assert all(125.0 <= v <= 875.0 for f in link.pubs for v in f[2:5])  # **每一拍**都在界内
    assert rep['ticks'] >= 3
