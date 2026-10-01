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

    def __init__(self, j0, v_max_deg=20.0, tip_err_m=(0.0, 0.0, 0.0), droop_deg=None, dead_deg=0.0):
        self.j = dict(j0)
        self.v = v_max_deg
        self.tip_err = tip_err_m
        # `droop_deg` = 舵机**位置环下垂**：稳态下真实位置停在"指令 − 下垂"处。
        # 2026-10-01 真机实测 p5/p4/p3 分别是 +16.7/+6.0/+8.4 count（= 4.0/1.5/2.0°）。
        self.droop = dict(droop_deg or {})
        self.dead = dead_deg

    def step(self, j_cmd, dt):
        for k in self.j:
            d = (j_cmd[k] - self.droop.get(k, 0.0)) - self.j[k]
            if abs(d) < self.dead:      # ★ 舵机**死区**：差得不够多就一动不动（真机 5~6 count）
                continue
            m = self.v * dt
            self.j[k] += max(-m, min(m, d))


class FakeLink:
    """实现 Link 协议：假臂 + 用 geom 当"真"相机。"""

    def __init__(self, plant, O_true, hz=10.0, noise_px=0.0, cut_after=None,
                 seed=1, grip_block=None, grip_rate=300.0, grip_lag_s=0.0,
                 fb_none=0, grip_noise=0.0, fb_lag_s=0.0, follow=False, cam_lag_s=0.0):
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
        # ★ 夹爪那一路的**真回读**（2026-10-01 补）：以前 `fb()` 恒报 240 ⇒ 假臂里
        #   "读回"和"指令"永远分不开 ⇒ `close_stuck`（合爪碰上东西就冻结）这条
        #   在假臂上**不可能红**，真机上坏了也测不出来。现在它按 `grip_rate` 追指令，
        #   `grip_block` 是"爪里有东西"时卡住的位置（count）。
        self.grip = 240.0
        self.grip_block = grip_block
        self.grip_rate = grip_rate
        # `grip_lag_s` = 回读**滞后**（真机实测：指令 240→354 走了 0.4s，读回还趴在 234）。
        # 上报的是 `grip` 在 (t − lag) 时刻的值。
        self.grip_lag_s = grip_lag_s
        self.grip_noise = grip_noise
        self.fb_none = fb_none
        self._ghist = [(0.0, 240.0)]
        # ★ 2026-10-01 补的两条"像真机"开关（默认关，老用例行为不变）：
        #   `fb_lag_s`：关节回读**整体滞后**（真机：固件 40ms + 总线轮询 60ms + 发布 40ms，
        #     曾实测到 ~0.4s）。没有它，"拿滞后回读当现在"的一整类 bug 在假臂上测不出来
        #     （下扎提前收工、下扎指令往回拉都是这么漏过去的）。
        #   `follow`：臂按 `Plant.v` 的速度**真的走过去**，而不是 publish 一下就瞬移到指令处。
        self.fb_lag_s = fb_lag_s
        self.follow = follow
        self._fhist = []
        #   `cam_lag_s`：相机这一路的延迟（拍照 → 框到 Pi），**run() 不知道它是多少**
        #     （obs 照样按"现在"打时间戳）⇒ 臂在动时拍的框配错关节姿态。
        self.cam_lag_s = cam_lag_s
        self._jhist = []

    def _fb_grip(self):
        return self._fb_grip_raw() + (self.rng.gauss(0, self.grip_noise)
                                      if self.grip_noise else 0.0)

    def _fb_grip_raw(self):
        if self.grip_lag_s <= 0.0:
            return self.grip
        tt = self.t - self.grip_lag_s
        g = self._ghist[0][1]
        for t, v in self._ghist:
            if t <= tt:
                g = v
            else:
                break
        return g

    def now(self):
        return self.t

    def spin(self, dt):
        self.t += dt

    def fb(self):
        # `fb_none` 拍之内假装还没读到回读（真机：跟踪器给框比串口反馈快）
        if self.t < self.fb_none / self.hz:      # 按**时间**算，不依赖发没发过指令
            return None
        f = to_fields(self.p.j, self._fb_grip(), 496.0)
        if self.fb_lag_s <= 0.0:
            return f
        self._fhist.append((self.t, f))
        tt = self.t - self.fb_lag_s
        out = self._fhist[0][1]
        for t, v in self._fhist:
            if t <= tt + 1e-9:
                out = v
            else:
                break
        return list(out)

    def obs(self):
        if self.cut_after is not None and self.pub_n > self.cut_after:
            return None
        try:
            self._jhist.append((self.t, dict(self.p.j)))
            j_cam = self._jhist[0][1]
            for t, jj in self._jhist:
                if t <= self.t - self.cam_lag_s + 1e-9:
                    j_cam = jj
                else:
                    break
            u, v = geom.project(j_cam, self.O)
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
        # 夹爪那一路：按速率追指令，追不动就卡在 grip_block（= 爪里有东西）
        step = self.grip_rate / self.hz
        d = float(fields[0]) - self.grip
        self.grip += max(-step, min(step, d))
        if self.grip_block is not None:
            self.grip = min(self.grip, float(self.grip_block))
        self._ghist.append((self.t, self.grip))
        if self.follow:
            self.p.step(from_fields(fields), 1.0 / self.hz)   # 按限速真的走过去
            return
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


@pytest.mark.parametrize('phase', ['descend', 'all'])
def test_measured_point_is_not_replaced_by_box_centre(phase):
    cfg = _cfg()._replace(max_seconds=60.0, max_ticks=900)
    plant = Plant(_start_joints(s_m=.05))
    class MeasuredLink(FakeLink):
        def obs(self):
            obs = super().obs()
            if obs is None:
                return None
            u, v = geom.project(self.p.j, CAP)
            x, y = u / 320, v / 180
            box = [max(0., x - .03), max(0., y - .03),
                   min(1., x + .08), min(1., y + .04)]
            return obs._replace(u_ai=u + 12, box_norm=box)
    link = MeasuredLink(plant, CAP, grip_block=400.0)
    rep = grasp.run(cfg, link, phase=phase, initial_point=CAP, log=lambda *_: None)
    assert rep['ok'], rep['stopped']
    assert rep['O_last'] == pytest.approx(CAP, abs=1e-12)
    if phase == 'all':
        assert [p[0] for p in rep['phases']] == ['aim', 'descend', 'close', 'lift']
        assert rep['s_ach_m'] > cfg.lift_m * .8


def test_wrong_box_stops_measured_grasp_before_publishing():
    cfg = _cfg()._replace(max_ticks=30)
    plant = Plant(_start_joints(s_m=.05))
    class WrongBoxLink(FakeLink):
        def obs(self):
            return super().obs()._replace(box_norm=[.01, .01, .04, .04])
    link = WrongBoxLink(plant, CAP)
    rep = grasp.run(cfg, link, initial_point=CAP, log=lambda *_: None)
    assert not rep['ok']
    assert '核验失败' in rep['stopped']
    assert not link.pubs


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
    # ⚠️ `obs_lost_s=1.0` 是**显式钉死**的：这条测的就是"观测丢失"分支，必须自己决定容忍度。
    #    默认值 2026-10-01 从 1.0 改成 5.0（相机移动时跟踪器会跟丢低纹理目标），
    #    沿用默认会让 aim 先把循环"正常完成"掉、这条分支根本走不到 ⇒ 假绿。
    cfg = _cfg()._replace(max_seconds=10.0, max_ticks=100, max_step_deg=0.3,
                          obs_lost_s=1.0)
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


def test_run_all_phases_closes_and_lifts():
    """★ `'all'` = 四相一次走完（接近 → 下扎 → 合爪 → 抬升），**估计器/时间缓冲不重建**。

    判据全是**物理量**，不看循环自己的说法：
      ① `rep['phases']` 恰好四相且顺序对；
      ② 最后发出去的 p1 ≥ close_field−20 ⇒ **夹爪真的合上了**（且没有过冲）；
      ③ 最终沿轴留量 ≥ lift_m×0.8、且远大于 s_stop ⇒ **真的抬起来了**。

    变异（实测过）：把 `advance_or_stop` 里的 `cur = seq[k]` 改成 `cur = seq[0]` ⇒
    相序变成 ['aim','aim',…] ⇒ 第 ① 条红。
    """
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900)
    plant = Plant(_start_joints(s_m=0.10))
    link = FakeLink(plant, CAP)
    rep = grasp.run(cfg, link, phase='all', log=lambda *a: None)
    assert [p[0] for p in rep['phases']] == ['aim', 'descend', 'close', 'lift'], rep['phases']
    assert rep['ok'], rep['stopped']
    assert cfg.close_field - 20.0 <= link.pub[0] <= cfg.close_field + 20.0, link.pub[0]
    assert rep['s_ach_m'] >= cfg.lift_m * 0.8, rep['s_ach_m']
    assert rep['s_ach_m'] > cfg.s_stop_m * 5.0, rep['s_ach_m']


def _close_reason(rep):
    r = [p[1] for p in rep['phases'] if p[0] == 'close']
    assert r, rep['phases']
    return r[0]


def test_close_freezes_on_contact_not_reported_as_empty():
    """★ 爪里有东西（读回卡在 400）⇒ 必须走"碰上东西、冻结"这条，**不许报"夹空"**。

    这条盯的是 2026-10-01 真抓暴露的那个 bug：`f_now[0]` 其实是**指令**
    （`to_fields(j, grip_cmd, …)` 把 grip_cmd 写进第 0 位），于是
      ① 收尾拿指令当"读回" ⇒ 明明握着东西却报"夹空了"；
      ② `close_stuck` 拿指令跟自己比（每拍恒差 close_step=30）⇒ 永远数不到 close_stall
         ⇒ **柔性冻结从不触发**、每次都一路合到底（用户明确否决过的"全力抓"）。
    变异（已实测）：把两处 `p1_read` 换回 `f_now[0]` ⇒ 这条立刻红。
    """
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900)
    plant = Plant(_start_joints(s_m=0.10))
    link = FakeLink(plant, CAP, grip_block=400.0)
    rep = grasp.run(cfg, link, phase='all', log=lambda *a: None)
    why = _close_reason(rep)
    assert '碰上东西' in why, why
    assert '夹空' not in why, why
    # 物理量：真回读**确实**卡在被挡的位置，没跟到 578
    assert 395.0 <= link.grip <= 405.0, link.grip
    # 而且必须是在**还没合到底**的时候就冻结的（否则"冻结"没有意义）。
    # ⚠️ 不能用 `link.pub`：那是**全程最后一帧**（lift 相本来就要求 p1 = close_field）。
    #    冻结点上的指令值只能从理由字符串里取。
    import re
    cmd_at_freeze = float(re.search(r'p1 令 (-?[\d.]+)', why).group(1))
    assert cmd_at_freeze < cfg.close_field - 20.0, why


def test_close_full_travel_admits_it_cannot_tell_empty_from_thin():
    """读回一路贴到指令（爪里可能真没东西）⇒ 只许报"分不出来"，**不许下"夹空了"的结论**。

    理由：2026-10-01 实测白条**被夹住了**、读回照样走到 578 ⇒ "读回到底"推不出"夹空"。
    """
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900)
    plant = Plant(_start_joints(s_m=0.10))
    link = FakeLink(plant, CAP)                 # grip_block=None ⇒ 爪子能一路合到底
    rep = grasp.run(cfg, link, phase='all', log=lambda *a: None)
    why = _close_reason(rep)
    assert '夹空' not in why, why
    assert '读回也到底了' in why, why
    assert abs(link.grip - cfg.close_field) < 10.0, link.grip


def test_close_does_not_freeze_before_the_readback_has_moved():
    """夹爪回读**滞后**（真机实测 0.4s 还没起振）⇒ **不许**一进 close 相就判"碰上东西"。

    变异（已实测）：把 `close_armed` 那道门去掉 ⇒ 这条立刻红。
    """
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900)
    plant = Plant(_start_joints(s_m=0.10))
    link = FakeLink(plant, CAP, grip_lag_s=0.5)      # 回读滞后 5 拍
    rep = grasp.run(cfg, link, phase='all', log=lambda *a: None)
    why = _close_reason(rep)
    assert '碰上东西' not in why, why
    assert '读回也到底了' in why, why


DROOP = {'shoulder': 2.0, 'elbow': 1.5, 'wrist_pitch': 1.5}   # 度（真机实测 4.0/1.5/2.0）


def _aim_final_err(comp):
    cfg = _cfg()._replace(hz=10.0, max_seconds=40.0, max_ticks=400, joint_comp=comp)
    plant = Plant(_start_joints(s_m=0.10), droop_deg=DROOP)
    link = FakeLink(plant, CAP)
    rep = grasp.run(cfg, link, phase='aim', log=lambda *a: None)
    return rep['err_m'], rep


def _descend_err(rep):
    """[descend] 收尾时"爪尖离目标"的 mm 数（从理由字符串里取，不另加字段）。"""
    import re
    for ph, why, _n in rep['phases']:
        if ph == 'descend':
            return float(re.search(r'爪尖离目标 ([\d.]+)mm', why).group(1)) / 1000.0
    raise AssertionError(rep['phases'])


def _run_all(comp, droop=None):
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900, joint_comp=comp)
    plant = Plant(_start_joints(s_m=0.10), droop_deg=(DROOP if droop is None else droop))
    link = FakeLink(plant, CAP)
    return grasp.run(cfg, link, phase='all', log=lambda *a: None)


def test_joint_comp_reduces_the_tip_error_under_droop():
    """★ 物理结局：有位置环下垂时，补偿必须让**下扎结束时爪尖离目标**明显变小。

    判据取的是 `[descend]` 收尾那行理由里的实际量（回读算出来的），不是"循环说自己补过了"。
    变异（已实测）：`joint_comp=0` ⇒ err_on == err_off ⇒ 这条立刻红。
    """
    e_off = _descend_err(_run_all(0.0))
    e_on = _descend_err(_run_all(1.0))
    assert e_off > 0.010, e_off                   # 前提：下垂确实造成可见误差
    assert e_on < e_off * 0.5, (e_on, e_off)


def test_joint_comp_measures_the_true_droop_at_the_handover():
    """★ 交接处量到的 (纯目标 − 回读) 必须≈真实下垂（度 → count：×500/120）。

    下垂是**稳态量**：交接那一刻臂刚报完到位、目标也不再动 ⇒ 量到的就是纯下垂，
    不是滞后。变异（实测）：把 `f_tgt - f_now` 换成 `fields - f_now`（即算上已加的 comp）
    ⇒ 恒等于下垂本身 ⇒ 每拍再加一次，数值无界 ⇒ 这条红。
    """
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900, joint_comp=1.0)
    plant = Plant(_start_joints(s_m=0.10), droop_deg=DROOP)
    link = FakeLink(plant, CAP)
    rep = grasp.run(cfg, link, phase='all', log=lambda *a: None)
    got = rep['comp']
    assert got is not None, rep['stopped']
    # ⚠️ p5 是**反号映射**（`to_fields`: p5 = 500 + F·(90 − shoulder)）⇒ 肩下垂在 field 上是**负**的。
    #    真机实测 `令 796.3 / 读 813.0` = −16.7 count，正是这个负号（= 肩下垂 4.0°）——对得上。
    want = [DROOP['wrist_pitch'] * 500 / 120, DROOP['elbow'] * 500 / 120,
            -DROOP['shoulder'] * 500 / 120]
    for g, w in zip(got, want):
        assert abs(g - w) < 3.0, (got, want)


# 2026-10-01 第五跑的真值：白盒 r=182mm / z=−106mm。
# aim 按"预抓点余量最大"挑走 α=−85 ⇒ 接触点余量只剩 49 ⇒ 真机上肩顶到 field 864
# （离固件上限 875 只差 11）**舵机撑不住**，下扎全程跟不上、40s 超时。
O_BOX = (0.1818, -0.0131, -0.1060)


def test_pick_target_balance_keeps_both_ends_feasible():
    """★ 下扎轴 α 必须**两头一起挑**：预抓点余量最大的那个 α，往往正是接触点余量最小的。

    变异（实测）：`balance=False` ⇒ 末态余量 49 < 50 ⇒ 这条红。
    """
    cfg = _cfg()._replace(alpha0=-84.0)

    def end_slack(a):
        return grasp.pick_target(O_BOX, cfg, cfg.s_stop_m, a, span=0.0).slack

    t_naive = grasp.pick_target(O_BOX, cfg, cfg.s_pre_m, cfg.alpha0)
    t_bal = grasp.pick_target(O_BOX, cfg, cfg.s_pre_m, cfg.alpha0, balance=True)
    e_naive, e_bal = end_slack(t_naive.alpha), end_slack(t_bal.alpha)
    assert t_naive.slack > 100.0, t_naive.slack          # 前提：现规则确实在预抓点余量很大
    assert e_naive < 50.0, (t_naive.alpha, e_naive)      # 可是它把接触点余量压到 50 以下
    assert e_bal >= 55.0, (t_bal.alpha, e_bal)           # 平衡后接触点余量够
    assert min(t_bal.slack, e_bal) > min(t_naive.slack, e_naive), (t_bal.slack, e_bal)


def test_descend_accepts_the_servo_deadband_instead_of_stalling():
    """★ 下扎到位必须认**舵机死区**这条硬件极限，不能只认"沿轴留量 ≤ s_stop"。

    真机第六跑：`s_ach` 停在 **5.7mm**（判据 3.5mm），舵机死区让它再也动不了 ⇒
    25 拍后判"卡住"、整轮白跑。`near`（回读落进死区量级连 3 拍）那条接近相一直有、下扎相漏了。
    变异（已实测）：把 `or near >= 3` 去掉 ⇒ 这条红。
    """
    # 用"肩沉降 −1.8°"（= 7.5 count ≤ deadband_counts 10）造一个**停在离目标几毫米**的稳态：
    # 这正是真机第六跑的现场（`s_ach` 停在 5.7mm 再也下不去，用户肉眼确认"几乎贴着"）。
    # 关掉 joint_comp，免得它把这个残余补掉、测试就失去意义。
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=1200, joint_comp=0.0)
    plant = Plant(_start_joints(s_m=0.10), droop_deg={'shoulder': -1.8})
    link = FakeLink(plant, CAP)
    rep = grasp.run(cfg, link, phase='descend', log=lambda *a: None)
    assert rep['phases'] and rep['phases'][0][0] == 'descend', rep['phases']
    why = rep['phases'][0][1]
    assert why.startswith('扎到位'), rep['stopped']
    assert '死区极限' in why, why                          # 必须走的是第二条判据


def test_run_survives_observations_arriving_before_any_feedback():
    """★ 观测（K230 框）比串口回读**先到**时，`run()` 不许崩。

    真机 2026-10-01 踩到：`trace.at()` 撞上空缓冲抛 `ValueError('Trace 是空的')`，
    而 `run()` 只接 `Refused` ⇒ 整轮直接崩（"❌ 出错了：Trace 是空的"）。
    变异（实测）：把 `if not trace.buf: continue` 去掉 ⇒ 这条抛 ValueError、红。
    """
    cfg = _cfg()._replace(hz=10.0, max_seconds=20.0, max_ticks=300)
    plant = Plant(_start_joints(s_m=0.10))
    link = FakeLink(plant, CAP, fb_none=4)      # 前 4 拍没有回读，但观测照来
    rep = grasp.run(cfg, link, phase='aim', log=lambda *a: None)
    # 变异（已实测）：去掉那道闸 ⇒ `ltrace.at` 抛 ValueError、**测试直接红**（异常本身就是判据）。
    assert rep['stopped'], rep['stopped']        # 跑完了，没崩
    assert rep['ticks'] >= 1, rep['ticks']       # 回读一到就正常发指令


def test_apply_bias_radial_moves_along_the_radius_not_a_fixed_vector():
    """★ 径向偏置：目标挪的方向必须**跟着物体方位角转**（固定向量做不到）。

    实测"爪尖几乎每次都偏后 ~1cm"是径向的（换方位角还是偏后）⇒ 必须按半径给。
    变异（实测）：把 `bias_r_m * ux` 的符号反过来 ⇒ 这一条红。
    """
    for (x, y) in ((0.20, -0.02), (-0.15, 0.10), (0.0, 0.18)):
        r = (x * x + y * y) ** 0.5
        out = grasp.apply_bias((x, y, -0.1), bias_r_m=0.01)      # 向外 1cm
        d = ((out[0] - x) ** 2 + (out[1] - y) ** 2) ** 0.5
        assert abs(d - 0.01) < 1e-9, (x, y, d)
        assert abs(out[0] - (x + 0.01 * x / r)) < 1e-9, (x, y, out)
        assert out[2] == -0.1                                   # z 不该被动
        inward = grasp.apply_bias((x, y, -0.1), bias_r_m=-0.01)
        din = ((inward[0] - x) ** 2 + (inward[1] - y) ** 2) ** 0.5
        assert abs(din - 0.01) < 1e-9
        # 向内必须是**反方向**
        assert (inward[0] - x) * (out[0] - x) <= 0.0
    # 零偏置 = 原样
    assert grasp.apply_bias((0.1, 0.2, 0.3)) == (0.1, 0.2, 0.3)
    assert grasp.apply_bias((0.1, 0.2, 0.3), (0.01, 0.0, 0.0)) == (0.11, 0.2, 0.3)


def test_close_arming_gate_survives_a_noisy_gripper_readback():
    """★ 起振门必须扛得住**回读噪声**。

    真机 2026-10-01 第六跑：`close` 报"p1 令 510 / 读 240 ⇒ 碰上东西"，可爪子根本没动
    —— 读回那点噪声蹭过了 10 count 的门，起振门形同虚设。门槛该按"**真的跟了一步**"
    （close_step=30 count）算，不是按死区宽度（10）。
    变异（已实测）：把门槛退回 `deadband_counts` ⇒ 这条红。
    """
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900)
    plant = Plant(_start_joints(s_m=0.10))
    link = FakeLink(plant, CAP, grip_lag_s=0.6, grip_noise=8.0, seed=7)
    rep = grasp.run(cfg, link, phase='all', log=lambda *a: None)
    why = _close_reason(rep)
    assert '碰上东西' not in why, why


# --------------------------------------------------------------------------
# 回读滞后下的下扎（2026-10-01 复查补的；假臂开 `fb_lag_s` + `follow`）
# --------------------------------------------------------------------------

def _laggy_descend(lag, **kw):
    """回读滞后 `lag` 秒、臂按限速真走的下扎。`z_plane_m` 钉成 CAP 的 z ⇒ 估计无系统偏差，
    测出来的残差只来自控制律本身。"""
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900, patience=25,
                          z_plane_m=CAP[2], **kw)
    plant = Plant(_start_joints(s_m=0.08))
    link = FakeLink(plant, CAP, fb_lag_s=lag, follow=True)
    rep = grasp.run(cfg, link, phase='descend', log=lambda *a: None)
    s_true = grasp.standoff_along_axis(grasp.tip_open_m(plant.j), CAP, rep['alpha'])
    return rep, link, s_true


@pytest.mark.parametrize('lag,s_lead', [(0.2, None), (0.4, None), (0.2, 0.002), (0.4, 0.002)])
def test_descend_reaches_s_stop_despite_feedback_lag(lag, s_lead):
    """★ 回读滞后时，下扎**不许在半路就宣布到位**。

    根因：`near`（回读离目标 ≤ 死区、连 3 拍）在下扎相比的是**每拍那个中间目标**
    （只比当前位置往前挪 2~5mm），不是终点 ⇒ 离目标 1~2cm 时每拍步长本身就 < 10 count，
    臂只要平稳地跟，`near` 就成立 ⇒ 提前收工。真机下扎收尾残差 9.5~16.6mm 随机，就是它。
    修法：下扎相只有**指令已经到了 s_stop** 之后，`near` 才开始数。
    变异（已实测）：
      * 旧代码（s_cmd 跟滞后回读走）：默认参数这两条就红（真实停在 7.7/8.3mm）。
      * 现代码去掉 `floor_ok`：默认参数**照绿**（指令领先回读、中间目标总比回读远 ⇒ `near`
        碰巧不成立），所以另加 `s_lead_m=2mm` 两条（指令几乎贴着回读走 = 旧行为）⇒
        红（停在 6.2/9.2mm）。`floor_ok` 不能靠"碰巧"，任何 s_lead_m 下都得对。
    """
    rep, _link, s_true = _laggy_descend(lag, **({} if s_lead is None else {'s_lead_m': s_lead}))
    assert rep['ok'], rep['stopped']
    assert s_true <= 0.003 + 0.0015, (lag, s_true, rep['stopped'])


def test_descend_command_does_not_back_up_under_feedback_lag():
    """★ 下扎**指令**不许被滞后回读往回拽：沿轴留量的**累计上行量** ≤ 3mm。

    根因：旧写法 `s_cmd = s_ach(滞后回读) − step`、限速也从滞后回读起步 ⇒ 回读落后时
    指令被算到臂**已经过去**的位置之上 ⇒ 臂被拽回去 ⇒ 带延迟的反馈环，来回顶、又慢。
    实测 0.4s 滞后：旧 6.7mm / 现 2.0mm（剩下的是 α 在 ±alpha_freeze_span 内微漂、
    量度用的是终态 α 造成的，不是往回拉）。单拍最大回退量两边都 ~1.3mm、区分不开，所以判累计量。
    """
    rep, link, _s = _laggy_descend(0.4)
    assert rep['ok'], rep['stopped']
    s_cmds = [grasp.standoff_along_axis(grasp.tip_open_m(from_fields(f)), CAP, rep['alpha'])
              for f in link.pubs]
    up = sum(max(0.0, b - a) for a, b in zip(s_cmds, s_cmds[1:]))
    assert up <= 0.003, up


def test_lift_keeps_the_frozen_grip_not_full_force():
    """★ 合爪"碰上东西就冻结"之后，**抬起时不许把夹爪拉到 close_field（全力夹）**。

    根因：`advance_or_stop` 把 `close_cmd` 清成 None，抬起相 `grip_cmd` 就落到 close_field=578
    ⇒ 用户明确否决过的全力合爪，在抬起那一刻照样发生。
    变异（已实测）：抬起相改回 `grip_cmd = cfg.close_field` ⇒ 这条红。
    """
    import re
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900)
    plant = Plant(_start_joints(s_m=0.10))
    link = FakeLink(plant, CAP, grip_block=400.0)
    rep = grasp.run(cfg, link, phase='all', log=lambda *a: None)
    assert [p[0] for p in rep['phases']] == ['aim', 'descend', 'close', 'lift'], rep['phases']
    why = _close_reason(rep)
    cmd_at_freeze = float(re.search(r'p1 令 (-?[\d.]+)', why).group(1))
    n_lift = rep['phases'][3][2]
    lift_p1 = [f[0] for f in link.pubs[-n_lift:]]
    assert lift_p1 and all(abs(v - cmd_at_freeze) < 1e-6 for v in lift_p1), (cmd_at_freeze, lift_p1)


def test_estimate_survives_unmodelled_camera_latency():
    """相机延迟 0.5s（run() 不知道）+ 回读滞后 0.2s：**下扎收尾时在用的 O** 离真值 ≤ 3mm。

    2026-10-01 复查时想过"只用臂静止时的观测 + aim 到位后原地等观测再下扎"，先拿这个
    场景量了一下：现状就只差 ≤2mm（下扎是沿着视线往目标走，目标像素几乎不动；
    接近相的偏差在臂停下后被 EWMA 洗掉）⇒ 不值得为它加一套门控和每次 ~1s 的等待。
    这条把"现状够用"钉住：将来谁改估计器/时间对齐把它弄坏了，这里会红。
    """
    import math as _m
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900, z_plane_m=CAP[2])
    plant = Plant(_start_joints(s_m=0.12))
    link = FakeLink(plant, CAP, fb_lag_s=0.2, follow=True, cam_lag_s=0.5)
    rep = grasp.run(cfg, link, phase='all', log=lambda *a: None)
    assert rep['ok'], rep['stopped']
    O = dict(rep['O_phase_end'])['descend']
    assert _m.dist(O, CAP) <= 0.003, O
