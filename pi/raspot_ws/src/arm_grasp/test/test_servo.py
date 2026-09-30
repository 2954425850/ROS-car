# -*- coding: utf-8 -*-
"""servo（视觉伺服控制律）的测试。

## 这套测试里哪条才算证据
* `test_*_pure` 那一批是**开环**：左右两边各自独立算，能红。
* 闭环那几条走的是**真的那份循环代码**（`servo.run_phase`），被控对象是本文件里的
  `Plant`（把「关节角 → 真实相机位姿 → 瓶盖圆盘投影 → 裁到画框 → 包围盒+质心」
  整条链**重新实现**一遍，不复用 geom 的 `project`）。
  ⚠️ 闭环比"像素误差收敛"是**自证**（那就是循环的停止条件）。
  **真判据是物理量**：`Plant` 把**真实瓶盖中心**投出来（不受画框限制），
  量它离瞄准像素多远 —— 这条与循环自己的说法无关。
* 最后两条是**负对照**：故意把观测量搞坏 / 让 alpha 不许动，
  断言循环**必须不收敛**。没有它们，"收敛了"这句话没有对照。

## Plant 的约定（与 servo 必须一致）
物理爪尖 = **张开态**（夹爪 240），相机偏移 = `CAM_OPEN`，两者由同一条刚体链给出：
`camera = open_tip + CAM_OPEN` 必须恒等于 `geom.camera_center`。
`test_plant_agrees_with_the_model` 就是钉这个的。
"""
import math

import pytest

from arm_grasp import geom, servo
from arm_grasp.arm_kin import fk, from_fields, ikine, to_fields
from arm_grasp.cam_model import tool_axes
from arm_grasp.servo import (ServoConfig, ServoRefused, cap_center, ik_open_m,
                            limit_step, pick_pose, tip_open_m)

K = geom.K_AI320x180_CHN2
FB0 = [578.0, 496.0, 126.0, 213.0, 483.0, -277.0]
CAP_TRUE = (-0.0754, -0.2482, 0.016)     # 米：2026-09-29 17:07 实拍那一拍推出来的瓶盖点
POSE0 = from_fields(FB0)


# ---------------------------------------------------------------------------
# 模拟被控对象
# ---------------------------------------------------------------------------

class Plant:
    """一台"真"臂 + 一个"真"相机 + 一个不动的瓶盖。

    * `tip_err_m`：臂的**零位误差**（模型以为爪尖在 A，真机在 A + err）。
    * `off_true`：**真实**的相机相对张开态爪尖的偏移；模型永远用 `geom.CAM_OPEN`。
    * `sigma`：真实图像的滚转（−1 = 相机倒装）。
    * `r_cap_m`：瓶盖半径（画出来的是个圆盘）。
    """

    def __init__(self, cap=CAP_TRUE, r_cap_m=0.0184, off_true=geom.CAM_OPEN,
                 tip_err_m=(0.0, 0.0, 0.0), sigma=geom.SIGMA,
                 w=geom.STREAM_W, h=geom.STREAM_H):
        # r_cap_m = 1.84cm：**实拍量出来的**（2026-09-29 干跑那一帧，包围盒宽 168px @ 24.3cm）
        self.cap, self.r_cap, self.off = cap, r_cap_m, off_true
        self.tip_err, self.sigma = tip_err_m, sigma
        self.w, self.h = w, h

    def tip(self, joints):
        t = tip_open_m(joints)
        return tuple(t[i] + self.tip_err[i] for i in range(3))

    def camera(self, joints):
        tip = self.tip(joints)
        tip_cm, axis = fk(joints)
        xh, yh, zh = tool_axes(tip_cm, axis)
        return tuple(tip[i] + self.off[0] * xh[i] + self.off[1] * yh[i]
                     + self.off[2] * zh[i] for i in range(3))

    def project(self, joints, P, k=K):
        """基座系点 → **AI 系**像素（含畸变、含 sigma）。"""
        C = self.camera(joints)
        tip_cm, axis = fk(joints)
        xh, yh, zh = tool_axes(tip_cm, axis)
        d = [P[i] - C[i] for i in range(3)]
        Z = sum(d[i] * zh[i] for i in range(3))
        X = sum(d[i] * xh[i] for i in range(3))
        Y = sum(d[i] * yh[i] for i in range(3))
        return geom.distort(k.cx + self.sigma * k.fx * X / Z,
                            k.cy + self.sigma * k.fy * Y / Z, k)

    def blob(self, joints, cap=None, k=K):
        """瓶盖圆盘 → 蓝块那种包围盒/质心（**1280x720 系**）。

        ★ 采样方式刻意做成"像真掩膜"：
        * **包围盒**用**边界一圈**投出来（投影的极值一定落在圆盘边界上）；
          哪条边出了画框就贴到画框上 —— 与逐像素掩膜 `blue_blob_px` 的行为一致。
          （用稀疏内部网格会差十几像素，裁剪判据就永远不触发，测试会假绿。）
        * **质心**用**可见部分**的内部网格取均值 —— 这正是"质心会骗人"的来源。
        """
        cap = self.cap if cap is None else cap
        nb = 720
        bu, bv = [], []
        for i in range(nb):
            th = 2.0 * math.pi * i / nb
            P = (cap[0] + self.r_cap * math.cos(th),
                 cap[1] + self.r_cap * math.sin(th), cap[2])
            u, v = self.project(joints, P, k)
            bu.append(u)
            bv.append(v)
        n = 41
        us, vs = [], []
        for i in range(n):
            for j in range(n):
                x = -1.0 + 2.0 * (i + 0.5) / n
                y = -1.0 + 2.0 * (j + 0.5) / n
                if x * x + y * y > 1.0:
                    continue
                P = (cap[0] + self.r_cap * x, cap[1] + self.r_cap * y, cap[2])
                u, v = self.project(joints, P, k)
                if 0.0 <= u <= k.w - 0.25 and 0.0 <= v <= k.h - 0.25:
                    us.append(u)
                    vs.append(v)
        if len(us) < 15:                       # 只剩一丝，量不准了 -> 就当没有
            return None
        f = self.w / float(k.w)
        # ⚠️ 贴边的那条边要夹在 **w-0.25**（即 1280x720 里的 719），
        #    不是 k.w-1.0（那是 716）—— 否则"已经贴到画框底"的块会被判成没裁剪，
        #    `cap_center` 就退回带偏的质心（实测那道偏差 26px ≈ 5mm）。
        return {'u0': max(0.0, min(bu)) * f, 'u1': min(k.w - 0.25, max(bu)) * f,
                'v0': max(0.0, min(bv)) * f, 'v1': min(k.h - 0.25, max(bv)) * f,
                'cu': sum(us) / len(us) * f, 'cv': sum(vs) / len(vs) * f,
                'n': len(us) * 16}


def make_io(plant, fb0=None):
    """造一对 observe()/command()。臂精确走到指令的关节角（完美臂）。"""
    st = {'j': from_fields(FB0 if fb0 is None else fb0)}

    def observe():
        return to_fields(st['j'], 240.0, 496.0), plant.blob(st['j'])

    def command(fields):
        st['j'] = from_fields(fields)
        return observe()

    return observe, command, st


def cfg(**kw):
    d = dict(cap_z_m=CAP_TRUE[2], max_step_deg=4.0, alpha0=-58.0, tol_px=10.0,
             max_iters=30)
    d.update(kw)
    return ServoConfig(**d)


AIM_PX = geom.to_stream(*geom.aim_pixel(closed=False))


def _run(plant, phase='aim', **kw):
    obs, cmd, st = make_io(plant)
    rep = servo.run_phase(cfg(**kw), obs, cmd, phase=phase, log=lambda *a: None)
    return rep, st


def _ground_truth_err_px(plant, joints):
    """**真**对准误差：真实瓶盖中心投到画面上，离瞄准像素多远（1280x720 系）。

    不受画框裁剪、不受特征估计器影响 —— 这条才是物理判据。
    """
    u, v = plant.project(joints, plant.cap)
    su, sv = u * 4.0, v * 4.0
    return math.hypot(su - AIM_PX[0], sv - AIM_PX[1])


def _lateral_mm(plant, joints):
    """真实爪尖到「光心→瓶盖」这条**直线**的垂距（mm）—— 对准的物理定义。"""
    C = plant.camera(joints)
    tip = plant.tip(joints)
    d = [plant.cap[i] - C[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in d))
    d = [v / n for v in d]
    w = [tip[i] - C[i] for i in range(3)]
    proj = sum(w[i] * d[i] for i in range(3))
    return math.sqrt(sum((w[i] - proj * d[i]) ** 2 for i in range(3))) * 1000.0


# ---------------------------------------------------------------------------
# 开环：约定与纯函数
# ---------------------------------------------------------------------------

def test_plant_agrees_with_the_model():
    """★ 开环：模拟被控对象和模型必须给出**同一个**相机位姿与投影。

    钉的是「物理爪尖 = 张开态、相机偏移 = CAM_OPEN」这一对约定：
    写成 fk 的夹紧态 + CAM_CLOSED 会差 1.7cm，这条会红。
    （下面这几个 fb 都是**实拍里真的看见瓶盖**的那几拍。）
    """
    p = Plant()
    for fb in (FB0, [578.0, 496.0, 129.0, 242.0, 492.0, -257.0],
               [578.0, 496.0, 150.0, 234.0, 580.0, -224.0]):
        j = from_fields(fb)
        assert math.dist(p.camera(j), geom.camera_center(j)) < 1e-12
        u1, v1 = p.project(j, CAP_TRUE)
        u2, v2 = geom.project(j, CAP_TRUE)
        assert abs(u1 - u2) < 1e-9 and abs(v1 - v2) < 1e-9
    # 反过来：把物理爪尖当成夹紧态就会差 1.7cm（这条是上面那个约定的对照）
    j = from_fields(FB0)
    closed_tip = tuple(v / 100.0 for v in fk(j)[0])
    assert math.dist(closed_tip, p.tip(j)) > 0.015


def test_the_starting_pose_reproduces_the_real_photo():
    """起手那一拍：模拟画出来的瓶盖必须落在**实拍**那个像素上（717.8, 213.6）。"""
    p = Plant()
    b = p.blob(POSE0)
    assert b is not None
    assert abs(b['cu'] - 717.79) < 1.5 and abs(b['cv'] - 213.58) < 1.5, b


def test_ik_open_m_puts_the_open_tip_exactly_on_the_target():
    """`ik_open_m` 解出来的姿态，其**张开态**爪尖必须落在给定点上（米）。

    ⚠️ (目标, alpha) 必须是**真的可行**的组合（下面这几组是扫出来的）——
    随手写一个够不着的点，这条测的就不是换算而是 IK 的报错。
    """
    cases = (((-0.055, -0.195, 0.100), -50.0),
             ((-0.055, -0.195, 0.100), -58.0),
             ((0.020, -0.220, 0.020), -60.0),
             ((0.020, -0.220, 0.020), -70.0),
             ((-0.100, -0.120, 0.050), -78.0),
             ((-0.070, -0.240, 0.030), -50.0))
    for tgt, a in cases:
        got = tip_open_m(ik_open_m(*tgt, a))
        assert math.dist(got, tgt) < 1e-8, (tgt, a, got)


def test_naive_ikine_would_miss_by_1_7cm():
    """负对照：不做那 1.7cm 换算（直接用 ikine，cm）会差 1.7cm。

    没有这条，上面那条测不出"换算到底有没有生效"。
    """
    for tgt_cm, a in (((-5.0, -19.0, 9.0), -58.0), ((2.0, -22.0, 2.0), -70.0)):
        got_cm = tuple(v * 100.0 for v in tip_open_m(ikine(*tgt_cm, a)))
        assert abs(math.dist(got_cm, tgt_cm) - 1.7) < 1e-6, tgt_cm
        assert got_cm[2] > tgt_cm[2]      # 物理爪尖更靠近腕（沿 ẑ 往回，z 更高）


def test_limit_step_scales_all_joints_by_the_same_ratio():
    cur = dict(zip(servo.JOINT_KEYS, (0, 90, -60, -80)))
    des = dict(zip(servo.JOINT_KEYS, (10, 130, -20, -60)))
    cmd, r, reached = limit_step(cur, des, 6.0)
    assert not reached and abs(r - 0.15) < 1e-12
    for k in cur:
        if abs(des[k] - cur[k]) > 1e-12:
            assert abs((cmd[k] - cur[k]) / (des[k] - cur[k]) - r) < 1e-12


def test_limit_step_passes_through_when_close_enough():
    cur = dict(zip(servo.JOINT_KEYS, (0, 90, -60, -80)))
    des = dict(zip(servo.JOINT_KEYS, (1, 92, -59, -81)))
    cmd, r, reached = limit_step(cur, des, 6.0)
    assert reached and r == 1.0 and cmd == des


def test_pick_pose_refuses_when_nothing_works():
    with pytest.raises(ServoRefused):
        pick_pose((0.60, 0.0, 0.60), (0.0, 0.0, -1.0), AIM_PX, 0.02, 0.05,
                  -58.0, span=4.0)


def test_pick_pose_prefers_a_slack_rich_reconfiguration():
    """★ 贴着实测那次失败的现场写的一条（2026-09-29 第一次上机）。

    那次循环锁在 alpha=−38°、p4=131（固件下限 125），肘关节一路贴死在 −90°、
    指令到 −88° 也走不到，误差停在 116px。同一状态的真实数据（从保存的帧重算）：

        钉死留量、只搜 alpha 时：argmin 在 −38°，余量只剩 6 count
        放开留量之后：一整族姿态都能对准，余量能到 100+

    这条断言"返回的姿态必须**离固件限位足够远**"——贴边的姿态执行不了，
    连模型自己算出来的那点优势都拿不到。
    """
    P = (-0.0771, -0.2737, 0.0160)          # 那次循环当时估出来的瓶盖点（米）
    d = (P[0] - (-0.028), P[1] - (-0.098), P[2] - 0.172)
    n = math.sqrt(sum(v * v for v in d))
    d = tuple(v / n for v in d)             # 那次的光心→瓶盖方向（米，单位向量）
    pose = pick_pose(P, d, AIM_PX, 0.040, 0.101, -38.0, lo=-88.0, hi=-30.0,
                     span=14.0, n_standoff=5, tie_px=5.0, min_slack=40.0)
    ok, f = servo.fields_in_range(pose.joints, 240.0, 496.0)
    assert ok
    assert pose.slack >= 40.0, pose.slack
    # 对照：钉死留量在"射线上离光心 L 处"、只搜 alpha，拿到的是贴边的那个
    T = servo.standoff_target(P, d, 0.101)
    raw = []
    for a in range(-56, -29):
        j = servo.ik_open_m(*T, a)
        ok, f = servo.fields_in_range(j, 240.0, 496.0)
        if ok:
            raw.append((servo.predicted_err_px(j, P, AIM_PX), a,
                        servo.field_slack(f)))
    best_raw = min(raw)
    assert best_raw[2] < 40.0, best_raw        # 纯 argmin 就是贴边的
    assert pose.err_px <= best_raw[0] + 5.0, (pose.err_px, best_raw)


def test_cap_center_reports_the_top_face_centre_not_the_blob_centroid():
    """★ 蓝块 = 顶面圆盘 **+ 柱侧面的可见部分** ⇒ 质心被往下拉 ~15~25px。

    实拍量到 168x210px：宽 = 圆盘直径，高比圆盘多出约 59px 的侧面。
    `v0`（上沿）和宽度都只由**顶面圆盘**决定 ⇒ 按上沿估出来的才是顶面圆心，
    也正是爪尖该去的那个点。
    """
    u, vc, r, k, side = 530.0, 500.0, 84.0, 0.9, 40.0
    v0 = vc - r * k                          # 424.4：圆盘上沿
    v1 = vc + r * k + side                   # 615.6：柱底
    centroid_v = vc + 15.0                   # 侧面把质心往下拽
    c, how = cap_center(u - r, u + r, v0, v1, u, centroid_v, -k)
    assert how == 'box'
    assert abs(c[0] - u) < 1e-9 and abs(c[1] - vc) < 1e-9, c
    assert abs(c[1] - centroid_v) > 10.0


def test_cap_center_recovers_a_bottom_clipped_disc():
    """★ 瓶盖对到瞄准像素那一刻**底边一定出画**（中心 709.8 + 半径 139 > 720）。

    几何要**自洽**：水平半径 139px、|d_z|=0.9 ⇒ 竖直半轴 125px、上沿 584.7、
    理论下沿 834.9（画框只到 719）。质心只按可见部分算 ⇒ 偏 ~50px。
    """
    u, vc, r_h, k = 530.0, 709.8, 139.0, 0.9
    v0 = vc - r_h * k                        # 584.7
    v1 = 719.0                               # 被裁在画框上
    frac = math.sqrt(max(0.0, 1.0 - ((v1 - vc) / (r_h * k)) ** 2))
    u0, u1 = u - r_h * frac, u + r_h * frac
    biased = 0.5 * (v0 + v1)                 # 可见部分的质心（会骗人）
    assert biased < vc - 40.0, biased
    c, how = cap_center(u0, u1, v0, v1, u, biased, -k)
    assert how == 'box'
    assert abs(c[0] - u) < 1e-9 and abs(c[1] - vc) < 1.5, c


def test_cap_center_falls_back_to_the_centroid_when_the_box_formula_runs_off_frame():
    """退路：条目齐、但式子算出来的圆心跑出画框（把几何那层单独拎出来测）。

    ⚠️ 真几何下这条几乎不可达（圆盘形状下 v 一定 ≤ v1）—— 它是防御代码，
    所以只能在单元层面给它非空语义。
    """
    c, how = cap_center(100.0, 400.0, 700.0, 715.0, 250.0, 706.0, -0.9)
    assert how == 'centroid' and c == (250.0, 706.0)


def test_cap_center_refuses_a_side_clipped_disc():
    c, how = cap_center(0.0, 300.0, 100.0, 400.0, 150.0, 250.0, -0.9)
    assert c is None and how == 'refused'


def test_cap_point_refuses_a_flat_ray():
    flat = dict(base=0.0, shoulder=20.0, elbow=0.0, wrist_pitch=-20.0)
    with pytest.raises(ServoRefused):
        servo.cap_point(flat, 160.0, 90.0, 0.016)


def test_aim_standoff_is_the_distance_from_the_cap_back_along_the_ray():
    """对准阶段的目标 = 「射线上离光心 L 处」—— 这正是爪尖**本来就该在**的位置。

    钉的是控制律的核心式子：`s = |P−C| − L` ⇒ `T = P − s·d = C + L·d`。
    """
    L = geom.handeye_range(False)
    u, v = geom.to_ai(717.79, 213.58)
    C, d = geom.pixel_ray(POSE0, u, v)
    P, _, _ = servo.cap_point(POSE0, u, v, 0.016)
    s = servo.aim_standoff(P, C)
    assert abs(s - (math.dist(P, C) - L)) < 1e-12
    T = servo.standoff_target(P, d, s)
    assert abs(math.dist(T, C) - L) < 1e-9
    assert math.dist(T, tuple(C[i] + L * d[i] for i in range(3))) < 1e-9


# ---------------------------------------------------------------------------
# 闭环
# ---------------------------------------------------------------------------

def test_aim_converges_and_lands_the_tip_on_the_ray():
    """★ 主测：从实测那一拍起手，4~8 拍内对准。判据全是物理量。"""
    p = Plant()
    rep, st = _run(p)
    assert rep['ok'], rep['stopped']
    assert len(rep["iters"]) <= 18, len(rep["iters"])

    # ★ 主判据是**横向**（物理对准）；像素残差是次一级的代理量，
    #   它同时含"沿射线"那一分量 —— 实测两者差一倍左右，都以 mm 计。
    assert _lateral_mm(p, st['j']) < 2.0, _lateral_mm(p, st['j'])
    gt = _ground_truth_err_px(p, st['j'])
    assert gt < 12.0, '真像素残差 %.1fpx' % gt              # ≈2mm @ 24cm
    # 而且不许下扎：爪尖必须还在瓶盖顶面之上
    assert p.tip(st['j'])[2] > p.cap[2] + 0.02, p.tip(st['j'])


def test_aim_makes_a_big_lateral_move_before_stopping():
    """起手离得远（横向 ~5cm）也必须走到 —— 不是"原地不动也算收敛"。"""
    p = Plant()
    rep, st = _run(p)
    t0 = p.tip(POSE0)
    t1 = p.tip(st['j'])
    assert math.hypot(t1[0] - t0[0], t1[1] - t0[1]) > 0.03, (t0, t1)


def test_aim_still_converges_when_the_camera_offset_is_wrong():
    """相机的真实偏移比模型差 3mm —— 循环还认不认得路？

    认得（像素是直接观测量）：`_lateral_mm` 残差会跟着模型误差走，
    量级 ≈ D·δ/‖手眼‖，断言它别超过瓶盖半径的一半并把数打出来。
    """
    p = Plant(off_true=tuple(geom.CAM_OPEN[i] + (0.003 if i == 1 else 0.0)
                             for i in range(3)))
    rep, st = _run(p)
    assert rep['ok'], rep['stopped']
    lat = _lateral_mm(p, st['j'])
    # 量级 ≈ D·δ/‖手眼‖ ≈ 22cm·3mm/12.4cm ≈ 5mm；给一倍余量，但必须**明显小于瓶盖半径**
    assert lat < p.r_cap * 1000.0, lat
    print('\n  [3mm 相机偏移] 横向残差 = %.2f mm（瓶盖半径 %.1f mm；模型无误差时 <2mm）'
          % (lat, p.r_cap * 1000))


def test_descend_puts_the_tip_on_the_cap():
    """下扎：留量压到 0。判据用**物理高度 + 物理距离**。"""
    p = Plant()
    rep, st = _run(p, phase='descend')
    assert rep['ok'], rep['stopped']
    tip = p.tip(st['j'])
    assert abs(tip[2] - p.cap[2]) < 0.005, tip
    assert math.dist(tip, p.cap) < 0.008, math.dist(tip, p.cap)


def test_the_freed_standoff_is_what_buys_the_workspace_margin():
    """★ **钉死留量 vs 放开留量** 的对照 —— 实测那个 bug 就长在这里。

    为什么不能靠跑一遍模拟来测：模拟里那台臂工作空间很宽松，**把留量钉死也照样收敛**
    （变异自检 M8 全绿）。真实约束只能从真实几何里复现：

        瓶盖在 r≈26cm、臂在肩87 肘−90 时
        钉死留量（n_standoff=1, s=0.10）→ 最优解余量只剩 6 count（执行不了）
        放开留量（[0.04, 0.12]）        → 找到余量 ≥40 的一族姿态（对准误差只差几 px）

    ⚠️ 这条测的是 **pick_pose 这一层**；"循环有没有用上这个自由度"只能靠实车。
    """
    P = (-0.0771, -0.2737, 0.0160)
    Cp = (-0.028, -0.098, 0.172)                # 那次的光心（米）
    d0 = tuple(P[i] - Cp[i] for i in range(3))
    n = math.sqrt(sum(v * v for v in d0))
    d = tuple(v / n for v in d0)
    kw = dict(lo=-88.0, hi=-30.0, span=14.0, tie_px=5.0, min_slack=40.0)
    freed = pick_pose(P, d, AIM_PX, 0.040, 0.120, -38.0, n_standoff=5, **kw)
    # 放开留量得到的解，**同时**在两项上都压过任何钉死的解：
    #   放开 [4,12]cm   -> err 101px  slack 50
    #   钉死 s=6cm      -> err 147px  slack 41
    #   钉死 s=10cm     -> err 225px  slack 43
    for s_pin in (0.060, 0.100):
        pin = pick_pose(P, d, AIM_PX, s_pin, s_pin, -38.0, n_standoff=1, **kw)
        assert freed.err_px < pin.err_px, (freed.err_px, s_pin, pin.err_px)
        assert freed.slack > pin.slack, (freed.slack, s_pin, pin.slack)


def test_the_loop_hands_the_standoff_down_as_a_free_variable(monkeypatch):
    """★ 循环必须把「沿视线留量」当**自由变量**传下去，不是钉成一个点。

    为什么只能查传参：模拟里那台臂工作空间太宽松，**把留量钉死也照样收敛**
    （变异自检 M8 全绿）—— 这个自由度的重要性只在真实几何下显出来。
    所以这里直接盯住"循环交给 pick_pose 的搜索窗口"。
    """
    seen = {}
    real = servo.pick_pose

    def spy(P_m, d, aim_px, s_lo, s_hi, prefer, *a, **kw):
        seen['s_lo'], seen['s_hi'] = s_lo, s_hi
        seen['n'] = kw.get('n_standoff', 1)
        return real(P_m, d, aim_px, s_lo, s_hi, prefer, *a, **kw)

    monkeypatch.setattr(servo, 'pick_pose', spy)
    p = Plant()
    obs, cmd, st = make_io(p)
    servo.run_phase(cfg(max_iters=1), obs, cmd, phase='aim', log=lambda *a: None)
    assert seen['s_hi'] > seen['s_lo'], seen      # 对准：一段区间
    assert seen['n'] > 1, seen
    # 下扎阶段相反：留量是**指令**值（一个点），臂自己不许乱跑
    seen.clear()
    p2 = Plant()
    obs2, cmd2, st2 = make_io(p2)
    servo.run_phase(cfg(max_iters=1), obs2, cmd2, phase='descend',
                    log=lambda *a: None)
    assert seen['s_hi'] == seen['s_lo'], seen
    assert seen['n'] == 1, seen


def test_loop_stops_when_the_cap_is_not_visible():
    """画面里没有瓶盖 -> **停**，不是硬着头皮动。"""
    p = Plant()
    obs, cmd, st = make_io(p)
    rep = servo.run_phase(cfg(), lambda: (to_fields(st['j'], 240.0, 496.0), None),
                          cmd, phase='aim', log=lambda *a: None)
    assert not rep['ok'] and '没有瓶盖' in rep['stopped'] and not rep['iters']


# ---------------------------------------------------------------------------
# 负对照：判据必须分得清"到没到"
# ---------------------------------------------------------------------------

def test_a_fixed_alpha_would_stall(monkeypatch):
    """★ 把 alpha 搜索**关掉**（始终用上一拍那个）—— 必须收敛不了。

    这条同时是"控制律为什么长这样"的证据：tip 钉在射线上之后，对准条件
    「光心→爪尖方向 ∥ 光心→瓶盖方向」**只能靠 alpha 满足**。
    （实测遗留方向退化：整体平移不改变观测误差，只靠腕子转一点点收敛极慢。）
    """
    def frozen(P_m, d, aim_px, s_lo, s_hi, prefer, gripper=240.0,
               wrist_roll=496.0, lo=-88.0, hi=-40.0, span=14.0, step=1.0,
               n_standoff=5, tie_px=5.0, min_slack=40.0):
        # 回到旧行为：留量钉死在"射线上离光心 L"处，alpha 也用上一拍那个
        s = s_hi
        T = servo.standoff_target(P_m, d, s)
        j = ik_open_m(T[0], T[1], T[2], prefer)
        ok, f = servo.fields_in_range(j, gripper, wrist_roll)
        return servo.Pose(servo.predicted_err_px(j, P_m, aim_px), prefer, s, j,
                          f, servo.field_slack(f))

    monkeypatch.setattr(servo, 'pick_pose', frozen)
    p = Plant()
    rep, _ = _run(p, max_iters=12)
    assert not rep['ok'], rep['stopped']


def test_a_plant_that_moves_the_wrong_way_must_not_converge():
    """★ 被控对象**反着走**（指令 +Δ 它走 −Δ）—— 必须不收敛。

    这条同时证明 `Plant` 真的在起作用：一个"怎么发都不动"的假臂也会让
    上面那些收敛测试里的**某些**断言通过。
    """
    p = Plant()
    obs, cmd, st = make_io(p)

    def cmd_bad(fields):
        j_was = st['j']
        j_new = from_fields(fields)
        st['j'] = {k: j_was[k] - (j_new[k] - j_was[k]) for k in servo.JOINT_KEYS}
        return obs()

    rep = servo.run_phase(cfg(max_iters=14), obs, cmd_bad, phase='aim',
                          log=lambda *a: None)
    assert not rep['ok'], rep['stopped']


def test_a_stuck_observation_must_not_converge():
    """观测永远不动（被控对象没反应）—— 必须停，不许报成功。"""
    p = Plant()
    obs, cmd, st = make_io(p)
    rep = servo.run_phase(cfg(), obs, lambda f: obs(), phase='aim',
                          log=lambda *a: None)
    assert not rep['ok'], rep['stopped']
