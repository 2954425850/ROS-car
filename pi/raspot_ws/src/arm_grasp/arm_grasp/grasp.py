# arm_grasp/grasp.py
# -*- coding: utf-8 -*-
"""抓取控制层：估计器 + 目标/姿态生成 + 相位机 + 10Hz 限速流式参考。**不碰硬件**。

设计见 `计划文档/【有效】2026-09-30-抓取全流程-design.md`。与 09-29 `servo.py` 的
根本差别只有一条，但它是根本的：

    目标点  T = O + s·(−ẑ_T(α))      ← 爪尖在**过目标点的下扎轴上**
                               ↑ 不再要求"爪尖落在视线（相机→目标那条线）上"

那条视线约束把臂锁在一条一维曲线上，是肘关节顶死固件限位 125 的根因（design §2.1）。
相机只要"看得见"目标，不再要求目标落在某个像素。

I/O 全部由调用方注入（`Link` 协议）：测试注入假臂，工具注入 ROS + K230。
"""
import math
from collections import namedtuple

from . import geom, ltrace
from .arm_kin import FIELD_HI, FIELD_LO, Unreachable, from_fields, to_fields
from .servo import (ServoRefused as Refused, field_slack, fields_in_range,
                    ik_open_m, limit_step, tip_open_m)


def _refuse(msg):
    return Refused(msg)


Obs = namedtuple('Obs', 't_arrive u_ai v_ai frame src box_norm')
Vis = namedtuple('Vis', 'u_lo u_hi v_lo v_hi')
Target = namedtuple('Target', 'alpha s_m joints fields slack err_m visible upx vpx')

GraspConfig = namedtuple('GraspConfig',
    'alpha_lo alpha_hi alpha_span alpha_step alpha0 '   # α 搜索
    'min_slack warn_slack slack_tie '                   # S0 三级 + 并列宽容
    'gripper wrist_roll '                               # p1/p2（张开 240 / 出厂 496）
    'vis '                                              # 看得见区域（AI 帧像素）
    's_pre_m s_stop_m s_min_step s_frac '               # 留量与下扎
    'max_step_deg hz max_seconds max_ticks '            # 流式
    'tol_m tol_px_err patience min_gain_m '             # 判据
    'z_plane_m latency_s k_ewma gate_m '                # 尺度 / 时间 / 估计器
    'close_field lift_m obs_lost_s')
GraspConfig.__new__.__defaults__ = (
    -88.0, -45.0, 14.0, 1.0, -58.0,
    40.0, 10.0, 5.0,
    240.0, 496.0,
    Vis(0.08 * 320.0, 0.92 * 320.0, 0.12 * 180.0, 0.86 * 180.0),
    # s_pre_m = 8cm 的来由（2026-09-30 实测 + 一条已证明的性质，见 pick_target docstring）：
    #   目标投影像素 v(s) = cy + fy·|dy|/(|dz|+s) = 97.1 + 9.394/(0.119+s) px —— **只由留量 s 决定**，
    #   与目标在哪无关。实测：s=0 → v=177.4（瞄准像素）；5cm → 153.3；8cm → 144.8；10cm → 140.4。
    #   跟踪器可锁带 v ≤ 160.7 ⇒ **目标中心在 s ≥ 2.9cm 时才可锁** ⇒ 末段 2.9cm 必然无观测
    #   （几何决定，改 α 没用；只有改相机安装才能动这条）。
    #   s_pre 的取舍：越大 → 目标在画面里越靠上（跟踪余量越大），但**接近姿态的余量越小**：
    #     5cm → slack 77 但 v=153（可锁带只剩 8px）；8cm → slack 58、v=145（16px）；10cm → slack 35。
    #   取 8cm：跟踪余量够，且 slack 仍在 min_slack(40) 之上。
    0.08, 0.003, 0.002, 0.30,
    1.5, 10.0, 40.0, 400,
    0.002, 10.0, 4, 0.0005,
    0.020, 0.040, 0.35, 0.03,
    578.0, 0.04, 1.0,
)


# --------------------------------------------------------------------------
# 目标 / 姿态
# --------------------------------------------------------------------------

def axis_point(O_m, s_m, alpha_deg):
    """过 O、沿下扎轴 ẑ_T(α) 退 s 的那一点（米）。s=0 就是 O 本身。

    ⚠️ ẑ_T 的水平方向取决于**爪尖自己的方位角**，不是目标点的方位角（两者差
    atan(s·cosα / r)，s=5cm、r=20cm 时约 7.8°，不是小量）⇒ 迭代一次即可收敛。
    """
    tb = math.atan2(O_m[1], O_m[0])
    for _ in range(2):
        a = math.radians(alpha_deg)
        ux, uy, uz = (math.cos(a) * math.cos(tb),
                      math.cos(a) * math.sin(tb), math.sin(a))
        T = (O_m[0] - s_m * ux, O_m[1] - s_m * uy, O_m[2] - s_m * uz)
        tb = math.atan2(T[1], T[0])
    return T


def visible(joints, O_m, vis):
    """此刻相机还看得见 O 吗。返回 (ok, (u_ai, v_ai))。"""
    u, v = geom.project(joints, O_m)
    return (vis.u_lo <= u <= vis.u_hi and vis.v_lo <= v <= vis.v_hi), (u, v)


def pick_target(O_m, cfg, s_m, prefer_alpha, vis=None):
    """在 α 上搜一个把**张开态爪尖**放到 `axis_point(O, s, α)` 的可行姿态。

    硬门：p3/p4/p5 ∈ [125,875]。排序：余量最大 → α 最接近 prefer_alpha（连续性，避免每拍换解）。

    ★★ **"看得见"不能当选择判据** —— 2026-09-30 由变异自检证明，**别再加回来**：
       候选之间 `visible` **恒相同**。证明：`ik_open_m` 恰好把爪尖放在过 O 的下扎轴上，于是
       `O − C = (O − T) − 1.7cm·ẑ − R·CAM_CLOSED = (s − 0.017)·ẑ − R·CAM_CLOSED`，在**工具系**里
       = `(0, 0, s−0.017) − CAM_CLOSED` —— **只含 s，与 α 无关，也与 O 在哪儿无关**。
       ⇒ 按 α 枚举的候选里，目标投影像素是同一个 ⇒ `[c for c in cands if c.visible]` 要么全留、
       要么全空，全空时 `or cands` 又原样补回来 ⇒ **恒等变换**（穷举 5 个 O × 7 个 s 验证过）。
       推论（有用）：**目标在画面里的位置是留量 s 的纯函数** ⇒ "接近阶段看得见吗"由 `s_pre` 决定，
       与目标在哪无关；`visible/upx/vpx` 只当**诊断量**（S0 报给人看），不参与选择。
    """

    vis = cfg.vis if vis is None else vis
    cands = []
    k = 0.0
    while k <= cfg.alpha_span + 1e-9:
        for a in ((prefer_alpha,) if k == 0.0
                  else (prefer_alpha - k, prefer_alpha + k)):
            if not (cfg.alpha_lo - 1e-9 <= a <= cfg.alpha_hi + 1e-9):
                continue
            T = axis_point(O_m, s_m, a)
            try:
                j = ik_open_m(T[0], T[1], T[2], a)
            except Unreachable:
                continue
            ok, f = fields_in_range(j, cfg.gripper, cfg.wrist_roll)
            if not ok:
                continue
            vis_ok, px = visible(j, O_m, vis)
            cands.append(Target(a, s_m, j, f, field_slack(f),
                                math.dist(tip_open_m(j), O_m), vis_ok, px[0], px[1]))
        k += cfg.alpha_step
    if not cands:
        raise _refuse('α∈[%.0f°,%.0f°]、留量 %.1fcm 下无可行姿态（不可解或越界）'
                      % (cfg.alpha_lo, cfg.alpha_hi, s_m * 100))
    good = cands          # ← 别加 visible 过滤：那是恒等变换（证明见 docstring）
    top = max(c.slack for c in good)
    near = [c for c in good if c.slack >= top - cfg.slack_tie]
    near.sort(key=lambda c: (abs(c.alpha - prefer_alpha), c.alpha))
    return near[0]


def tier(slack, cfg):
    """余量三级（**纯函数，单独测**，不用去凑一个工作空间边缘的点）：
    ≥min_slack → 'ok'；warn_slack~min_slack → 'warn'；<warn_slack → 'refuse'。
    """
    if slack < cfg.warn_slack:
        return 'refuse'
    if slack < cfg.min_slack:
        return 'warn'
    return 'ok'


def plan_verdict(O_m, cfg, vis=None):
    """S0 预检：末态（爪尖就在 O）能不能干。返回 (level, Target|None, 说明)。

    level：'ok'＝余量 ≥min_slack；'warn'＝warn_slack~min_slack（照跑但告警）；
           'refuse'＝解不出或余量 <warn_slack（拒绝并报该往哪挪）。
    """
    try:
        t = pick_target(O_m, cfg, 0.0, cfg.alpha0, vis)
    except Refused as e:
        return 'refuse', None, '末态解不出：%s' % e
    # 诊断（见 pick_target docstring 的推论）：目标投影像素只由留量 s 决定 ⇒
    # "接近阶段看得见吗 / 在不在可锁带里"是 s_pre 的函数，与目标在哪无关。S0 就报出来给人看。
    try:
        tp = pick_target(O_m, cfg, cfg.s_pre_m, cfg.alpha0, vis)
        pix = '；s_pre=%.1fcm 处：预测像素 (%.0f, %.0f) %s有效区（可锁带 v ≤ %.0f）、slack %.0f' % (
            cfg.s_pre_m * 100, tp.upx, tp.vpx,
            '在' if tp.visible else '**不在**', 0.893 * 180.0, tp.slack)
    except Refused as e:
        pix = '；但 s_pre=%.1fcm 处解不出姿态：%s' % (cfg.s_pre_m * 100, e)
    lvl = tier(t.slack, cfg)
    if lvl == 'refuse':
        return ('refuse', t,
                '余量只剩 %d count（< %d）——臂已到工作空间边缘，'
                '把目标往底座方向挪 5~8cm 再来%s' % (t.slack, cfg.warn_slack, pix))
    if lvl == 'warn':
        return ('warn', t, '余量 %d count（< %d）——接近限位，成功率可能下降%s'
                % (t.slack, cfg.min_slack, pix))
    return 'ok', t, '余量 %d count%s' % (t.slack, pix)


def standoff_along_axis(tip_m, O_m, alpha_deg):
    """爪尖离目标**沿下扎轴**还有多远（m）。下扎的限速用这个（用观测+回读算，不用指令）。

    **符号：爪尖在目标上方 ⇒ 正**（`standoff_along_axis(axis_point(O,s,α), O, α) == +s`）。
    ⚠️ 2026-09-30 Task 5 抓到并修掉的符号 bug（Task 4 遗留）：原来少了最外层那个负号，
      于是"在目标上方 8cm"返回 **−0.08**。这条不是形式问题——`run()` 下扎阶段的
      `max(s_ach, 0.0)` 会把负数钳成 0、`s_ach <= s_stop_m` 又在**第一拍**就成立 ⇒
      下扎循环一拍就"扎到位"退出，**测试还会假绿**（`test_loop_descends...` 就是靠它
      蒙过去的）。判据是 `servo.py:343` 的同名量：`s_ach = -Σ(tip−P)·d`，那里的 − 号是对的。
    下扎轴 u = ẑ_T(α) 由爪尖指向目标（z 分量为负），所以 `T − O = −s·u`。
    """
    tb = math.atan2(O_m[1], O_m[0])
    a = math.radians(alpha_deg)
    uz = math.sin(a)
    ux = math.cos(a) * math.cos(tb)
    uy = math.cos(a) * math.sin(tb)
    return -((tip_m[0] - O_m[0]) * ux + (tip_m[1] - O_m[1]) * uy
             + (tip_m[2] - O_m[2]) * uz)


# --------------------------------------------------------------------------
# 估计器 + 流式主循环（M2）
#
# 设计 §2.5 / §5.1 / §5.3 / §4：一条观测 → 时间对齐 → 射线∩平面 → 门限+EWMA
# → 按相位给目标 → 限速发一帧。**没有任何一处会阻塞在"等到位"上**。
# --------------------------------------------------------------------------

def estimate_point(trace, obs, z_plane_m, latency_s):
    """一条观测 → 基座系目标点 O。

    ① 时间对齐：用 `obs.t_arrive − latency_s` 去 Trace 里取"拍那张图那一刻"的关节状态
       （K230 的框是"过去"的，拿"现在"的关节去算几何会把误差按臂速放大）
    ② 射线 ∩ 平面(z=z_plane_m) → O

    返回 (O, C, d, joints, t_frame)。算不出来一律 `Refused`（**绝不返回垃圾深度**）。
    """
    t_frame = obs.t_arrive - latency_s
    fbf = trace.at(t_frame, 'auto')
    j = from_fields(fbf)
    try:
        P, _, _ = geom.target_from_pixel(j, obs.u_ai, obs.v_ai, z_plane_m)
    except ValueError as e:
        raise _refuse('算不出目标点：%s' % e)
    C, d = geom.pixel_ray(j, obs.u_ai, obs.v_ai)
    if d[2] > -0.3:
        raise _refuse('射线太平（d_z=%.3f），深度不可用' % d[2])
    return P, C, d, j, t_frame


class Est:
    """静止目标的 O 估计：门限 + EWMA + 盲段保持（不上卡尔曼，理由见 design §2.5）。

    门限是**必要的**：跳变的 O_new（错框 / 跟踪跳目标）如果直接进 EWMA，
    会把估计慢慢拖到错误的地方——门限把它整个丢掉并计数（`rejects`）。
    """

    def __init__(self, k=0.35, gate_m=0.03):
        self.k, self.gate_m = k, gate_m
        self.O, self.n, self.rejects = None, 0, 0

    def update(self, O_new):
        if self.O is None:
            self.O, self.n = tuple(O_new), 1
            return True
        if math.dist(O_new, self.O) > self.gate_m:
            self.rejects += 1
            return False
        self.O = tuple(self.O[i] + self.k * (O_new[i] - self.O[i]) for i in range(3))
        self.n += 1
        return True


class Link:
    """run() 需要的全部 I/O（接口，由调用方实现）：

        now() -> float          单调时钟（秒）——实现方用 time.monotonic()
        spin(dt)                非阻塞推进一小步（spin_once + 睡到 dt），**绝不等臂到位**
        fb() -> list | None     最新 6 个 field（可能含 0）
        obs() -> Obs | None     自上一拍以来最新的一条观测（已去重）
        publish(fields)         发一条 /arm/command
    """


def run(cfg, link, phase='aim', log=print):
    """流式主循环：每拍只算一次目标并限速发一帧，**没有任何一处会阻塞在"等到位"**。

    phase ∈ {'aim','descend','close','lift'}（本任务只实现 aim/descend 的判据，
    close/lift 走同一套限速参考，判据留给 Task 10）。

    返回 report dict（见下面 `rep` 的初始化；收敛判据一律用**回读算出来的实际量**）。
    """
    assert phase in ('aim', 'descend', 'close', 'lift')
    trace = ltrace.Trace()
    est = Est(cfg.k_ewma, cfg.gate_m)
    rep = {'phase': phase, 'ticks': 0, 'stopped': None, 'ok': False,
           'O_last': None, 'obs_n': 0, 'obs_bad': 0, 'obs_lost': False,
           'max_step_deg_actual': 0.0, 'err_m': None, 'alpha': None,
           'target': None, 's_ach_m': None, 'rows': []}
    dt = 1.0 / cfg.hz
    j_ref, cmd_sent = None, None
    prev_alpha = cfg.alpha0
    stall, best = 0, None
    s_cmd = cfg.s_pre_m
    t0 = None
    obs_age = None

    for it in range(cfg.max_ticks):
        link.spin(dt)                                   # ① 收消息
        now = link.now()
        if t0 is None:
            t0 = now
        fbf = link.fb()
        if fbf is not None:
            trace.add(now, cmd_sent, fbf)
            if j_ref is None:
                # ★ `j_ref` 的起点必须是**起手那一下的真实关节**，不是"第一帧指令"。
                #   否则第一拍的步长（往往是全程最大的一跳）落在测量窗口之外 ⇒
                #   `max_step_deg_actual` 抓不到"限速没生效"（变异自检真抓到过：
                #   把上限改成 1000 后，第一拍直接跳到位、之后每拍步长 0，指标恒为 0）。
                j_ref = to_fields(from_fields(fbf), cfg.gripper, cfg.wrist_roll)
        # ② 观测 → 估计
        obs = link.obs()
        if obs is not None:
            obs_age = 0.0
            try:
                O_new, C_obs, d_obs, j_obs, t_fr = estimate_point(
                    trace, obs, cfg.z_plane_m, cfg.latency_s)
                if est.update(O_new):
                    rep['obs_n'] += 1
                else:
                    rep['obs_bad'] += 1
            except Refused as e:
                rep['obs_bad'] += 1
                log('  [obs] 丢：%s' % e)
        else:
            if obs_age is not None:
                obs_age += dt
        # 阶段推进前的守卫
        if now - t0 > cfg.max_seconds:
            rep['stopped'] = '超时（%.0fs）' % cfg.max_seconds
            break
        if est.O is None:
            if fbf is not None:                          # 还没见到目标 → 原地不动
                cmd_sent = j_ref = to_fields(from_fields(fbf), cfg.gripper, cfg.wrist_roll)
                link.publish(cmd_sent)
            continue

        # ③ 目标（按阶段）
        if phase == 'descend':
            tip = tip_open_m(from_fields(trace.at(link.now(), 'auto')))
            s_ach = standoff_along_axis(tip, est.O, prev_alpha)
            step = max(cfg.s_min_step, cfg.s_frac * max(s_ach, 0.0))
            s_cmd = max(cfg.s_stop_m, s_ach - step)
        try:
            tgt = pick_target(est.O, cfg, s_cmd, prev_alpha)
        except Refused as e:
            rep['stopped'] = 'refuse：%s' % e
            break
        prev_alpha = tgt.alpha

        # ④ 限速参考 + 发
        # ★ 判据一律用**回读**算出来的实际量：`tgt.err_m` 是"目标姿态自己到 O 的距离"，
        #   它恒等于 s（因为姿态就是按 T=O+s·(−ẑ) 解出来的），拿它当收敛判据 ⇒ 永远不收敛。
        j_cur = from_fields(trace.at(link.now(), 'auto'))
        tip_act = tip_open_m(j_cur)
        T_goal = axis_point(est.O, s_cmd, tgt.alpha)
        err_act = math.dist(tip_act, T_goal)
        s_ach = standoff_along_axis(tip_act, est.O, tgt.alpha)
        j_cmd, ratio, reached = limit_step(j_cur, tgt.joints, cfg.max_step_deg)
        # ★★ 判据查的是**目标**的 field，不是这一拍的**指令**：
        #   实机开场姿态可能是"歇在机械限位上"的 —— 2026-09-30 实测开机 p4=121，**在固件下限 125 之下**，
        #   于是第一拍的限速指令必然也在界外。拿指令当判据 ⇒ 整条链在第一拍就 refuse（永远动不了）。
        #   界外的**指令**直接夹进 [125,875] 再发：固件本来就会夹，我们先夹一遍是为了
        #   让自己发出去的值与后续回读一致（否则白挨一次"目标 field 出界"）。
        # （`pick_target` 已保证候选 field 在界内 ⇒ 下面这条正常**永不触发**，留着当不变式断言）
        f_tgt = to_fields(tgt.joints, cfg.gripper, cfg.wrist_roll)
        if not all(FIELD_LO <= v <= FIELD_HI for v in f_tgt[2:5]):
            rep['stopped'] = 'refuse：目标 field 出界 %s' % ['%.0f' % v for v in f_tgt[2:5]]
            break
        fields = to_fields(j_cmd, cfg.gripper, cfg.wrist_roll)
        for i in (2, 3, 4):                    # 只夹 p3/p4/p5；p1/p2(夹爪/自转)、p6(底座 ±1000) 原样带过
            fields[i] = min(FIELD_HI, max(FIELD_LO, fields[i]))
        if j_ref is not None:
            dmax = max(abs(fields[k] - j_ref[k]) for k in range(6))
            rep['max_step_deg_actual'] = max(rep['max_step_deg_actual'], dmax / 4.1667)
        j_ref, cmd_sent = fields, list(fields)
        link.publish(fields)
        rep['ticks'] += 1
        rep['O_last'] = est.O
        rep['alpha'] = tgt.alpha
        rep['target'] = tgt
        rep['err_m'] = err_act
        rep['s_ach_m'] = s_ach
        rep['rows'].append((now, err_act, s_ach, tgt.alpha, tgt.slack,
                            0.0 if obs is None else 1.0))

        # ⑤ 判据 / 阶段切换（全用实际量）
        if best is None or err_act < best - cfg.min_gain_m:
            best, stall = err_act, 0
        else:
            stall += 1
        if phase == 'aim' and err_act <= cfg.tol_m:
            rep['stopped'] = '对准完成：爪尖离目标点 %.1fmm（余量 %.1fcm，α=%.1f°）' % (
                err_act * 1000, s_cmd * 100, tgt.alpha)
            break
        # ★ 判据带 `min_gain_m`(0.5mm) 容差，**不是**裸的 `s_ach <= cfg.s_stop_m`：
        #   上面的 `s_cmd = max(cfg.s_stop_m, ...)` 把**指令**地板钉死在 s_stop_m 上，而
        #   `s_ach` 是拿**当前**的 est.O 量的（观测随臂移动而变 ⇒ O 的 EWMA 一直在动）
        #   ⇒ 实测稳定停在 **3.012~3.037mm**，比地板高几十微米，`<= 0.003` 永远差一点点，
        #   于是被 stall 判据误报成"发散"（爪尖其实就停在离目标 3mm 处）。
        #   容差用 `min_gain_m`（"小到算没进展"的那个尺度，0.5mm）：既覆盖 O 的抖动
        #   （几十微米，13 倍余量），又不会提前开闸（实测在 3.016mm 处断，不是 4.8mm）。
        if phase == 'descend' and s_ach <= cfg.s_stop_m + cfg.min_gain_m:
            rep['stopped'] = '扎到位：实际余量 %.1fmm（爪尖离目标 %.1fmm）' % (
                s_ach * 1000, err_act * 1000)
            break
        if stall >= cfg.patience:
            rep['stopped'] = '发散：连续 %d 拍没进展（现在 %.1fmm，最好 %.1fmm）' % (
                stall, err_act * 1000, best * 1000)
            break
        if phase in ('aim', 'descend') and obs_age is not None \
                and obs_age > cfg.obs_lost_s:
            rep['obs_lost'] = True
            if phase == 'aim':
                rep['stopped'] = '观测丢失 %.1fs（接近阶段丢了就停下）' % obs_age
                break

    rep['ok'] = bool(rep['stopped']) and rep['stopped'].startswith(('对准完成', '扎到位'))
    return rep
