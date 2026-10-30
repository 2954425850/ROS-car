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
from .arm_kin import FIELD_HI, FIELD_LO, from_fields, to_fields
from .reach import (NoFeasible, axis_point, enumerate_feasible,
                    feasible_alpha_intervals, path_samples, select)
from .servo import ServoRefused as Refused, limit_step, tip_open_m

JOINT_KEYS = ('base', 'shoulder', 'elbow', 'wrist_pitch')


def _refuse(msg):
    return Refused(msg)


Obs = namedtuple('Obs', 't_arrive u_ai v_ai frame src box_norm depth_mm',
                 defaults=(None,))
Vis = namedtuple('Vis', 'u_lo u_hi v_lo v_hi')
Target = namedtuple('Target',
                    'alpha s_m joints fields slack err_m visible upx vpx slack_path')
# slack      = 当前 s_m 这一点的余量（兼容旧字段名）
# slack_path = 整段 s_stop→s_pre 的最小余量（新选解用的安全项；旧构造器不传时为 None）
Target.__new__.__defaults__ = (None,)

# ★ 下扎“到位”的 3D 误差上限（米）。`s_ach` 只是**沿轴投影**：爪尖横向偏 2cm 时
#   它仍可能只有 3mm。2026-10-30 真机就是“s_ach=3.2mm、爪尖离目标 22.8mm”被当成
#   扎到位，合爪后没抓到。必须同时要求实际 3D 误差够小。
DESCEND_ERR_TOL_M = 0.010

GraspConfig = namedtuple('GraspConfig',
    'alpha_lo alpha_hi alpha_span alpha_step alpha0 '   # α 搜索
    'min_slack warn_slack slack_tie '                   # S0 三级 + 并列宽容
    'gripper wrist_roll '                               # p1/p2（张开 240 / 出厂 496）
    'vis '                                              # 看得见区域（AI 帧像素）
    's_pre_m s_stop_m s_min_step s_frac '               # 留量与下扎
    'max_step_deg hz max_seconds max_ticks '            # 流式
    'tol_m tol_px_err patience min_gain_m '             # 判据
    'z_plane_m latency_s k_ewma gate_m '                # 尺度 / 时间 / 估计器
    'close_field lift_m obs_lost_s deadband_counts close_ticks '
    'min_step_deg close_step close_stall alpha_freeze_span bias_m '
    'base_comp base_comp_max '
    'joint_comp joint_comp_max '
    'bias_r_m '
    's_lead_m lead_max_counts require_depth '
    'close_reach_m aim_hold_m')
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
    # ⚠️ max_step_deg = **6.0**（不是 1.5）：`limit_step` 按最大行程关节等比缩放，小行程关节的
    #    每拍步长 = max_step × (小行程/大行程)。实测肩/腕行程比 ~3.5:1，1.5°/拍时腕只有 ~2 count、
    #    落进舵机死区**一动不动** ⇒ 姿态永远到不了（2026-09-30 真跑两次都栽在这）。6.0 ⇒ 最慢 ~7 count。
    # ⚠️ max_seconds/max_ticks 2026-10-01 放宽到 **150s / 1500 拍**：真机上"下扎"收敛很慢
    #   （实测耗了 ~390 拍 = 39s，而旧上限正是 400 拍 ⇒ `[close]` 刚起步就被掐掉、整轮白跑）。
    #   拍数上限必须**明显大于**最慢那一相，否则会出现"前面都对、最后一步没时间做"。
    6.0, 10.0, 150.0, 1500,
    # patience = 25 拍（10Hz = 2.5s）：和 grasp_once 真机默认一致。它必须**明显长于回读滞后**
    #   （~0.2~0.4s）——否则每次换相刚起步、回读还没开始动，就被误判"卡住"。
    0.002, 10.0, 25, 0.0005,
    0.020, 0.040, 0.35, 0.03,
    # deadband_counts = 10：实测**带载**（臂自重）下位置环死区稳态误差 5.6~6.6 count，
    # 取 10 留一档余量（定 6.0 时刚好卡在门外 ⇒ 判据不满足 ⇒ 白嗡嗡 2.5 秒才被"卡住"接管）。
    # close_ticks：合爪相最多走几拍（30 拍 @10Hz = 3s，兜底）。**不能是 12**：指令从 240 爬到
    #   578 正好 12 拍，而读回滞后很大（实测 0.4s 还没起振）⇒ 12 拍时读回可能还没走完，
    #   兜底会抢在"真的合到底"之前开闸（2026-10-01 第三跑就卡在这个边界上）。
    # min_step_deg：每个"要动"的关节每拍**至少**走多少度（=3.1 count，跨过舵机死区）
    # close_step  ：合爪每拍 p1 只走多少 count（**慢合**）—— 一次写到 578 是全力合，用户明确否决
    # close_stall ：合爪时"读回连续几拍几乎不动"算碰上东西 ⇒ 冻结不再加压
    # alpha_freeze_span：下扎/合爪/抬起时 α 只在 prefer±这个范围里搜（**冻住轴线**）。
    #   不能真取 0：可行 α 区间随留量 s 移动，钉死单点会在半路"无可行姿态"⇒ refuse
    #   （实测：s=3.2cm 时钉死 −59° 就解不出来了）。3° 的漂移只带 ~4mm 横向误差。
    # obs_lost_s = 5.0（原 1.0）：**跟踪器在相机移动时会跟丢低纹理目标**（2026-10-01 实测：
     #   瓶盖这种纯色大平面，臂一动就丢、臂一停就稳 8 秒零漂移）。1.0s 会把整轮拦掉。
     #   放长是安全的：丢观测期间**估计器保持 O 不变**、目标点也不变，臂只是继续走向同一个点。
    # bias_m：**静态偏置**（基座系，米）。**先留 0** —— 2026-10-01 查明"指尖偏左 1cm"的
    #   真正来源是**底座滞后 ~10 count（3.85°）**（令 −227.7 / 读 −217.0），20cm 上正好 1.34cm。
    #   底座靠 `base_comp`（外环补偿）修，不再用手调的 (x,y) 偏置（那还会把方向搞反）。
    #   留着这个字段是给"补偿后仍有残差"时兜底用的。
    # base_comp：**底座外环补偿**增益。底座是 motor 模式速度环、总差 ~10 count 到不了指令位置
    #   （2026-10-01 实测 令 −227.7/读 −217.0 = 3.85° ⇒ 20cm 上 1.34cm = 就是"偏左 1cm"）。
    #   每拍把 (指令−回读)×增益 补回指令，直到回读真的到位；0 = 关掉。
    # base_comp_max：单次补偿的上限（count），防止一次补过头把底座甩出去。
    # ⚠️ 2026-10-01 再抬到 **30s**：跟踪器在相机移动时**必丢**低纹理目标（瓶盖、白色细长条都试过），
    #   而目标是**静止**的、几何解算是**绝对量** ⇒ 开头一条干净观测就够了，之后靠"估计器保持"完全安全。
    #   前提：这期间**人不能碰目标物**。
    578.0, 0.04, 30.0, 10.0, 30, 0.75, 30.0, 3, 3.0, (0.0, 0.0, 0.0), 0.5, 80.0,
    # joint_comp：肩/肘/腕位置环下垂补偿增益（0 = 关）；joint_comp_max：单拍补多少 count 封顶。
    #   实测下垂 +8.4/+6.0/+16.7 count（p3/p4/p5）⇒ 爪尖差 1.5~2.2cm。稳态残差 = 下垂/(1+增益)。
    #   先取 1.0（砍一半）——**别一次性往大调**，底座那边有过"增益大会一起震荡"的教训。
    1.0, 60.0,
    # bias_r_m：**沿半径**的静态偏置（米，+|向外/远离底座）。实测爪尖**几乎每次都偏后 ~1cm**
    #   = 舵机死区稳态残差 + 臂自重沉向那边；换方位角它还是径向 ⇒ 用径向的、不用固定向量。
    0.0,
    # s_lead_m：下扎时留量**指令**最多领先回读多少（米）。指令按自己上一拍单调往下走，
    #   回读落后超过这个量就原地等它（回读滞后 ~0.2~0.4s，不能拿滞后回读当起点，见 run()）。
    # lead_max_counts：每拍限速的起点用**上一拍指令**；指令领先回读超过这么多 count
    #   （= 臂跟不上 / 被挡住）就**原地等**（指令保持），免得指令一路跑飞。
    #   不能"退回从回读起步"：那样指令会往回跳（0.4s 滞后下实测来回顶）。
    0.015, 40.0,
    # require_depth：盲区硬闸（默认关，既有用例不受影响）。开 = **进 descend 之前**
    #   本轮必须见过至少一次有效实测深度（`Obs.depth_mm` 非 None）；一次都没见过就
    #   **拒绝、一步不动**，绝不悄悄退化成"假设目标躺在 1.6cm 平面上"去硬抓。
    #   为什么必须在进 descend 前判死：ToF 在 <15cm 上瞎，而进 descend 之后 cmd 就开始
    #   往目标扎 ⇒ 那时再看深度已经晚了（见设计 §6）。
    False,
    # close_reach_m：下扎到 s_stop **解不出姿态**时的合爪窗口（米）。
    #   2026-10-21 真机：黄椒在工作空间边缘（S0 slack 24），aim 收在 α=−85°，
    #   下扎时 α 每拍重选、漂到 −79°，到 s=25mm 处 shoulder 要 872/上限 875 ⇒
    #   `pick_target` refuse。可**臂实际停在 40.6mm、爪尖离目标点只有 16.2mm**，
    #   爪已经套在椒上 —— 再拒绝就是整轮白跑。
    #   ⇒ 下扎已到 s_stop 地板却无解、且爪尖离目标点 ≤ 这个窗口时，**直接进合爪**
    #   （`run()` 的 descend 分支）。0 = 关掉这个兜底（退回旧的"整轮 refuse"）。
    #   取 30mm：爪张开时指尖到指根的跨度约 5~6cm，目标中心进到 3cm 以内才认为
    #   "已经套住"；比它远就仍然按几何拒绝，不去赌。
    0.030,
    # aim_hold_m：接近相**冻结 O**的窗口（米）。爪尖离当前目标点 ≤ 这个距离后，
    #   不再用新观测更新 O、α 搜索窗也从 ±alpha_span 收到 ±alpha_freeze_span。
    #   2026-10-21 真机（椒在 r≈20.8cm 的工作空间边缘）：α 稳在 −75°，但 p3 指令
    #   仍在 ±27 count 来回、err 10~42mm 抖满 150s 到超时。原因是接近时相机还在动，
    #   K230 的框有 0.1~0.2s 龄、ToF 是当前帧，两者配出来的 O 每拍都在漂 ⇒ 目标点
    #   跟着漂 ⇒ 舵机被指挥着来回追，"回读进死区连续 3 拍"永远不成立。
    #   冻结后臂在一个**固定** O 上收敛；前几拍已经把 O 收敛到 1~2cm 内。
    #   0 = 关掉（退回旧行为，靠近时仍每拍更新 O）。
    0.030,
)


# --------------------------------------------------------------------------
# 目标 / 姿态
# --------------------------------------------------------------------------

def apply_bias(O_m, bias_m=(0.0, 0.0, 0.0), bias_r_m=0.0):
    """目标点的**静态偏置**。两种：

    * `bias_m`：固定向量（基座系，米）。换个方位角就不对了 —— 只适合"偏差方向不变"的场合。
    * `bias_r_m`：**沿半径方向**（+|向外/远离底座）。实测"几乎每次都偏后 ~1cm"是**径向**的
      （舵机死区稳态残差 + 臂自重往那边沉），换物体的方位角它还是径向 ⇒ 用这个才对。

    ★ 为什么要人给：这 1cm 是**硬件死区**留下的稳态残差，闭环自己收不掉（判据本来就在死区上）；
      要真收掉得给每个关节加**积分**补偿，那是另一件事（今晚试过、和到位判据打架，已回退）。
    """
    x, y, z = O_m
    r = math.hypot(x, y)
    if r < 1e-9:
        ux = uy = 0.0
    else:
        ux, uy = x / r, y / r
    return (x + bias_m[0] + bias_r_m * ux,
            y + bias_m[1] + bias_r_m * uy,
            z + bias_m[2])


# axis_point 现在在 reach.py（grasp import 它，避免 reach ↔ grasp 循环依赖）。
# 对外 `grasp.axis_point` 仍然可用。


def visible(joints, O_m, vis):
    """此刻相机还看得见 O 吗。返回 (ok, (u_ai, v_ai))。"""
    u, v = geom.project(joints, O_m)
    return (vis.u_lo <= u <= vis.u_hi and vis.v_lo <= v <= vis.v_hi), (u, v)


def pick_target(O_m, cfg, s_m, prefer_alpha, vis=None, span=None, balance=False,
                j_prev=None, branches=(-1, 1), step_deg=None, s_samples=None):
    """在 α 上选一个把**张开态爪尖**放到 `axis_point(O, s, α)` 的可行姿态。

    ★ 2026-10-28 起是“全包络可行集 + 整段路径余量 + 字典序选解”：
      - `span=None`  ⇒ 扫**整个配置包络** `[alpha_lo, alpha_hi]`（aim 用）；
      - `span=<x>`  ⇒ 只扫 `prefer_alpha ± x`（下扎冻结窗用）；
      - 每个候选都要在 `s_stop → s_pre` 的整段路径上 field 合法；
      - 选解：① 整段最小余量最大 → ② α 离 prefer 最近 → ③ 关节行程最少
        → ④ |α−prefer|、α 本身。详见 `reach.select()`。
      - `balance` 保留为兼容参数：整段路径余量已经包含“两头一起看”，不再单独处理。

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
    if step_deg is None:
        step_deg = getattr(cfg, 'alpha_step', 1.0)
    if span is None:
        lo, hi = cfg.alpha_lo, cfg.alpha_hi
    else:
        lo = max(cfg.alpha_lo, prefer_alpha - span)
        hi = min(cfg.alpha_hi, prefer_alpha + span)
    cands = enumerate_feasible(O_m, cfg, s_m, alpha_lo=lo, alpha_hi=hi,
                               step_deg=step_deg, branches=branches,
                               s_samples=s_samples)
    if not cands:
        # 2026-10-28：现在印的是**真正扫过的窗口**（全包络时就是包络本身），
        # 并且把“几何/field 都不可行”说清楚；不再把种子窗伪装成包络。
        tag = ('全包络 [%.0f°,%.0f°]' % (lo, hi) if span is None
               else '[%.0f°,%.0f°]（prefer %.0f° ± %.0f°）'
                    % (lo, hi, prefer_alpha, span))
        # 额外诊断：只查当前留量 s_m 时有没有可行 α（区分“几何/限位不行”
        # 和“末态行、整段下扎不行”）。
        endpoint = enumerate_feasible(O_m, cfg, s_m, alpha_lo=lo, alpha_hi=hi,
                                      step_deg=step_deg, branches=branches,
                                      s_samples=[s_m])
        if endpoint:
            iv = ', '.join('[%.0f°,%.0f°]' % (a, b) for a, b in
                           feasible_alpha_intervals(endpoint))
            extra = ('；只查留量 %.1fcm 时这些 α 可行 %s，但整段路径不可行'
                     % (s_m * 100.0, iv))
        else:
            extra = '；当前留量处也没有可行 α（几何不可解或 field 越界）'
        raise _refuse('搜过的 α∈%s、留量 %.1fcm 下无整段可行姿态%s'
                      % (tag, s_m * 100.0, extra))
    try:
        c = select(cands, cfg, alpha_prev=prefer_alpha,
                   prefer_alpha=prefer_alpha, j_prev=j_prev)
    except NoFeasible as e:
        raise _refuse(str(e))
    vis_ok, px = visible(c.joints, O_m, vis)
    return Target(c.alpha, s_m, c.joints, c.fields, c.slack,
                  math.dist(tip_open_m(c.joints), O_m), vis_ok, px[0], px[1],
                  c.min_slack)


def pick_target_descend(O_m, cfg, s_m, anchor_alpha, j_prev=None):
    """下扎相选解：**只接受进相时钉死的 α**，不换解、不漂。

    2026-10-30 真机：aim 选 −86、下扎中途放宽到 −81，最后爪尖离目标 17.7mm
    （用户看到“又往下扎了一点，偏了”）。下扎改成硬冻结：
      · 成功路径是 aim 选出的 α（它已通过整段路径检查）；
      · 精确 α 在某个 s 解不出 ⇒ 抛 Refused，交给 run 的“就地合爪”兜底，
        **不要换一个 3° 外的 α 继续扎**。
    这样轴线是一条直线，不会因为逐拍放宽而漂。
    """
    return pick_target(O_m, cfg, s_m, anchor_alpha, span=0.0, j_prev=j_prev)


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
        sp = tp.slack_path if tp.slack_path is not None else tp.slack
        pix = '；s_pre=%.1fcm 处：预测像素 (%.0f, %.0f) %s有效区（可锁带 v ≤ %.0f）、整段余量 %.0f' % (
            cfg.s_pre_m * 100, tp.upx, tp.vpx,
            '在' if tp.visible else '**不在**', 0.893 * 180.0, sp)
    except Refused as e:
        pix = '；但 s_pre=%.1fcm 处解不出姿态：%s' % (cfg.s_pre_m * 100, e)
    sl = t.slack_path if t.slack_path is not None else t.slack
    lvl = tier(sl, cfg)
    if lvl == 'refuse':
        return ('refuse', t,
                '整段最小余量只剩 %d count（< %d）——臂已到工作空间边缘，'
                '把目标往底座方向挪 5~8cm 再来%s' % (sl, cfg.warn_slack, pix))
    if lvl == 'warn':
        return ('warn', t, '整段最小余量 %d count（< %d）——接近限位，成功率可能下降%s'
                % (sl, cfg.min_slack, pix))
    return 'ok', t, '整段最小余量 %d count%s' % (sl, pix)


def limit_step_floor(j_cur, j_des, max_step_deg, min_step_deg):
    """等比限幅 + **每个"要动"的关节至少走 min_step_deg**。

    为什么不能只用 `servo.limit_step` 的等比缩放：比例由**最大行程那个关节**定，
    小行程关节的每拍步长 = max_step × (小行程 / 大行程)。2026-10-01 第一次真抓实测：
    抬升阶段 p4 要 24 count、p5 要 39 count，按 2°/拍等比缩后 **p3 每拍只剩 0.12 count**
    ⇒ 落进舵机死区 ⇒ 它一动不动、姿态永远到不了（trace 里"发"的 p4 在 191 附近抖、
    "读"的 186 从头到尾不动）。
    """
    j_cmd, ratio, reached = limit_step(j_cur, j_des, max_step_deg)
    if reached:
        return j_cmd, ratio, True
    out = dict(j_cmd)
    for k in JOINT_KEYS:
        d = j_des[k] - j_cur[k]
        if abs(d) < 1e-12:
            out[k] = j_cur[k]
            continue
        lo = min(min_step_deg, abs(d), max_step_deg)
        if abs(out[k] - j_cur[k]) < lo:
            out[k] = j_cur[k] + math.copysign(lo, d)
    return out, ratio, False


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

def estimate_point(trace, obs, z_plane_m, latency_s, require_depth=False):
    """一条观测 → 基座系目标点 O。

    ★ 两条路：
      * `obs.depth_mm` 有值（ToF 实测）⇒ **射线 × 实测光轴深度**。这是首选。
      * 否则 ⇒ 老的 **射线 ∩ 假设平面 z=z_plane_m**（`--h` 兜底路径）。

    ★ `require_depth=True` 时**禁止**走第二条路：没有实测深度就 Refused。
      这是"绝不拿 1.6cm 车身瓶盖平面蒙一个 O"的硬闸。它按**每一条观测**生效，
      不只是第一条 —— 上一版本的 `run()` 只在进 descend 前查过"见过一次深度"，
      后续观测丢深度时仍会静默回落，这个参数就是补那个洞。

    返回 (O, C, d, joints, t_frame)。**绝不返回垃圾深度** —— 算不出来一律 Refused。
    """
    t_frame = obs.t_arrive - latency_s
    fbf = trace.at(t_frame, 'auto')
    j = from_fields(fbf)
    C, d = geom.pixel_ray(j, obs.u_ai, obs.v_ai)
    depth_mm = getattr(obs, 'depth_mm', None)
    if depth_mm is not None:
        try:
            P, _t = geom.target_from_pixel_depth(
                j, obs.u_ai, obs.v_ai, float(depth_mm) / 1000.0)
        except ValueError as e:
            raise _refuse('算不出目标点（实测深度）：%s' % e)
        return P, C, d, j, t_frame
    if require_depth:
        raise _refuse('这一条观测没有实测深度（require_depth=True）—— '
                      '拒绝用 z=%.1fmm 的假设平面蒙一个目标点；'
                      '请检查 ToF/结果流，或手动给 --h'
                      % (z_plane_m * 1000.0))
    try:
        P, _, _ = geom.target_from_pixel(j, obs.u_ai, obs.v_ai, z_plane_m)
    except ValueError as e:
        raise _refuse('算不出目标点：%s' % e)
    if d[2] > -0.3:
        raise _refuse('射线太平（d_z=%.3f），深度不可用' % d[2])
    return P, C, d, j, t_frame


def validate_measured_observation(trace, obs, point, latency_s):
    """A tracked box confirms visibility of a measured point, not its 3D depth.

Requires actual feedback and a finite box. Inconsistent boxes do not refresh
the blind-motion timer. The scene must remain static after the scan.
"""
    box = obs.box_norm
    if (box is None or len(box) != 4 or not all(math.isfinite(v) for v in box)
            or not (0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1)):
        raise _refuse('多视角目标需要有效跟踪框核验')
    try:
        j = from_fields(trace.at(obs.t_arrive - latency_s, 'fb'))
        u, v = geom.project(j, point)
    except ValueError as e:
        raise _refuse('多视角目标投影失败：%s' % e)
    u, v = u / geom.AI_W, v / geom.AI_H
    margin = .025
    if not (box[0] - margin <= u <= box[2] + margin
            and box[1] - margin <= v <= box[3] + margin):
        raise _refuse('跟踪框不包含已测目标（目标移动/错框/位姿偏差）')


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


PHASES = ('aim', 'descend', 'close', 'lift')


def run(cfg, link, phase='aim', log=print, initial_point=None):
    """流式主循环：每拍只算一次目标并限速发一帧，**没有任何一处会阻塞在"等到位"**。

    phase ∈ {'aim','descend','close','lift','all'}。
    **`'all'` = 四相依次走完，而且是一次调用** —— 估计器/时间缓冲不重建，相位切换只是换个
    目标点，所以动作**不断流**（分段调用会在两次调用之间丢掉估计与连续性）。

    每相结束由 `finish()` 记进 `rep['phases']` 并切下一相。

    返回 report dict（见下面 `rep` 的初始化；收敛判据一律用**回读算出来的实际量**）。
    """
    assert phase in PHASES + ('all',)
    seq = list(PHASES) if phase == 'all' else [phase]
    trace = ltrace.Trace()
    est = Est(cfg.k_ewma, cfg.gate_m)
    if initial_point is not None:
        # Multiview target is a real measured surface point. Detector-box centres
        # are not the same physical point and must not overwrite this estimate.
        initial_point = tuple(float(v) for v in initial_point)
        if len(initial_point) != 3 or not all(math.isfinite(v) for v in initial_point):
            raise _refuse('无效的多视角目标点')
        est.update(initial_point)
    rep = {'phase': phase, 'ticks': 0, 'stopped': None, 'ok': False,
           'O_last': None, 'obs_n': 0, 'obs_bad': 0, 'obs_lost': False,
           'depth_seen': False, 'depth_refused': 0,
           'descend_fallback': False, 'fallback_err_m': None,
           'base_glitch': 0,
           'max_step_deg_actual': 0.0, 'err_m': None, 'alpha': None, 'comp': None,
           'target': None, 's_ach_m': None, 'rows': [], 'phases': [], 'O_phase_end': []}
    dt = 1.0 / cfg.hz
    j_ref, cmd_sent, f_prev, near = None, None, None, 0
    f_now = None          # 上一拍的 field（底座外环补偿要用；本拍的更晚才算）
    prev_alpha = cfg.alpha0
    descend_alpha = None   # 下扎相的 α 锚（进相时定；只有窗口放宽才改）
    base_fb_prev = None    # p6 回读连续性过滤（电位器毛刺会让 base_comp 乱补）
    stall, best = 0, None
    s_cmd = cfg.s_pre_m
    t0 = None
    obs_age = 0.0 if initial_point is not None else None
    cur = seq[0]          # 当前相
    hold_j = None         # 'close' 相：进相时冻住的四个关节
    close_cmd = None      # 'close' 相：正在往 close_field 爬的 p1 指令
    close_stuck = 0       # 'close' 相：读回连续几拍没动的计数（= 碰上东西了）
    p1_prev = None        # 'close' 相：上一拍读回的 p1
    comps = None             # 肩/肘/腕的位置环下垂（p3/p4/p5，count）—— 交接处量一次
    p1_start = 0.0        # 'close' 相：进相时的 p1 回读（判"起振"的基准）
    close_armed = False   # 'close' 相：读回"真的动过"了没有（见 close 相那段注释）
    grip_hold = None      # 'lift' 相：沿用 close 收尾时的 p1 指令（冻结值，**不是** close_field）
    j_prev_cmd = None     # 上一拍的**未补偿**关节指令（限速的起点，见 ④）
    phase_ticks = 0       # 本相已走几拍
    aim_hold = False      # 'aim' 相：已贴到目标 ⇒ 冻结 O 与 α（见 cfg.aim_hold_m）

    def advance_or_stop(reason):
        """本相结束：有下一相就切过去（返回 True 继续），最后一相就定稿（返回 False）。

        只在这里改 `cur` 和每相计数 ⇒ 五个停止点各两行，不会漏复位某一项。
        """
        nonlocal cur, near, stall, best, phase_ticks, hold_j, j_ref, s_cmd
        nonlocal close_cmd, close_stuck, p1_prev, p1_start, close_armed, comps, grip_hold
        nonlocal descend_alpha
        # 第三项 = 本相走了几拍。**量"时间花在哪一相"全靠它**（47s 里到底是接近慢、
        # 下扎慢、还是合爪慢，不量就只能猜）。放最后一位，读 `p[0]/p[1]` 的调用方不受影响。
        rep['phases'].append((cur, reason, phase_ticks))
        rep['O_phase_end'].append((cur, est.O))      # 每相收尾时在用的 O（查"下扎用的是哪个 O"）
        k = seq.index(cur) + 1
        if cur == 'aim' and k < len(seq) and seq[k] == 'descend' \
                and cfg.require_depth and not rep.get('depth_seen'):
            rep['stopped'] = ('进 descend 前从未拿到有效深度 —— '
                              'ToF 没给过数，拒绝下扎（不做平面假设兜底）')
            return False
        if k >= len(seq):
            rep['stopped'] = reason
            return False
        log('  [%s] 完成：%s  （本相 %d 拍 = %.1fs） ⇒ 进入 [%s]'
            % (cur, reason, phase_ticks, phase_ticks / cfg.hz, seq[k]))
        if (cur == 'aim' and seq[k] == 'descend' and cfg.joint_comp > 0.0
                and f_now is not None and f_tgt is not None):
            # 臂已经停稳（[aim] 刚报完到位）⇒ (纯目标 − 回读) 就是位置环下垂。量一次、带到底。
            v = [max(-cfg.joint_comp_max,
                     min(cfg.joint_comp_max, cfg.joint_comp * (f_tgt[i] - f_now[i])))
                 for i in (2, 3, 4)]
            comps = v
            log('  [下垂补偿] 量到 (目标−回读) = %s count ⇒ 下扎/抬起带着走'
                % ['%+.1f' % x for x in v])
        if cur == 'close':
            # ★ 合爪收尾那一刻的 p1 指令带进抬起相（2026-10-01 复查）。旧写法下面把 close_cmd
            #   清成 None、抬起相就落到 close_field(578) ⇒ "碰上东西就冻结"之后，**抬起时照样全力夹**。
            grip_hold = close_cmd
        cur = seq[k]
        if cur == 'descend':
            # 下扎锚 = aim 收尾时的 α。之后窗口中心固定，不再逐拍游走。
            descend_alpha = prev_alpha
        near, stall, best, phase_ticks = 0, 0, None, 0
        hold_j, j_ref, s_cmd = None, None, cfg.s_pre_m
        close_cmd, close_stuck, p1_prev = None, 0, None
        # ⚠️ `comps` **不在这里复位**：它就是要在 aim→descend 量到之后一路带着走。
        #    （第一版把这行写在这儿 ⇒ 量完立刻被清成 None，等于没补。）
        p1_start, close_armed = 0.0, False
        return True

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
        # ★ 还没读到**任何**可用回读就先别往下算：`trace.at()` 撞上空缓冲会抛
        #   `ValueError('Trace 是空的')`，而 `run()` 只接了 `Refused` ⇒ **整轮直接崩**。
        #   2026-10-01 踩到：跟踪器给框比串口反馈快，观测先到、回读还没到。
        #   （老隐患，只是今天才撞上；上面 `fbf is None` 时 `trace.add` 不会执行。）
        if not trace.buf:
            continue
        # ② 观测 → 估计
        # Once the object is grasped it moves with the gripper. The measured
        # contact point remains the lift-path anchor, not a visibility target.
        obs = None if initial_point is not None and cur == 'lift' else link.obs()
        if initial_point is not None and cur == 'lift':
            obs_age = None
        elif initial_point is not None and obs_age is not None:
            obs_age += dt
        if obs is not None:
            # obs_age 从**第一条观测**开始计时（哪怕它随后被 Refused 掉）；
            # 只有算出 O 才清零。否则"有框但没深度"会永远走不到丢失判据。
            if obs_age is None and initial_point is None:
                obs_age = 0.0
            # 盲区硬闸（require_depth）：记下"本轮到底见过没有真实 ToF 深度"。
            # 用 `rep` 而不是局部变量，省得在闭包 `advance_or_stop` 里纠结 `nonlocal`，
            # 而且报告里也留了证据。（见设计 §6。）
            if getattr(obs, 'depth_mm', None) is not None:
                rep['depth_seen'] = True
            try:
                if initial_point is not None:
                    validate_measured_observation(trace, obs, initial_point, cfg.latency_s)
                    obs_age = 0.0
                    rep['obs_n'] += 1
                elif (cur == 'aim' and aim_hold) or (cur == 'descend' and est.O is not None):
                    # ★ 贴住目标（aim_hold）或进入下扎后：**不再更新 O**。
                    #   2026-10-30 真机：下扎期间 O 随观测漂 → 可行的 α 区间变 →
                    #   选解从 −86 放宽到 −81 → 爪尖横向偏 17.7mm。
                    #   目标是静止的，aim 阶段一条干净观测足够；下扎只沿固定 O 走。
                    obs_age = 0.0
                    rep['obs_n'] += 1
                else:
                    O_new, C_obs, d_obs, j_obs, t_fr = estimate_point(
                        trace, obs, cfg.z_plane_m, cfg.latency_s,
                        require_depth=cfg.require_depth)
                    # ★ 只有**算出了 O** 才算"见到目标"：estimate_point 因缺深度
                    #   被拒绝时不能把 obs_age 清零，否则 require_depth 会一直等到
                    #   max_ticks 也不报"没有深度"（2026-10-28 补）。
                    obs_age = 0.0
                    if est.update(O_new):
                        rep['obs_n'] += 1
                    else:
                        rep['obs_bad'] += 1
            except Refused as e:
                rep['obs_bad'] += 1
                if cfg.require_depth and '没有实测深度' in str(e):
                    rep['depth_refused'] += 1
                    # ★ 一直算不出 O（连一条深度都没见过）时别干等到 max_ticks：
                    #   连续 patience 拍没有深度就明确停住。等 patience 是为了容忍
                    #   ToF 的瞬时掉帧；patience=25 @10Hz = 2.5s。
                    if est.O is None and rep['depth_refused'] >= cfg.patience:
                        rep['stopped'] = (
                            '没有实测深度（require_depth=True）：连续 %d 拍没有深度，'
                            '拒绝用 z=%.1fmm 平面兜底；请检查 ToF/结果流，'
                            '或手动给 --h' % (rep['depth_refused'],
                                             cfg.z_plane_m * 1000.0))
                        break
                # ★ 观测**存在但不可用**也要推进 obs_age：否则缺深度时每拍都
                #   Refused、obs_age 恒 0，永远走不到"观测丢失"的停止点，
                #   最后只是撞 max_ticks 且 stopped=None（2026-10-28 补）。
                if obs_age is not None and initial_point is None:
                    obs_age += dt
                log('  [obs] 丢：%s' % e)
                if initial_point is not None:
                    rep['stopped'] = '已测目标核验失败：%s' % e
                    break
        else:
            if obs_age is not None and initial_point is None:
                obs_age += dt
        # 阶段推进前的守卫
        if now - t0 > cfg.max_seconds:
            rep['stopped'] = '超时（%.0fs）' % cfg.max_seconds
            break
        if est.O is None:
            # ★ 还没算出 O 时也要判"观测丢失/缺深度"：否则每拍都在这里 continue，
            #   永远走不到后面那个 obs_lost 检查点，最后撞 max_ticks 且 stopped=None
            #   （2026-10-28：require_depth + 全程无深度时实测）。
            if obs_age is not None and obs_age > cfg.obs_lost_s:
                rep['obs_lost'] = True
                if cfg.require_depth and rep['depth_refused']:
                    rep['stopped'] = (
                        '没有实测深度（require_depth=True）：已拒绝 %d 次 '
                        'z=%.1fmm 平面兜底；始终没算出目标点'
                        % (rep['depth_refused'], cfg.z_plane_m * 1000.0))
                else:
                    rep['stopped'] = '观测丢失 %.1fs（还没算出目标点）' % obs_age
                break
            if fbf is not None:                          # 还没见到目标 → 原地不动
                cmd_sent = j_ref = to_fields(from_fields(fbf), cfg.gripper, cfg.wrist_roll)
                j_prev_cmd = None
                link.publish(cmd_sent)
            continue
        # ★ 目标点 + **静态偏置**：实测指尖总落在目标左边 1cm ⇒ 目标往右挪 1cm 抵掉。
        #   下面所有几何（目标姿态/残差/沿轴留量）**一律用 `O_u`**，别混用 est.O。
        O_u = apply_bias(est.O, cfg.bias_m, cfg.bias_r_m)

        # ③ 目标（按**当前相** `cur`）
        # ★★ `f_raw` = 这一拍时间对齐后的 6 个 field（回读优先）。**p1 的真值只在 `f_raw[0]`**：
        #   `from_fields` 把 p1/p2 丢掉，`to_fields(j, grip_cmd, …)` 又把 grip_cmd 写回第 0 位
        #   ⇒ 下面 ④ 的 `f_now[0]` **恒等于"我这一拍刚下发的指令"**。
        #   2026-10-01 真抓（桌面目标）踩到：拿 `f_now[0]` 当"读回"用 ⇒ ① 合爪收尾报
        #   "夹空了——盖不在爪子里"，其实手里**正握着那个白条**；② `close_stuck`
        #   （"读回停住 = 碰上东西"）拿指令跟自己比，永远为 0 ⇒ **慢合的柔性冻结从未触发**，
        #   每次都一路合到 close_field —— 用户明确否决过的"全力抓"其实一直没被修掉。
        #   合爪相的一切判据**只许用 `p1_read`**。
        f_raw = trace.at(link.now(), 'auto')
        # ★ 2026-10-30 黑匣子抓到：底座 p6 回读会在 40ms 内 -9→+129→-342
        #   （电位器毛刺，不是真实运动）。base_comp 拿它补底座 ⇒ 底座指令乱摆 ⇒
        #   爪尖横向偏 2cm。p6 每拍变化超过 60 count（21.6°，物理不可能）就沿用上一拍。
        if len(f_raw) >= 6:
            _b = float(f_raw[5])
            if base_fb_prev is not None and abs(_b - base_fb_prev) > 60.0:
                f_raw = list(f_raw)
                f_raw[5] = base_fb_prev
                rep['base_glitch'] += 1
                if rep['base_glitch'] <= 3:
                    log('  [base] p6 回读跳变 %.0f→%.0f（>60 count），本拍沿用上一拍'
                        % (base_fb_prev, _b))
            else:
                base_fb_prev = _b
        j_cur_m = from_fields(f_raw)
        p1_read = float(f_raw[0])
        if cur == 'descend':
            tip = tip_open_m(j_cur_m)
            s_ach_now = standoff_along_axis(tip, O_u, prev_alpha)
            step = max(cfg.s_min_step, cfg.s_frac * max(s_ach_now, 0.0))
            # ★★ 留量指令从**自己上一拍**单调往下走，回读只用来"别领先太多"（2026-10-01 复查）。
            #   旧写法 `s_cmd = s_ach_now − step`：回读滞后 0.2~0.4s ⇒ `s_ach_now` 是臂**过去**
            #   的位置 ⇒ 指令被算到臂已经走过的地方之上 ⇒ 臂被往回拽（带延迟的反馈环，又慢又顶）。
            #   `min(s_cmd, s_ach_now)`：臂真比指令还低时以回读为准，不往回拉。
            if s_ach_now - s_cmd < cfg.s_lead_m:
                s_cmd = max(cfg.s_stop_m, min(s_cmd, s_ach_now) - step)
        elif cur == 'lift':
            # 抬起 = 沿 −ẑ 把留量从 s_stop 加到 s_stop+lift_m（目标点固定，不追）
            s_cmd = cfg.s_stop_m + cfg.lift_m
        try:
            if cur == 'close':
                # 合爪：四个关节**冻住**在进相时的位置；p1 **慢慢合**（每拍 close_step counts）。
                # ⚠️ 2026-10-01 第一次真抓我把它一次写到 578 ⇒ 舵机**全力合到底**，用户明确否决
                #    （"不能这样"）。现在慢合 + 读回一停就冻结（见下面的停止判据）。
                if hold_j is None:
                    hold_j = dict(j_cur_m)
                    # ★ 判"碰上东西"之前必须先**起振**（2026-10-01 真机第三跑踩到）：
                    #   这个固件的夹爪回读滞后很大（实测指令 240→354 走了 0.4s，读回还趴在
                    #   234 不动）⇒ 一进 close 相就"读回不动 = 碰上东西"，其实它只是**还没开始动**。
                    #   所以只有读回**真的动过**（离起点超过一个死区宽）之后，卡住计数才算数。
                    # ⚠️ p1=0 = **该拍没读到**（协议约定），不是位置 0：
                    #   2026-10-21 真机 close 相读到 0（令 553 那拍起），旧代码把
                    #   "0 和 0 差 <5"当成"连续 3 拍不动 = 碰上东西"，提前冻结。
                    p1_start = p1_read if p1_read > 0.0 else cfg.gripper
                    close_armed = False
                if close_cmd is None:
                    # 起点 = **真回读**；没读到就用张开位（cfg.gripper）当起点，
                    # 绝不拿 0 起步（那会让 p1 先"从 0 往 578 爬"）。
                    close_cmd = p1_read if p1_read > 0.0 else cfg.gripper
                close_cmd = min(cfg.close_field, close_cmd + cfg.close_step)
                tgt = Target(prev_alpha, cfg.s_stop_m, dict(hold_j),
                             to_fields(hold_j, close_cmd, cfg.wrist_roll),
                             0.0, 0.0, True, 0.0, 0.0)
            else:
                # ★ 下扎必须是一条**直线**（2026-10-01 实测：α 漂 7° ⇒ 轴线转向
                #   ⇒ 爪尖横向偏 4.6cm，夹到瓶盖后面）。2026-10-28 起 aim 选出的
                #   α 已经通过**整段路径**检查，所以下扎可以**钉死这个锚**；只有
                #   采样间隙导致中间点无解时，才在 `pick_target_descend()` 里
                #   0°→3°→7°→14°→全包络逐级放宽，并更新锚。
                # aim 贴住之后也把搜索窗收到 ±alpha_freeze_span：否则 O 虽冻结，
                # 候选仍会随余量排序换来换去。
                if cur == 'descend' and descend_alpha is None:
                    # 单独跑 --phase descend：没有 aim 给的锚。只在**第一拍**用
                    # 全包络选一个整段可行的 α，选完立即锁死；之后不再换解。
                    tgt = pick_target(O_u, cfg, s_cmd, prev_alpha,
                                      span=None, j_prev=j_cur_m)
                    descend_alpha = tgt.alpha
                elif cur == 'descend':
                    # 硬冻结：只接受进相时的 α；解不出就抛给下面的“就地合爪”兜底，
                    # 绝不换一个 α 继续扎（2026-10-30：−86 漂到 −81 后偏 17.7mm）。
                    tgt = pick_target_descend(O_u, cfg, s_cmd, descend_alpha,
                                              j_prev=j_cur_m)
                else:
                    span = (None if (cur == 'aim' and not aim_hold)
                            else cfg.alpha_freeze_span)
                    # ★ 抬起只走 s_stop → s_stop+lift，不该要求整段下扎路径都可行。
                    #   2026-10-30 真机：用默认路径检查把抬起解误拒成 REFUSED。
                    s_samples = None
                    if cur == 'lift':
                        # 从**当前实际留量**开始查，不从 s_stop 开始：
                        # 下扎可能停在 s_stop 之上（实测 3.9mm），要求它先回到
                        # 3mm 再抬是多余的，而且可能把本来可行的抬起解拒掉。
                        s_now = standoff_along_axis(
                            tip_open_m(j_cur_m), O_u, prev_alpha)
                        s_lo = min(s_cmd, max(cfg.s_stop_m, s_now))
                        s_samples = path_samples(cfg, s_lo=s_lo, s_hi=s_cmd)
                    tgt = pick_target(O_u, cfg, s_cmd, prev_alpha,
                                      span=span, j_prev=j_cur_m,
                                      s_samples=s_samples)
        except Refused as e:
            # ★★ 下扎已经到 s_stop 地板、几何上解不出更深的姿态，但**爪尖已经进合爪窗口**：
            #   这不是"这轮抓不了"，而是"机械极限先到了" ⇒ 直接进合爪。
            #   2026-10-21 真机：s=25mm 处 α=−79° 无解（shoulder 要 872/875），
            #   但臂实际停在 40.6mm、爪尖离目标点 16.2mm，爪已套在黄椒上。
            #   旧行为是整轮 refuse、臂已经到位却不合爪 —— 用户明确说这是 bug。
            # ★★★ 2026-10-21 用户：「位置刚好，直接夹就行，防御措施导致失败没必要」。
            #   之前只在下扎到 s_stop 地板、且爪尖离目标 ≤ close_reach 时才兜底合爪；
            #   真机 s=16mm 处无解（臂已对准、爪尖离目标 13.7mm）却整轮 refuse。
            #   现在：**下扎途中任何一处解不出更深的姿态 ⇒ 就地合爪**，不再看距离。
            if cur == 'descend':
                T_stop = axis_point(O_u, cfg.s_stop_m, prev_alpha)
                err_now = math.dist(tip_open_m(j_cur_m), T_stop)
                # 给 grasp_server 判“这轮是否真的到位”：偏差大时不能报 OK。
                rep['descend_fallback'] = True
                rep['fallback_err_m'] = err_now
                if not advance_or_stop(
                        '下扎到机械极限：锚定 α 在 s=%.1fmm 处无可行姿态（%s）；'
                        '爪尖离 s_stop 目标点 %.1fmm ⇒ 就地合爪'
                        % (s_cmd * 1000.0, e, err_now * 1000.0)):
                    break
                continue
            # 2026-10-28：aim 相现在默认扫**整个包络**，不再需要“窗口无解 → 铺满
            # 包络重试”。预抓位解不出就直接下扎兜底，别整轮放弃（2026-10-22 用户：
            # 瓶盖 25cm 实测够得着，aim 留量处解不出却报“够不着”）。
            if cur == 'aim':
                log('  [aim] 预抓位解不出（%s）⇒ 直接下扎兜底' % e)
                if not advance_or_stop('refuse：接近相解不出预抓位（直接下扎兜底）'):
                    break
                continue
            else:
                rep['stopped'] = 'refuse：%s' % e
                break
        if cur != 'close':
            prev_alpha = tgt.alpha
        # 合爪/抬起期间**夹爪保持闭合**（抬起时张开 = 把东西放回去 ✗）
        if cur == 'close':
            grip_cmd = close_cmd
        elif cur == 'lift':
            # ★ 沿用合爪收尾时的指令（碰上东西冻结在哪就是哪），**不再拉到 close_field**。
            #   单独跑 `--phase lift`（没走过 close）⇒ 保持进相时的真回读，不改夹爪状态。
            if grip_hold is None:
                grip_hold = p1_read
            grip_cmd = grip_hold
        else:
            grip_cmd = cfg.gripper
        phase_ticks += 1

        # ④ 限速参考 + 发
        # ★ 判据一律用**回读**算出来的实际量：`tgt.err_m` 是"目标姿态自己到 O 的距离"，
        #   它恒等于 s（因为姿态就是按 T=O+s·(−ẑ) 解出来的），拿它当收敛判据 ⇒ 永远不收敛。
        j_cur = from_fields(trace.at(link.now(), 'auto'))
        tip_act = tip_open_m(j_cur)
        T_goal = axis_point(O_u, s_cmd, tgt.alpha)
        err_act = math.dist(tip_act, T_goal)
        s_ach = standoff_along_axis(tip_act, O_u, tgt.alpha)
        # ★ 贴近 s_pre 之后冻结 O/α（见 cfg.aim_hold_m）：第一次进窗口就锁住，
        #   之后臂在一个固定的目标点上收敛。窗口外的接近段照常每拍更新。
        if cur == 'aim' and not aim_hold and err_act <= cfg.aim_hold_m:
            aim_hold = True
        # ★★ 限速的**起点**用上一拍的指令（未补偿的那份），不用回读（2026-10-01 复查）：
        #   回读滞后 ~0.2~0.4s，从回读起步 ⇒ 每拍指令都被拽回臂"过去"的位置（和上面 s_cmd
        #   是同一个病）。指令领先回读超过 `lead_max_counts`（臂跟不上/被挡住）⇒ **原地等**
        #   （指令保持上一拍），不是退回从回读起步 —— 那样指令会往回跳（实测 0.4s 滞后下来回顶）。
        #   被挡住时回读不动 ⇒ 照样由"卡住"判据接管。合爪相照旧从回读起步（关节本来就冻住）。
        #   舵机死区也由此自然跨过：小步在指令上累积，攒够死区宽度舵机就动。
        if j_prev_cmd is None or cur == 'close':
            j_cmd, ratio, reached = limit_step_floor(j_cur, tgt.joints,
                                                     cfg.max_step_deg, cfg.min_step_deg)
        else:
            f_prev_cmd = to_fields(j_prev_cmd, 0.0, 0.0)
            f_rb = to_fields(j_cur, 0.0, 0.0)
            if max(abs(f_prev_cmd[k] - f_rb[k]) for k in (2, 3, 4, 5)) <= cfg.lead_max_counts:
                j_cmd, ratio, reached = limit_step_floor(j_prev_cmd, tgt.joints,
                                                         cfg.max_step_deg, cfg.min_step_deg)
            else:
                j_cmd, ratio, reached = dict(j_prev_cmd), 0.0, False
        j_prev_cmd = dict(j_cmd)
        # ★★ 判据查的是**目标**的 field，不是这一拍的**指令**：
        #   实机开场姿态可能是"歇在机械限位上"的 —— 2026-09-30 实测开机 p4=121，**在固件下限 125 之下**，
        #   于是第一拍的限速指令必然也在界外。拿指令当判据 ⇒ 整条链在第一拍就 refuse（永远动不了）。
        #   界外的**指令**直接夹进 [125,875] 再发：固件本来就会夹，我们先夹一遍是为了
        #   让自己发出去的值与后续回读一致（否则白挨一次"目标 field 出界"）。
        # （`pick_target` 已保证候选 field 在界内 ⇒ 下面这条正常**永不触发**，留着当不变式断言）
        f_tgt = to_fields(tgt.joints, grip_cmd, cfg.wrist_roll)
        # ★ 合爪相的"目标"四个关节取自**回读**（hold_j = j_cur_m），而臂可能歇在
        #   机械限位上（实测 p4=121~122，低于固件下限 125，见上面 2026-09-30 那条）。
        #   2026-10-22 真机：aim 无解 → 兜底直接进 close → 这里拿 122 当"目标出界"整轮拒绝，
        #   可指令本来就会在下面被夹进 [125,875] ⇒ 对 close 相跳过这条断言。
        if cur != 'close' and not all(FIELD_LO <= v <= FIELD_HI for v in f_tgt[2:5]):
            rep['stopped'] = 'refuse：目标 field 出界 %s' % ['%.0f' % v for v in f_tgt[2:5]]
            break
        fields = to_fields(j_cmd, grip_cmd, cfg.wrist_roll)
        for i in (2, 3, 4):                    # 只夹 p3/p4/p5；p1/p2(夹爪/自转)、p6(底座 ±1000) 原样带过
            fields[i] = min(FIELD_HI, max(FIELD_LO, fields[i]))
        # ★★ 底座外环补偿（2026-10-01 查明"偏左 1cm"的根源）：底座是 **motor 模式速度环**，
        #   速度 0 时还是松的，实测它**总差 ~10 count 到不了指令位置**（令 −227.7 / 读 −217.0
        #   = 3.85°，在 20cm 半径上正好 1.34cm）。做法：把"指令 − 回读"的差按增益补回指令，
        #   直到回读真的到位。⚠️ 用的是**上一拍**的回读（f_now 在本拍更后面才算）—— 外环慢，
        #   这是刻意的；增益 <1 也是刻意的（底座那边还有它自己的速度环，别一起震荡）。
        if cfg.base_comp > 0.0 and f_now is not None and len(f_now) >= 6:
            err6 = fields[5] - f_now[5]
            corr = max(-cfg.base_comp_max, min(cfg.base_comp_max, cfg.base_comp * err6))
            fields[5] = max(-1000.0, min(1000.0, fields[5] + corr))
        # ★★ 肩/肘/腕的**位置环下垂补偿**（2026-10-01 第四跑查明）。
        #   实测稳态下三关节的回读**一致地越过指令**：令 383.6/读 392.0、令 209.0/读 215.0、
        #   令 796.3/读 813.0 ⇒ +8.4/+6.0/+16.7 count（p5 那 16.7 = 4.0°）。臂伸到桌面下方时
        #   重力一直拽着，舵机位置环就停在"差一点"的地方 ⇒ **爪尖系统性差 1.5~2.2cm**，
        #   用户看到的就是"总夹偏、还偏左"。**这不是手眼标定误差，别再去标手眼了。**
        #   做法和"每拍跟"不一样：**只在 [aim]→[descend] 交接处量一次**（那时臂已经停稳、
        #   目标也不动了，量到的就是纯下垂），之后当下扎/抬起的**常量**带着走。
        #   为什么不做成每拍积分：第一版那么写，`test_loop_converges_to_the_object` 的爪尖
        #   残差从 <10mm 变成 11.8mm —— 积分器和"回读落在死区内连 3 拍算到位"那条判据互相打架，
        #   到位瞬间还在往前推。交接处一次量、全程常量，既没有这个耦合，也没有积分饱和。
        #   ⚠️ 只补 p3/p4/p5：p1 是夹爪、p2 自转恒定、p6 底座已有 `base_comp`。
        #   ⚠️ 单独的 `--phase descend` **学不到** comps（没走 aim），只有 `all`/`aim` 才学。
        if comps is not None and cur in ('descend', 'lift'):
            for _n, _i in enumerate((2, 3, 4)):
                fields[_i] = min(FIELD_HI, max(FIELD_LO, fields[_i] + comps[_n]))
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
        rep['comp'] = comps
        rep['rows'].append((now, err_act, s_ach, tgt.alpha,
                            tgt.slack_path if tgt.slack_path is not None else tgt.slack,
                            0.0 if obs is None else 1.0))

        # ⑤ 判据 / 阶段切换（全用实际量）
        # ★★ "发散"判的是**臂有没有在动**，不是"到目标的距离有没有单调下降"。
        #   2026-09-30 实机实测：`limit_step` 按"最大行程那个关节"等比缩放 ⇒ 小行程关节每拍
        #   只走 ~1 count（**落进舵机死区 ⇒ 它一动不动**）；于是肩/肘在按时腕卡住，爪尖走成
        #   一条弧线，**到目标点的距离反而从 27.8 → 30.2mm 涨** ⇒ 旧判据把它误报成"发散"、
        #   在第 25 拍停手（臂其实还在动）。多关节 + 每拍限速下"距离先增后减"是**正常**的。
        #   正确判据：**回读的 field 有没有超过 1 count 地变化**（= 臂还在不在跟）。
        f_now = to_fields(j_cur, grip_cmd, cfg.wrist_roll)
        moved = (999.0 if f_prev is None
                 else max(abs(f_now[k] - f_prev[k]) for k in range(6)))
        f_prev = f_now
        # ★★ 到位判据要**认硬件的死区**：2026-09-30 实测，命令收敛到目标之后，
        #   **回读稳定地差 ~5 count（≈1.2°/关节）**就再也不动了 —— 那是 LX 舵机带载（臂自重）
        #   下的位置环死区稳态误差，是硬件极限。拿"tip 误差 ≤ 2mm"当判据 ⇒ 永远不满足、
        #   白跑到超时（实测 40s）。正确判据：**回读落在目标的死区量级内连续 3 拍**。
        lag = max(abs(f_now[k] - f_tgt[k]) for k in range(2, 6))     # p3..p6 实际 vs 目标
        # ★★ 下扎相：只有**指令已经到了 s_stop** 之后 `near` 才开始数（2026-10-01 复查）。
        #   下扎的 `tgt` 是**每拍的中间点**（只比当前位置往前 2~5mm），不是终点 ⇒ 离目标
        #   1~2cm 时每拍步长本身就 < deadband_counts，臂只要平稳地跟，`near` 就成立 ⇒
        #   **半路宣布扎到位**。假臂加回读滞后实测：真实停在 ~8mm（自报 10~11mm）；
        #   真机下扎收尾残差 9.5~16.6mm 随机，大概率就是它（不是舵机死区）。
        floor_ok = cur != 'descend' or s_cmd <= cfg.s_stop_m + 1e-9
        near = near + 1 if (lag <= cfg.deadband_counts and floor_ok) else 0
        if cur in ('aim', 'descend'):     # 合爪/抬起时臂本来就该几乎不动 ⇒ 那两相不判"卡住"
            if moved >= 1.0:              # 1 count = 回读的量化单位 ⇒ 任何真实运动都能清掉它
                stall = 0
                if best is None or err_act < best:
                    best = err_act
            else:
                stall += 1
        if cur == 'aim' and (err_act <= cfg.tol_m or near >= 3):
            if not advance_or_stop('对准完成：爪尖离目标点 %.1fmm（余量 %.1fcm，α=%.1f°；'
                                   '回读与目标差 %d count ⇒ 已到舵机死区边界）'
                                   % (err_act * 1000, s_cmd * 100, tgt.alpha, round(lag))):
                break
            continue
        # ★ 判据带 `min_gain_m`(0.5mm) 容差，**不是**裸的 `s_ach <= cfg.s_stop_m`：
        #   上面的 `s_cmd = max(cfg.s_stop_m, ...)` 把**指令**地板钉死在 s_stop_m 上，而
        #   `s_ach` 是拿**当前**的 est.O 量的（观测随臂移动而变 ⇒ O 的 EWMA 一直在动）
        #   ⇒ 实测稳定停在 **3.012~3.037mm**，比地板高几十微米，`<= 0.003` 永远差一点点，
        #   于是被 stall 判据误报成"发散"（爪尖其实就停在离目标 3mm 处）。
        #   容差用 `min_gain_m`（"小到算没进展"的那个尺度，0.5mm）：既覆盖 O 的抖动
        #   （几十微米，13 倍余量），又不会提前开闸（实测在 3.016mm 处断，不是 4.8mm）。
        # ★ 下扎到位 = 两种之一：① 沿轴留量真的到 s_stop（理想）；② **回读落进舵机死区量级
        #   连续 3 拍**（= 硬件给得出的极限）。第二条是 2026-10-01 第六跑补的：那次 `s_ach` 停在
        #   **5.7mm**，离 3.5mm 的判据只差 2mm，而舵机死区让它**再也动不了** ⇒ 25 拍后
        #   被"卡住"判死、白跑一整轮。接近相**一直**用第二种判据（`tol_m` 那条比硬件还细，
        #   真机永远满足不了），下扎相漏了。**别再把这条删掉。**
        if cur == 'descend':
            at_floor = s_ach <= cfg.s_stop_m + cfg.min_gain_m
            # ★ 必须**两个条件同时成立**：沿轴到 s_stop / 回读进死区，**且**实际 3D
            #   误差 ≤ 10mm。只看 s_ach 会把“横向偏 2cm”误判成扎到位（2026-10-30）。
            if (at_floor or near >= 3) and err_act <= DESCEND_ERR_TOL_M:
                if not advance_or_stop('扎到位：实际余量 %.1fmm（爪尖离目标 %.1fmm；%s）'
                                       % (s_ach * 1000, err_act * 1000,
                                          '留量到 s_stop' if at_floor
                                          else '回读已达舵机死区极限 %d 拍' % near)):
                    break
                continue
        if cur == 'close' and close_cmd is not None and close_cmd >= cfg.close_field - 1.0 \
                and p1_read >= cfg.close_field - cfg.deadband_counts:
            # ★ **读回也到了底**才算"合到底"。判据必须查读回、不能只查指令：2026-10-01 第三跑
            #   指令早就到 578，读回还停在 382（爪里有东西 / 卡住），拿指令收尾就会把这种情况
            #   报成"夹空了"（上一次真抓就是这么被骗的）。而且**读回到底也推不出"夹空"**：
            #   实测白条被夹住了、读回照样走到 578 ⇒ 这里只报事实，不下"夹空"的结论。
            if not advance_or_stop('合爪完成：p1 令 %.0f / **读 %.0f**（读回也到底了 ⇒ '
                                   '爪里没东西 / 夹着一个细物体**都可能**，本判据分不出来）'
                                   % (close_cmd, p1_read)):
                break
            continue
        if cur == 'close':
            # 读回连续 close_stall 拍几乎不动（每拍变化 <5 count）= 指尖碰上东西了 ⇒ **冻结**
            # ⚠️ 这里比的必须是 `p1_read`（真回读）。用 `f_now[0]`（= 每拍 +close_step 的指令）
            #    ⇒ 差值恒为 close_step ⇒ 永远数不到 close_stall ⇒ 冻结永不触发。
            # ⚠️ 而且要 `close_armed` 之后才算（见 `p1_start` 那段：回读滞后很大，
            #    没起振就判"不动"是误触发）。
            # ★ 起振门槛用 **close_step（30 count）**，不是 deadband_counts（10）。
            #   ⚠️ 这是**更保守的调参，不是已证实的修复** —— 老实说：
            #   真机 2026-10-01 第六跑出现"`p1 令 510 / 读 240` 却报碰上东西"（读回压根没动），
            #   说明 10 count 的门被跨过了；但我在假臂上**复现不出来**
            #   （加了 0.6s 滞后 + ±8 count 噪声的模型，两种门槛都不会误触发）⇒
            #   **没有能红的测试覆盖这条**。留着它是因为它只可能更保守（要求夹爪真的跟了一步），
            #   代价是"移动不到 30 count 就真碰上"的情况会漏判、退到合到底/兜底（不会过力）。
            #   真因待查：下次上机要抓 `close` 相每拍的 (令, 读) 原始对。
            # ⚠️ p1_read = 0 是**该拍没读到**（协议约定），不是"爪停在 0"：
            #   2026-10-21 真机 close 相从令 553 那拍起读回恒 0，旧代码按
            #   "0 与 0 差 <5"数满 3 拍 ⇒ 误报"碰上东西"、提前冻结。
            #   没读到的样本**不参与**起振/卡住判据，只让合爪指令继续按步长爬。
            if p1_read > 0.0:
                if not close_armed and abs(p1_read - p1_start) > cfg.close_step:
                    close_armed = True      # ★ 读回真的跟了一步，卡住计数从这里才开始算
                if close_armed and p1_prev is not None and abs(p1_read - p1_prev) < 5.0:
                    close_stuck += 1
                else:
                    close_stuck = 0
                p1_prev = p1_read
            else:
                close_stuck = 0
            if close_stuck >= cfg.close_stall:
                if not advance_or_stop('合爪完成：读回连续 %d 拍不动（p1 令 %.0f/读 %.0f）'
                                       '⇒ 碰上东西、冻结不再加压'
                                       % (close_stuck, close_cmd, p1_read)):
                    break
                continue
        if phase_ticks >= cfg.close_ticks and cur == 'close':
            if not advance_or_stop('合爪超时兜底：走了 %d 拍（p1 令 %.0f / **读 %.0f**）'
                                   '—— 读回没到底 ⇒ 爪里有东西或舵机卡住，'
                                   '**别当"夹空"**' % (phase_ticks, close_cmd, p1_read)):
                break
            continue
        if cur == 'lift' and s_ach >= cfg.lift_m * 0.8:
            # 抬起来了：沿轴留量从 s_stop 涨到 lift_m 的 80%
            if not advance_or_stop('抬升完成：沿轴留量 %.1fcm（目标 %.1fcm）'
                                   % (s_ach * 100, cfg.lift_m * 100)):
                break
            continue
        if cur == 'descend' and s_ach <= cfg.s_stop_m + cfg.min_gain_m \
                and err_act > DESCEND_ERR_TOL_M and stall >= cfg.patience:
            # 已经到 s_stop 地板、但 3D 偏差仍 >10mm，且臂连续 patience 拍不动：
            # 不是“扎到位”，是横向/机械极限。就地合爪并让 server 报未确认抓住。
            rep['descend_fallback'] = True
            rep['fallback_err_m'] = err_act
            if not advance_or_stop(
                    '下扎到机械极限：s=%.1fmm；爪尖离目标 %.1fmm（>10mm）且 %d 拍不动 ⇒ 就地合爪'
                    % (s_cmd * 1000.0, err_act * 1000.0, stall)):
                break
            continue
        if stall >= cfg.patience:
            rep['stopped'] = '卡住：连续 %d 拍**臂没动**（回读 field 变化 <1 count；现在 %.1fmm，最好 %.1fmm）' % (
                stall, err_act * 1000, (best or 0.0) * 1000)
            break
        if cur in ('aim', 'descend') and obs_age is not None \
                and obs_age > cfg.obs_lost_s:
            rep['obs_lost'] = True
            if cur == 'aim':
                if cfg.require_depth and rep['depth_refused']:
                    rep['stopped'] = (
                        '没有实测深度（require_depth=True）：已拒绝 %d 次 '
                        'z=%.1fmm 平面兜底；观测丢失 %.1fs'
                        % (rep['depth_refused'], cfg.z_plane_m * 1000.0, obs_age))
                else:
                    rep['stopped'] = '观测丢失 %.1fs（接近阶段丢了就停下）' % obs_age
                break

    rep['ok'] = bool(rep['stopped']) and rep['stopped'].startswith(
        ('对准完成', '扎到位', '合爪完成', '抬升完成'))
    return rep
