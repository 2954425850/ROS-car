# -*- coding: utf-8 -*-
"""视觉伺服：把爪尖驱到瓶盖上。**纯算术 + 一个可注入 I/O 的循环，不碰硬件。**

## 控制律（只有两条式子）
设 `C` = 光心、`d` = 「当前观测像素」的射线单位方向、`P` = 该射线∩瓶盖顶面、
`L` = ‖手眼‖ = 0.124m（张开态）、`s` = **沿射线的留量**（爪尖离瓶盖还有多远）：

```
目标 T = P − s·d           # 爪尖该待的地方
对准:  s ← max(0, |P−C| − L)    # ★ 见下面「为什么」
下扎:  s ← s − 每拍步长（压到 0）
```

然后 **在 alpha 上做一维搜索**：挑一个让「模型预测的像素误差」最小的可行 alpha。

## 为什么对准阶段的 s 取「|P−C| − L」而不是「保持高度」或「直接放到 P」
`|爪尖 − 光心| ≡ L` 是**刚体的**（相机拧在腕上）。所以「爪尖落在射线上」这个条件
**等价于**「把爪尖放到射线上离光心 L 处」，即 `s = |P−C| − L`。

⚠️ **这一条是被实测打出来的**。原先我用「目标 = 射线 ∩ 水平面 z=当前高度」，
在模拟里 20 多拍都收不干净、末段还过冲；原因是它有一个**退化的平移方向**：
把整条臂平着挪 δ，光心和爪尖一起挪，那条「射线∩水平面」的目标点也挪 δ
（水平分量全抵消）——观测误差**一点不降**。只有靠腕子转一点点才缓慢收敛
（实测每拍只缩 0.9 倍）。换成 `T = C + L·d` 之后：把爪尖放上去，光心**几乎不动**
（因为 `C = T − L·ĥ`，而 `ĥ` 被 alpha 搜索转到了 `d` 上）⇒ 4 拍收敛到 1mm。

## 三个必须记住的坑（都踩过）
1. **单位**：`arm_kin.ikine()/fk()` 收/吐 **cm**，`geom` 吐 **m**。
   本模块内部**一律 m**，只在调 `ikine` 的那一行换算。交接文档 §3 的伪码
   `find_grasp_solution(P.x, P.y, P.z)` 直接照抄会差 100 倍。
2. **张开态**：`ikine()` 内部固定用 L4 = 17.7（夹紧态），而伺服期间夹爪是**张开**的
   （240）：物理爪尖比它沿 ẑ 后退 1.7cm ⇒ 必须走 `ik_open_m()`。不做这步差 1.7cm
   —— 约一个瓶盖半径，够让爪子夹空。
3. **瞄准像素在画框最下沿**（1280x720 里 v = 709.8/720）。瓶盖对上去那一刻它等效半径
   约 139px、中心就在 710 ⇒ **下面一大截在画外**。所以**蓝块质心当特征是收敛不了的**：
   会停在约 60px ≈ 7mm 的假偏差上（而且偏多少随距离变）。特征改用
   「包围盒上沿 + 半宽 × |cos 倾角|」估圆心（`cap_center()`）—— 与裁剪无关。

## 这一层不碰硬件
`observe()` / `command()` 由调用方注入：**测试里注入模拟被控对象，工具里注入 ROS + K230**。
两边跑的是**同一份循环代码**，所以本地测试绿了才有意义。
"""
import math
from collections import namedtuple

from . import geom
from .arm_kin import (FIELD_HI, FIELD_LO, Unreachable, fk, from_fields, ikine,
                      to_fields)
from .cam_model import tool_axes

JOINT_KEYS = ('base', 'shoulder', 'elbow', 'wrist_pitch')

# 张开态爪尖比 `fk`/`ikine` 的夹紧态爪尖沿 ẑ 后退多少（cm）
L4_BACK_CM = geom.L4_CLOSED - geom.L4_OPEN          # = 1.7

# 射线太平就拒（与 geom.target_from_pixel 同一个门限）
MIN_DZ = 0.3

STREAM_W, STREAM_H = geom.STREAM_W, geom.STREAM_H


class ServoRefused(Exception):
    """伺服**主动拒绝**继续 —— 绝不"算不出来就塞个垃圾值"。"""


# --------------------------------------------------------------------------
# 纯函数（一律 m）
# --------------------------------------------------------------------------

def tip_open_m(joints):
    """**张开态**爪尖在基座系下的坐标，米（z = 0 = 车体安装面）。"""
    tip_cm, axis = fk(joints)
    xh, yh, zh = tool_axes(tip_cm, axis)
    return tuple((tip_cm[i] - L4_BACK_CM * zh[i]) / 100.0 for i in range(3))


def ik_open_m(x_m, y_m, z_m, alpha_deg):
    """把**张开态**爪尖放到 (x, y, z) **米** 的关节角。坑 #2 的正解。

    做法：先沿 ẑ(alpha) 把目标推 1.7cm 换成"夹紧态的点"，再交给 `ikine`（它要 cm）。
    """
    x, y, z = x_m * 100.0, y_m * 100.0, z_m * 100.0
    a = math.radians(alpha_deg)
    tb = math.atan2(y, x)
    return ikine(x + L4_BACK_CM * math.cos(a) * math.cos(tb),
                 y + L4_BACK_CM * math.cos(a) * math.sin(tb),
                 z + L4_BACK_CM * math.sin(a),
                 alpha_deg)


def fields_in_range(joints, gripper, wrist_roll):
    """(是否在固件钳位内, fields)。只有 p3/p4/p5 受钳位 —— 与 `pose_is_safe` 同判据。"""
    f = to_fields(joints, gripper, wrist_roll)
    return all(FIELD_LO <= v <= FIELD_HI for v in (f[2], f[3], f[4])), f


def field_slack(fields):
    """腕/肘/肩这三条离固件限位还有多少 count —— 越小越贴边。

    ★ 贴边就意味着**臂已经到工作空间边缘**：那次上机肘关节贴在 −90°（p4=125=下限），
      指令到 −88° 也走不到，回读饱和 ⇒ 伺服再怎么算也到不了目标。
    """
    return min(min(v - FIELD_LO, FIELD_HI - v) for v in fields[2:5])


def cap_point(joints, u_ai, v_ai, z_plane_m):
    """观测像素 → (瓶盖点 P, 单位射线方向 d, 光心 C)，米。P 与 d 共线，P 在 z_plane 上。

    ⚠️ `target_from_pixel` 会拒掉太平的射线；这里也一样，把它的 ValueError 换成
    `ServoRefused`（伺服语义：停，而不是崩）。
    """
    try:
        P, _, _ = geom.target_from_pixel(joints, u_ai, v_ai, z_plane_m)
    except ValueError as e:
        raise ServoRefused('算不出瓶盖点：%s' % e)
    C, d = geom.pixel_ray(joints, u_ai, v_ai)
    if d[2] > -MIN_DZ:
        raise ServoRefused('射线太平（d_z = %.3f），深度不可用' % d[2])
    return P, d, C


def standoff_target(P, d, standoff_m):
    """目标 = 射线上、离瓶盖 `standoff_m` 的那一点。"""
    return tuple(P[i] - standoff_m * d[i] for i in range(3))


def aim_standoff(P, C):
    """对准阶段的留量 = |P−C| − L（爪尖待在"它在射线上本该在的地方"）。"""
    return max(0.0, math.dist(P, C) - geom.handeye_range(closed=False))


def predicted_err_px(joints, P, aim_px):
    """模型预测：把臂摆成这样之后，瓶盖中心会落在哪、离瞄准像素多远（1280x720 系）。"""
    u, v = geom.project(joints, P)
    su, sv = geom.to_stream(u, v)
    return math.hypot(su - aim_px[0], sv - aim_px[1])


Pose = namedtuple('Pose', 'err_px alpha standoff_m joints fields slack')


def pick_pose(P_m, d, aim_px, s_lo, s_hi, prefer, gripper=240.0,
              wrist_roll=496.0, lo=-88.0, hi=-40.0, span=14.0, step=1.0,
              n_standoff=5, tie_px=5.0, min_slack=40.0):
    """在 **(沿视线的留量 s) × (alpha)** 两个自由度上搜一个姿态。

    返回 `Pose(err_px, alpha, standoff_m, joints, fields, slack)`。

    ## 为什么必须两维（★ 2026-09-29 第一次上机栽在这里）
    只搜 alpha 时，「爪尖钉在射线上离光心 L 处」这个约束会把臂逼到**工作空间边缘**：
    实测肘关节一路贴死在 −90°（p4=124，固件下限 125），指令到 −88° 也走不到，
    回读饱和 ⇒ 伺服算得再对也到不了位，误差停在 ~120px 不动。

    **放开 s**（= 允许臂沿视线重新分配姿态，人话叫"提腕压肩"）之后，同一个目标点
    周围有一整族姿态都能满足对准条件，余量差 20 倍：

        alpha −40° → p4=153 余量  28      ← 钉死 s 时循环就锁在这
        alpha −44° → p4=197 余量  72
        alpha −48° → p4=247 余量 121

    挑选规则：先卡**离固件限位 ≥ min_slack**（这个是硬的 —— 贴边的姿态根本执行不了），
    再在里面挑预测像素误差最小；误差只差 tie_px 以内的，挑余量最大的。

    ⚠️ 顺带排除过"另一个肘分支"（`ikine` 只给 k2<0 那支）：k2>0 那支 p3 会掉到 0 以下，
    机械上不可用 —— 所以不是漏了分支，是漏了**留量**这个自由度。
    """
    cands = []
    ns = max(1, int(n_standoff))
    for i in range(ns):
        s = s_lo if ns == 1 else s_lo + (s_hi - s_lo) * i / float(ns - 1)
        T = standoff_target(P_m, d, s)
        k = 0
        while k * step <= span + 1e-9:
            for a in ((prefer,) if k == 0 else (prefer - k * step,
                                                prefer + k * step)):
                if not (lo - 1e-9 <= a <= hi + 1e-9):
                    continue
                try:
                    j = ik_open_m(T[0], T[1], T[2], a)
                except Unreachable:
                    continue
                ok, f = fields_in_range(j, gripper, wrist_roll)
                if ok:
                    cands.append(Pose(predicted_err_px(j, P_m, aim_px),
                                      round(a, 9), s, j, f, field_slack(f)))
            k += 1
    if not cands:
        raise ServoRefused(
            '留量 %.3f~%.3f m × alpha [%.0f°, %.0f°] ∩ [%.0f±%.0f°] 里找不到可行解'
            % (s_lo, s_hi, lo, hi, prefer, span))
    loose = [c for c in cands if c.slack >= min_slack] or cands
    best_e = min(c.err_px for c in loose)
    near = [c for c in loose if c.err_px <= best_e + tie_px]
    near.sort(key=lambda c: (-c.slack, abs(c.alpha - prefer), -c.standoff_m))
    return near[0]


def limit_step(j_cur, j_des, max_step_deg):
    """关节空间限幅：**所有关节按同一比例缩**（保住方向，别各缩各的）。

    返回 (j_cmd, ratio, reached)。ratio < 1 表示这一拍没走到，还得再来几拍。
    """
    d = {k: j_des[k] - j_cur[k] for k in JOINT_KEYS}
    m = max(abs(v) for v in d.values())
    if m <= 1e-12:
        return {k: j_cur[k] for k in JOINT_KEYS}, 0.0, True
    if m <= max_step_deg:
        return {k: j_des[k] for k in JOINT_KEYS}, 1.0, True
    r = max_step_deg / m
    return {k: j_cur[k] + r * d[k] for k in JOINT_KEYS}, r, False


def cap_center(u0, u1, v0, v1, cu, cv, dz,
               w=STREAM_W, h=STREAM_H, margin=2.0):
    """瓶盖**顶面圆心**的像素估计。返回 ((u, v), 用了哪个估计) 或 (None, 'refused')。

    首选「上沿 + 半宽 × |cos 倾角|」：顶面圆盘在画面里投影成椭圆，
    短半轴 = 长半轴 × cos(视线与盘面法线的夹角)，而这个余弦就是射线方向的 |d_z|。

    ★ **为什么不是质心**（两条独立的理由，都实测过）：
    1. **裁剪**：瞄准像素在画框最下沿（v = 709.8/720），瓶盖对上去那一刻底边一定出画，
       质心只按可见部分算 ⇒ 稳定偏 ~60px（≈7mm），而且偏多少随距离变。
    2. **圆柱侧面**：蓝块 = 顶面圆盘 **+ 柱侧面的可见部分**（实拍量到 ~168x210px：
       宽是圆盘直径、高多出 ~59px 的侧面）⇒ 质心被往下拉 ~25px。
       而 `v0`（上沿）和宽度都只由**顶面圆盘**决定，所以这条式子两种情形都对、
       而且是**同一个物理点**（顶面圆心）—— 质心在裁剪前后含义会跳。
    （侧边或上沿也被裁时估不出来 ⇒ 退到质心；连质心都不可靠 ⇒ **拒绝**，不猜。）
    """
    if u0 > margin and u1 < w - 1 - margin and v0 > margin:
        u = 0.5 * (u0 + u1)
        v = v0 + 0.5 * (u1 - u0) * abs(dz)
        if 0.0 <= v <= h - 1.0:
            return (u, v), 'box'
    if (u0 > margin and u1 < w - 1 - margin and v0 > margin
            and v1 < h - 1 - margin):
        return (cu, cv), 'centroid'
    return None, 'refused'


# --------------------------------------------------------------------------
# 配置
# --------------------------------------------------------------------------

ServoConfig = namedtuple('ServoConfig', (
    'cap_z_m',         # 瓶盖顶面高度 —— 下扎的目标面（那 ±1cm 的不确定就在这一个数上）
    'max_step_deg',    # 每个关节每拍最多走多少度
    'alpha0',          # 起始 alpha（deg，负=往下）
    'alpha_span',      # 每拍在 alpha 上搜多宽
    'alpha_tie_px',    # 预测误差差这么点以内，就算"一样好"
    'min_slack',       # 规划姿态**最少**离固件限位多少 count（硬门限）
    'aim_clear_m',     # 对准阶段爪尖离瓶盖至少留多少（留量下界）
    'aim_far_m',       # 对准阶段最多退到离瓶盖多远（留量上界）
    'n_standoff',      # 对准阶段沿视线取几个留量采样点
    'gripper',         # p1：张开 240（**伺服期间必须张开** —— 坑 #2）
    'wrist_roll',      # p2：496（出厂）
    'tol_px',          # 对准判据（1280x720 系）
    'standoff_min_m',  # 下扎每拍至少压掉多少留量
    'standoff_frac',   # 下扎每拍再按比例压掉（几何逼近，末段自动变细）
    'tol_standoff_m',  # 留量剩多少算扎到底
    'patience',        # 连续几拍误差不下降就判发散
    'min_gain_px',     # 算"有进展"的最小改善量
    'max_iters',
    'alpha_lo',
    'alpha_hi',
))
ServoConfig.__new__.__defaults__ = (0.016, 6.0, -58.0, 14.0, 5.0, 40.0, 0.040,
                                    0.120, 5, 240.0, 496.0, 10.0, 0.005, 0.35,
                                    0.003, 4, 1.0, 40, -88.0, -30.0)


# --------------------------------------------------------------------------
# 循环
# --------------------------------------------------------------------------

Iteration = namedtuple('Iteration', (
    'k fb joints fields target_m alpha ratio err_px pred_px standoff_cm '
    's_ach_cm d_cap_cm tip_cap_cm feature tip_m note'))


def _fmt_j(j):
    return '肩%6.1f 肘%6.1f 腕%6.1f 底%7.1f' % (j['shoulder'], j['elbow'],
                                               j['wrist_pitch'], j['base'])


def run_phase(cfg, observe, command, phase='aim', log=print):
    """跑一个阶段，返回报告 dict。

    * `observe() -> (fb, box)`：`fb` = `/arm/feedback` 的 6 个数（已稳过），
      `box` = 瓶盖蓝块的包围盒 `{'u0','u1','v0','v1','cu','cv','n'}`（**1280x720 系**）
      或 **None**（画面里没有瓶盖 —— 这是"停"，不是"猜一个"）。
    * `command(fields) -> (fb, box)`：发一条 `/arm/command`、**等到位**、再观察一拍。

    ★ 每一拍的几何都用**回读**算，不用指令值：回差会被下一拍自动收掉。
    """
    assert phase in ('aim', 'descend')
    aim_px = geom.to_stream(*geom.aim_pixel(closed=False))
    L = geom.handeye_range(closed=False)
    z_plane = cfg.cap_z_m

    rep = {'phase': phase, 'iters': [], 'stopped': None, 'ok': False,
           'aim_px': list(aim_px), 'z_plane_m': z_plane, 'handeye_m': L,
           'best_err_px': None, 'alpha_used': None, 'standoff_cmd_m': None}

    fb, box = observe()
    j = from_fields(fb)
    prev_alpha = cfg.alpha0
    standoff_cmd = None
    best_err, stall = None, 0
    log('  [%s] 起手：%s  爪尖 (%.3f, %.3f, %.3f) m'
        % (phase, _fmt_j(j), *tip_open_m(j)))

    for k in range(1, cfg.max_iters + 1):
        def stop(why):
            rep['stopped'] = why
            log('  [%s] 停：%s' % (phase, why))

        if box is None:
            stop('画面里没有瓶盖 —— 不动')
            break

        # ① 特征像素：先用包围盒中心代理一版射线（只为拿 |d_z| 定盘面倾角）
        u_r = 0.5 * (box['u0'] + box['u1'])
        v_r = 0.5 * (box['v0'] + box['v1'])
        try:
            _, d_ref = geom.pixel_ray(j, *geom.to_ai(u_r, v_r))
        except Exception as e:                               # noqa: BLE001
            stop('代理射线不可用：%s' % e)
            break
        feat, how = cap_center(box['u0'], box['u1'], box['v0'], box['v1'],
                               box['cu'], box['cv'], d_ref[2])
        if feat is None:
            stop('瓶盖被画框裁到估不出中心（%s）—— 不动' % how)
            break

        # ② 瓶盖点 / 留量 / 目标
        try:
            P, d, C = cap_point(j, *geom.to_ai(*feat), z_plane)
        except ServoRefused as e:
            stop(str(e))
            break
        d_cap = math.dist(P, C)
        tip = tip_open_m(j)
        # 爪尖**实际**离瓶盖多远（沿射线方向）—— 下扎的速率限制要用它
        s_ach = -sum((tip[i] - P[i]) * d[i] for i in range(3))

        # ③ 在 (留量 s, alpha) 两维上搜一个姿态
        if phase == 'aim':
            # 对准阶段留量是**自由变量**：[离瓶盖至少留点余量, 最多退到多远]。
            # 中间一整族姿态都满足对准条件，挑的时候优先要**离固件限位远**的
            # （否则臂被顶到边缘就执行不了）。上界**不能**取"爪尖现在待的那条
            # 射线上离光心 L 处"—— 起手比它更近时会塌缩成单点、两维退化成单维。
            s_lo, n_s = cfg.aim_clear_m, cfg.n_standoff
            s_hi = max(s_lo, cfg.aim_far_m)
        else:
            if standoff_cmd is None:
                standoff_cmd = s_ach
                log('  [descend] 起始留量 %.1f cm' % (s_ach * 100))
            # ★ 速率限制在**实际**留量上，不是在上一拍的**指令**上：
            #   臂一步走不到那么远，指令一路往下压就会跑到前面去（实测指令到 0 时
            #   爪尖离盖还有 6.5cm，然后开始在错误的深度上磨）。
            step = max(cfg.standoff_min_m, cfg.standoff_frac * max(s_ach, 0.0))
            standoff_cmd = max(0.0, s_ach - step)
            s_lo = s_hi = standoff_cmd          # 下扎阶段留量是**指令**值，不放开
            n_s = 1
        try:
            pose = pick_pose(P, d, aim_px, s_lo, s_hi, prev_alpha, cfg.gripper,
                             cfg.wrist_roll, cfg.alpha_lo, cfg.alpha_hi,
                             cfg.alpha_span, n_standoff=n_s,
                             tie_px=cfg.alpha_tie_px, min_slack=cfg.min_slack)
        except ServoRefused as e:
            stop(str(e))
            break
        standoff, alpha, j_des, pred, slack = (pose.standoff_m, pose.alpha,
                                               pose.joints, pose.err_px,
                                               pose.slack)
        prev_alpha = alpha
        if phase == 'aim' and standoff_cmd is None:
            standoff_cmd = standoff
        rep['standoff_cmd_m'] = standoff_cmd
        target = standoff_target(P, d, standoff)
        j_cmd, ratio, reached = limit_step(j, j_des, cfg.max_step_deg)
        fields = to_fields(j_cmd, cfg.gripper, cfg.wrist_roll)
        ok = all(FIELD_LO <= v <= FIELD_HI for v in fields[2:5])

        # ④ 指标（全部用观测 + 回读）
        err_px = math.hypot(feat[0] - aim_px[0], feat[1] - aim_px[1])
        tip_cap = math.dist(tip, P)
        it = Iteration(k, list(fb), dict(j), list(fields), list(target), alpha,
                       ratio, err_px, pred, standoff * 100.0, s_ach * 100.0,
                       d_cap * 100.0, tip_cap * 100.0, how, list(tip),
                       'slack=%.0f' % slack)
        rep['iters'].append(it)
        rep['last_slack'] = slack
        log('  [%s] #%02d %s  a=%5.1f° 走%3.0f%%  err=%6.1fpx(预测%6.1f)  '
            '留量 令%5.1f/实%5.1fcm  爪尖离盖%5.2fcm  余量%3.0f  (%s)'
            % (phase, k, _fmt_j(j), alpha, ratio * 100, err_px, pred,
               standoff * 100, s_ach * 100, tip_cap * 100, slack, how))
        rep['s_ach_cm'] = s_ach * 100.0

        if not ok:
            stop('目标 field 出界 %s' % ['%.0f' % v for v in fields[2:5]])
            break

        # ⑤ 停不停
        # ★ 已经进了容差就不算"没进展" —— 否则下扎末段（误差本来就在 0.几 px 抖）
        #   会被发散守卫误杀（实测就是这么红的）。
        if err_px <= cfg.tol_px:
            best_err, stall = min(best_err or err_px, err_px), 0
        elif best_err is None or err_px < best_err - cfg.min_gain_px:
            best_err, stall = err_px, 0
        else:
            stall += 1
        rep['best_err_px'] = best_err
        if phase == 'aim' and err_px <= cfg.tol_px:
            stop('对准完成：偏差 %.1fpx ≤ %.1fpx（≈%.1fmm）'
                 % (err_px, cfg.tol_px,
                    err_px / (geom.K_AI320x180_CHN2.fx * 4.0) * d_cap * 1000)
                 )
            break
        if (phase == 'descend' and s_ach <= cfg.tol_standoff_m
                and err_px <= cfg.tol_px):
            stop('扎到底：实际留量 %.1fmm ≤ %.1fmm（爪尖离盖 %.2fcm）'
                 % (s_ach * 1000, cfg.tol_standoff_m * 1000, tip_cap * 100))
            break
        if stall >= cfg.patience:
            stop('连续 %d 拍误差没下降（现在 %.1fpx，最好 %.1fpx）—— 判发散'
                 % (stall, err_px, best_err))
            break

        # ⑥ 走一拍
        try:
            fb, box = command(fields)
        except Exception as e:                               # noqa: BLE001
            stop('发指令/观察失败：%s' % e)
            break
        j = from_fields(fb)

    rep['ok'] = bool(rep['stopped']) and rep['stopped'].startswith(
        ('对准完成', '扎到底'))
    rep['alpha_used'] = prev_alpha
    return rep
