# -*- coding: utf-8 -*-
"""流式走到一个钉死的目标点上方（**不依赖视觉硬件**）。M2 的验收工具。

    python3 tools/stream_move.py --selftest                # 本地/上机都能跑，不碰硬件
    python3 tools/stream_move.py --dry-run                 # 只读：算一遍打出来，不发指令
    python3 tools/stream_move.py --target 20,-2,1.6 --yes  # 真跑（**会动臂！**）

默认目标点是**名义点** `(0.20, -0.020, 0.016)`（米）= 正前方略偏右 20cm、平台高度。
本步只为验证"连续流式 / 余量 / 限速"，**不依赖瓶盖在哪**（09-29 那个 CAP_TRUE 已作废，
瓶盖挪过位置了）。

为什么偏偏是这个点（2026-09-30 上机前拿 `/arm/feedback` 实测算出来的）：
  当前歇着的姿态 base=-4.0° shoulder=111.8° elbow=-91.0° wrist=-78.0°（爪尖半径 13.1cm、高 8.9cm）
  · `(0,-0.200,0.016)` → 底座要摆 **-86°**（一次 86° 的大横扫，要 17 秒；底座还有 3.7° 松量、
    是速度环 motor 模式，横扫还会扫过台面），而且**从起手姿态相机根本看不见它**（见下）
  · `(0.20,-0.020,0.016)` → 底座只摆 -2°，最大关节动作 19°（4 秒），末态余量 **112 count**（`ok`）、
    预抓取余量 57，肘关节还是**离开**它现在歇着的限位（-91° → -77°）
  第一次动臂不该先来一个 86° 的底座横扫。另外半径扫描也支持这个选择
  （z=0.016、α∈[−88°,−45°] 的末态 slack：r≤16cm 无解、17cm→14(warn)、18cm→46、20cm→110、25cm→163）。

## ★ 这个工具**不用相机**
`grasp.run()` 在 `est.O is None` 时只会**原地不动**（`grasp.py` 那条"还没见到目标"分支），
所以"钉死目标"必须**通过 Obs 喂进去**。本工具用一个**模型相机**（`virt_obs`）：
把目标点 O 用 `geom.project` 投到**当前关节姿态**下，当成"这一刻的观测"，再让
`grasp.estimate_point` 反解回 O。几何上这跟"真相机看着一个静止目标"是同一件事：
同一个 3D 点在任何姿态下投出的像素，过它的射线都穿 O ⇒ 反解恒等于 O（稳定、不漂）。
**它验证的是限速流式 / 余量 / S0 / 几何自洽，不验证视觉链路**（K230、8555、跟踪框
一概不碰；那条在 Task 7~9）。

⚠️ **推论：目标点必须在起手姿态的相机视野里**，否则模型相机给不出观测 ⇒
`run()` 会一直"原地不动"到超时（**看起来像卡住，其实什么都没发**）。实测算过
（2026-09-30 的那个起手姿态）：`(0.20,-0.020,0.016)` 投到像素 **(148, 116)**，落在有效区；
而 `(0,-0.200,0.016)` 投出 **40446** —— 远在画框外，用它这一步会**干等 40 秒**。
所以 `main()` 在**停服务之前**就只读地算一遍这个可见性，看不见就当场喊出来。

## 安全（每条都是踩出来的，照抄 `tools/servo_grasp.py`）
* **先 `systemctl stop ps2-teleop.service`（不许 kill**，`Restart=always` 会被拉起
  → 两个实例抢串口，dropped 疯涨）。停完**核实** inactive 才动臂。
* 停服务会**把驱动一起停掉**（两者同一个 service）⇒ 必须**单独起**
  `l150pro_driver_node`，否则收不到 `/arm/feedback`。
* 停/恢复全部焊在 `try/finally` 里（含 Ctrl-C）—— `tools/jacobian.py` 的隐患
  就是停服务那两步在 `try` 之前，一抛异常服务就留在停止态。**别照抄它。**
* ⚠️ **恢复服务 = 机械臂走 INIT_HOME 归位（会动！）** —— `finally` 里 `_svc('start')`
  那一步就会挥臂。跑之前先把臂周围清空、人也让开；跑完也别马上伸手进去。
* 发指令前把要发的关节 field 打印出来给人看。
* `l150pro-driver.service` **故意不启**（它和 `ps2-teleop` 抢同一个 /dev/l150pro）。
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')
# 本地（Windows）也能跑 --selftest：源码树就在脚本上一层
_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if not os.path.isdir('/home/cy/raspot_ws/src/arm_grasp') and os.path.isdir(_HERE):
    sys.path.insert(0, _HERE)

# 顶层只 import **纯模块**（grasp/geom/arm_kin/servo 都不 import rclpy）。
# `arm_grasp.collect` 照计划也放进函数内 import（见 `_collect()`）—— 这样将来它要是
# 把 `rclpy` 提到顶层，本地 Windows 的 `--selftest` 也不会被连累。
# （实测 2026-09-30：**现在**顶层 import collect 其实也行，它的 `import rclpy` 是
#  `ArmIO.__init__` 里才做的；真正要 ROS 的是 `ArmIO()` 本身，不是 import。）
from arm_grasp import geom, grasp
from arm_grasp.arm_kin import from_fields, to_fields
from arm_grasp.servo import limit_step

# 名义点（米）：正前方略偏右 20cm、平台高度。理由见模块头（底座横扫 86° vs 2°）。
DEFAULT_TARGET = (0.20, -0.020, 0.016)

# 2026-09-30 上机前从 `/arm/feedback` 实测的"歇着的姿态"。只给 `--selftest` 当
# 目击证人用（臂动过之后它就过期了）；"目标点在不在视野里"的**真检查**在 `main()` 里，
# 用的是当时真实回读。
START_POSE_2026_09_30 = {'base': -4.0, 'shoulder': 111.8, 'elbow': -91.0,
                         'wrist_pitch': -78.0}


def _collect():
    """`arm_grasp.collect` 在函数内取（碰硬件的那半边，`ArmIO` 里才 `import rclpy`）。"""
    from arm_grasp import collect
    return collect


def parse_target(s):
    """`'0,-20,1.6'`（**cm**）→ `(0.0, -0.20, 0.016)`（米）。"""
    parts = [float(v) for v in s.replace(' ', '').split(',')]
    if len(parts) != 3:
        raise ValueError('--target 要写成 x,y,z（cm），收到 %r' % s)
    return tuple(v / 100.0 for v in parts)


def virt_obs(t_arrive, joints, O_m, frame=0):
    """模型相机：O 在 `joints` 这个姿态下的像素，当成一条观测（纯函数，好自检）。

    点落在相机平面之后时 `geom.project` 抛 ValueError —— **由调用方决定怎么办**
    （`ArmLink.obs` 把它当成"这拍没观测"）。
    """
    u, v = geom.project(joints, O_m)
    return grasp.Obs(float(t_arrive), u, v, int(frame), 'virt', None)


def vis_report(joints, O_m, vis):
    """目标点在这个姿态的相机视野里吗？返回 (ok|None, (u,v)|None, 说明)。

    `ok is None` = 连投影都算不出来（点在相机平面之后）。**必须**看得见才跑得动：
    看不见 ⇒ `ArmLink.obs()` 恒为 None ⇒ `run()` 原地不动到超时（见模块头）。
    """
    try:
        u, v = geom.project(joints, O_m)
    except ValueError as e:
        return None, None, '投影失败：%s' % e
    ok = vis.u_lo <= u <= vis.u_hi and vis.v_lo <= v <= vis.v_hi
    return ok, (u, v), ('预测像素 (%.0f, %.0f)（有效区 u∈[%.0f,%.0f] v∈[%.0f,%.0f]）'
                        % (u, v, vis.u_lo, vis.u_hi, vis.v_lo, vis.v_hi))


class ArmLink:
    """把 `ArmIO` 包成 `grasp.Link`：**每一次 spin 都必须很轻**，绝不等臂到位。"""

    def __init__(self, io, O_m, hz=10.0):
        self.io = io
        self.O = tuple(O_m)          # 钉死的目标点（喂给模型相机的那个）
        self.hz = hz
        self.n = 0                   # 已发观测条数
        self.miss = 0                # 投影失败（点跑到相机后面）的拍数

    def now(self):
        return time.monotonic()

    def spin(self, dt):
        # 不睡满 dt：先 spin 一小段收消息，剩下的时间睡掉 —— 保证节拍稳定在 ~hz
        t_end = time.monotonic() + dt
        self.io.rclpy.spin_once(self.io.node, timeout_sec=dt * 0.4)
        rest = t_end - time.monotonic()
        if rest > 0:
            time.sleep(rest)

    def fb(self):
        return self.io.fb

    def obs(self):
        """模型相机这一刻的观测；`None` = 这拍没有观测（`run()` 按丢观测处理）。"""
        fbf = self.io.fb
        if fbf is None:
            return None
        try:
            o = virt_obs(self.now(), from_fields(fbf), self.O, self.n)
        except ValueError:
            self.miss += 1
            return None
        self.n += 1
        return o

    def publish(self, fields):
        from arm_grasp.collect import JOINT_NAMES      # 函数内：collect 依赖 rclpy
        msg = self.io._js()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in fields]
        self.io.pub.publish(msg)


# --------------------------------------------------------------------------
# 打印
# --------------------------------------------------------------------------

def _fmt_fields(fields):
    return ' '.join('%4.0f' % v for v in fields)


def _print_rows(rep, n=10):
    rows = rep['rows'][-n:]
    if not rows:
        print('   （一拍都没发过 —— 一次目标点都没算出来）')
        return
    t00 = rows[0][0]
    for t, err, s_ach, alpha, slack, has_obs in rows:
        print('   t=%5.1fs  爪尖→目标点 %7.2fmm  沿轴留量 %6.1fmm  α=%6.1f°  '
              'slack=%4.0f  观测=%s'
              % (t - t00, err * 1000.0, s_ach * 1000.0, alpha, slack,
                 '有' if has_obs else '无'))


# --------------------------------------------------------------------------
# 自检（不需要硬件、不需要 ROS、不发任何指令）
# --------------------------------------------------------------------------

def selftest():
    """走的是**接下来真用的那几个函数**：`parse_target` / `virt_obs` /
    `grasp.plan_verdict` / `grasp.estimate_point`（不含 `ArmLink` 的 ROS 那半边）。"""
    ok = True
    print('=' * 74)
    print('stream_move --selftest（无硬件、无 ROS、不发指令）')
    print('=' * 74)

    # ① 目标点解析（cm → m）
    for s, want in (('0,-20,1.6', (0.0, -0.20, 0.016)),
                    ('0,-17,1.6', (0.0, -0.17, 0.016)),
                    (' 1.5 , -3 , 0 ', (0.015, -0.03, 0.0))):
        got = parse_target(s)
        tag = '✓' if all(abs(a - b) < 1e-12 for a, b in zip(got, want)) else '✗'
        if tag == '✗':
            ok = False
        print('%s parse_target(%-14r) = %s' % (tag, s, got))
    for bad in ('0,-20', '0,-20,1.6,7', 'a,b,c'):
        try:
            parse_target(bad)
            print('✗ parse_target(%r) 应该报错却没有' % bad)
            ok = False
        except ValueError:
            print('✓ parse_target(%-14r) 按预期报 ValueError' % bad)

    cfg = grasp.GraspConfig()
    print('\nGraspConfig：s_pre=%.1fcm  s_stop=%.1fmm  max_step=%.2f°/拍  hz=%.0f  '
          'min_slack=%.0f  z_plane=%.0fmm'
          % (cfg.s_pre_m * 100, cfg.s_stop_m * 1000, cfg.max_step_deg, cfg.hz,
             cfg.min_slack, cfg.z_plane_m * 1000))

    # ② S0 三级判定：默认名义点必须落在 ok（"默认值别被改回贴着限位那档"的回归闸）
    print('\nS0（默认目标点 %.3f, %.3f, %.3f m）：' % DEFAULT_TARGET)
    lvl, tgt, msg = grasp.plan_verdict(DEFAULT_TARGET, cfg)
    print('  %s —— %s' % (lvl, msg))
    if lvl != 'ok':
        print('  ✗ 默认名义点应是 ok（(0.20,-0.02) 有 112 count；'
              '(0,-0.20) 是 110 但底座要横扫 86°；(0,-0.17) 只有 14 ⇒ warn）')
        ok = False
    if tgt is None:
        print('  ✗ level 不是 refuse 却给了空 Target')
        ok = False
    else:
        print('  末态 α=%.1f°  field=[%s]  余量=%d  count'
              % (tgt.alpha, _fmt_fields(tgt.fields[2:5]), tgt.slack))
        if tgt.slack < cfg.min_slack:
            print('  ✗ 余量 %.0f < min_slack %.0f' % (tgt.slack, cfg.min_slack))
            ok = False
    # 旁证（半径扫描，z=0.016、末态 slack）。slack 印一位小数：`plan_verdict` 的说明里
    # 用的是 `%d`（截断），110.89 会印成 "110" —— 别把截断当成另一个数。
    print('  旁证（半径扫描，z=0.016、末态 slack）：')
    for r in (0.16, 0.17, 0.20, 0.25):
        l2, t2, _ = grasp.plan_verdict((0.0, -r, 0.016), cfg)
        print('    r=%2.0fcm  %-6s slack=%s' % (
            r * 100, l2, '-' if t2 is None else '%.1f' % t2.slack))

    # ②b 模型相机看得见吗 —— 这是"跑起来会不会动"的前提（看不见 ⇒ run() 原地不动到超时）
    print('\n模型相机（用 2026-09-30 实测的起手姿态当目击证人；臂动过就过期了）：')
    print('  起手 field=%s  爪尖 %s m'
          % (_fmt_fields(to_fields(START_POSE_2026_09_30, cfg.gripper, cfg.wrist_roll)),
             tuple(round(v, 4) for v in grasp.tip_open_m(START_POSE_2026_09_30))))
    for name, O in (('默认点', DEFAULT_TARGET),
                    ('正前方 (0,-0.20,0.016)', (0.0, -0.200, 0.016))):
        v_ok, px, why = vis_report(START_POSE_2026_09_30, O, cfg.vis)
        mark = '✓' if v_ok else '⚠️'
        print('  %s %-22s %s' % (mark, name, why))
        if not v_ok:
            print('     ↑ 看不见 ⇒ 模型相机给不出观测 ⇒ `run()` 会**原地不动**到超时')
    v_ok, _, _ = vis_report(START_POSE_2026_09_30, DEFAULT_TARGET, cfg.vis)
    if not v_ok:
        print('  ✗ 默认目标点在起手姿态看不见，这一步会干等到超时')
        ok = False

    # ③ refuse 不能是"抛异常"，必须是三级里的一个值
    lvl_far, t_far, msg_far = grasp.plan_verdict((0.0, 0.0, 3.0), cfg)
    print('\n不可达点 (0,0,3)m → %s —— %s' % (lvl_far, msg_far))
    if lvl_far != 'refuse' or t_far is not None:
        ok = False
        print('  ✗ 应返回 refuse + None')

    # ④ 模型相机（本工具不用相机，靠它喂观测）：投出去 → 反解回来，必须闭合
    print('\n模型相机（virt_obs → estimate_point）：')
    for O in (DEFAULT_TARGET, (0.02, -0.22, 0.016)):
        j0 = grasp.pick_target(O, cfg, cfg.s_pre_m, cfg.alpha0).joints
        fb = to_fields(j0, cfg.gripper, cfg.wrist_roll)
        u, v = geom.project(j0, O)
        # 单条样本的轨迹：`at()` 越界会夹到最近端 ⇒ 正好模拟"只有起手那一拍"
        tr = __import__('arm_grasp.ltrace', fromlist=['Trace']).Trace()
        tr.add(100.0, fb, fb)
        obs = virt_obs(100.2, j0, O, 0)
        try:
            P, C, d, jj, t_fr = grasp.estimate_point(tr, obs, cfg.z_plane_m,
                                                     cfg.latency_s)
        except Exception as e:                                    # noqa: BLE001
            print('  ✗ %s 反解抛了 %s' % (O, e))
            ok = False
            continue
        # 注意：`estimate_point` 用的是 cfg.z_plane_m（默认 20mm）**不是** O 的 z
        # ⇒ 反解点落在那张平面上，与 O 的差 = 平面高度差带来的斜视偏移（几 mm）。
        # 关键是**它不随姿态变** ⇒ 估计稳定、环路能收敛（这就是喂给 run() 的那个 O）。
        err = math.dist(P, O)
        print('  O=%s 像素(%.1f, %.1f) 反解 %s  |Δ|=%.2fmm（z 平面 %.0fmm vs 目标 %.0fmm）'
              % (tuple(round(x, 3) for x in O), u, v,
                 tuple(round(x, 4) for x in P), err * 1000.0,
                 cfg.z_plane_m * 1000, O[2] * 1000))
        if err > 0.02:
            print('  ✗ 反解误差 %.1fmm 太大（相机模型/内参对不上了）' % (err * 1000))
            ok = False

    # ⑤ 打印格式不炸（真跑时那几行就是它们打的）
    print('\n打印格式：')
    fake = {'rows': [(10.0, 0.0231, 0.0400, -58.0, 110.0, 1.0),
                     (10.1, 0.0008, 0.0802, -58.0, 110.0, 1.0)]}
    try:
        _print_rows(fake)
        _print_rows({'rows': []})
        # 真跑时 main() 打的就是这一行：从"起手姿态"到"末态姿态"限速一步后的 field
        j_start = grasp.pick_target(DEFAULT_TARGET, cfg, cfg.s_pre_m, cfg.alpha0).joints
        j_last = grasp.pick_target(DEFAULT_TARGET, cfg, 0.0, cfg.alpha0).joints
        j_first, ratio, reached = limit_step(j_start, j_last, cfg.max_step_deg)
        print('  第一拍将发 field=[%s]（限速比 %.2f，%s）'
              % (_fmt_fields(to_fields(j_first, cfg.gripper, cfg.wrist_roll)),
                 ratio, '已到位' if reached else '还在追'))
    except Exception as e:                                        # noqa: BLE001
        print('  ✗ 打印格式炸了：%s' % e)
        ok = False

    print('\n%s' % ('自检通过 ✅' if ok else '自检**失败** ❌'))
    return 0 if ok else 1


# --------------------------------------------------------------------------
# 真跑
# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description='流式走到钉死的目标点上方（会动臂）')
    ap.add_argument('--selftest', action='store_true',
                    help='不需要硬件/ROS 的自检（不发任何指令）')
    ap.add_argument('--target', default=None,
                    help='x,y,z（cm）；默认名义点 %s' % (DEFAULT_TARGET,))
    ap.add_argument('--s', type=float, default=0.08,
                    help='留量（米），默认 8cm（= grasp.GraspConfig().s_pre_m）')
    ap.add_argument('--dry-run', action='store_true',
                    help='只读：算一遍打出来，不发任何指令')
    ap.add_argument('--yes', action='store_true', help='跳过回车确认（真跑）')
    ap.add_argument('--max-step', type=float, default=1.5,
                    help='每拍每关节最多多少度（默认 1.5）')
    ap.add_argument('--t-ms', type=int, default=100,
                    help='arm_t_ms：每条 0xAC 的移动时长（默认 100 ≈ 10Hz 一拍）')
    ap.add_argument('--max-seconds', type=float, default=None,
                    help='覆盖 cfg.max_seconds（默认 40）')
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()

    try:
        tgt_m = parse_target(args.target) if args.target else DEFAULT_TARGET
    except ValueError as e:
        print('❌ %s' % e)
        return 2

    # ★ `--s` 必须真的进 cfg：`run()` 的接近阶段用的就是 `cfg.s_pre_m`。
    cfg = grasp.GraspConfig(max_step_deg=args.max_step, s_pre_m=args.s)
    if args.max_seconds is not None:
        cfg = cfg._replace(max_seconds=args.max_seconds)

    print('目标点 O = (%.4f, %.4f, %.4f) m   留量 s = %.1f cm   限速 %.2f°/拍   %.0f Hz'
          % (tgt_m + (args.s * 100, args.max_step, cfg.hz)))
    print('（目标点只喂**模型相机**：本步不碰 K230/8555，验证的是流式/余量/限速）')

    try:
        collect = _collect()
    except ImportError as e:
        print('❌ 没有 ROS（%s）—— 真跑要在树莓派上（source install/setup.bash）；'
              '本地 Windows 只能跑 --selftest' % e)
        return 2

    io = None
    drv, stopped = None, False
    try:
        io = collect.ArmIO()
        io.wait_feedback()
        fb = list(io.fb)
        j = from_fields(fb)
        print('\n当前回读 field=[%s]' % _fmt_fields(fb))
        print('当前爪尖 (%.4f, %.4f, %.4f) m' % grasp.tip_open_m(j))

        lvl, tgt, msg = grasp.plan_verdict(tgt_m, cfg)
        print('S0 判定：%s —— %s' % (lvl, msg))
        if tgt is not None:
            print('  末态 α=%.1f°  field=[%s]  余量=%d  count'
                  % (tgt.alpha, _fmt_fields(tgt.fields[2:5]), tgt.slack))

        # ★ 先只读地看一眼"模型相机能不能看见它" —— 看不见就别停服务了（见模块头）
        vis_ok, px, why = vis_report(j, tgt_m, cfg.vis)
        print('目标点在**当前姿态**的相机视野里：%s —— %s'
              % ('是' if vis_ok else '**不是**', why))
        if not vis_ok:
            print('⚠️ 模型相机给不出观测 ⇒ `grasp.run` 会**原地不动**直到 %.0f 秒超时'
                  '（看起来像卡住，**不是**限速/余量的问题；一条 /arm/command 也不会发'
                  '得动）. 换一个当前能看见的目标点，或先把臂摆到能看见它的姿态。'
                  % cfg.max_seconds)

        if lvl == 'refuse' and not args.yes:
            print('⛔ 拒绝执行。要硬来加 --yes（不建议）。')
            return 3

        if args.dry_run:
            print('\n--dry-run：一条指令都没发。')
            return 0

        # 发指令前把**第一拍要发的那条**打出来（人看的就是这个）
        if tgt is not None:
            j_first, ratio, reached = limit_step(j, tgt.joints, cfg.max_step_deg)
            print('\n第一拍将发 field=[%s]   （限速比 %.2f，%s）'
                  % (_fmt_fields(to_fields(j_first, cfg.gripper, cfg.wrist_roll)),
                     ratio, '已到位' if reached else '还在追'))
        print('⚠️ 这一步会：停 %s → 单独起驱动 → **真发 /arm/command（臂会动）**'
              % collect.SERVICE)
        print('   结束时恢复服务 = 机械臂走 INIT_HOME 归位（**也会动**），'
              '跑完先让臂周围空出来。')
        if not args.yes:
            try:
                input('确认臂的行程里没有东西、人离远点，回车开跑（Ctrl-C 取消）：')
            except EOFError:
                print('没有交互输入，用 --yes 才能跑。退出。')
                return 2

        rc = collect._svc('stop')
        if rc.returncode != 0:
            print('❌ 停服务失败：%s' % (rc.stderr or rc.stdout).strip())
            return 1
        stopped = True
        time.sleep(1.0)
        st = collect._svc_active()
        if st != 'inactive':
            print('❌ %s 还是 %s —— **没有动臂**' % (collect.SERVICE, st))
            return 1
        print('✅ %s 已停' % collect.SERVICE)

        drv = collect._start_driver(args.t_ms)
        time.sleep(2.0)
        io.wait_feedback()
        print('→ /arm/feedback 有了（fb_n=%d），开始流式' % io.fb_n)

        link = ArmLink(io, tgt_m, hz=cfg.hz)
        rep = grasp.run(cfg, link, phase='aim', log=print)
        print('\n[aim] %s' % rep['stopped'])
        _print_rows(rep)
        print('   实际最大帧间增量 %.2f°（上限 %.2f°）  拍了 %d 拍  观测 %d 条'
              '（投影失败 %d 次）'
              % (rep['max_step_deg_actual'], cfg.max_step_deg, rep['ticks'],
                 link.n, link.miss))
        if rep['err_m'] is not None:
            print('   最终 爪尖→目标点 %.2fmm  沿轴留量 %.1fmm  余量 slack=%s'
                  % (rep['err_m'] * 1000.0, rep['s_ach_m'] * 1000.0,
                     '-' if rep['target'] is None else '%.0f' % rep['target'].slack))
        # 验收（Task 6 Step 4 的 1/2/4 条，在这里直接判给人看）
        if rep['ok']:
            print('✅ 走到位（≤2mm）')
        else:
            print('❌ 没走到位：%s' % rep['stopped'])
        if rep['max_step_deg_actual'] > cfg.max_step_deg + 1e-6:
            print('❌ 限速没生效：实际 %.2f° > 上限 %.2f°'
                  % (rep['max_step_deg_actual'], cfg.max_step_deg))
        return 0 if rep['ok'] else 1
    except KeyboardInterrupt:
        print('\n⚠️ Ctrl-C 中断')
        return 1
    except Exception as e:                                        # noqa: BLE001
        print('\n❌ 出错了：%s' % e)
        return 1
    finally:
        # 无论成败、包括 Ctrl-C：驱动 -> 服务，一步步收回来。
        if io is not None:
            try:
                io.close()
            except Exception:                                     # noqa: BLE001
                pass
        if drv is not None:
            # ★ 收驱动**不许**把恢复服务那一步连累掉：`_stop_driver` 里的
            #   `_kill_all_drivers` 杀不干净时会抛 RuntimeError —— 让它冒出去，
            #   下面 `_svc('start')` 就永远不执行，服务留在停止态（臂没人管）。
            try:
                collect._stop_driver(drv)
            except Exception as e:                                # noqa: BLE001
                print('⚠️ 收驱动出错（继续恢复服务）：%s' % e)
        if stopped:
            collect._svc('start')
            # ★ 这一步会让机械臂走 INIT_HOME **归位（会动）**
            print('✅ 服务已恢复：%s（臂已 INIT_HOME 归位，别伸手进去）'
                  % collect._svc_active())


if __name__ == '__main__':
    sys.exit(main())
