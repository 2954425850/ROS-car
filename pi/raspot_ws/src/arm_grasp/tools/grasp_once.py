# -*- coding: utf-8 -*-
"""真抓流程 CLI（Task 9）：**唯一的"视觉 + 臂"接线处**。

把 K230 的框接到 `grasp.run()` 的限速流式主循环上，整条链是：

    app 画框 → 8557 锁框 → Pi 每拍读框中心 → 射线∩平面得 O → (α,s) 搜索
      → 限速 10Hz 发 /arm/command → 视觉自己带着臂收敛（不停在"等到位"上）

    python3 tools/grasp_once.py --selftest                       # 不碰硬件的自检
    python3 tools/grasp_once.py --box l,t,r,b --dry-run          # 只读
    python3 tools/grasp_once.py --box l,t,r,b --yes              # S1+S2（**会动臂**）
    python3 tools/grasp_once.py --box l,t,r,b --phase all --yes  # 再接着下扎 S3

★ **sudo 密码**：Pi 上的 `cy` **没有免密 sudo**（见 `collect._sudo` 的 docstring），
  非交互跑必须先给密码，否则 `_svc('stop')` 必然失败、停在半路。所以用法里一律带：

      GRASP_SUDO_PASS=1 python3 tools/grasp_once.py --box ... --yes

  （`GRASP_SUDO_PASS` 的**值就是 sudo 密码本身**，`collect._sudo` 拿它喂 `sudo -S`；
    这条 T6 上机时踩过。密码不是 `1` 时换成真的。）

## `--box` 是**人画的那个框**
归一化 `l,t,r,b`，与板子 8557 的约定**逐字相同**（左上原点、基准 1280x720 推流画面）
⇒ Pi 不需要任何坐标换算，直接转发（避开"通道/分辨率不同"那个静默坑）。

## 高度来源

`--auto-height`：先通过腕部单目多视角照片与真实关节回读自动测量支撑平面/目标顶面。
测量失败不抓取，不使用默认高度。`--measure-only` 只测高，`--scan-preview` 只读查路径。
`--height-session ... --dry-run` 离线重算与预演，不接触 ROS/相机。
详见 `docs/2026-10-01-auto-height.md`。

手动模式的 `--h` 是尺度来源：
单目没有距离：用"射线 ∩ z = h 平面"当尺度（design §2.3）。目标面高度默认 0.016 m
（车身安装面上 1.6cm）。**它进 `cfg.z_plane_m`**，不是随手一个常数 —— 填错 =
整条链的落点整体偏（`--dry-run` 打印的就是它算出来的 O）。

## 安全（每条都是踩出来的，照抄 `tools/stream_move.py`）
* **先 `systemctl stop ps2-teleop.service`（不许 kill**，`Restart=always` 会被拉起
  → 两个实例抢串口，dropped 疯涨）。停完**核实** inactive 才动臂。
* 停服务会把驱动一起停掉 ⇒ 必须**单独起** `l150pro_driver_node`，否则没有 `/arm/feedback`。
* 停/恢复全部焊在 `try/finally`（含 Ctrl-C）里。
* ⚠️ **恢复服务 = 机械臂走 INIT_HOME 归位（会动！）** —— 跑前清空行程、人让开，
  跑完也别马上伸手进去。
* `finally` 里还要 `{"cmd":"stop"}` **释放跟踪器**（一次性语义，design §8）——
  不然板子一直锁着上次那个框，下一次锁框就锁不上。
* `--dry-run` **只读**：会锁框、会读关节、会算 O 与 S0，但**一条 /arm/command 都不发、
  服务一秒都不停**（板子侧只有"锁框"和收尾的"释放"两条网络写，那不叫动臂）。
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')
# 同目录的 `stream_move` 要 import（复用它的 ArmLink / _stable_fb / 打印函数）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# 本地（Windows）也能跑 --selftest：包目录就在脚本的上两层
_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if not os.path.isdir('/home/cy/raspot_ws/src/arm_grasp') and os.path.isdir(_HERE):
    sys.path.insert(0, _HERE)

# 顶层只 import **纯模块**（grasp/geom/arm_kin/servo/observe/ltrace 都不 import rclpy）。
# `arm_grasp.collect`（碰硬件那半边）在函数内 import —— 见 `_collect()`。
from arm_grasp import grasp, ltrace, observe
from arm_grasp.arm_kin import from_fields, to_fields
from arm_grasp.servo import limit_step
from stream_move import ArmLink, _fmt_fields, _print_rows, _print_trace, _stable_fb

# 默认目标面高度（米）：车身安装面之上 1.6cm（与 Task 6/9 的写法一致）
DEFAULT_H_M = 0.016
# 锁框之后等结果流刷出这个框的最长时间（板子 6~10Hz ⇒ 秒级足够）
OBS_WAIT_S = 5.0


def _collect():
    """`arm_grasp.collect` 在函数内取（`ArmIO` 里才 `import rclpy`）。"""
    from arm_grasp import collect
    return collect


# --------------------------------------------------------------------------
# 纯函数（自检也走它们，别再写第二份）
# --------------------------------------------------------------------------

def parse_box(s):
    """`'0.41,0.49,0.56,0.72'` → `[0.41, 0.49, 0.56, 0.72]`（归一化 l,t,r,b）。

    个数不对、越界、l≥r / t≥b 一律 `ValueError` —— **别让一个坏框走到锁框那一步**
    （锁上一个 0.9,0.9,0.1,0.1 的框，板子会锁到奇怪的地方，而后面看起来"一切正常"）。
    """
    parts = [float(v) for v in s.replace(' ', '').split(',')]
    if len(parts) != 4:
        raise ValueError('--box 要写成 l,t,r,b（归一化，4 个数），收到 %r' % s)
    for v in parts:
        if not (0.0 <= v <= 1.0):
            raise ValueError('--box 的每一项都要在 0~1（归一化），收到 %r' % s)
    if not (parts[0] < parts[2] and parts[1] < parts[3]):
        raise ValueError('--box 要满足 l<r 且 t<b，收到 %r' % s)
    return parts


def build_cfg(h_m, max_step=None, patience=None, max_seconds=None, alpha0=None,
              joint_comp=None, hz=None, bias_r=None):
    """把 CLI 参数装进 `GraspConfig`。**`--h` 必须真的进 `z_plane_m`**（自检里有闸）。"""
    kw = {'z_plane_m': float(h_m)}
    if max_step is not None:
        kw['max_step_deg'] = float(max_step)
    if patience is not None:
        kw['patience'] = int(patience)
    if alpha0 is not None:
        kw['alpha0'] = float(alpha0)
    if joint_comp is not None:
        kw['joint_comp'] = float(joint_comp)
    if bias_r is not None:
        kw['bias_r_m'] = float(bias_r) / 100.0      # CLI 用 cm，内部用米
    cfg = grasp.GraspConfig(**kw)
    if hz is not None:
        # ★ 提频时**必须把"按拍数"的判据一起折算**，否则它们在**墙上时间**上被砍短：
        #   hz 10→20 会让"卡住 25 拍(2.5s)"变成 1.25s、"合爪兜底 30 拍(3s)"变成 1.5s
        #   ⇒ 判据比硬件还急、整轮变脆。按 hz/10 同比例放大，**秒数不变**。
        hz = float(hz)
        k = hz / cfg.hz
        cfg = cfg._replace(hz=hz,
                           patience=max(1, int(round(cfg.patience * k))),
                           close_ticks=max(1, int(round(cfg.close_ticks * k))),
                           max_ticks=max(1, int(round(cfg.max_ticks * k))))
    if max_seconds is not None:
        cfg = cfg._replace(max_seconds=float(max_seconds))
    return cfg


def one_obs(want_norm, timeout=OBS_WAIT_S):
    """从结果流里**挑一条**框观测（只读、不碰臂）。挑不到返回 None。

    锁框刚下发的那一小会儿，`/tmp/k230/latest-result.json` 里还是**锁之前**那一帧
    ⇒ 立刻读会挑不出框。所以这里是有界重试，不是读一次就认。
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = observe.fresh_result()
        if r is not None:
            b = observe.pick_box(r, want_norm)
            if b is not None:
                u, v = observe.box_center_ai(b['box'])
                return grasp.Obs(time.monotonic(), u, v, r.get('frame'),
                                 b.get('src', 'det'), b['box'])
        time.sleep(0.05)
    return None


# --------------------------------------------------------------------------
# Link：在 stream_move.ArmLink 上接上 K230 的框
# --------------------------------------------------------------------------

class GraspLink(ArmLink):
    """`ArmLink`（ROS 那半边：spin/fb/publish 全复用）+ 真观测。

    `obs()` 的契约（`grasp.Link`）：**自上一拍以来最新的一条**，没有就 `None`。
    去重靠板子的 `frame` 号（板子时间戳只有秒级，用不了；design §5.1）。

    2026-10-01 复查改的两处：
      * 挑框用 `observe.BoxFollower`（参考框跟着目标走）。旧写法每拍拿**人画的框**比 IoU，
        相机一动目标在画面里挪走 ⇒ IoU<0.2 ⇒ 板子还在发跟踪框，这里整条扔掉。
      * 观测时刻 = 现在 − 结果文件的年龄（mtime），不是"读到的那一刻"——文件最旧可到 1.5s。
    """

    def __init__(self, io, host, want_norm, hz=10.0):
        super().__init__(io, (0.0, 0.0, 0.0), hz=hz)   # ArmLink 的模型相机在这里用不上
        self.host, self.want = host, want_norm
        self.follow = observe.BoxFollower(want_norm)
        self.last_frame = None
        self.last_box = None
        self.last_src = None
        self.n_obs = 0        # 交出去的观测条数
        self.n_none = 0       # 结果流里挑不出框（跟踪丢了 / IoU 不达标）的拍数

    def obs(self):
        got = observe.fresh_result_aged()
        if got is None:
            return None
        r, age = got
        if r.get('frame') == self.last_frame:
            return None                      # 没新帧 / 文件还没刷新 ⇒ 这拍没有观测
        self.last_frame = r.get('frame')
        b = self.follow.pick(r)
        if b is None:
            self.n_none += 1
            return None
        self.last_box, self.last_src = b['box'], b.get('src', 'det')
        u, v = observe.box_center_ai(b['box'])
        self.n_obs += 1
        return grasp.Obs(self.now() - age, u, v, r.get('frame'), self.last_src, b['box'])


# --------------------------------------------------------------------------
# 自检（不需要硬件 / ROS / 板子，不发任何指令）
# --------------------------------------------------------------------------

def selftest():
    """走的是**接下来真用的那几个函数**：`parse_box` / `build_cfg` / `grasp.plan_verdict`。"""
    ok = True
    print('=' * 74)
    print('grasp_once --selftest（无硬件、无 ROS、无板子、不发指令）')
    print('=' * 74)

    # ① --box 解析
    print('① --box 解析（归一化 l,t,r,b）：')
    for s, want in (('0.41,0.49,0.56,0.72', [0.41, 0.49, 0.56, 0.72]),
                    (' 0.1 , 0.2 , 0.3 , 0.4 ', [0.1, 0.2, 0.3, 0.4]),
                    ('0,0,1,1', [0.0, 0.0, 1.0, 1.0])):
        got = parse_box(s)
        tag = '✓' if got == want else '✗'
        if tag == '✗':
            ok = False
        print('  %s parse_box(%-22r) = %s' % (tag, s, got))
    for bad in ('0.41,0.49,0.56', '0.41,0.49,0.56,0.72,0.9', '1.1,0,0.5,0.5',
                '-0.1,0,0.5,0.5', 'a,b,c,d', '0.6,0.2,0.5,0.4', 'nan,0,0.5,0.5'):
        try:
            parse_box(bad)
            print('  ✗ parse_box(%r) 应该报错却没有' % bad)
            ok = False
        except ValueError:
            print('  ✓ parse_box(%-22r) 按预期报 ValueError' % bad)

    # ② ★ 变异闸：--h 必须真的进 cfg.z_plane_m
    #     （把 build_cfg 里的 z_plane_m 写死成 0.02 ⇒ 这一条立刻变红）
    print('\n② --h → cfg.z_plane_m（写死常数会在这里变红）：')
    for h in (0.016, 0.03, 0.045):
        c = build_cfg(h)
        tag = '✓' if c.z_plane_m == h else '✗'
        if tag == '✗':
            ok = False
        print('  %s build_cfg(h=%.3f).z_plane_m = %.4f' % (tag, h, c.z_plane_m))
    c_def = build_cfg(DEFAULT_H_M)
    if c_def.z_plane_m != DEFAULT_H_M:
        print('  ✗ 默认高度没进 cfg')
        ok = False

    # ②b ★ 变异闸：--alpha0 必须真的进 cfg.alpha0（把它从 kw 里删掉 ⇒ 这一条立刻变红）
    #     ⚠️ 不传时必须**原样保留 GraspConfig 的默认**（别顺手写成硬编码的 −84）。
    print('\n②b --alpha0 → cfg.alpha0（不传 = 保留默认，这一条会红）：')
    for a in (-84.0, -70.0, -88.0):
        c = build_cfg(DEFAULT_H_M, alpha0=a)
        tag = '✓' if c.alpha0 == a else '✗'
        if tag == '✗':
            ok = False
        print('  %s build_cfg(alpha0=%.0f).alpha0 = %.1f' % (tag, a, c.alpha0))
    if build_cfg(DEFAULT_H_M).alpha0 != grasp.GraspConfig().alpha0:
        print('  ✗ 不传 --alpha0 时没有保留 GraspConfig 的默认值')
        ok = False

    # ②c ★ 变异闸：--hz 必须真的进 cfg.hz，**且按拍数的判据要同比折算**
    #     （把 cfg._replace 里那三项删掉 ⇒ 这一条立刻变红）
    print('\n②c --hz → cfg.hz 且按拍判据同比折算（秒数不变）：')
    base = build_cfg(DEFAULT_H_M)
    for h in (10.0, 20.0, 25.0):
        c = build_cfg(DEFAULT_H_M, hz=h)
        k = h / base.hz
        want = (round(base.patience * k), round(base.close_ticks * k),
                round(base.max_ticks * k))
        got = (c.patience, c.close_ticks, c.max_ticks)
        tag = '✓' if (c.hz == h and got == want) else '✗'
        if tag == '✗':
            ok = False
        print('  %s hz=%-5.0f patience/close_ticks/max_ticks = %s (期望 %s；'
              '秒数 patience=%.1fs close=%.1fs)'
              % (tag, h, got, want, c.patience / h, c.close_ticks / h))
    if build_cfg(DEFAULT_H_M).hz != grasp.GraspConfig().hz:
        print('  ✗ 不传 --hz 时没有保留 GraspConfig 的默认值')
        ok = False

    # ③ 名义点上跑 S0（打印**原文**，断言不抛异常）
    print('\n③ 名义点 S0（grasp.plan_verdict 原文）：')
    cfg = build_cfg(DEFAULT_H_M)
    print('  GraspConfig：s_pre=%.1fcm  s_stop=%.1fmm  max_step=%.2f°/拍  hz=%.0f  '
          'z_plane=%.0fmm'
          % (cfg.s_pre_m * 100, cfg.s_stop_m * 1000, cfg.max_step_deg, cfg.hz,
             cfg.z_plane_m * 1000))
    for name, O in (('名义点 (0.20,-0.02,0.016)', (0.20, -0.020, 0.016)),
                    ('Task4 夹具点 (0,-0.17,0.016)', (0.0, -0.170, 0.016))):
        try:
            lvl, tgt, msg = grasp.plan_verdict(O, cfg)
        except Exception as e:                                # noqa: BLE001
            print('  ✗ %s 抛了异常：%r' % (name, e))
            ok = False
            continue
        print('  %s → %s —— %s' % (name, lvl, msg))
        if lvl not in ('ok', 'warn', 'refuse'):
            print('  ✗ 不是三级之一')
            ok = False
        if (lvl == 'refuse') != (tgt is None):
            print('  ✗ refuse 与 "有没有 Target" 不吻合')
            ok = False
        if tgt is not None:
            print('    末态 α=%.1f°  field=[%s]  余量=%d count'
                  % (tgt.alpha, _fmt_fields(tgt.fields[2:5]), tgt.slack))

    # ③b 不可达点必须是 refuse（这条能红：把 tier 改了就会变）
    try:
        lvl_far, tgt_far, msg_far = grasp.plan_verdict((0.0, 0.0, 3.0), cfg)
        print('  不可达点 (0,0,3)m → %s —— %s' % (lvl_far, msg_far))
        if lvl_far != 'refuse' or tgt_far is not None:
            print('  ✗ 应返回 refuse + None')
            ok = False
    except Exception as e:                                    # noqa: BLE001
        print('  ✗ 不可达点抛异常（应该温和拒绝）：%r' % e)
        ok = False

    print('\n%s' % ('自检通过 ✅' if ok else '自检**失败** ❌'))
    return 0 if ok else 1


# --------------------------------------------------------------------------
# 真跑
# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='grasp_once.py',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='闭环抓取一次：K230 的框 → grasp.run（**真跑会动臂**）',
        epilog='例：GRASP_SUDO_PASS=1 python3 tools/grasp_once.py '
               '--box 0.41,0.49,0.56,0.72 --yes\n'
               '（`cy` 没有免密 sudo，非交互跑必须给 GRASP_SUDO_PASS，否则停服务那步必失败；'
               '该变量的值 = sudo 密码本身，不是 1 时就换成真的）')
    ap.add_argument('--selftest', action='store_true',
                    help='不需要硬件/ROS 的自检（不发任何指令）')
    ap.add_argument('--box', default=None,
                    help='归一化 l,t,r,b（与板子 8557 同约定，1280x720 推流系）—— '
                         '**人画的那个框**')
    ap.add_argument('--h', type=float, default=None,
                    help='目标面高度（米），默认 %.3f（安装面上 1.6cm）**进 cfg.z_plane_m**'
                         % DEFAULT_H_M)
    height_mode = ap.add_mutually_exclusive_group()
    height_mode.add_argument('--auto-height', action='store_true',
                             help='先动臂采集多视角照片并自动测高；失败不抓取')
    height_mode.add_argument('--height-session',
                             help='离线重算 session.json 并预演抓取；必须带 --dry-run')
    ap.add_argument('--measure-only', action='store_true',
                    help='配合 --auto-height：采集并测高后结束，不下扎/合爪')
    ap.add_argument('--scan-preview', action='store_true',
                    help='配合 --auto-height：只读当前回读并打印扫描路径，不动臂')
    ap.add_argument('--support-z-mm', type=float, default=None,
                    help='自动测高：物体所在平面相对车体安装面的高度（mm，向下为负，'
                         '如桌面 -135.5）。给出后按已知水平面测量，绝对高度更稳')
    ap.add_argument('--height-out', default=None,
                    help='自动测高数据根目录；每次创建独立会话，默认 ~/arm-height（重启不丢）')
    ap.add_argument('--k230-host', default=None,
                    help='显式指定 K230 当前地址；默认从结果流来源读取')
    ap.add_argument('--dry-run', action='store_true',
                    help='只读：锁框 → 读关节 → 算 O/S0 → 打印；不发 /arm/command、不停服务')
    ap.add_argument('--phase', choices=('aim', 'descend', 'close', 'lift', 'all'), default='aim',
                    help='aim=S1+S2 接近对准；descend=只下扎；all=先接近再下扎')
    ap.add_argument('--yes', action='store_true', help='跳过回车确认（真跑）')
    ap.add_argument('--max-step', type=float, default=6.0,
                    help='每拍每关节最多走多少度（真机默认 6.0，理由见 grasp.GraspConfig 注释）')
    ap.add_argument('--patience', type=int, default=25,
                    help='连续几拍**臂没动**就判卡住（真机默认 25 = 2.5s）')
    ap.add_argument('--t-ms', type=int, default=100,
                    help='arm_t_ms：每条 0xAC 的移动时长（默认 100 ≈ 10Hz 一拍）')
    ap.add_argument('--max-seconds', type=float, default=None,
                    help='覆盖 cfg.max_seconds（默认 40）')
    ap.add_argument('--alpha0', type=float, default=None,
                    help='覆盖 cfg.alpha0（下扎轴角的**搜索种子**，默认 −58°）。'
                         '**α0 不是"选哪个 α"，是"从哪开始搜"**：每拍每相位都拿 `prev_alpha` 当'
                         '中心、`span` 当半径去搜，第一拍的中心就是 α0 ⇒ α0 错了、'
                         '真解又在窗外，第一拍就 refuse（臂一步不动）。'
                         '目标越低越远，需要的 α 越陡：桌面(−12.6cm)、半径 17.8cm 时要 −73°…−88°。')
    ap.add_argument('--hz', type=float, default=None,
                    help='控制频率（默认 10）。真机可到 ~20——**硬上限是回读滞后 ~0.4s**，'
                         '再快就是拿过期数据发指令。会按 hz/10 同比折算 patience/close_ticks/'
                         'max_ticks，保证各判据的**秒数**不变。')
    ap.add_argument('--bias-r', type=float, default=None,
                    help='**沿半径**的静态偏置（cm，+ = 目标向外/远离车）。'
                         '实测爪尖几乎每次偏后 ~1cm（= 舵机死区稳态残差），'
                         '要消掉就填 **-1**。')
    ap.add_argument('--joint-comp', type=float, default=None,
                    help='肩/肘/腕位置环下垂补偿增益（默认 1.0；**0 = 关掉**）。'
                         'A/B 用：第 2 跑（无它）下扎通、第 4~6 跑（有它）下扎卡住。')
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()

    if args.auto_height or args.height_session:
        if args.h is not None:
            print('❌ 自动测高/离线测高不能同时指定 --h；不使用默认高度兜底。')
            return 2
        if args.auto_height and args.dry_run and not args.scan_preview:
            print('❌ --auto-height 采集会动臂，不能作为 --dry-run；'
                  '用 --scan-preview 只读检查路径，或 --height-session 离线预演。')
            return 2
        if args.auto_height and not (args.measure_only or args.scan_preview) and args.phase not in ('aim', 'all'):
            print('❌ 自动测高从观察姿态开始，只支持 --phase aim 或 all。')
            return 2
        if args.height_session and (not args.dry_run or args.measure_only or args.scan_preview):
            print('❌ 历史测高会话仅支持 --height-session ... --dry-run，不能驱动真机。')
            return 2
        try:
            from auto_height_grasp import run_auto_height
            return run_auto_height(args)
        except ImportError as e:
            print('❌ 自动测高依赖不可用：%s；见 requirements-height.txt。' % e)
            return 2
    if args.measure_only or args.scan_preview or args.height_out or args.support_z_mm is not None:
        print('❌ --measure-only / --scan-preview / --height-out / --support-z-mm 需要 --auto-height。')
        return 2
    if args.h is None:
        args.h = DEFAULT_H_M

    if not args.box:
        print('❌ 缺少 --box l,t,r,b（用 --selftest 可以不带硬件自检）')
        return 2
    try:
        want = parse_box(args.box)
    except ValueError as e:
        print('❌ %s' % e)
        return 2

    cfg = build_cfg(args.h, max_step=args.max_step, patience=args.patience,
                    alpha0=args.alpha0, joint_comp=args.joint_comp, hz=args.hz,
                    bias_r=args.bias_r,
                    max_seconds=args.max_seconds)
    print('框 box=%s（归一化）  目标面高 h=%.4f m（%.1f mm）→ cfg.z_plane_m'
          % (want, args.h, args.h * 1000))
    print('相位 %s   限速 %.2f°/拍   %.0f Hz' % (args.phase, cfg.max_step_deg, cfg.hz))

    collect = None
    io, drv, stopped, locked = None, None, False, False
    host = None
    try:
        try:
            collect = _collect()
        except ImportError as e:
            print('❌ 没有 ROS（%s）—— 真跑要在树莓派上（source install/setup.bash）；'
                  '本地 Windows 只能跑 --selftest' % e)
            return 2

        # ---------------------------------------------------------------- ① 锁框
        try:
            host = args.k230_host or collect.k230_host_from_result()
        except Exception as e:                                # noqa: BLE001
            print('❌ 拿不到板子地址：%s' % e)
            return 2
        print('\n板子 %s' % host)
        rep_lock = observe.k230_cmd(host, {'box': want})
        print('锁框应答：%s' % (rep_lock,))
        if not rep_lock.get('ok'):
            print('❌ 板子没锁上框 —— **不动臂**，退出。（检查：VLC 关了吗？'
                  '板子在线吗？框在画面里吗？）')
            return 1
        locked = True

        # ---------------------------------------------------------------- ② 起手读一帧
        io = collect.ArmIO()
        io.wait_feedback()
        fb = _stable_fb(io)          # 起手必须读**可信**的一帧（实测 1/3 概率有 0）
        j = from_fields(fb)
        print('当前回读 field=[%s]' % _fmt_fields(fb))
        print('当前爪尖 (%.4f, %.4f, %.4f) m' % grasp.tip_open_m(j))
        trace = ltrace.Trace()
        trace.add(time.monotonic(), fb, fb)     # 塞进这一帧（cmd 用 fb 顶，auto 会取 fb）

        # ---------------------------------------------------------------- ③ 一条观测 → O
        print('\n等结果流刷出这个框（最多 %.0fs）...' % OBS_WAIT_S)
        obs = one_obs(want)
        if obs is None:
            print('❌ 结果流里挑不出这个框（跟踪丢了 / IoU 不达标 / 结果文件太旧）'
                  '—— 不动臂。')
            return 1
        print('框：src=%s box=[%s]  中心（AI 帧）=(%.1f, %.1f)  frame=%s'
              % (obs.src, ' '.join('%.3f' % v for v in obs.box_norm),
                 obs.u_ai, obs.v_ai, obs.frame))
        try:
            O, C, d, j_fr, t_fr = grasp.estimate_point(trace, obs, cfg.z_plane_m,
                                                       cfg.latency_s)
        except Exception as e:                                # noqa: BLE001
            print('❌ 算不出目标点 O：%s —— 不动臂。' % e)
            return 1
        print('O = (%.4f, %.4f, %.4f) m   （射线∩z=%.1fmm 平面；延迟 %.0fms）'
              % (O + (cfg.z_plane_m * 1000, cfg.latency_s * 1000)))
        tip = grasp.tip_open_m(j_fr)
        print('拍这张图时的爪尖 (%.4f, %.4f, %.4f) m   爪尖→O = %.1f mm'
              % (tip + (math.dist(tip, O) * 1000,)))

        lvl, tgt, msg = grasp.plan_verdict(O, cfg)
        print('S0 判定：%s —— %s' % (lvl, msg))
        if tgt is not None:
            print('  末态 α=%.1f°  field=[%s]  余量=%d count'
                  % (tgt.alpha, _fmt_fields(tgt.fields[2:5]), tgt.slack))
        else:
            print('  （S0 没给出末态姿态 ⇒ 这个点现在干不了）')

        # ---------------------------------------------------------------- ④ dry-run 收工
        if args.dry_run:
            print('\n--dry-run：一条 /arm/command 都没发、服务一秒都没停。')
            return 0

        # ---------------------------------------------------------------- ⑤ 真跑
        if lvl == 'refuse' and not args.yes:
            print('⛔ 拒绝执行（S0 refuse）。要硬来加 --yes（不建议）。')
            return 3

        # 发指令前把**第一拍要发的那条**打出来（人看的就是这个）
        if tgt is not None:
            # `limit_step` 给的是**关节 dict**，要过 `to_fields` 才是发出去的 6 个 field
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

        # ---- 动臂段：自己的 try/finally（跟踪器的释放在**外层** finally）----
        try:
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

            link = GraspLink(io, host, want, hz=cfg.hz)
            # ★ 'all' 交给 grasp.run **一次**走完四相：分段调用会丢估计器与时间缓冲，
            #   相位之间就"断流"了（而设计要求的正是连续流式）。
            phases = [args.phase]
            rc_all = 0
            for ph in phases:
                rep = grasp.run(cfg, link, phase=ph, log=print)
                print('\n[%s] %s' % (ph, rep['stopped']))
                _print_rows(rep)
                _print_trace(link)
                if rep.get('target') is not None:
                    print('   目标姿态 field=[%s]（α=%.1f°）'
                          % (['%5.1f' % v for v in rep['target'].fields],
                             rep['target'].alpha))
                print('   拍了 %d 拍  观测 %d 条（挑不出框 %d 次）'
                      % (rep['ticks'], link.n_obs, link.n_none))
                if rep['err_m'] is not None:
                    print('   最终 爪尖→目标点 %.2fmm  余量 slack=%s'
                          % (rep['err_m'] * 1000.0,
                             '-' if rep['target'] is None
                             else '%.0f' % rep['target'].slack))
                if not rep['ok']:
                    print('❌ [%s] 没到位：%s' % (ph, rep['stopped']))
                    rc_all = 1
                    break
            return rc_all
        finally:
            # 驱动 -> 服务，一步步收回来（收驱动出错**不许**连累恢复服务）
            if drv is not None:
                try:
                    collect._stop_driver(drv)
                except Exception as e:                        # noqa: BLE001
                    print('⚠️ 收驱动出错（继续恢复服务）：%s' % e)
            if stopped:
                collect._svc('start')
                print('✅ 服务已恢复：%s（臂已 INIT_HOME 归位，别伸手进去）'
                      % collect._svc_active())
    except KeyboardInterrupt:
        print('\n⚠️ Ctrl-C 中断')
        return 1
    except Exception as e:                                    # noqa: BLE001
        print('\n❌ 出错了：%s' % e)
        return 1
    finally:
        if io is not None:
            try:
                io.close()
            except Exception:                                 # noqa: BLE001
                pass
        if locked:
            # 一次性语义（design §8）：锁过就必须释放，否则板子一直占着跟踪器。
            try:
                print('释放跟踪器：%s' % (observe.k230_cmd(host, {'cmd': 'stop'}),))
            except Exception as e:                            # noqa: BLE001
                print('⚠️ 释放跟踪器失败：%s' % e)


if __name__ == '__main__':
    sys.exit(main())
