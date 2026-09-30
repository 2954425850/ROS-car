# -*- coding: utf-8 -*-
"""视觉伺服：把爪尖**动到**瓶盖上（第一、二步）。会动臂。

    # ① 自检（不需要硬件、不需要板子）
    python3 tools/servo_grasp.py --selftest

    # ② 只读预演：读当前姿态 + 抓一帧，把**整条计划**算出来打给你看，**不发任何指令**
    python3 tools/servo_grasp.py --dry-run

    # ③ 真跑：对准（只横着走、不下扎）
    python3 tools/servo_grasp.py --phase aim

    # ④ 下扎（对准之后，把留量压到 0）
    python3 tools/servo_grasp.py --phase descend

## 安全（每条都是踩出来的）
* **先 `systemctl stop ps2-teleop.service`（不许 kill**，`Restart=always` 会被拉起
  → 两个实例抢串口，dropped 疯涨）。停完**核实** inactive 才动臂。
* 停服务会**把驱动一起停掉**（两者同一个 service）⇒ 必须**单独起**
  `l150pro_driver_node`，否则收不到 `/arm/feedback`。
* 停/恢复全部焊在 `try/finally` 里（含 Ctrl-C）—— `tools/jacobian.py` 的隐患
  就是停服务那两步在 `try` 之前，一抛异常服务就留在停止态。**别照抄它。**
* 全程用 `arm_t_ms`（默认 600）把每条 0xAC 放慢；每条 `/arm/command` 即发一帧。
* **VLC 开着时拍照会失败**（板子那边一会话只能一条管线），跑之前先关掉。
* 发指令前把要发的关节值打印出来；`--yes` 才跳过回车确认。

## 这一步只做「对准」
对准阶段只横着走、爪尖始终在瓶盖顶面之上 —— 不会碰任何东西。
要看下扎，先确认对准落下来的那张标注图（红十字在不在两只爪中间）。
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')
# 本地（Windows）也能跑 --selftest：源码树就在脚本上一层
_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if not os.path.isdir('/home/cy/raspot_ws/src/arm_grasp') and os.path.isdir(_HERE):
    sys.path.insert(0, _HERE)

from arm_grasp import geom, servo
from arm_grasp.arm_kin import from_fields, to_fields
from arm_grasp.collect import (ArmIO, SERVICE, _start_driver, _stop_driver,
                               _svc, _svc_active, blue_blob_box, capture_frame,
                               k230_host_from_result)

SHOT_DIR = '/tmp/k230/servo'


def _writable_dir():
    """自检要落一张合成图；本地 Windows 没有 /tmp。"""
    for d in (SHOT_DIR, os.path.join(_HERE, 'docs', 'servo-selftest')):
        try:
            os.makedirs(d, exist_ok=True)
            return d
        except OSError:
            continue
    import tempfile
    return tempfile.mkdtemp()


# --------------------------------------------------------------------------
# 只读的小工具
# --------------------------------------------------------------------------

def stable_fb(io, timeout=6.0, tol=4.0):
    """读一帧**稳**的回读（底座电位器本底噪 ±1 count；p3/p4/p5 有 0 就是该拍没读到）。"""
    t0, prev = time.time(), None
    while time.time() - t0 < timeout:
        io.spin(0.1)
        if io.fb is None:
            continue
        fb = [float(v) for v in io.fb]
        if len(fb) < 6 or 0 in fb[2:6]:
            prev = None
            continue
        if prev is not None and all(abs(fb[k] - prev[k]) <= tol for k in range(6)):
            return fb
        prev = fb
    raise RuntimeError('读不到稳的回读（p3/p4/p5 有 0，或者一直在跳）')


def measure(j, box, cap_z_m):
    """(特征像素, 用的哪个估计, 瓶盖三维点, 射线方向, 光心)；估不出来返回 (None,)。"""
    if box is None:
        return None, 'no-blob', None, None, None
    u_r = 0.5 * (box['u0'] + box['u1'])
    v_r = 0.5 * (box['v0'] + box['v1'])
    _, dref = geom.pixel_ray(j, *geom.to_ai(u_r, v_r))
    feat, how = servo.cap_center(box['u0'], box['u1'], box['v0'], box['v1'],
                                 box['cu'], box['cv'], dref[2])
    if feat is None:
        return None, how, None, None, None
    P, d, C = servo.cap_point(j, *geom.to_ai(*feat), cap_z_m)
    return feat, how, P, d, C


def annotate(jpg, out, aim_px, box, feat, text):
    """在**原始帧**上画：瞄准红十字 / 蓝块包围盒 / 估计的瓶盖中心 / 状态文字。

    这张图是给**人眼**判的：红十字落在两只爪中间没有、瓶盖在不在爪子下面。
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return None
    im = Image.open(jpg).convert('RGB')
    W, H = im.size
    d = ImageDraw.Draw(im)
    for x in range(0, W, 100):
        d.line([(x, 0), (x, H)], fill=(150, 150, 150))
        d.text((x + 3, 3), str(x), fill=(255, 255, 0))
    for y in range(0, H, 100):
        d.line([(0, y), (W, y)], fill=(150, 150, 150))
        d.text((3, y + 3), str(y), fill=(255, 255, 0))
    ax, ay = aim_px
    d.line([(ax - 80, ay), (ax + 80, ay)], fill=(255, 0, 0), width=3)
    d.line([(ax, ay - 80), (ax, ay + 80)], fill=(255, 0, 0), width=3)
    d.text((ax + 10, ay - 70), 'AIM', fill=(255, 0, 0))
    if box is not None:
        d.rectangle([box['u0'], box['v0'], box['u1'], box['v1']],
                    outline=(0, 255, 0), width=3)
    if feat is not None:
        cx, cy = feat
        d.line([(cx - 30, cy), (cx + 30, cy)], fill=(0, 255, 255), width=2)
        d.line([(cx, cy - 30), (cx, cy + 30)], fill=(0, 255, 255), width=2)
        d.text((cx + 12, cy + 6), 'CAP', fill=(0, 255, 255))
    for i, line in enumerate(text.split('\n')):
        d.text((10, 60 + i * 16), line, fill=(255, 255, 255))
    im.save(out, quality=88)
    return out


def selftest():
    """不需要硬件的自检：走的是**接下来真用的那几个函数**。"""
    ok = True
    aim_ai = geom.aim_pixel(closed=False)
    aim_px = geom.to_stream(*aim_ai)
    print('瞄准像素  AI(320x180)=(%.3f, %.3f)  根(1280x720)=(%.3f, %.3f)'
          % (aim_ai[0], aim_ai[1], aim_px[0], aim_px[1]))
    for got, want, name in ((aim_ai[0], 132.546, 'aim u(AI)'),
                            (aim_ai[1], 177.450, 'aim v(AI)'),
                            (aim_px[1], 709.80, 'aim v(根)')):
        if abs(got - want) > 0.02:
            print('  ✗ %s = %.3f（应为 %.3f）' % (name, got, want))
            ok = False
    print('手眼欧氏 张开 %.4f m / 夹紧 %.4f m（差 %.1f mm）'
          % (geom.handeye_range(False), geom.handeye_range(True),
             (geom.handeye_range(True) - geom.handeye_range(False)) * 1000))
    for tgt, a in (((-0.055, -0.195, 0.100), -50.0),
                   ((0.020, -0.220, 0.020), -60.0)):
        err = math.dist(tgt, servo.tip_open_m(servo.ik_open_m(*tgt, a))) * 1000
        print('ik_open_m 往返误差 %.2e mm（目标 %s, alpha %.0f°）' % (err, tgt, a))
        if err > 1e-3:
            ok = False
    # 合成一张「下沿被裁的蓝盘」，走**真正用的**那条 读图 -> 包围盒 -> 估中心 的链
    try:
        from PIL import Image, ImageDraw
        p = os.path.join(_writable_dir(), 'selftest.jpg')
        im = Image.new('RGB', (1280, 720), (90, 80, 70))
        d = ImageDraw.Draw(im)
        # ★ 这一组数是**对准那一刻的真实几何**（见 servo 模块头坑 #3）：
        #   中心正好落在瞄准像素上（v=709.8），水平半径 139px，盘面倾角 |d_z|=0.9
        #   ⇒ 竖直半轴 125px ⇒ 下沿在 834.9，画框只有 720 —— 底边一定被裁。
        cx, cy, r = 530.0, 709.8, 139.0
        d.ellipse([cx - r, cy - 0.9 * r, cx + r, cy + 0.9 * r], fill=(37, 133, 255))
        im.save(p, quality=95)
        box = blue_blob_box(p)
        c, how = servo.cap_center(box['u0'], box['u1'], box['v0'], box['v1'],
                                  box['cu'], box['cv'], -0.9)
        print('合成裁剪盘 包围盒 v[%.0f,%.0f] 质心 v=%.0f -> 估计 v=%.1f'
              '（真值 %.1f，用 %s）'
              % (box['v0'], box['v1'], box['cv'], c[1] if c else float('nan'),
                 cy, how))
        if how != 'box' or abs(c[1] - cy) > 20.0:
            print('  ✗ 裁剪盘的圆心估计不对（质心会偏 ~60px，就是靠这条躲开的）')
            ok = False
    except ImportError:
        print('  (没有 PIL/numpy，跳过合成图检查)')
    print('\n%s  闭环控制律本身在 <包根>/test/test_servo.py 里覆盖（22 条，'
          '含 5 处变异自检）：\n    python3 -m pytest test/test_servo.py -q'
          % ('自检通过' if ok else '自检**失败**'))
    return 0 if ok else 1


# --------------------------------------------------------------------------
# 硬件
# --------------------------------------------------------------------------

def make_hooks(io, host, cfg, out_dir, tag, shot_timeout):
    """造 `run_phase` 要的 observe()/command()。每拍落一张标注图。"""
    st = {'k': 0, 'files': []}
    aim_px = geom.to_stream(*geom.aim_pixel(closed=False))

    def observe():
        st['k'] += 1
        fb = stable_fb(io)
        j = from_fields(fb)
        jpg = os.path.join(out_dir, '%s-%02d.jpg' % (tag, st['k']))
        if not capture_frame(jpg, host, timeout=shot_timeout):
            raise RuntimeError('拍照失败（VLC 还开着？板子在推流吗？）')
        box = blue_blob_box(jpg)
        feat, how, P, d, C = measure(j, box, cfg.cap_z_m)
        tip = servo.tip_open_m(j)
        if feat is None:
            txt = 'fb %s\ntip (%.3f, %.3f, %.3f) m\n**画面里没有瓶盖/量不出来**'
            txt = txt % tuple([' '.join('%4.0f' % v for v in fb)] + list(tip))
        else:
            txt = ('fb %s\ntip (%.3f, %.3f, %.3f) m   盖 (%.3f, %.3f, %.3f) m\n'
                   '爪尖离盖 %.2f cm   像素偏差 %.1f px   特征用 %s'
                   % tuple([' '.join('%4.0f' % v for v in fb)] + list(tip)
                           + list(P)
                           + [math.dist(tip, P) * 100,
                              math.hypot(feat[0] - aim_px[0], feat[1] - aim_px[1]),
                              how]))
        out = os.path.join(out_dir, '%s-%02d-annot.jpg' % (tag, st['k']))
        if annotate(jpg, out, aim_px, box, feat, txt):
            st['files'].append(out)
        return fb, box

    def command(fields):
        got, dt = io.wait_arrived(fields, settle=1.5, tol=8.0, timeout=10.0)
        if got is None:
            raise RuntimeError('等了 %.1fs 回读还没跟上指令（这一拍不算数）' % dt)
        return observe()

    return observe, command, st


def main(argv=None):
    ap = argparse.ArgumentParser(description='机械臂视觉伺服（会动臂！）')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--dry-run', action='store_true',
                    help='只读：算一遍并打出来，**不发任何指令**')
    ap.add_argument('--phase', choices=('aim', 'descend', 'all'), default='aim')
    ap.add_argument('--cap-z', type=float, default=0.016,
                    help='瓶盖顶面高度（米）。⚠️ 这个数有 ±1cm 的不确定')
    ap.add_argument('--max-step', type=float, default=6.0, help='每拍每关节最多走多少度')
    ap.add_argument('--tol-px', type=float, default=10.0,
                    help='对准判据（1280x720 系）；10px ≈ 2mm')
    ap.add_argument('--alpha0', type=float, default=-58.0)
    ap.add_argument('--shot-timeout', type=float, default=6.0, help='抓一帧最多等几秒')
    ap.add_argument('--t-ms', type=int, default=600, help='0xAC 每帧移动时长（驱动参数）')
    ap.add_argument('--k230-host', default=None)
    ap.add_argument('--out-dir', default=SHOT_DIR)
    ap.add_argument('--no-open-gripper', action='store_true',
                    help='不先把夹爪开到 240（**不建议**：几何用的是张开态）')
    ap.add_argument('--yes', action='store_true', help='跳过回车确认')
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()

    os.makedirs(args.out_dir, exist_ok=True)
    host = args.k230_host
    if not host:
        try:
            host = k230_host_from_result()
        except (OSError, RuntimeError):
            print('拿不到板子地址（结果文件不可用）。`ip neigh` 里有这些：')
            import subprocess
            print('  ' + (subprocess.run(['ip', 'neigh'], capture_output=True,
                                         text=True).stdout or '').replace('\n', '\n  ').strip())
            print('用 --k230-host 手工指定板子 IP 再跑。')
            return 2

    cfg = servo.ServoConfig(cap_z_m=args.cap_z, max_step_deg=args.max_step,
                            alpha0=args.alpha0, tol_px=args.tol_px)
    print('=' * 74)
    print('机械臂视觉伺服 · 板子 %s · cap_z=%.3f m  max_step=%.1f°  tol=%.1fpx  '
          'phase=%s' % (host, cfg.cap_z_m, cfg.max_step_deg, cfg.tol_px, args.phase))

    io, drv, stopped, reports = None, None, False, []
    try:
        # ---------------- ① 先只读地看一眼（走的是接下来真用的那套读法） -------------
        io = ArmIO()
        io.wait_feedback()
        fb = stable_fb(io)
        j = from_fields(fb)
        tip = servo.tip_open_m(j)
        raw = os.path.join(args.out_dir, 'dryrun.jpg')
        if not capture_frame(raw, host, timeout=args.shot_timeout):
            print('✗ 抓不到画面（VLC 开着？板子没推流？）')
            return 2
        box = blue_blob_box(raw)
        print('回读    %s   关节 肩%.1f 肘%.1f 腕%.1f 底%.1f'
              % (' '.join('%4.0f' % v for v in fb), j['shoulder'], j['elbow'],
                 j['wrist_pitch'], j['base']))
        print('爪尖    (%.3f, %.3f, %.3f) m' % tip)
        aim_px = geom.to_stream(*geom.aim_pixel(closed=False))
        feat, how, P, d, C = measure(j, box, cfg.cap_z_m)
        if feat is None:
            print('✗ 量不出瓶盖（%s）—— 换个能看见它的姿态再来。' % how)
            annotate(raw, os.path.join(args.out_dir, 'dryrun-annot.jpg'), aim_px,
                     box, None, '*** 量不出瓶盖: %s ***' % how)
            return 2
        err0 = math.hypot(feat[0] - aim_px[0], feat[1] - aim_px[1])
        print('瓶盖    特征像素 (%.1f, %.1f)[%s]   估计三维 (%.3f, %.3f, %.3f) m'
              % (feat[0], feat[1], how, P[0], P[1], P[2]))
        print('        相机离盖 %.1f cm   爪尖离盖 %.1f cm   当前像素偏差 %.1f px'
              % (math.dist(P, C) * 100, math.dist(tip, P) * 100, err0))
        print('对准计划 留量（沿视线离瓶盖多远）在 [%.1f, %.1f] cm 里自由搜'
              % (cfg.aim_clear_m * 100, cfg.aim_far_m * 100))
        try:
            pose = servo.pick_pose(P, d, aim_px, cfg.aim_clear_m,
                                   cfg.aim_far_m, cfg.alpha0,
                                   cfg.gripper, cfg.wrist_roll, cfg.alpha_lo,
                                   cfg.alpha_hi, cfg.alpha_span,
                                   n_standoff=cfg.n_standoff,
                                   tie_px=cfg.alpha_tie_px,
                                   min_slack=cfg.min_slack)
            tgt = servo.standoff_target(P, d, pose.standoff_m)
            jc, ratio, reached = servo.limit_step(j, pose.joints, cfg.max_step_deg)
            f = to_fields(jc, cfg.gripper, cfg.wrist_roll)
            print('第一拍   目标 (%.3f, %.3f, %.3f) m（留量 %.1f cm）alpha=%.1f°'
                  % (tgt[0], tgt[1], tgt[2], pose.standoff_m * 100, pose.alpha))
            print('         发 %s  %s' % (' '.join('%4.0f' % v for v in f),
                                          '（一步到位）' if reached else '（限幅，分几拍）'))
            print('         该姿态 field = %s  预计走 %.0f%%  预测残差 %.1f px  离限位 %d count'
                  % (['%.0f' % v for v in pose.fields[2:5]], ratio * 100,
                     pose.err_px, pose.slack))
            # ★ 2026-09-29 第一次上机就是没看这一条：姿态贴到工作空间边缘，
            #   肘关节一路顶死在 −90°（p4=124 < 下限 125）、指令到 −88° 也走不到，
            #   伺服算得再对也到不了位。**宁可现在停下来说一句。**
            if pose.slack < cfg.min_slack:
                print('   ⚠️⚠️ 离固件限位只剩 %d count（想要 ≥%d）—— 这个瓶盖位置'
                      '贴到臂的工作空间边缘了。\n        伺服大概率走到一半就顶死'
                      '（实测过）。把瓶盖往底座方向挪近 5~8cm 再跑。'
                      % (pose.slack, cfg.min_slack))
                if not args.yes:
                    return 3
        except servo.ServoRefused as e:
            print('✗ 算不出可行解：%s' % e)
            return 2
        annotate(raw, os.path.join(args.out_dir, 'dryrun-annot.jpg'), aim_px, box,
                 feat, 'DRY-RUN  fb %s\n爪尖离盖 %.2f cm   像素偏差 %.1f px'
                 % (' '.join('%4.0f' % v for v in fb), math.dist(tip, P) * 100, err0))
        print('        标注图 -> %s' % os.path.join(args.out_dir, 'dryrun-annot.jpg'))

        if args.dry_run:
            print('\n--dry-run：到此为止，**一条指令都没发**，臂和 service 都没碰。')
            return 0

        # ---------------- ② 真要动了 ----------------
        print('\n⚠️  下一步：')
        print('   1) sudo systemctl stop %s（不 kill！）' % SERVICE)
        print('   2) 单独起 l150pro_driver_node（驱动和摇杆在同一个 service 里）')
        print('   3) %s然后流式发 /arm/command'
              % ('不动夹爪；' if args.no_open_gripper else '先把夹爪开到 240；'))
        print('   期间摇杆无效；结束（无论成败）自动杀掉驱动、start 回服务。')
        if not args.yes:
            try:
                input('确认臂的行程里没有东西、人离远点，回车开跑（Ctrl-C 取消）：')
            except EOFError:
                print('没有交互输入，用 --yes 才能跑。退出。')
                return 2

        rc = _svc('stop')
        if rc.returncode != 0:
            print('❌ 停服务失败：%s' % (rc.stderr or rc.stdout).strip())
            return 1
        stopped = True
        time.sleep(1.0)
        st = _svc_active()
        if st != 'inactive':
            print('❌ %s 还是 %s —— **没有动臂**' % (SERVICE, st))
            return 1
        print('✅ %s 已停' % SERVICE)

        drv = _start_driver(args.t_ms or None)
        time.sleep(2.0)
        # ★ **只能有一个 rclpy 节点**：`rclpy.init()` 在一个进程里只能调一次。
        #   第一版这里又建了一个 ArmIO 想"换个节点名"，直接
        #   `Context.init() must only be called once` —— 好在它发生在发任何指令之前。
        io.wait_feedback()
        if not args.no_open_gripper:
            jj = from_fields(stable_fb(io))
            f_open = to_fields(jj, 240.0, 496.0)
            print('→ 先把夹爪开到 240（几何是张开态的）：%s'
                  % ' '.join('%4.0f' % v for v in f_open))
            io.wait_arrived(f_open, settle=1.5, tol=8.0, timeout=8.0)

        phases = ('aim', 'descend') if args.phase == 'all' else (args.phase,)
        for ph in phases:
            print('\n' + '-' * 74)
            print('阶段 [%s]' % ph)
            obs, cmd, stk = make_hooks(io, host, cfg, args.out_dir, ph,
                                       args.shot_timeout)
            rep = servo.run_phase(cfg, obs, cmd, phase=ph, log=print)
            rep['shots'] = stk['files']
            reports.append(rep)
            print('阶段 [%s] 结果：%s' % (ph, rep['stopped']))
            if not rep['ok'] and ph != phases[-1]:
                print('⚠️  这一阶段没成功，不再往下走。')
                break
    except KeyboardInterrupt:
        print('\n⚠️  被 Ctrl-C 打断。')
    except Exception as e:                                   # noqa: BLE001
        print('\n❌ 出错了：%s' % e)
    finally:
        if io is not None:
            try:
                io.close()
            except Exception:                                # noqa: BLE001
                pass
        if drv is not None:
            _stop_driver(drv)
        if stopped:
            rc = _svc('start')
            st = _svc_active()
            print('\n%s systemctl start -> %s' % ('✅' if st == 'active' else '❌', st))

    if not reports:
        return 1
    path = os.path.join(args.out_dir,
                        'servo-report-%s.json' % time.strftime('%Y%m%d-%H%M%S'))
    with open(path, 'w', encoding='utf-8') as f:
        json.dump([dict(r, iters=[list(i) for i in r['iters']]) for r in reports],
                  f, ensure_ascii=False, indent=1)
    print('报告 -> %s' % path)
    for r in reports:
        print('  [%s] %s' % (r['phase'], r['stopped']))
    return 0 if all(r['ok'] for r in reports) else 1


if __name__ == '__main__':
    sys.exit(main())
