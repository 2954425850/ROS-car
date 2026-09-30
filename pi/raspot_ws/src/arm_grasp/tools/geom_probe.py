# -*- coding: utf-8 -*-
"""几何链路的验收探针（**只读**：不发指令、不碰 ps2-teleop.service）。

摆姿态用遥控器（或手推底座）。脚本只读 /arm/feedback 和 K230 的画面。

    # 自检（不需要硬件、不需要板子；但走的是下面真正用的那几个函数）
    python3 tools/geom_probe.py --selftest

    # P3：把爪尖摆到瓶盖上 → 瓶盖必须落在**瞄准像素**上
    python3 tools/geom_probe.py --mode p3 --out docs/p3.json

    # P4：瓶盖在尺子量过的已知点、摆个看得见它的姿态 → 算出来的坐标 vs 尺子
    python3 tools/geom_probe.py --mode p4 --truth 0,-8.3,1.6 --out docs/p4.json

## P3 为什么不需要找爪尖
「爪尖落在瓶盖上」等价于「瓶盖落在爪尖投影处」= 瞄准像素。
瞄准像素是个**常数**（只由手眼 4 个数 + 标定内参决定），所以只要量**瓶盖**的像素。
（旧的 `tip_px` 那条路已作废：实测 16 条里 v 全挤在 296~359 的"上半幅边界"，
锁到的是画面边界不是爪尖；`docs/tip-pixels.json` 至今是空的。）

## 像素坐标系
检测/跟踪/拍照都在**根分辨率 1280x720**；标定的 K 在 **AI 通道 320x180**（均匀 ÷4）。
本脚本一律在**归一化坐标**里比，并把 1280x720 的绝对值也打出来给你核对。
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp import geom
from arm_grasp.arm_kin import fk, from_fields


# ---------------- 只读回读（与 tools/pose.py 里的那份相同，待合并） ----------------

def read_feedback(timeout=5.0):
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node('geom_probe')
    box = {'fbs': []}
    node.create_subscription(JointState, '/arm/feedback',
                             lambda m: box['fbs'].append([float(x) for x in m.position]), 10)
    t0 = time.time()
    while time.time() - t0 < timeout and len(box['fbs']) < 6:
        rclpy.spin_once(node, timeout_sec=0.1)
    fbs = box['fbs']
    node.destroy_node()
    rclpy.shutdown()
    if not fbs:
        raise RuntimeError('收不到 /arm/feedback')
    for i in range(len(fbs) - 1, 0, -1):
        a, b = fbs[i], fbs[i - 1]
        if len(a) == 6 and len(b) == 6 and all(abs(a[k] - b[k]) <= 4 for k in range(6)):
            return a
    return fbs[-1]


# ---------------- 两个比较函数（selftest 也走这两个） ----------------

def p3_compare(cap_px_stream, predicted_px_stream, z_tip=0.119):
    """瓶盖像素 vs 瞄准像素。都在 1280x720 系。返回 dict。"""
    du = cap_px_stream[0] - predicted_px_stream[0]
    dv = cap_px_stream[1] - predicted_px_stream[1]
    err_px = math.hypot(du, dv)
    fx_stream = geom.K_AI320x180_CHN2.fx * (geom.STREAM_W / geom.K_AI320x180_CHN2.w)
    return {'err_px': err_px, 'du_px': du, 'dv_px': dv,
            'err_mm': err_px / fx_stream * z_tip * 1000.0}


def p4_compare(target, truth):
    """算出来的目标坐标 vs 尺子量出来的。都在基座系、米。返回 dict。"""
    rt = math.hypot(truth[0], truth[1])
    rp = math.hypot(target[0], target[1])
    at = math.degrees(math.atan2(truth[1], truth[0]))
    ap = math.degrees(math.atan2(target[1], target[0]))
    daz = (ap - at + 180.0) % 360.0 - 180.0
    return {'dist_m': math.dist(target, truth),
            'dr_cm': (rp - rt) * 100.0,
            'daz_deg': daz,
            'dz_cm': (target[2] - truth[2]) * 100.0,
            'r_model_cm': rp * 100.0, 'r_truth_cm': rt * 100.0,
            'target': list(target)}


# ---------------- 采一拍 ----------------

def capture_cap_px(host, jpg):
    """抓一帧、找蓝色块（瓶盖）。返回 (u, v, n) 在 1280x720 系，或 None（=没找到瓶盖）。"""
    from arm_grasp.collect import capture_frame, blue_blob_px
    if not capture_frame(jpg, host):
        raise RuntimeError(
            '抓不到画面（rtsp://%s:8554/k230）—— 板子没在推流，或者地址不对。\n'
            '    最常见的原因：**板子上跑的是标定那支 t_cam_ai_rgb.py，不是正常的 app.py**。\n'
            '    （「板子没网 ≈ app.py 根本没跑」—— WiFi 是 app.py 起的）\n'
            '    → 让板子重启回正常 app，再看 `ip neigh` 里有没有板子、8554 开没开' % host)
    return blue_blob_px(jpg)


def annotate(jpg, out, aim_stream, cap_px=None):
    """在**原始帧**上画：瞄准像素红十字 + 每 100px 网格 + 瓶盖质心。

    这是最干净的一次性检验：**人眼直接看爪尖在不在红十字上**，
    不需要任何测量、不依赖 FK、不依赖颜色质心。
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print('   (没有 PIL，跳过标注)')
        return None
    im = Image.open(jpg).convert('RGB')
    W, H = im.size
    d = ImageDraw.Draw(im)
    for x in range(0, W, 100):
        d.line([(x, 0), (x, H)], fill=(200, 200, 200))
        d.text((x + 3, 3), str(x), fill=(255, 255, 0))
    for y in range(0, H, 100):
        d.line([(0, y), (W, y)], fill=(200, 200, 200))
        d.text((3, y + 3), str(y), fill=(255, 255, 0))
    ax, ay = aim_stream
    d.line([(ax - 70, ay), (ax + 70, ay)], fill=(255, 0, 0), width=3)
    d.line([(ax, ay - 70), (ax, ay + 70)], fill=(255, 0, 0), width=3)
    d.text((ax + 10, ay - 60), 'AIM (%.0f,%.0f)' % (ax, ay), fill=(255, 0, 0))
    if cap_px:
        cx, cy = cap_px
        d.ellipse([cx - 10, cy - 10, cx + 10, cy + 10], outline=(0, 255, 0), width=3)
        d.text((cx + 12, cy + 8), 'CAP', fill=(0, 255, 0))
    im.save(out, quality=88)
    return out


def sample(host, jpg):
    fb = read_feedback()
    if 0 in fb[2:6]:
        raise RuntimeError('回读里有 0（底座/肩/肘/腕任一）—— 再叫一次')
    j = from_fields(fb)
    tip, _ = fk(j)
    alpha = j['shoulder'] + j['elbow'] + j['wrist_pitch']
    cap = capture_cap_px(host, jpg)
    return fb, j, tip, alpha, cap


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['p3', 'p4'], default='p3')
    ap.add_argument('--truth', default=None, help='P4 的尺子真值 x,y,z (cm)')
    ap.add_argument('--z-plane', type=float, default=geom.CAP_HEIGHT,
                    help='目标所在平面高度（米）；瓶盖在车身上=顶面 0.016')
    ap.add_argument('--tol-px', type=float, default=40.0,
                    help='P3 判据：1280x720 系里的像素容差（40px ≈ 4.3mm）')
    ap.add_argument('--k230-host', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--note', default='')
    ap.add_argument('--jpg', default='/tmp/geom_probe.jpg')
    ap.add_argument('--annotate', default=None,
                    help='把瞄准像素画到图上并存到这个路径，人眼核对用')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args(argv)

    aim_ai = geom.aim_pixel(closed=False)
    aim_st = geom.to_stream(*aim_ai)

    if args.selftest:
        # 合成：取一个姿态，把爪尖投出去（= 瞄准像素），再走一遍比较函数。
        # 走的是和实拍**同一套函数**，所以能验管道；红了就是代码错。
        pose = dict(base=6.0, shoulder=110.88, elbow=-89.28, wrist_pitch=-77.04)
        tip = geom.gripper_tip(pose, closed=False)
        px = geom.project(pose, tip)
        st = geom.to_stream(*px)
        r = p3_compare(st, aim_st)
        assert r['err_px'] < 0.01, r
        # 归一化坐标必须跨分辨率不变
        assert abs(geom.normalized(*aim_st, geom.STREAM_W, geom.STREAM_H)[0]
                   - geom.normalized(*aim_ai)[0]) < 1e-12
        # P4 比较函数也要能红：故意把 truth 挪 3cm
        p4 = p4_compare((0.10, -0.05, 0.016), (0.10, -0.05, 0.016))
        assert p4['dist_m'] < 1e-12, p4
        # 把真值挪 3cm：比较函数必须报出这 3cm，且径向/方位角都要动。
        # ⚠️ 别指望 dr_cm == 3.0 —— 沿 x 挪 3cm 时**半径**只变了 2.75cm（方位角也变了）。
        #    （第一版就是这么写错的，被这段自检抓住。）
        bad = p4_compare((0.10, -0.05, 0.016), (0.13, -0.05, 0.016))
        assert abs(bad['dist_m'] - 0.03) < 1e-9, bad
        assert bad['dr_cm'] < -2.0 and abs(bad['daz_deg']) > 1.0, bad
        print('自检通过：P3 err=%.4f px；P4 挪 3cm -> |Δ|=%.2f cm, 径向 %+.2f cm, 方位 %+.2f°'
              % (r['err_px'], bad['dist_m'] * 100, bad['dr_cm'], bad['daz_deg']))
        print('瞄准像素  AI(320x180)=(%.2f, %.2f)   根(1280x720)=(%.2f, %.2f)   归一化=(%.4f, %.4f)'
              % (aim_ai[0], aim_ai[1], aim_st[0], aim_st[1],
                 aim_ai[0] / geom.AI_W, aim_ai[1] / geom.AI_H))
        return 0

    host = args.k230_host
    if not host:
        from arm_grasp.collect import k230_host_from_result
        try:
            host = k230_host_from_result()
        except (OSError, RuntimeError) as e:
            print('✗ 拿不到板子的地址：%s' % e)
            print('  本脚本**不需要**结果文件，只要板子的 IP（用来拉 RTSP 画面）。二选一：')
            print('   ① 直接给：   --k230-host 192.168.5.xxx')
            print('   ② 让 latest-result.json 出现（板子跑正常 app 并把结果推过来）')
            print('  查板子在不在网上：  ip neigh   （只有网关 = 板子没连上）')
            return 2
    print('板子 %s   模式 %s   目标面 z = %.3f m' % (host, args.mode, args.z_plane))

    try:
        fb, j, tip, alpha, cap = sample(host, args.jpg)
    except RuntimeError as e:
        print('✗ %s' % e)
        return 2
    print('回读  %s' % ' '.join('%5.0f' % x for x in fb))
    print('tip   x=%6.2f y=%6.2f z=%6.2f cm   α %6.1f°   底座 %6.1f°'
          % (tip[0], tip[1], tip[2], alpha, j['base']))
    if not cap:
        print('✗ 画面里没找到蓝色块（瓶盖）—— 确认瓶盖在画面里、且没被遮挡')
        return 2
    cu, cv, n = cap
    print('瓶盖  根(1280x720)=(%8.2f, %8.2f)  蓝像素 %d   归一化=(%.4f, %.4f)'
          % (cu, cv, n, cu / geom.STREAM_W, cv / geom.STREAM_H))

    if args.annotate:
        out = annotate(args.jpg, args.annotate, aim_st, (cu, cv))
        if out:
            print('标注图 -> %s   （红十字 = 预测的瞄准像素；看爪尖在不在红十字上）' % out)

    rec = {'mode': args.mode, 'when': time.strftime('%F %T'), 'note': args.note,
           'fb': fb, 'alpha': alpha, 'cap_px_stream': [cu, cv], 'blue_px': n,
           'tip_cm': list(tip), 'z_plane_m': args.z_plane}

    if args.mode == 'p3':
        r = p3_compare((cu, cv), aim_st)
        rec.update(r)
        print()
        print('瞄准像素（根系）=(%8.2f, %8.2f)   归一化=(%.4f, %.4f)'
              % (aim_st[0], aim_st[1], aim_ai[0] / geom.AI_W, aim_ai[1] / geom.AI_H))
        print('误差  Δ=(%+7.1f, %+7.1f) px   |Δ|=%6.1f px ≈ %.1f mm'
              % (r['du_px'], r['dv_px'], r['err_px'], r['err_mm']))
        ok = r['err_px'] < args.tol_px
        fx_stream = geom.K_AI320x180_CHN2.fx * (geom.STREAM_W / geom.K_AI320x180_CHN2.w)
        mm_per_px = 0.119 / fx_stream * 1000.0
        print('      这个 Δ 只是**参考**，不是判据 —— 见下。')
        print()
        print('★ P3 的正确判法：**人眼看标注图**（加 --annotate）。')
        print('  判据是「**爪尖**在不在红十字上」。')
        print('  ⚠️ 上面那个 Δ 是「瓶盖蓝色块**质心** vs 瞄准像素」—— 质心**不是**抓取点：')
        print('     夹爪夹的是瓶盖**边缘**，质心在瓶盖中间，所以 Δ 大是正常的。')
        print('  （2026-09-29 实拍核对：红十字正落在两只爪中间，肉眼判为**通过**。）')
        print('  爪尖归一化 v = %.4f，超出跟踪器可锁带 [0.107, 0.893] —— '
              '所以只能用颜色块或人眼，不能用跟踪框。' % (aim_ai[1] / geom.AI_H))
        print('  要量化到 mm 级：把爪尖摆到红十字上，再看标注图里爪尖离红十字几像素。')
        rec['passed'] = None
        rec['note'] = (rec['note'] + ' | P3 判据=人眼看标注图（爪尖是否在红十字上）').strip()
    else:
        ua, va = geom.to_ai(cu, cv)
        try:
            P, Zax, D = geom.target_from_pixel(j, ua, va, args.z_plane)
        except ValueError as e:
            print('✗ 这条姿态算不出深度：%s' % e)
            return 2
        rec['target'] = list(P)
        rec['C'] = list(geom.camera_center(j))
        rec['Z_axial_m'] = Zax
        rec['D_m'] = D
        print()
        print('相机推出  (%.2f, %.2f, %.2f) cm   深度 Z=%.2f cm (欧氏 %.2f cm)'
              % (P[0] * 100, P[1] * 100, P[2] * 100, Zax * 100, D * 100))
        if args.truth:
            truth = tuple(float(x) / 100.0 for x in args.truth.split(','))
            r = p4_compare(P, truth)
            rec.update(r)
            rec['truth'] = list(truth)
            print('尺子真值  (%.2f, %.2f, %.2f) cm' % tuple(x * 100 for x in truth))
            print('误差      径向 %+6.2f cm   方位角 %+6.2f°   高度 %+6.2f cm   |Δ| %.2f cm'
                  % (r['dr_cm'], r['daz_deg'], r['dz_cm'], r['dist_m'] * 100))
            rec['passed'] = bool(r['dist_m'] < 0.01)
        else:
            print('（没给 --truth ⇒ 只记点。把瓶盖放**同一个位置不动**，多摆几个姿态各跑一次，')
            print('  然后：python3 tools/p4_spread.py docs/p4.json —— 看这些点是不是同一个点。）')
            rec['passed'] = None

    if args.out:
        old = []
        if os.path.exists(args.out):
            try:
                old = json.load(open(args.out, encoding='utf-8'))
                if not isinstance(old, list):
                    old = [old]
            except (ValueError, OSError):
                old = []
        old.append(rec)
        with open(args.out, 'w', encoding='utf-8') as f:
            json.dump(old, f, ensure_ascii=False, indent=1)
        print('已追加到 %s（共 %d 条）' % (args.out, len(old)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
