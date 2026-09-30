# -*- coding: utf-8 -*-
"""手动采一条标定样本。

用户用手柄把臂摆到"能看见瓶盖"的姿态，然后叫我跑这个脚本记一条。
（服务**不用停** —— 我们只是"听"，不发任何指令。）

    rec.py <out.json> --point 17,0,-12.3 [--note "高一点"]

每次都会：
  1. 拍一张照片，用**颜色**找瓶盖（独立于跟踪器）
  2. 检查跟踪器有没有漂；漂了就**重新锁定**在颜色质心上，等它稳住
  3. 读 /arm/feedback（要连续两条一致才算稳，避免拿到 INIT_HOME 的假值）
  4. 存一条样本（含诊断信息：照片坐标、跟踪框、两者差多少、离盖距离）
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.arm_kin import fk, from_fields
from arm_grasp.calib_solve import Sample
from arm_grasp.collect import (blue_blob_px, box_to_px, capture_frame,
                               k230_host_from_result, lock_target, pick_track,
                               read_result)

SHOT = '/tmp/k230/manual'


def read_feedback(timeout=8.0):
    """读两条一致的回读（稳了才算）。返回 6 个 field。"""
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node('rec_sample')
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
    # 从后往前找两条一致的（±4 count）
    for i in range(len(fbs) - 1, 0, -1):
        a, b = fbs[i], fbs[i - 1]
        if len(a) == 6 and len(b) == 6 and all(abs(a[k] - b[k]) <= 4 for k in range(6)):
            return a, len(fbs)
    return fbs[-1], len(fbs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--point', required=True)
    ap.add_argument('--note', default='')
    args = ap.parse_args()
    cap = tuple(float(x) for x in args.point.split(','))
    os.makedirs(SHOT, exist_ok=True)
    host = k230_host_from_result()

    fb, nfb = read_feedback()
    if 0 in fb[2:5]:
        print('❌ 回读里有 0（该拍没读到）—— 等一秒再试'); return 1
    j = from_fields(fb)
    tip, _ = fk(j)
    dcap = math.dist(tip, cap)

    idx = len(json.load(open(args.out))) if os.path.exists(args.out) else 0
    jpg = os.path.join(SHOT, 'm%02d.jpg' % (idx + 1))
    if not capture_frame(jpg, host):
        print('❌ 拍照失败'); return 1
    blob = blue_blob_px(jpg)
    if blob is None:
        print('❌ **这一拍画面里没有瓶盖** —— 换个姿态再说一声'); return 1

    d = read_result(max_age=2.0)
    o = pick_track(d.get('objs'))
    relock = False
    if o is None:
        relock = True
    else:
        b = box_to_px(o['box'], d['w'], d['h'])
        if math.hypot(b['u'] - blob[0], b['v'] - blob[1]) > 60.0:
            relock = True
    if relock:
        print('   跟踪器没跟上（或漂了），按颜色重新锁定...')
        lock_target(host, blob[0] / 1280.0, blob[1] / 720.0)
        for _ in range(16):
            time.sleep(0.5)
            d = read_result(max_age=2.0)
            o = pick_track(d.get('objs'))
            if o:
                b = box_to_px(o['box'], d['w'], d['h'])
                if math.hypot(b['u'] - blob[0], b['v'] - blob[1]) < 60.0:
                    break

    if o is None:
        print('❌ 锁定后还是拿不到跟踪框 —— 跳过'); return 1
    b = box_to_px(o['box'], d['w'], d['h'])
    du = math.hypot(b['u'] - blob[0], b['v'] - blob[1])
    snap = Sample(joints=j, point=cap, uv=(b['u'], b['v']))

    rows = json.load(open(args.out)) if os.path.exists(args.out) else []
    rows.append({'joints': j, 'point': list(cap), 'uv': [b['u'], b['v']]})
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)
    meta_p = args.out[:-5] + '.meta.json'
    metas = json.load(open(meta_p)) if os.path.exists(meta_p) else []
    metas.append({'i': idx + 1, 'fb_fields': fb, 'n_fb_msgs': nfb,
                  'uv_tracker': [b['u'], b['v']], 'uv_color': [blob[0], blob[1]],
                  'track_vs_color_px': du, 'blue_px': blob[2],
                  'box_w': b['w'], 'box_h': b['h'], 'relocked': relock,
                  'tip_to_cap_cm': dcap, 'note': args.note, 'jpg': jpg})
    with open(meta_p, 'w', encoding='utf-8') as f:
        json.dump(metas, f, ensure_ascii=False, indent=1)

    print('✅ 第 %d 条  px=(%.1f,%.1f) | 照片 %.1f,%.1f 差 %.0fpx | 瓶盖面积 %d | '
          '回读 %s | 肩%.1f 肘%.1f 腕%.1f 底座%.1f | 尖离盖 %.1fcm%s'
          % (idx + 1, b['u'], b['v'], blob[0], blob[1], du, blob[2],
             ' '.join('%4.0f' % x for x in fb), j['shoulder'], j['elbow'],
             j['wrist_pitch'], j['base'], dcap, '  [已重锁]' if relock else ''))
    return 0


sys.exit(main())
