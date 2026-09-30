# -*- coding: utf-8 -*-
"""记一个「爪尖对准已知点」的运动学标定点（只读回读，不拍照）。

    kin_point.py <out.json> --xy 17,0 [--note "高伸"]

真值：爪尖水平位置 = (x,y)。用于反解真实连杆/关节模型。
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.arm_kin import fk, from_fields


def read_feedback(timeout=6.0):
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node('kin_point')
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--xy', required=True)
    ap.add_argument('--note', default='')
    args = ap.parse_args()
    truth = tuple(float(x) for x in args.xy.split(','))

    fb = read_feedback()
    if 0 in fb[2:6]:
        print('⚠️  回读里有 0 —— 再叫一次'); return 1
    j = from_fields(fb)
    tip, _ = fk(j)
    dx, dy = tip[0] - truth[0], tip[1] - truth[1]

    rows = json.load(open(args.out)) if os.path.exists(args.out) else []
    rows.append({'joints': j, 'truth_xy': list(truth), 'fb': fb, 'note': args.note})
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)

    print('✅ 第 %d 点  回读 %s' % (len(rows), ' '.join('%5.0f' % x for x in fb)))
    print('   关节  肩 %6.1f 肘 %6.1f 腕 %6.1f 底座 %6.1f' % (
        j['shoulder'], j['elbow'], j['wrist_pitch'], j['base']))
    print('   真值 (%g,%g)   模型 (%6.2f,%6.2f)   -> 水平误差 (%+.2f, %+.2f) = %.2f cm'
          % (truth[0], truth[1], tip[0], tip[1], dx, dy, (dx * dx + dy * dy) ** 0.5))
    return 0


sys.exit(main())
