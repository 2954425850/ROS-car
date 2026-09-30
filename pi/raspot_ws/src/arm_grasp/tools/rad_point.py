# -*- coding: utf-8 -*-
"""记一个「尺子量出的爪尖水平半径」标定点（只读回读）。

    rad_point.py <out.json> --r 17.0 [--az 0] [--note "高伸"]

真值：爪尖到**底座转轴**的水平距离 = --r（cm）；--az 是方位角（度，正=左），可省。
每记一条会报「模型 r」与「差多少」—— **差恒定 = 水平偏移；差随姿态变 = 肩/肘拆分错**。
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.arm_kin import fk, from_fields


def read_feedback(timeout=6.0):
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node('rad_point')
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
    ap.add_argument('--r', type=float, required=True, help='爪尖到底座转轴的水平距离 cm')
    ap.add_argument('--az', type=float, default=None, help='方位角 deg（正=左），可省')
    ap.add_argument('--note', default='')
    args = ap.parse_args()

    fb = read_feedback()
    if 0 in fb[2:6]:
        print('⚠️  回读里有 0 —— 再叫一次'); return 1
    j = from_fields(fb)
    tip, _ = fk(j)
    r_model = math.hypot(tip[0], tip[1])
    az_model = math.degrees(math.atan2(tip[1], tip[0]))

    rows = json.load(open(args.out)) if os.path.exists(args.out) else []
    rows.append({'joints': j, 'fb': fb, 'r_true': args.r, 'az_true': args.az,
                 'r_model': r_model, 'az_model': az_model, 'note': args.note})
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)

    print('✅ 第 %d 点  回读 %s' % (len(rows), ' '.join('%5.0f' % x for x in fb)))
    print('   关节  肩 %6.1f  肘 %6.1f  腕 %6.1f  底座 %6.1f' % (
        j['shoulder'], j['elbow'], j['wrist_pitch'], j['base']))
    print('   r  真值 %7.2f   模型 %7.2f   **差 %+6.2f cm**' % (args.r, r_model, r_model - args.r))
    if args.az is not None:
        print('   方位角 真值 %6.1f   模型 %6.1f   差 %+6.1f°' % (args.az, az_model, az_model - args.az))
    return 0


sys.exit(main())
