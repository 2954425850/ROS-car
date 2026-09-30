# -*- coding: utf-8 -*-
"""记一个「爪尖轻贴桌面」的运动学标定点（只读回读）。

    desk_point.py <out.json> --desk -13.6 [--note "伸远"]

真值：爪尖 z = 桌面高度。**只有 z 是精确已知的**，水平位置不用量、也不用管。
每记一条，顺手算「如果其余连杆都对，L4 应该是多少」—— 若这一列恒定，
说明错的是 L4；若它随姿态变，说明错在别的项（尤其是 L3·sin(肩+肘)）。
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.arm_kin import L1, L2, L3, L4, fk, from_fields


def read_feedback(timeout=6.0):
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node('desk_point')
    box = {'fbs': []}
    node.create_subscription(JointState, '/arm/feedback',
                             lambda m: box['fbs'].append([float(x) for x in m.position]), 10)
    t0 = time.time()
    while time.time() - t0 < timeout and len(box['fbs']) < 6:
        rclpy.spin_once(node, timeout=0.1) if False else rclpy.spin_once(node, timeout_sec=0.1)
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
    ap.add_argument('--desk', type=float, required=True, help='桌面 z（cm，负）')
    ap.add_argument('--note', default='')
    args = ap.parse_args()

    fb = read_feedback()
    if 0 in fb[2:6]:
        print('⚠️  回读里有 0 —— 再叫一次'); return 1
    j = from_fields(fb)
    tip, _ = fk(j)
    k1 = math.radians(j['shoulder']); k2 = math.radians(j['elbow'])
    alpha = k1 + math.radians(j['elbow']) + math.radians(j['wrist_pitch'])
    other = L1 + L2 * math.sin(k1) + L3 * math.sin(k1 + k2)
    need_l4 = (args.desk - other) / math.sin(alpha)

    err = tip[2] - args.desk
    rows = json.load(open(args.out)) if os.path.exists(args.out) else []
    rows.append({'joints': j, 'fb': fb, 'desk_z': args.desk, 'note': args.note,
                 'model_tip_z': tip[2], 'err_z': err, 'need_l4': need_l4})
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)

    print('✅ 第 %d 点  回读 %s' % (len(rows), ' '.join('%5.0f' % x for x in fb)))
    print('   关节 肩 %6.1f 肘 %6.1f 腕 %6.1f 底座 %6.1f   α %7.1f°' % (
        j['shoulder'], j['elbow'], j['wrist_pitch'], j['base'], math.degrees(alpha)))
    print('   模型 tip z = %7.2f   真值 %6.2f   **误差 %+6.2f cm**' % (tip[2], args.desk, err))
    print('   若其余连杆都对，L4 应为 %6.2f cm（模型 %.2f）' % (need_l4, L4))
    return 0


sys.exit(main())
