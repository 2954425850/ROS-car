# -*- coding: utf-8 -*-
"""扫关节标定：记一个「爪尖离桌面多少 cm」的点（只读回读）。
真值 z = 桌面高度(-13.6) + h。不涉及相机，纯校验 field->角度 那一环。"""
import argparse, json, math, os, sys, time
sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')
from arm_grasp.arm_kin import fk, from_fields
DESK = -13.6

def read_feedback(timeout=6.0):
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init(); node = rclpy.create_node('sweep'); box = {'fbs': []}
    node.create_subscription(JointState, '/arm/feedback',
                             lambda m: box['fbs'].append([float(x) for x in m.position]), 10)
    t0 = time.time()
    while time.time()-t0 < timeout and len(box['fbs']) < 6:
        rclpy.spin_once(node, timeout_sec=0.1)
    fbs = box['fbs']; node.destroy_node(); rclpy.shutdown()
    if not fbs: raise RuntimeError('收不到 /arm/feedback')
    for i in range(len(fbs)-1, 0, -1):
        a, b = fbs[i], fbs[i-1]
        if len(a) == 6 and len(b) == 6 and all(abs(a[k]-b[k]) <= 4 for k in range(6)):
            return a
    return fbs[-1]

ap = argparse.ArgumentParser()
ap.add_argument('out'); ap.add_argument('--h', type=float, required=True)
ap.add_argument('--note', default='')
a = ap.parse_args()
fb = read_feedback()
if 0 in fb[2:6]:
    print('回读里有 0 -- 再叫一次'); sys.exit(1)
j = from_fields(fb); tip, _ = fk(j)
z_true = DESK + a.h
rows = json.load(open(a.out)) if os.path.exists(a.out) else []
rows.append({'fb': fb, 'joints': j, 'h_true': a.h, 'z_true': z_true,
             'model_tip_z': tip[2], 'note': a.note})
json.dump(rows, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
print('OK 第 %d 点  回读 %s' % (len(rows), ' '.join('%5.0f' % v for v in fb)))
print('   肩 %7.2f  肘 %7.2f  腕 %7.2f  底座 %7.2f'
      % (j['shoulder'], j['elbow'], j['wrist_pitch'], j['base']))
print('   真值 z = %7.2f   模型 z = %7.2f   误差 %+5.2f cm'
      % (z_true, tip[2], tip[2]-z_true))
