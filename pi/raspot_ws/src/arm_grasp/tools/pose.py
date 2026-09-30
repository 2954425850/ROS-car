# -*- coding: utf-8 -*-
"""只读当前姿态（不发指令、不拍照），约 1 秒。

    pose.py [--point 17,0,-12.3]

给摆姿态的人一个即时反馈：现在 tip 在哪、α 多少、离瓶盖多远、
以及按实测经验这条「看不看得见」。
"""
import argparse
import math
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.arm_kin import fk, from_fields

# §3 实测：α ≈ 仰角(tip->盖) − 18.4° 时刚好看得见。
# ⚠️「浅」= 末端更接近水平 = α 的**代数值更大**（−58° 比 −79° 浅）。别写反。
VIS_DELTA = -18.4
ALPHA_SHALLOW_LIMIT = -65.0


def alpha_is_too_shallow(alpha):
    """α 是否浅过 −65°（更接近水平）。−58 -> True，−79 -> False，−65 -> False。"""
    return alpha > ALPHA_SHALLOW_LIMIT


def read_feedback(timeout=5.0):
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node('pose')
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
    ap.add_argument('--point', default='17,0,-12.3')
    args = ap.parse_args()
    cap = tuple(float(x) for x in args.point.split(','))

    fb = read_feedback()
    if 0 in fb[2:6]:
        print('⚠️  回读里有 0（底座/肩/肘/腕任一，该拍没读到）—— 再叫一次')
        return 1
    j = from_fields(fb)
    tip, axis = fk(j)
    alpha = j['shoulder'] + j['elbow'] + j['wrist_pitch']

    dx, dy, dz = cap[0] - tip[0], cap[1] - tip[1], cap[2] - tip[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    horiz = math.hypot(dx, dy)
    elev = math.degrees(math.atan2(dz, horiz))          # tip->盖 的仰角
    want = elev + VIS_DELTA                              # 实测经验：这条 α 看得见

    print('回读  %s' % ' '.join('%5.0f' % x for x in fb))
    print('tip   x=%6.2f y=%6.2f z=%6.2f cm   底座 %6.1f°' % (tip[0], tip[1], tip[2], j['base']))
    print('关节  肩 %6.1f  肘 %6.1f  腕 %6.1f   ->  α %6.1f°' % (
        j['shoulder'], j['elbow'], j['wrist_pitch'], alpha))
    print('离盖  %5.2f cm   (水平 %5.2f / 垂直 %+5.2f)   仰角 %+5.1f°' % (
        dist, horiz, dz, elev))
    # ★ 这条经验式**只在 α ∈ [−88, −65] 被实测验证过**（§3）。
    #   出区间就失效：第 3 条 α=−99.4° 它预测"看不见"，看图实际看得见。
    #   所以出区间不报"差多少"，别拿它当判据。
    if -88.0 <= alpha <= ALPHA_SHALLOW_LIMIT:
        print('经验  这条看得见的 α ≈ %+5.1f°   现在 差 %+5.1f°' % (want, alpha - want))
    else:
        print('经验  α=%+.1f° 出验证区间 [−88,−65]，经验式不适用 —— 以看画面为准' % alpha)
    if dist < 3.0:
        print('⚠️  离盖只有 %.1f cm —— 太近，退到 3cm 以上' % dist)
    if alpha_is_too_shallow(alpha):
        print('⚠️  α 浅于 %.0f°：相机基本平着往前看，瓶盖多半不在画面'
              % ALPHA_SHALLOW_LIMIT)
    return 0


sys.exit(main())
