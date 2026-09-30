# -*- coding: utf-8 -*-
"""看采集进度：每条的姿态 + 相机到瓶盖的距离覆盖范围。

    progress.py <samples.json> [--point 17,0,-12.3] [--t 12.3]

★ 「相机->盖」是**算出来的**：相机位置 = tip − t·axis（t 用实测 12.3，
   axis 是夹爪下扎方向）。这个约定已用两个独立样本的瓶盖**成像面积**反验过
  （预测面积比 1.54/0.783 vs 实测 1.48/0.822）。
  标定真正吃的是**这个距离的变化**，不是 tip 到盖的变化——§4 说上一轮它只有 15%。
"""
import argparse
import json
import math
import sys

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.arm_kin import fk
from arm_grasp.calib_solve import Sample


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('samples')
    ap.add_argument('--point', default='17,0,-12.3')
    ap.add_argument('--t', type=float, default=12.3)
    args = ap.parse_args()
    cap = tuple(float(x) for x in args.point.split(','))

    rows = json.load(open(args.samples))
    print('#  底座°    tip z   肩     肘     腕     α°    尖->盖  相机->盖')
    cams = []
    for i, r in enumerate(rows, 1):
        s = Sample(joints=r['joints'], point=r['point'], uv=tuple(r['uv']))
        tip, axis = fk(s.joints)
        cam = tuple(tip[k] - args.t * axis[k] for k in range(3))
        cam_d = math.dist(cam, cap)
        cams.append(cam_d)
        a = s.joints['shoulder'] + s.joints['elbow'] + s.joints['wrist_pitch']
        print('%2d %6.1f %7.2f %6.1f %6.1f %6.1f %7.1f %6.2f %8.2f' % (
            i, s.joints['base'], tip[2], s.joints['shoulder'], s.joints['elbow'],
            s.joints['wrist_pitch'], a, math.dist(tip, cap), cam_d))

    n = len(rows)
    print()
    print('共 %d 条' % n)
    if n >= 2:
        lo, hi = min(cams), max(cams)
        print('相机->盖 覆盖 %.2f ~ %.2f cm  =  跨度 %.0f%%' % (lo, hi, (hi / lo - 1) * 100))
        print('  （§4：上一轮只有 ~15% -> f/t 分不开。这个数越大越好）')
    j = [r['joints'] for r in rows]
    if n >= 2:
        print('肩 %.1f~%.1f   肘 %.1f~%.1f   腕 %.1f~%.1f   底座 %.1f~%.1f' % (
            min(x['shoulder'] for x in j), max(x['shoulder'] for x in j),
            min(x['elbow'] for x in j), max(x['elbow'] for x in j),
            min(x['wrist_pitch'] for x in j), max(x['wrist_pitch'] for x in j),
            min(x['base'] for x in j), max(x['base'] for x in j)))
    return 0


sys.exit(main())
