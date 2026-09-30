# -*- coding: utf-8 -*-
"""实测图像雅可比：臂动一小段，瓶盖像素跟着移动多少。

    jacobian.py --point 17,0,-12.3 [--delta 1.0]

为什么做这个（2026-09-28）：
  「合成数据能完美还原」是**闭环**——用 project() 生成、又用 project() 拟合，
  任何 project() 自身的系统性错误都会互相抵消，**测不出相机模型对不对**。
  这个实验是**开环**的：拿真实的臂、真实的相机，量**差分**关系。

  差分关系比绝对像素稳健得多：绝对误差可能有几百 px，但只要
  「臂动 1cm → 像素动多少」这个斜率对，就说明相机模型的**尺度**是对的，
  那几百 px 就得由某个**常数**来解释。

判据：
  Δ像素(实测)  与  Δ像素(模型)  一致 -> 相机模型尺度对
  差很多                              -> 相机模型错，而且这个差值直接量化它
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.arm_kin import (FIELD_HI, FIELD_LO, Unreachable, fk,
                               from_fields, ikine, to_fields)
from arm_grasp.calib_solve import Sample
from arm_grasp.cam_model import CamParams, project
from arm_grasp.collect import (ArmIO, _start_driver, _stop_driver, _svc,
                               _svc_active, blue_blob_px, capture_frame,
                               k230_host_from_result, pose_is_safe)

SHOT = '/tmp/k230/manual/jac'
CAM = CamParams(f=1100.0, t=12.3, u_star=640.0, v_star=360.0, roll=math.pi)


def shoot(host, tag):
    p = os.path.join(SHOT, tag + '.jpg')
    if not capture_frame(p, host):
        raise RuntimeError('拍照失败')
    b = blue_blob_px(p)
    if b is None:
        raise RuntimeError('画面里没有瓶盖（%s）' % tag)
    return (b[0], b[1], b[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--point', default='17,0,-12.3')
    ap.add_argument('--delta', type=float, default=1.0, help='每个方向的位移 cm')
    ap.add_argument('--out', default='/home/cy/raspot_ws/src/arm_grasp/docs/jacobian.json')
    args = ap.parse_args()
    cap = tuple(float(v) for v in args.point.split(','))
    os.makedirs(SHOT, exist_ok=True)
    host = k230_host_from_result()
    print('K230 host = %s' % host)

    if _svc_active() != 'active':
        print('⚠️  ps2-teleop.service 不是 active —— 先确认状态再用')
    print('→ 停 ps2-teleop.service（控臂期间摇杆会 25Hz 抢发 /arm/command）')
    r = _svc('stop')
    if r.returncode != 0:
        print('❌ 停服务失败：%s' % (r.stderr or r.stdout)); return 1

    drv = None
    io = None
    out = {'point': list(cap), 'delta': args.delta, 'axes': []}
    try:
        drv = _start_driver()
        io = ArmIO()
        io.wait_feedback()
        fb0 = io.wait_fresh_feedback(0, timeout=3.0)
        if fb0 is None:
            print('❌ 收不到回读'); return 1
        J0 = from_fields(fb0)
        tip0, axis0 = fk(J0)
        a0 = math.degrees(math.asin(max(-1.0, min(1.0, axis0[2]))))
        print('\n基准姿态  回读 %s' % ' '.join('%5.0f' % v for v in fb0))
        print('  关节 肩%7.2f 肘%7.2f 腕%7.2f 底座%7.2f  α%7.2f°'
              % (J0['shoulder'], J0['elbow'], J0['wrist_pitch'], J0['base'], a0))
        print('  模型 tip (%.2f, %.2f, %.2f)' % tip0)

        Q0 = shoot(host, 'base')
        P0 = project(CAM, cap, tip0, axis0)
        print('  瓶盖像素 实测 (%.1f, %.1f)   模型 (%.1f, %.1f)   绝对差 %.1f px'
              % (Q0[0], Q0[1], P0[0], P0[1], math.hypot(Q0[0]-P0[0], Q0[1]-P0[1])))
        out['base'] = {'fb': fb0, 'uv': Q0[:2], 'pred': list(P0), 'tip': list(tip0)}

        print('\n%-6s %-22s %-22s %-8s %-8s' %
              ('轴', 'Δ像素 实测', 'Δ像素 模型', '比值x', '比值y'))
        for name, d in (('+x', (1, 0, 0)), ('+y', (0, 1, 0)), ('+z', (0, 0, 1))):
            tgt = tuple(tip0[i] + d[i]*args.delta for i in range(3))
            try:
                J1 = ikine(*tgt, a0)
            except Unreachable as e:
                print('%-6s IK 不可达: %s' % (name, e)); continue
            if not pose_is_safe(J1):
                print('%-6s 姿态不安全，跳过' % name); continue
            f1 = to_fields(J1, fb0[0], fb0[1])
            if not all(FIELD_LO <= v <= FIELD_HI for v in f1[2:5]):
                print('%-6s 目标 field 出界 %s，跳过' % (name, ['%.0f' % v for v in f1[2:5]])); continue
            # 容差收到 10 count（≈0.7cm @17cm 半径）—— 上一轮 20 count
            # 在 1cm 位移上等于 ±1.4cm，误差和信号同量级（实测 tip 位移走成了斜的）。
            got, dt = io.wait_arrived(f1, settle=2.0, tol=10.0, timeout=12.0)
            if got is None:
                print('%-6s 没到位（%.1fs），跳过' % (name, dt)); continue
            J1a = from_fields(got)
            tip1a, axis1a = fk(J1a)
            try:
                Q1 = shoot(host, 'j' + name[1:])
            except RuntimeError as e:
                print('%-6s 拍照/找盖失败：%s —— 这一轴跳过' % (name, e))
                io.wait_arrived(fb0, settle=1.0, timeout=12.0)
                continue
            P1 = project(CAM, cap, tip1a, axis1a)
            dQ = (Q1[0]-Q0[0], Q1[1]-Q0[1])
            dP = (P1[0]-P0[0], P1[1]-P0[1])
            rx = dQ[0]/dP[0] if abs(dP[0]) > 3 else float('nan')
            ry = dQ[1]/dP[1] if abs(dP[1]) > 3 else float('nan')
            print('%-6s (%+7.1f,%+7.1f)      (%+7.1f,%+7.1f)      %+6.2f  %+6.2f'
                  % (name, dQ[0], dQ[1], dP[0], dP[1], rx, ry))
            out['axes'].append({'axis': name, 'd_obs': list(dQ), 'd_pred': list(dP),
                                'tip_moved': [tip1a[i]-tip0[i] for i in range(3)],
                                'fb': got})
            io.wait_arrived(fb0, settle=1.0, timeout=12.0)
    finally:
        if io is not None:
            io.close()
        if drv is not None:
            _stop_driver(drv)
        print('→ 恢复 ps2-teleop.service')
        _svc('start')
        time.sleep(1.0)
        print('  服务状态: %s' % _svc_active())
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print('已写出 %s' % args.out)
    return 0


sys.exit(main())
