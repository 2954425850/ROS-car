# -*- coding: utf-8 -*-
"""假设：关节有**零位偏置**（设计 §9 风险 #3，当时写"会被标定吸收"，
但 5 参数的相机模型吸收不了 —— 它只能变成残差）。

做法：把 f、t 固定在网格上，除了 (u*, v*, roll) 再放开 4 个关节偏置
（肩/肘/腕/底座各一个常量，单位度），看残差掉不掉。
"""
import math
import sys

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.arm_kin import fk
from arm_grasp.calib_solve import load_samples
from arm_grasp.cam_model import _cross, _dot, _norm, _sub
from arm_grasp.lm import least_squares

S = load_samples('/home/cy/raspot_ws/src/arm_grasp/docs/samples-20260928-190746.json')
U0, V0 = 640.0, 360.0


def _frame(axis, roll):
    z = _norm(axis)
    up = (0.0, 0.0, 1.0)
    if abs(_dot(up, z)) > 0.999:
        up = (1.0, 0.0, 0.0)
    e1 = _norm(_cross(up, z))
    e2 = _cross(z, e1)
    cr, sr = math.cos(roll), math.sin(roll)
    return (tuple(cr * e1[i] + sr * e2[i] for i in range(3)),
            tuple(-sr * e1[i] + cr * e2[i] for i in range(3)), z)


def project(f, t, us, vs, roll, tip, axis, P):
    x, y, z = _frame(axis, roll)
    k = t / f
    off = tuple(k * ((us - U0) * x[i] + (vs - V0) * y[i]) for i in range(3))
    C = tuple(tip[i] - t * z[i] - off[i] for i in range(3))
    v = _sub(P, C)
    d = _dot(v, z)
    if abs(d) < 1e-9:
        return None
    return (U0 + f * _dot(v, x) / d, V0 + f * _dot(v, y) / d)


def make_resid(f, t, with_offsets):
    pre = [(dict(s.joints), s.point, s.uv) for s in S]
    n0 = 3
    x0 = [670.0, 85.0, math.pi] + ([0.0, 0.0, 0.0, 0.0] if with_offsets else [])

    def resid(vec):
        us, vs, roll = vec[0], vec[1], vec[2]
        ds, de, dw, db = (vec[3], vec[4], vec[5], vec[6]) if with_offsets \
            else (0.0, 0.0, 0.0, 0.0)
        out = []
        for j, P, uv in pre:
            jj = dict(j)
            jj['shoulder'] += ds
            jj['elbow'] += de
            jj['wrist_pitch'] += dw
            jj['base'] += db
            tip, axis = fk(jj)
            q = project(f, t, us, vs, roll, tip, axis, P)
            if q is None:
                return [1e6] * (2 * len(pre))
            out.append(q[0] - uv[0])
            out.append(q[1] - uv[1])
        return out

    return resid, x0


print('%-22s | %-34s | %s' % ('(f, t)', '无偏置 px_rms', '有偏置 px_rms  (肩/肘/腕/底座 偏置°)'))
print('-' * 108)
for f, t in ((600.0, 10.0), (900.0, 40.0), (1400.0, 40.0), (2500.0, 40.0)):
    r, x0 = make_resid(f, t, False)
    x5, _, _ = least_squares(r, x0, max_iter=4000)
    rr = r(x5)
    a = math.sqrt(sum(v * v for v in rr) / len(rr))

    r2, x02 = make_resid(f, t, True)
    x7, _, _ = least_squares(r2, x02, max_iter=4000)
    rr2 = r2(x7)
    b = math.sqrt(sum(v * v for v in rr2) / len(rr2))
    print('f=%5.0f t=%5.1f        | %10.3f                         | %8.3f     %+6.2f %+6.2f %+6.2f %+6.2f'
          % (f, t, a, b, x7[3], x7[4], x7[5], x7[6]))
