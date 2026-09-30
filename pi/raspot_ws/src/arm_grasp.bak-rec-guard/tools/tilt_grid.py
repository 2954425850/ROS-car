# -*- coding: utf-8 -*-
"""把 f、t 固定在网格上（避免退化），只优化 (u*, v*, roll) 以及可选的
光轴倾斜 (tx, ty)。看两件事：
  1. 有没有**物理上合理**的 (f, t) 能把残差压下去？
  2. 加上光轴倾斜那 2 个自由度，残差掉不掉？掉多少？
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
PRE = [(fk(s.joints), s.point, s.uv) for s in S]


def _frame(axis, roll, tx, ty):
    z0 = _norm(axis)
    up = (0.0, 0.0, 1.0)
    if abs(_dot(up, z0)) > 0.999:
        up = (1.0, 0.0, 0.0)
    e1 = _norm(_cross(up, z0))
    e2 = _cross(z0, e1)
    cr, sr = math.cos(roll), math.sin(roll)
    x = tuple(cr * e1[i] + sr * e2[i] for i in range(3))
    y = tuple(-sr * e1[i] + cr * e2[i] for i in range(3))
    z = _norm(tuple(z0[i] + tx * x[i] + ty * y[i] for i in range(3)))
    return x, y, z


def project(f, t, us, vs, roll, tx, ty, tip, axis, P):
    x, y, z = _frame(axis, roll, tx, ty)
    k = t / f
    off = tuple(k * ((us - U0) * x[i] + (vs - V0) * y[i]) for i in range(3))
    C = tuple(tip[i] - t * z[i] - off[i] for i in range(3))
    v = _sub(P, C)
    d = _dot(v, z)
    if abs(d) < 1e-9:
        return None
    return (U0 + f * _dot(v, x) / d, V0 + f * _dot(v, y) / d)


def fit_at(f, t, with_tilt):
    x0 = [670.0, 85.0, math.pi] + ([0.0, 0.0] if with_tilt else [])

    def resid(vec):
        us, vs, roll = vec[0], vec[1], vec[2]
        tx, ty = (vec[3], vec[4]) if with_tilt else (0.0, 0.0)
        out = []
        for tip, P, uv in PRE:
            q = project(f, t, us, vs, roll, tx, ty, tip, (0, 0, 0), P) \
                if False else None
            axis = None
        return out
    return None


def fit_at2(f, t, with_tilt):
    """(用预先算好的 tip/axis)"""
    pre = [(fk(s.joints), s.point, s.uv) for s in S]
    x0 = [670.0, 85.0, math.pi] + ([0.0, 0.0] if with_tilt else [])

    def resid(vec):
        us, vs, roll = vec[0], vec[1], vec[2]
        tx, ty = (vec[3], vec[4]) if with_tilt else (0.0, 0.0)
        out = []
        for (tip, axis), P, uv in pre:
            q = project(f, t, us, vs, roll, tx, ty, tip, axis, P)
            if q is None:
                return [1e6] * (2 * len(pre))
            out.append(q[0] - uv[0])
            out.append(q[1] - uv[1])
        return out

    x, cost, _ = least_squares(resid, x0, max_iter=3000)
    r = resid(x)
    rms = math.sqrt(sum(a * a for a in r) / len(r))
    return rms, x


print('f(px)  t(cm) | 无倾斜 px_rms (u*,v*,roll) | 有倾斜 px_rms (u*,v*,roll,tx,ty)  倾角')
print('-' * 96)
for f in (400.0, 600.0, 900.0, 1400.0, 2500.0):
    for t in (4.0, 10.0, 20.0, 40.0):
        r5, _ = fit_at2(f, t, False)
        r7, x7 = fit_at2(f, t, True)
        tilt = math.degrees(math.hypot(x7[3], x7[4]))
        mark = '  <<< 明显改善' if r7 < r5 * 0.5 else ''
        print('%6.0f %6.1f | %10.3f                  | %10.3f                        %6.2f 度%s'
              % (f, t, r5, r7, tilt, mark))
