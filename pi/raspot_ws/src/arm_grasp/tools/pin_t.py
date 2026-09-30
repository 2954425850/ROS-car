# -*- coding: utf-8 -*-
"""把 t 钉在实测值（相机镜头沿夹爪轴离爪尖 12.3cm），看 f 能不能解出来。"""
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


def proj(f, t, us, vs, roll, tip, axis, P):
    z = _norm(axis)
    up = (0.0, 0.0, 1.0)
    if abs(_dot(up, z)) > 0.999:
        up = (1.0, 0.0, 0.0)
    e1 = _norm(_cross(up, z))
    e2 = _cross(z, e1)
    cr, sr = math.cos(roll), math.sin(roll)
    x = tuple(cr * e1[i] + sr * e2[i] for i in range(3))
    y = tuple(-sr * e1[i] + cr * e2[i] for i in range(3))
    k = t / f
    off = tuple(k * ((us - U0) * x[i] + (vs - V0) * y[i]) for i in range(3))
    C = tuple(tip[i] - t * z[i] - off[i] for i in range(3))
    v = _sub(P, C)
    d = _dot(v, z)
    if abs(d) < 1e-9:
        return None
    return (U0 + f * _dot(v, x) / d, V0 + f * _dot(v, y) / d)


def fit(fixed_t):
    """fixed_t=None -> 5 参数都放开；否则 t 固定。"""
    def resid(vec):
        if fixed_t is None:
            f, t, us, vs, roll = vec
        else:
            f = vec[0]
            t, us, vs, roll = fixed_t, vec[1], vec[2], vec[3]
        out = []
        for (tip, axis), P, uv in PRE:
            q = proj(f, t, us, vs, roll, tip, axis, P)
            if q is None:
                return [1e6] * 2 * len(PRE)
            out += [q[0] - uv[0], q[1] - uv[1]]
        return out

    x0 = ([900.0, 670.0, 85.0, math.pi] if fixed_t is not None
          else [900.0, 12.3, 670.0, 85.0, math.pi])
    x, _, _ = least_squares(resid, x0, max_iter=8000)
    r = resid(x)
    rms = math.sqrt(sum(a * a for a in r) / len(r))
    return rms, x


print('%d 个样本（%d 个方程）' % (len(S), 2 * len(S)))
print('%-40s %-10s %s' % ('模型', 'px_rms', '解'))
print('-' * 100)
for t in (6.0, 8.0, 10.0, 12.3, 15.0, 20.0, 30.0):
    rms, x = fit(t)
    print('%-40s %-10.3f f=%9.1f  u*=%7.2f  v*=%7.2f  roll=%7.4f'
          % ('t 钉在 %.1f cm，拟合 f,u*,v*,roll' % t, rms, x[0], x[1], x[2], x[3]))
rms, x = fit(None)
print('%-40s %-10.3f f=%9.1f  t=%8.2f u*=%7.2f  v*=%7.2f  roll=%7.4f'
      % ('t 也放开（5 参数全自由）', rms, x[0], x[1], x[2], x[3], x[4]))
