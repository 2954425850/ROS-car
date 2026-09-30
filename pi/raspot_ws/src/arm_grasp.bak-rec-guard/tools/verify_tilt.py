# -*- coding: utf-8 -*-
"""把两种写法各拟合一遍，确认它们是同一件事：
  写法甲：光轴=夹爪轴，主点 (u0,v0) 自由
  写法乙：主点在画面中心，光轴相对夹爪轴倾斜 (tx,ty)
若两者残差和"等效倾角"对得上，就说明**相机光轴确实和夹爪轴不平行**。
"""
import math
import sys

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.arm_kin import fk
from arm_grasp.calib_solve import load_samples
from arm_grasp.cam_model import _cross, _dot, _norm, _sub
from arm_grasp.lm import least_squares

S = load_samples('/home/cy/raspot_ws/src/arm_grasp/docs/samples-20260928-190746.json')
PRE = [(fk(s.joints), s.point, s.uv) for s in S]
GOOD = [778.4, 673.77, 133.47, math.pi]        # 上面解出来的好起点


def frame(axis, roll, tx, ty, tilt):
    z0 = _norm(axis)
    up = (0.0, 0.0, 1.0)
    if abs(_dot(up, z0)) > 0.999:
        up = (1.0, 0.0, 0.0)
    e1 = _norm(_cross(up, z0))
    e2 = _cross(z0, e1)
    cr, sr = math.cos(roll), math.sin(roll)
    x = tuple(cr * e1[i] + sr * e2[i] for i in range(3))
    y = tuple(-sr * e1[i] + cr * e2[i] for i in range(3))
    if tilt:
        z = _norm(tuple(z0[i] + tx * x[i] + ty * y[i] for i in range(3)))
    else:
        z = z0
    return x, y, z


def run(mode):
    """mode='principal' 或 'tilt'"""
    def resid(vec):
        f, us, vs, roll = vec[0], vec[1], vec[2], vec[3]
        if mode == 'principal':
            u0, v0, tx, ty = vec[4], vec[5], 0.0, 0.0
        else:
            u0, v0 = 640.0, 360.0
            tx, ty = vec[4], vec[5]
        out = []
        for (tip, axis), P, uv in PRE:
            x, y, z = frame(axis, roll, tx, ty, mode == 'tilt')
            k = 12.3 / f
            off = tuple(k * ((us - u0) * x[i] + (vs - v0) * y[i])
                        for i in range(3))
            C = tuple(tip[i] - 12.3 * z[i] - off[i] for i in range(3))
            v = _sub(P, C)
            d = _dot(v, z)
            if abs(d) < 1e-9:
                return [1e6] * 2 * len(PRE)
            out += [u0 + f * _dot(v, x) / d - uv[0],
                    v0 + f * _dot(v, y) / d - uv[1]]
        return out

    x0 = GOOD + ([659.5, 4.6] if mode == 'principal' else [0.0, 0.0])
    x, _, _ = least_squares(resid, x0, max_iter=20000)
    r = resid(x)
    rms = math.sqrt(sum(a * a for a in r) / len(r))
    return rms, x


r1, x1 = run('principal')
print('写法甲（光轴=夹爪轴，主点自由）')
print('  px_rms = %.3f   f=%.1f  u*=%.2f v*=%.2f roll=%.4f  主点=(%.1f, %.1f)'
      % (r1, x1[0], x1[1], x1[2], x1[3], x1[4], x1[5]))
print('  -> 等效倾角: 竖向 %.1f 度, 横向 %.1f 度'
      % (math.degrees(math.atan((360.0 - x1[5]) / x1[0])),
         math.degrees(math.atan((x1[4] - 640.0) / x1[0]))))

r2, x2 = run('tilt')
tilt = math.degrees(math.hypot(x2[4], x2[5]))
print()
print('写法乙（主点=画面中心，光轴可倾斜）')
print('  px_rms = %.3f   f=%.1f  u*=%.2f v*=%.2f roll=%.4f  tx=%+.4f ty=%+.4f'
      % (r2, x2[0], x2[1], x2[2], x2[3], x2[4], x2[5]))
print('  -> 等效倾角 = %.2f 度（分轴: 竖向 %.1f, 横向 %.1f）'
      % (tilt, math.degrees(x2[5]), math.degrees(x2[4])))
print()
print('两种写法残差 %.3f vs %.3f  =>  %s'
      % (r1, r2, '一致，倾角是真的' if abs(r1 - r2) < 1.0 else '不一致，还得再查'))
