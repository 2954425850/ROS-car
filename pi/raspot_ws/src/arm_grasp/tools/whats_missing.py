# -*- coding: utf-8 -*-
"""t 钉在实测 12.3cm 的前提下，逐个试"还缺什么"，看哪个能让残差掉下来。

候选：
  A. 主点 u0,v0 放开（现在写死在画面中心 640,360）
  B. 单个关节零位偏置（肩/肘/腕/底座 各试一个）
  C. 径向畸变 k1（针孔模型之外的镜头畸变）
"""
import math
import sys

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.arm_kin import fk
from arm_grasp.calib_solve import load_samples
from arm_grasp.cam_model import _cross, _dot, _norm, _sub
from arm_grasp.lm import least_squares

S = load_samples('/home/cy/raspot_ws/src/arm_grasp/docs/samples-20260928-190746.json')
T_FIX = 12.3
BASE = [(dict(s.joints), s.point, s.uv) for s in S]


def build(extra):
    """extra: 'none' | 'principal' | 'off_shoulder' | ... | 'k1'"""
    def resid(vec):
        i = 0
        f = vec[i]; i += 1
        us = vec[i]; i += 1
        vs = vec[i]; i += 1
        roll = vec[i]; i += 1
        u0, v0, k1 = 640.0, 360.0, 0.0
        offs = dict(shoulder=0.0, elbow=0.0, wrist_pitch=0.0, base=0.0)
        if extra == 'principal':
            u0 = vec[i]; i += 1
            v0 = vec[i]; i += 1
        elif extra.startswith('off_'):
            offs[extra[4:]] = vec[i]; i += 1
        elif extra == 'k1':
            k1 = vec[i]; i += 1
        out = []
        for j, P, uv in BASE:
            jj = dict(j)
            for k, d in offs.items():
                jj[k] += d
            tip, axis = fk(jj)
            z = _norm(axis)
            up = (0.0, 0.0, 1.0)
            if abs(_dot(up, z)) > 0.999:
                up = (1.0, 0.0, 0.0)
            e1 = _norm(_cross(up, z))
            e2 = _cross(z, e1)
            cr, sr = math.cos(roll), math.sin(roll)
            x = tuple(cr * e1[k] + sr * e2[k] for k in range(3))
            y = tuple(-sr * e1[k] + cr * e2[k] for k in range(3))
            kk = T_FIX / f
            off = tuple(kk * ((us - u0) * x[k] + (vs - v0) * y[k])
                        for k in range(3))
            C = tuple(tip[k] - T_FIX * z[k] - off[k] for k in range(3))
            v = _sub(P, C)
            d = _dot(v, z)
            if abs(d) < 1e-9:
                return [1e6] * 2 * len(BASE)
            uu = u0 + f * _dot(v, x) / d
            vv = v0 + f * _dot(v, y) / d
            # 径向畸变（以主点为心）
            if k1:
                ru = (uu - u0) / f
                rv = (vv - v0) / f
                r2 = ru * ru + rv * rv
                uu = u0 + f * ru * (1.0 + k1 * r2)
                vv = v0 + f * rv * (1.0 + k1 * r2)
            out += [uu - uv[0], vv - uv[1]]
        return out

    x0 = [900.0, 670.0, 85.0, math.pi]
    if extra == 'principal':
        x0 += [640.0, 360.0]
    elif extra.startswith('off_'):
        x0 += [0.0]
    elif extra == 'k1':
        x0 += [0.0]
    return resid, x0


print('t 固定 = %.1f cm，%d 个样本（%d 个方程）' % (T_FIX, len(S), 2 * len(S)))
print('%-34s %-10s %s' % ('模型', 'px_rms', '解'))
print('-' * 96)
for extra, label in (('none', '基准 4 参数 f,u*,v*,roll'),
                     ('principal', '+ 主点 u0,v0'),
                     ('off_shoulder', '+ 肩零位偏置'),
                     ('off_elbow', '+ 肘零位偏置'),
                     ('off_wrist_pitch', '+ 腕零位偏置'),
                     ('off_base', '+ 底座零位偏置'),
                     ('k1', '+ 径向畸变 k1')):
    r, x0 = build(extra)
    x, _, _ = least_squares(r, x0, max_iter=8000)
    rr = r(x)
    rms = math.sqrt(sum(a * a for a in rr) / len(rr))
    tail = ''
    if extra == 'principal':
        tail = 'u0=%.1f v0=%.1f' % (x[4], x[5])
    elif extra.startswith('off_'):
        tail = '%s 偏置 = %+.2f 度' % (extra[4:], x[4])
    elif extra == 'k1':
        tail = 'k1 = %+.4f' % x[4]
    print('%-34s %-10.3f f=%9.1f u*=%7.2f v*=%7.2f roll=%7.4f  %s'
          % (label, rms, x[0], x[1], x[2], x[3], tail))
