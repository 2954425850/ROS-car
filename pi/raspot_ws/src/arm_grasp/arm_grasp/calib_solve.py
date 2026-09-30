# -*- coding: utf-8 -*-
"""从采集样本里解退化手眼参数，并给出残差报告。

## 残差判据（里程碑 1 的核心交付）
两条都要报，缺一不可：
  px_*  像素残差 —— 说明**模型**对不对
  cm_*  厘米残差 —— 说明**能不能抓**（像素误差折算到地面上的距离）
并且**必须留出未参与拟合的样本**（solve 的 holdout 参数），否则就是自欺。
"""
import json
import math
from collections import namedtuple

from .arm_kin import fk
from .cam_model import CamParams, project, pixel_ray, ray_plane
from .lm import least_squares

Sample = namedtuple('Sample', 'joints point uv')

# ★ 初值按实测给：
#   t    = 12.3 cm（用户量的相机镜头到爪尖；design 里的 120 是旧的）
#   roll = 0
#
#   ⚠️ 2026-09-29 修正：这段原来写着「roll=0 才代表相机倒置，§3 那个 pi 是反的」，
#      并称 e2 指向世界上方 —— 那是**照着当时错的 camera_pose 推出来的**，别再引用。
#
#      原实现用 `e1 = _norm(_cross(世界竖直, z))` 搭相机帧。up x z 的水平分量是
#      (cos a cos tb, cos a sin tb, 0)，**归一化把 sign(cos a) 丢掉了**，于是
#      e1 = sign(cos a) · (臂平面法线)：夹爪轴一越过竖直向下（alpha = -90 度），
#      e1/e2 同时取反 ==> 相机绕光轴翻 180 度。相机是刚体，不会这么动。
#
#      **有效 roll 因此在悬崖两侧差 pi。** 这才是「roll 到底是 0 还是 pi 谁也定不
#      下来」和「绝对拟合在 roll 上退化」（roll≈11 度 与 191 度 的 px_rms 几乎相同，
#      259.76 vs 259.57）的真正原因 —— 一个**常**滚转角当然拼不出一个**随位姿翻转**
#      的帧。当年为此写下的「只能靠差分实验定 roll」也只是因为这个。
#
#      已改：e1 现在取臂平面法线 (-sin tb, cos tb, 0)，与 alpha 无关。
#      对 alpha > -90 度那一侧与旧实现**逐位等价**，所以 tools/jacobian.py 那次
#      差分实验的结论（roll≈0）依然成立 —— 它当时正好落在没翻的那一侧。
#      **roll=0 仍然是对的初值。**

DEFAULT_INIT = CamParams(f=1100.0, t=12.3, u_star=704.0, v_star=22.0,
                         roll=0.0)


def _residual(vec, samples, fit_principal):
    p = CamParams.from_vector(vec, fit_principal)
    out = []
    for s in samples:
        tip, axis = fk(s.joints)
        u, v = project(p, s.point, tip, axis)
        out.append(u - s.uv[0])
        out.append(v - s.uv[1])
    return out


def _px_cm_scale(p, sample):
    """像素 -> 地面厘米：u/v 各扰动 1px，看地面交点移动多远（取大者）。

    射线打不到地面（估计已经离谱）时返回 nan —— **不要返回 0**，
    0 会让 `cm_max < 1.5` 这种判据在垃圾解上反而变绿。
    """
    tip, axis = fk(sample.joints)
    best = float('nan')
    for du, dv in ((1.0, 0.0), (0.0, 1.0)):
        try:
            c1, d1 = pixel_ray(p, sample.uv[0], sample.uv[1], tip, axis)
            c2, d2 = pixel_ray(p, sample.uv[0] + du, sample.uv[1] + dv,
                               tip, axis)
            h1 = ray_plane(c1, d1, sample.point[2])
            h2 = ray_plane(c2, d2, sample.point[2])
        except ValueError:
            continue
        d = math.dist(h1, h2)
        best = d if math.isnan(best) else max(best, d)
    return best


def _stats(p, samples):
    errs_px, errs_cm = [], []
    for s in samples:
        tip, axis = fk(s.joints)
        u, v = project(p, s.point, tip, axis)
        e_px = math.hypot(u - s.uv[0], v - s.uv[1])
        errs_px.append(e_px)
        errs_cm.append(e_px * _px_cm_scale(p, s))
    return {'px_rms': _rms(errs_px), 'px_max': max(errs_px),
            'cm_rms': _rms(errs_cm), 'cm_max': max(errs_cm),
            'px': errs_px, 'cm': errs_cm}


def solve(samples, fit_principal=False, init=None, holdout=0):
    """返回 (CamParams, report)。

    holdout: 末尾留出几条不参与拟合，只用于报残差（**这是唯一诚实的做法**）。

    report 顶层 `px_rms/px_max/cm_rms/cm_max` 报的是**留出集**
    （holdout=0 时退化成拟合集）；`fit` / `test` 两个子字典分别给两组数。
    判据要用顶层，顶层不能是"自己批改自己"的拟合集。
    """
    need = 7 if fit_principal else 5
    if len(samples) < need:
        raise ValueError('样本太少：%d（至少 %d 条）' % (len(samples), need))
    if holdout and len(samples) - holdout < need:
        raise ValueError('留出 %d 条后拟合样本不足（剩 %d，至少 %d）'
                         % (holdout, len(samples) - holdout, need))
    if holdout:
        fit_set, test_set = samples[:-holdout], samples[-holdout:]
    else:
        fit_set, test_set = samples, samples
    x0 = (init or DEFAULT_INIT).as_vector(fit_principal)
    x, cost, iters = least_squares(
        lambda v: _residual(v, fit_set, fit_principal), x0)
    p = CamParams.from_vector(x, fit_principal)

    rep = {'iters': iters, 'cost': cost, 'n_fit': len(fit_set),
           'n_test': len(test_set), 'params': p,
           'fit_principal': fit_principal}
    rep['fit'] = _stats(p, fit_set)
    rep['test'] = _stats(p, test_set)
    for k in ('px_rms', 'px_max', 'cm_rms', 'cm_max'):
        rep[k] = rep['test'][k]
    rep['per_sample'] = [{'px': a, 'cm': b}
                         for a, b in zip(rep['test']['px'], rep['test']['cm'])]
    return p, rep


def _rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else float('nan')


def load_samples(path):
    with open(path, 'r', encoding='utf-8') as f:
        raw = json.load(f)
    return [Sample(joints=r['joints'], point=tuple(r['point']),
                   uv=tuple(r['uv'])) for r in raw]


def save_samples(samples, path):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump([{'joints': s.joints, 'point': list(s.point),
                    'uv': list(s.uv)} for s in samples], f,
                  ensure_ascii=False, indent=1)
