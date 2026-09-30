import math
import random

import pytest

from arm_grasp.arm_kin import fk
from arm_grasp.cam_model import CamParams, project
from arm_grasp.calib_solve import Sample, solve, load_samples, save_samples

TRUE = CamParams(f=1100.0, t=120.0, u_star=704.0, v_star=22.0, roll=0.03)


def _rand_joints(rnd):
    return dict(base=rnd.uniform(-20, 20), shoulder=rnd.uniform(60, 120),
                elbow=rnd.uniform(-80, -20), wrist_pitch=rnd.uniform(-90, -40))


def _make_samples(params, n=10, noise_px=0.0, seed=7):
    rnd = random.Random(seed)
    out = []
    for i in range(n):
        j = _rand_joints(rnd)
        tip, axis = fk(j)
        pt = (16.0 + rnd.uniform(-2, 2), rnd.uniform(-3, 3), -12.0)
        u, v = project(params, pt, tip, axis)
        u += rnd.gauss(0, noise_px)
        v += rnd.gauss(0, noise_px)
        out.append(Sample(joints=j, point=pt, uv=(u, v)))
    return out


def test_recovers_parameters_from_clean_synthetic_data():
    samples = _make_samples(TRUE, n=12)
    est, rep = solve(samples)
    assert abs(est.f - TRUE.f) / TRUE.f < 0.02, est
    assert abs(est.t - TRUE.t) / TRUE.t < 0.05, est
    assert abs(est.u_star - TRUE.u_star) < 3.0, est
    assert abs(est.v_star - TRUE.v_star) < 3.0, est
    assert abs(est.roll - TRUE.roll) < 0.01, est


def test_clean_fit_residual_is_near_zero():
    samples = _make_samples(TRUE, n=12)
    _, rep = solve(samples)
    assert rep['px_rms'] < 1e-3, rep
    assert rep['cm_rms'] < 1e-3, rep


def test_survives_pixel_noise():
    """1.5 px 检测噪声下，参数仍应收敛到可用精度。"""
    samples = _make_samples(TRUE, n=25, noise_px=1.5)
    est, rep = solve(samples)
    assert abs(est.f - TRUE.f) / TRUE.f < 0.05, est
    assert abs(est.u_star - TRUE.u_star) < 8.0, est
    assert rep['px_rms'] < 2.5, rep


def test_degraded_data_shows_up_in_residual():
    """★ 这条是「必须能红」的判据。

    如果标定数据里混进了「光轴不平行」的痕迹（模型假设被破坏），
    残差必须显著变大——否则这套判据就是摆设。
    做法：用一组带 roll 变化的假数据（模拟相机非刚性/装偏），
    看残差是否明显超过噪声水平。
    """
    bad = []
    rnd = random.Random(11)
    for i in range(20):
        j = _rand_joints(rnd)
        tip, axis = fk(j)
        # 每条样本的 roll 都不一样 —— 模型里 roll 是常数，这必然拟合不上
        p = CamParams(f=1100.0, t=120.0, u_star=704.0, v_star=22.0,
                      roll=rnd.uniform(-0.25, 0.25))
        pt = (16.0 + rnd.uniform(-2, 2), rnd.uniform(-3, 3), -12.0)
        bad.append(Sample(joints=j, point=pt, uv=project(p, pt, tip, axis)))
    _, rep_bad = solve(bad)
    _, rep_ok = solve(_make_samples(TRUE, n=20, noise_px=1.5))
    assert rep_bad['px_rms'] > 20.0, rep_bad
    assert rep_bad['px_rms'] > 5 * rep_ok['px_rms'], (rep_bad, rep_ok)


def test_holdout_is_really_held_out():
    """留出集必须真的不参与拟合，而且报告要如实标出 n_fit / n_test。"""
    samples = _make_samples(TRUE, n=12, noise_px=1.0)
    est_all, rep_all = solve(samples)
    est_ho, rep_ho = solve(samples, holdout=3)
    assert rep_ho['n_test'] == 3 and rep_ho['n_fit'] == 9, rep_ho
    assert rep_all['n_test'] == 12 and rep_all['n_fit'] == 12, rep_all
    # 少喂了 3 条，参数必然变（否则说明留出根本没生效）
    assert abs(est_ho.f - est_all.f) > 1e-6, (est_ho, est_all)
    # 干净数据下留出残差也要小
    assert rep_ho['test']['px_rms'] < 3.0, rep_ho


def test_top_level_residual_is_the_holdout_ones():
    """顶层 px_rms/cm_rms 必须报的是**留出集**（holdout=0 时才是拟合集）。

    判据用顶层数，所以顶层不能是"自己批改自己"的拟合集。
    """
    samples = _make_samples(TRUE, n=12, noise_px=1.5)
    _, rep = solve(samples, holdout=4)
    assert rep['px_rms'] == rep['test']['px_rms']
    assert rep['n_test'] == 4


def test_too_few_samples_raises():
    with pytest.raises(ValueError):
        solve(_make_samples(TRUE, n=3))


def test_save_load_roundtrip(tmp_path):
    samples = _make_samples(TRUE, n=6)
    p = str(tmp_path / 's.json')
    save_samples(samples, p)
    back = load_samples(p)
    assert len(back) == 6
    for a, b in zip(samples, back):
        assert a.joints == b.joints
        assert math.dist(a.point, b.point) < 1e-12
        assert math.dist(a.uv, b.uv) < 1e-12


def test_per_sample_report_is_per_sample():
    samples = _make_samples(TRUE, n=12, noise_px=1.5)
    _, rep = solve(samples, holdout=3)
    assert len(rep['per_sample']) == 3
    for e in rep['per_sample']:
        assert e['px'] >= 0.0 and e['cm'] >= 0.0

def test_px_cm_scale_is_nan_when_no_ray_reaches_the_plane():
    """射线打不到地面时必须给 nan，**不能给 0**。

    ★ 2026-09-28 补：变异自检时发现把 `best = nan` 改成 `best = 0.0`
      没有任何用例能抓住 —— 等于「防止 cm 指标假绿」这道护栏根本没上锁。
      0 会让 `cm_max < 1.5` 在垃圾解上反而变绿。

    构造：正常朝下的臂姿态，但把标记点的高度报成 300 cm（平面在相机**背后**），
    两条扰动射线都够不到 -> 没有可定义的换算比例 -> nan。
    """
    from arm_grasp.calib_solve import _px_cm_scale, DEFAULT_INIT
    j = dict(base=0.0, shoulder=110.0, elbow=-60.0, wrist_pitch=-70.0)
    s = Sample(joints=j, point=(14.0, 0.0, 300.0), uv=(640.0, 360.0))
    assert math.isnan(_px_cm_scale(DEFAULT_INIT, s))


def test_cm_stats_go_nan_not_zero_when_the_geometry_is_degenerate():
    """整条链路上都要保持这个性质：垃圾解 -> cm 指标是 nan，不是漂亮的 0。"""
    samples = _make_samples(TRUE, n=8)
    samples = [s._replace(point=(s.point[0], s.point[1], 300.0))
               for s in samples]
    _, rep = solve(samples)
    assert math.isnan(rep['cm_rms']), rep
    assert math.isnan(rep['cm_max']), rep
