import json
import math
import random

import pytest

from arm_grasp.arm_kin import fk
from arm_grasp.calib_report import (build_report, print_report, write_report,
                                    PX_RMS_PASS, CM_MAX_PASS)
from arm_grasp.calib_solve import Sample, save_samples
from arm_grasp.cam_model import CamParams, project

TRUE = CamParams(f=1100.0, t=120.0, u_star=704.0, v_star=22.0, roll=0.03)


def _samples(params, n=12, noise_px=0.0, seed=3, vary_roll=False):
    rnd = random.Random(seed)
    out = []
    for _ in range(n):
        j = dict(base=rnd.uniform(-20, 20), shoulder=rnd.uniform(70, 110),
                 elbow=rnd.uniform(-80, -40), wrist_pitch=rnd.uniform(-85, -45))
        tip, axis = fk(j)
        p = params
        if vary_roll:      # 模型假设被破坏：roll 不是常数
            p = CamParams(params.f, params.t, params.u_star, params.v_star,
                          rnd.uniform(-0.25, 0.25))
        pt = (16.0 + rnd.uniform(-2, 2), rnd.uniform(-3, 3), -12.0)
        u, v = project(p, pt, tip, axis)
        out.append(Sample(joints=j, point=pt,
                          uv=(u + rnd.gauss(0, noise_px),
                              v + rnd.gauss(0, noise_px))))
    return out


def _write(tmp_path, samples):
    p = str(tmp_path / 'samples-x.json')
    save_samples(samples, p)
    return p


def test_passes_on_clean_data(tmp_path, capsys):
    p = _write(tmp_path, _samples(TRUE, noise_px=0.6))
    params, rep, meta, _ = build_report(p, holdout=3)
    assert math.hypot(params.u_star - TRUE.u_star,
                      params.v_star - TRUE.v_star) < 8.0
    ok = print_report(p, params, rep, meta)
    assert ok is True


def test_fails_when_the_parallel_axis_assumption_is_broken(tmp_path, capsys):
    """★ 判据必须能红：光轴不平行（roll 每条都不同）时结论必须是不通过。"""
    p = _write(tmp_path, _samples(TRUE, noise_px=0.6, vary_roll=True))
    params, rep, meta, _ = build_report(p, holdout=3)
    assert print_report(p, params, rep, meta) is False
    assert rep['px_rms'] > PX_RMS_PASS


def test_judge_requires_both_conditions():
    """★ 两条判据是 **and**，不是 or。

    变异自检发现：把 and 改成 or 时，真实数据上跑不出红 ——
    因为"模型被破坏"时 px 和 cm 总是一起超标。所以直接钉语义。
    """
    from arm_grasp.calib_report import judge
    assert judge({'px_rms': 1.0, 'cm_max': 1.0}) is True
    assert judge({'px_rms': 6.0, 'cm_max': 1.0}) is False   # px 超标
    assert judge({'px_rms': 1.0, 'cm_max': 2.0}) is False   # cm 超标
    assert judge({'px_rms': 5.0, 'cm_max': 1.0}) is False   # 边界：正好等于不过
    assert judge({'px_rms': 1.0, 'cm_max': 1.5}) is False


def test_thresholds_are_the_agreed_numbers(capsys):
    """阈值是**事先约定**的判据，不能被人悄悄改宽。"""
    assert PX_RMS_PASS == 5.0
    assert CM_MAX_PASS == 1.5


def test_write_report_writes_json_and_markdown(tmp_path, capsys):
    p = _write(tmp_path, _samples(TRUE, noise_px=0.6))
    params, rep, meta, _ = build_report(p, holdout=3)
    ok = print_report(p, params, rep, meta)
    j, m = write_report(p, params, rep, meta, ok)
    d = json.load(open(j, encoding='utf-8'))
    assert d['passed'] is ok
    assert d['n_test'] == 3
    assert d['thresholds'] == {'px_rms': PX_RMS_PASS, 'cm_max': CM_MAX_PASS}
    txt = open(m, encoding='utf-8').read()
    assert '留出集（真判据）' in txt


def test_report_works_without_meta_sidecar(tmp_path, capsys):
    """没有 .meta.json 时也要能出报告（不能因为缺个可选文件就崩）。"""
    p = _write(tmp_path, _samples(TRUE, noise_px=0.6))
    params, rep, meta, _ = build_report(p, holdout=0)
    assert meta is None
    assert print_report(p, params, rep, meta) in (True, False)


def _shift_z(samples, dz):
    return [Sample(joints=x.joints,
                   point=(x.point[0], x.point[1], x.point[2] + dz),
                   uv=x.uv) for x in samples]


def test_a_large_marker_coordinate_error_fails_the_criterion(tmp_path, capsys):
    """标记点坐标量错 10 cm 时必须判不通过（计划排查第 1 条的最粗一档）。"""
    p = _write(tmp_path, _shift_z(_samples(TRUE, noise_px=0.6), -10.0))
    params, rep, meta, _ = build_report(p, holdout=3)
    assert print_report(p, params, rep, meta) is False


def test_small_marker_coordinate_error_slips_past_the_residual(tmp_path, capsys):
    """★ ★ 残差判据**抓不住小的**标记点坐标误差 —— 这条测试是把它钉在明面上。

    2026-09-28 合成数据实测（f 真值 1100）：
        标记点 z 偏 -3 cm -> px_rms 2.70、cm_max 0.63   **判据通过**
                              但拟合出的 f 已经偏 **12.1%**
        偏 -5 cm          -> px_rms 4.71、cm_max 1.19   **仍然通过**，f 偏 22.4%
        偏 -10 cm         -> 这才不通过

    也就是说计划 Task 6 Step 5 写的「坐标量错 -> 残差整体偏大」**不成立**：
    小的量错会被 (f, t, u*, v*) 悄悄吸收 —— 又一类"自洽但错"。
    所以报告必须**额外**给出参数敏感度，光看残差绿了不能放心。

    这条测试同时钉住两件事：(1) 这个盲区确实存在（别以为残差能兜住）；
    (2) 报告确实把敏感度算了出来、量级对得上（f 每 cm 跑 ~4%）。
    """
    p = _write(tmp_path, _shift_z(_samples(TRUE, noise_px=0.6), -3.0))
    params, rep, meta, _ = build_report(p, holdout=3)
    assert print_report(p, params, rep, meta) is True          # 盲区：通过了！
    assert abs(params.f - TRUE.f) / TRUE.f > 0.08              # 但 f 已经偏了 >8%
    sens = rep['sensitivity']
    assert abs(sens['df_per_cm'] / params.f) > 0.02, sens      # 每 cm 至少跑 2%


def test_sensitivity_is_reported_in_the_files(tmp_path, capsys):
    p = _write(tmp_path, _samples(TRUE, noise_px=0.6))
    params, rep, meta, _ = build_report(p, holdout=3)
    ok = print_report(p, params, rep, meta)
    j, m = write_report(p, params, rep, meta, ok)
    d = json.load(open(j, encoding='utf-8'))
    assert d['sensitivity'] and abs(d['sensitivity']['df_per_cm']) > 0.0
    assert '参数敏感度' in open(m, encoding='utf-8').read()
