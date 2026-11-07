# -*- coding: utf-8 -*-
"""预合爪快照：采集（build/save）、离线拆解（deviation）、以及"进 close 那一拍真的会调"。

**判据一律是物理量**（毫米偏移的方向和大小），不看快照自己说自己对不对：
把物体放在"爪尖 + 已知偏移"处，`deviation()` 必须把那个偏移原样还回来。
把 `SIGMA`/轴方向/半径公式任何一个符号写反，下面的 `x`/`radial`/`lat` 会立刻红。
"""
import json
import math
import os

import pytest

from arm_grasp import geom, grasp, shot
from arm_grasp.arm_kin import from_fields, to_fields

from .test_grasp import CAP, FakeLink, Plant, _start_joints


def _pose(s_m=0.05):
    """一个真实的整臂姿态（目标上方 s 处）——别手搓关节角，直接问逆解。"""
    return to_fields(_start_joints(s_m), 240.0, 496.0)


def _obs_for_point(f_pose, P):
    """造一条"拍到 P"的观测：像素用正解，深度用 (P−光心)·轴 反解，和
    `geom.target_from_pixel_depth` 严格互逆。"""
    j = from_fields(f_pose)
    u, v = geom.project(j, P)
    C = geom.camera_center(j)
    _tip_cm, axis = __import__('arm_grasp.arm_kin', fromlist=['fk']).fk(j)
    z_axial = sum((P[i] - C[i]) * axis[i] for i in range(3))
    return u, v, z_axial * 1000.0


def _rec(offset=(0.0, 0.0, 0.0), O=None, f_pose=None, j_obs=None):
    f_pose = _pose() if f_pose is None else f_pose
    j_obs = f_pose if j_obs is None else j_obs
    tip = geom.gripper_tip(from_fields(j_obs), closed=False)
    P = tuple(tip[i] + offset[i] for i in range(3))
    u, v, d = _obs_for_point(f_pose, P)
    obs = grasp.Obs(t_arrive=0.0, u_ai=u, v_ai=v, frame=1, src='track',
                    box_norm=[0.4, 0.5, 0.6, 0.7], depth_mm=d)
    return shot.build(phase_from='descend', reason='扎到位', O=list(O or P), O_u=list(O or P),
                      est_n=5, est_rejects=0, alpha=math.radians(-70.0),
                      f_fb=list(f_pose), f_cmd=list(f_pose), f_tgt=list(f_pose),
                      s_ach=0.003, s_cmd=0.003, err_m=0.002, comps=[0.0, 0.0, 0.0],
                      obs=obs, obs_age=0.3, j_obs=list(j_obs), obs_n=9, obs_bad=0,
                      depth_refused=0, cfg=grasp.GraspConfig(), t_mono=1.0,
                      latency_s=0.0)


def test_deviation_returns_zero_when_object_is_exactly_at_the_tip():
    dev = shot.deviation(_rec())
    assert dev['d_tip'] == pytest.approx(0.0, abs=2e-6)
    assert dev['lat'] == pytest.approx(0.0, abs=2e-6)
    assert dev['radial'] == pytest.approx(0.0, abs=2e-6)


def test_deviation_recovers_a_known_10mm_lateral_offset():
    """目标比爪尖偏 +x 1cm ⇒ `x` 必须是 +10mm，`lat` 也≈10mm（轴近竖直）。"""
    dev = shot.deviation(_rec(offset=(0.010, 0.0, 0.0)))
    assert dev['x'] == pytest.approx(0.010, abs=2e-6)
    assert dev['y'] == pytest.approx(0.0, abs=2e-6)
    assert dev['lat'] == pytest.approx(0.010, abs=3e-4)
    assert dev['d_tip'] == pytest.approx(0.010, abs=3e-4)


def test_deviation_sign_flips_with_the_offset():
    """反号必须跟着反 —— 防"正负写反却绝对值漂亮"这种静默错误。"""
    a = shot.deviation(_rec(offset=(0.010, 0.0, 0.0)))['x']
    b = shot.deviation(_rec(offset=(-0.010, 0.0, 0.0)))['x']
    assert a > 0 > b
    assert a == pytest.approx(-b, abs=1e-6)


def test_deviation_splits_frozen_O_drift_from_landing_error():
    """`d_O`（冻结的 O 到物体真值）和 `d_tip`（爪尖到物体真值）必须是**两个**数。

    造一个"冻结的 O 偏了 8mm、爪尖却正好落在物体上"的局面：
    ⇒ `d_O≈8mm` 而 `d_tip≈0`。这正是"冻结不准"独占误差的那种情况。
    """
    rec = _rec(offset=(0.0, 0.0, 0.0))
    tip = geom.gripper_tip(from_fields(rec['f_fb']), closed=False)
    rec['O'] = [tip[0] + 0.008, tip[1], tip[2]]
    dev = shot.deviation(rec)
    assert dev['d_O'] == pytest.approx(0.008, abs=2e-6)
    assert dev['d_tip'] == pytest.approx(0.0, abs=2e-6)


def _axis():
    from arm_grasp.arm_kin import fk
    return fk(from_fields(_pose()))[1]


def test_offset_along_the_axis_lands_in_axial_not_lateral():
    """沿夹爪轴偏 1cm ⇒ `axial≈10mm`、`lat≈0`。写反了这条立刻红。"""
    ax = _axis()
    dev = shot.deviation(_rec(offset=tuple(v * 0.010 for v in ax)))
    assert dev['axial'] == pytest.approx(0.010, abs=3e-4)
    assert dev['lat'] == pytest.approx(0.0, abs=3e-4)


def test_offset_perpendicular_to_the_axis_is_pure_lateral():
    """垂直于轴偏 1cm ⇒ `lat≈10mm`、`axial≈0`（"扎到位"的判据不能被横向误差骗过）。"""
    ax = _axis()
    # ax × x̂ = (0, a_z, −a_y)：由构造与 ax 正交（ax 不平行于 x̂）
    perp = [0.0, ax[2], -ax[1]]
    n = math.sqrt(sum(v * v for v in perp))
    perp = [v / n for v in perp]
    dev = shot.deviation(_rec(offset=tuple(v * 0.010 for v in perp)))
    assert dev['lat'] == pytest.approx(0.010, abs=3e-4)
    assert dev['axial'] == pytest.approx(0.0, abs=3e-4)


def test_record_is_json_serializable_and_keeps_raw_inputs():
    rec = _rec()
    json.dumps(rec)                     # 不能有 namedtuple / numpy 混进来
    assert len(rec['f_fb']) == 6 and rec['obs']['depth_mm'] is not None
    assert rec['tip_fb'] == pytest.approx(
        list(geom.gripper_tip(from_fields(rec['f_fb']), closed=False)), abs=1e-9)


def test_missing_observation_degrades_to_None_instead_of_lying():
    rec = _rec()
    rec['obs'] = None
    dev = shot.deviation(rec)
    assert dev['d_tip'] is None and dev['lat'] is None and dev['d_O'] is None
    shot.summarize(rec)                 # 汇总不许炸


def test_save_load_update_roundtrip(tmp_path):
    rec = _rec()
    path = shot.save(str(tmp_path), rec)
    assert os.path.exists(path)
    got = shot.load_all(str(tmp_path))
    assert len(got) == 1 and got[0]['O'] == rec['O']
    shot.update(path, image='a.jpg', result={'ok': True, 'code': 'OK'})
    got = shot.load_all(str(tmp_path))
    assert len(got) == 1 and got[0]['image'] == 'a.jpg' and got[0]['result']['ok'] is True
    assert got[0]['result']['code'] == 'OK'
    assert shot.aggregate(got)['ok']['n'] == 1


# ---------------------------------------------------------------- 真的会在 close 调

def _run_all(link, **cfg_kw):
    # 用 test_grasp 那套**已验证跑得通**的配置（α 包络/夹具都是配套的），只改计时与快照
    from .test_grasp import _cfg
    cfg = _cfg()._replace(hz=10.0, max_seconds=60.0, max_ticks=900, **cfg_kw)
    return grasp.run(cfg, link, phase='all', log=lambda *a: None)


def _plant_and_link(shot_dir):
    plant = Plant(_start_joints(s_m=0.10))
    link = FakeLink(plant, CAP, grip_block=450.0)
    shots = []
    if shot_dir is not None:
        link.shot = shots.append          # 只加一个属性，FakeLink 其余行为不变
    return link, shots


def test_snapshot_fires_exactly_once_on_entering_close():
    link, shots = _plant_and_link('/tmp/x')
    rep = _run_all(link, shot_dir='/tmp/x')
    assert [p[0] for p in rep['phases']] == ['aim', 'descend', 'close', 'lift']
    assert len(shots) == 1, '进 close 只该拍一次（close 内部每次都调就错了）'
    rec = shots[0]
    assert rec['phase_from'] == 'descend'
    assert rec['O'] is not None and rec['f_fb'] is not None
    # 快照里的爪尖必须和那份回读自洽（否则离线算出来的偏差是假的）
    assert rec['tip_fb'] == pytest.approx(
        list(geom.gripper_tip(from_fields(rec['f_fb']), closed=False)), abs=1e-9)
    assert rep['close_shot'] is rec


def test_no_snapshot_when_shot_dir_is_off():
    link, shots = _plant_and_link(None)
    _run_all(link)
    assert shots == []


def test_snapshot_failure_never_breaks_the_grasp():
    """落盘炸了也必须照常抓完 —— 诊断代码不许影响生产路径。"""
    link, _ = _plant_and_link('/tmp/x')

    def boom(_rec):
        raise OSError('磁盘满了')
    link.shot = boom
    rep = _run_all(link, shot_dir='/tmp/x')
    assert [p[0] for p in rep['phases']] == ['aim', 'descend', 'close', 'lift']
    assert rep['close_shot']['phase_from'] == 'descend'
