# -*- coding: utf-8 -*-
"""gaze：视线 (az, el) ↔ 关节。"""
import pytest

from arm_grasp.arm_kin import FIELD_HI, FIELD_LO, from_fields, to_fields
from arm_grasp.gaze import gaze_of, joints_ok, solve_gaze
from arm_grasp.motion_spec import load_config

CFG = load_config()

# 上电复位位姿（grasp_server.HOME_JOINTS：p2..p5,base = 496,180,129,415,0）：肩 110.4 / 肘 -89.0 / 腕 -76.8，el=-55.4
HOME = [240.0, 496.0, 180.0, 129.0, 415.0, 0.0]
HOME_J = from_fields(HOME)
# 「伸出去低头」：底座左 30°、肩 50 / 肘 -40 / 腕 -70，el=-60
REACH_J = {'base': 30.0, 'shoulder': 50.0, 'elbow': -40.0, 'wrist_pitch': -70.0}


def _fields_in_limits(j):
    f = to_fields(j, 240, 496)
    return all(FIELD_LO - 1e-6 <= v <= FIELD_HI + 1e-6 for v in f[2:5])


def test_gaze_of_home():
    az, el = gaze_of(HOME_J)
    assert az == pytest.approx(0.0)
    assert el == pytest.approx(-55.44, abs=0.05)


@pytest.mark.parametrize('start', [HOME_J, REACH_J], ids=['home', 'reach'])
@pytest.mark.parametrize('az, el', [(0, -15), (90, -15), (-45, -40), (20, -63), (0, -75),
                                    (0, 30), (-90, -88), (180, -15), (260, 10)])
def test_roundtrip_through_fields(start, az, el):
    j, notes = solve_gaze(az, el, start, CFG)
    assert notes == []
    assert _fields_in_limits(j)
    # 经过 field 编解码（真正发给舵机的是 field）再算视线
    az2, el2 = gaze_of(from_fields(to_fields(j, 240, 496)))
    assert az2 == pytest.approx(az, abs=0.5)
    assert el2 == pytest.approx(el, abs=0.5)


def test_wrist_only_when_feasible():
    # home 腕 -76.8，往上抬 20°（el -35.4）只动腕就够
    j, _ = solve_gaze(0, -35.44, HOME_J, CFG)
    assert j['shoulder'] == pytest.approx(HOME_J['shoulder'])
    assert j['elbow'] == pytest.approx(HOME_J['elbow'])
    assert gaze_of(j)[1] == pytest.approx(-35.44)


def test_needs_shoulder_elbow_and_change_is_minimal():
    # home 往下 20°（el -75.4）⇒ 只动腕要 -96.8，超腕限位 -90 共 6.8° ⇒ 网格最小 L1 改动 = 7°
    el = gaze_of(HOME_J)[1] - 20
    j, notes = solve_gaze(0, el, HOME_J, CFG)
    assert notes == []
    d = abs(j['shoulder'] - HOME_J['shoulder']) + abs(j['elbow'] - HOME_J['elbow'])
    assert d == pytest.approx(7.0)
    assert _fields_in_limits(j)
    assert gaze_of(j)[1] == pytest.approx(el)


def test_base_truncated_with_note():
    j, notes = solve_gaze(300, -15, HOME_J, CFG)
    assert j['base'] == pytest.approx(270)
    assert any('还差 30' in n for n in notes)
    j, notes = solve_gaze(-280, -15, HOME_J, CFG)
    assert j['base'] == pytest.approx(-270)
    assert any('还差 10' in n and '右边' in n for n in notes)
    assert joints_ok(j, CFG)


def test_el_out_of_range_clamped_with_note():
    j, notes = solve_gaze(0, -120, HOME_J, CFG)
    assert gaze_of(j)[1] == pytest.approx(-90)
    assert any('最低' in n for n in notes)
    assert _fields_in_limits(j)
