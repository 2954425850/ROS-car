# -*- coding: utf-8 -*-
"""motion_plan：spec + 当前姿态 → 关键帧。"""
import json
import math
import re

import pytest

from arm_grasp.arm_kin import FIELD_HI, FIELD_LO, Unreachable, fk, from_fields, ikine, to_fields
from arm_grasp.gaze import gaze_of, joints_ok
from arm_grasp.motion_plan import nearest_equivalent_az, plan
from arm_grasp.motion_spec import load_config

CFG = load_config()

HOME = [240.0, 496.0, 180.0, 129.0, 415.0, 0.0]          # 上电复位位姿（grasp_server.HOME_JOINTS），夹爪张开
REACH_J = {'base': 30.0, 'shoulder': 50.0, 'elbow': -40.0, 'wrist_pitch': -70.0}
REACH = to_fields(REACH_J, 578.0, 496.0)                 # 「伸出去低头」，夹爪合上
MID = to_fields({'base': 0.0, 'shoulder': 80.0, 'elbow': -60.0, 'wrist_pitch': -60.0}, 240.0, 496.0)  # 平移余量较大


def P(spec, cur=HOME, history=None):
    return plan(json.dumps(spec, ensure_ascii=False), cur, history or [], CFG)


def J(fields):
    return from_fields(fields)


def gaze(fields):
    return gaze_of(J(fields))


def tip(fields):
    return fk(J(fields))[0]


def assert_valid(fields):
    assert len(fields) == 6
    assert all(FIELD_LO - 1e-6 <= v <= FIELD_HI + 1e-6 for v in fields[2:5]), fields
    assert abs(fields[5]) <= CFG.base_limit_deg * 1000 / 360 + 1e-6


# ---------------------------------------------------------------- turn（相对量）

def test_turn_same_amount_same_increment_different_endpoint():
    spec = {'steps': [{'turn': [{'dir': '下', 'amount': '一点'}]}]}
    ends = []
    for cur in (HOME, REACH):
        p = P(spec, cur)
        assert len(p.keyframes) == 1 and not p.partial
        f = p.keyframes[0].fields
        assert_valid(f)
        (az0, el0), (az1, el1) = gaze(cur), gaze(f)
        assert el1 - el0 == pytest.approx(-8, abs=0.5)     # 增量相同 = 档位表「一点」8°
        assert az1 == pytest.approx(az0, abs=0.5)
        ends.append((az1, el1))
    assert abs(ends[0][1] - ends[1][1]) > 2 or abs(ends[0][0] - ends[1][0]) > 2   # 终点不同


def test_turn_degrees_used_verbatim():
    p = P({'steps': [{'turn': [{'dir': '左', 'degrees': 30}]}]}, REACH)
    assert gaze(p.keyframes[0].fields)[0] == pytest.approx(60, abs=0.5)


def test_turn_two_dirs_combined():
    p = P({'steps': [{'turn': [{'dir': '左', 'amount': '一点'}, {'dir': '下', 'amount': '一点'}]}]})
    az, el = gaze(p.keyframes[0].fields)
    assert az == pytest.approx(8, abs=0.5)
    assert el == pytest.approx(gaze(HOME)[1] - 8, abs=0.5)


def test_turn_base_truncated_reports_gap():
    cur = list(HOME)
    cur[5] = 260 * 1000 / 360
    p = P({'steps': [{'turn': [{'dir': '左', 'amount': '很多'}]}]}, cur)
    assert gaze(p.keyframes[0].fields)[0] == pytest.approx(270, abs=0.5)
    assert p.partial and any('还差 30' in n for n in p.notes)


def test_turn_to_the_end_is_not_partial():
    p = P({'steps': [{'turn': [{'dir': '下', 'amount': '到底'}]}]})
    assert gaze(p.keyframes[0].fields)[1] == pytest.approx(-90, abs=0.5)
    assert not p.partial and p.notes


# ---------------------------------------------------------------- toward

def test_toward_named_direction():
    for cur in (HOME, REACH):
        p = P({'steps': [{'toward': '左边'}]}, cur)
        az, el = gaze(p.keyframes[0].fields)
        assert az == pytest.approx(90, abs=0.5) and el == pytest.approx(-15, abs=0.5)
        assert_valid(p.keyframes[0].fields)


def test_nearest_equivalent_az_avoids_winding():
    assert nearest_equivalent_az(-90, 200, 270) == 270     # 已经在左后方 200° ⇒ 「右边」走到 270 而不是倒回 -90
    assert nearest_equivalent_az(180, 100, 270) == 180
    assert nearest_equivalent_az(180, -100, 270) == -180
    assert nearest_equivalent_az(0, 200, 270) == 0          # 360 超限，只能回 0
    cur = list(HOME)
    cur[5] = 200 * 1000 / 360
    p = P({'steps': [{'toward': '右边'}]}, cur)
    assert gaze(p.keyframes[0].fields)[0] == pytest.approx(270, abs=0.5)


def test_toward_previous_pose_pops_history():
    hist = [list(REACH), list(HOME)]      # 最后一项是最近的
    cur = to_fields({'base': -40, 'shoulder': 90, 'elbow': -60, 'wrist_pitch': -20}, 400, 496)
    p = plan('{"steps": [{"toward": "上一个姿态"}]}', cur, hist, CFG)
    f = p.keyframes[0].fields
    assert f[2:] == pytest.approx(HOME[2:])
    assert f[0] == 400 and p.history_used == 1 and not p.partial
    # 一次 spec 里两次「上一个姿态」⇒ 依次往回
    p = plan('{"steps": [{"toward": "上一个姿态"}, {"toward": "上一个姿态"}]}', cur, hist, CFG)
    assert p.keyframes[1].fields[2:] == pytest.approx(REACH[2:])
    assert p.history_used == 2


def test_toward_previous_pose_without_history():
    p = P({'steps': [{'toward': '上一个姿态'}]})
    assert p.partial and any('上一个姿态' in n for n in p.notes)
    assert p.keyframes[0].fields == pytest.approx(HOME)


# ---------------------------------------------------------------- shift / approach

@pytest.mark.parametrize('cur, items, delta', [
    (HOME, [{'dir': '前', 'amount': '一些'}], (5, 0, 0)),
    (MID, [{'dir': '前', 'amount': '一点'}], (2, 0, 0)),
    (MID, [{'dir': '后', 'amount': '一些'}], (-5, 0, 0)),
    (MID, [{'dir': '上', 'cm': 2}], (0, 0, 2)),
    (MID, [{'dir': '前', 'amount': '一点'}, {'dir': '下', 'amount': '一点'}], (2, 0, -2)),
    (MID, [{'dir': '左', 'amount': '一些'}], (0, 5, 0)),
    (MID, [{'dir': '右', 'cm': 3}], (0, -3, 0)),
])
def test_shift_moves_tip_by_delta_keeping_el(cur, items, delta):
    p = P({'steps': [{'shift': items}]}, cur)
    f = p.keyframes[0].fields
    assert not p.partial, p.notes
    t0, t1 = tip(cur), tip(f)
    for i in range(3):
        assert t1[i] - t0[i] == pytest.approx(delta[i], abs=0.2)
    assert gaze(f)[1] == pytest.approx(gaze(cur)[1], abs=0.5)
    assert_valid(f)


def test_shift_unreachable_is_shortened_and_reported():
    # 复位位姿肘已贴 field 129，「往后」任何俯仰下都挪不满 5cm
    p = P({'steps': [{'shift': [{'dir': '后', 'amount': '一些'}]}]}, HOME)
    f = p.keyframes[0].fields
    assert p.partial
    note = [n for n in p.notes if '挪了' in n]
    assert note, p.notes
    moved = float(re.search(r'挪了 ([\d.]+)cm', note[0]).group(1))
    assert 0.1 < moved < 5.0
    assert math.dist(tip(HOME), tip(f)) == pytest.approx(moved, abs=0.2)
    assert tip(HOME)[0] - tip(f)[0] == pytest.approx(moved, abs=0.2)      # 方向还是「后」
    assert_valid(f)


def _reachable(t, el):
    """独立判定：爪尖 t、俯仰 el 时是否有 field 合法的 IK 解（两个肘支）。"""
    for br in (-1, 1):
        try:
            j = ikine(t[0], t[1], t[2], el, branch=br)
        except Unreachable:
            continue
        if joints_ok(j, CFG):
            return True
    return False


def _sight_note_deg(notes):
    m = [re.search(r'俯仰变了 (\d+)°', n) for n in notes]
    m = [x for x in m if x]
    return int(m[0].group(1)) if m else 0


@pytest.mark.parametrize('step', [
    {'shift': [{'dir': '上', 'cm': 5}]},
    {'shift': [{'dir': '下', 'cm': 5}]},
    {'approach': {'dir': '靠近', 'cm': 5}},
    {'approach': {'dir': '远离', 'cm': 5}},
], ids=['up', 'down', 'closer', 'away'])
def test_shift_from_home_reaches_by_changing_el_minimally(step):
    """设计 §7 修订：保持俯仰到不了 ⇒ 改俯仰，取 |Δel| 最小、能完整到达的解，不算截断。"""
    p = P({'steps': [step]}, HOME)
    f = p.keyframes[0].fields
    assert_valid(f)
    assert not p.partial, p.notes
    t0, t1 = tip(HOME), tip(f)
    if 'shift' in step:
        want = [0, 0, 5 if step['shift'][0]['dir'] == '上' else -5]
    else:                                   # approach 方向用**起始**视线
        _, axis0 = fk(J(HOME))
        sign = 1 if step['approach']['dir'] == '靠近' else -1
        want = [5 * sign * a for a in axis0]
    for i in range(3):
        assert t1[i] - t0[i] == pytest.approx(want[i], abs=0.2)
    el0, el1 = gaze(HOME)[1], gaze(f)[1]
    d_el = abs(el1 - el0)
    assert d_el > 0.5                       # 复位位姿下保持俯仰确实不行
    assert not _reachable(t1, el0)
    assert _sight_note_deg(p.notes) == round(d_el)
    # 最小性：比它少 1° 的两个俯仰都到不了
    for el in (el0 + (d_el - 1), el0 - (d_el - 1)):
        assert not _reachable(t1, el), el


def test_approach_moves_along_line_of_sight():
    p = P({'steps': [{'approach': {'dir': '靠近', 'amount': '一点'}}]}, REACH)
    f = p.keyframes[0].fields
    assert not p.partial
    _, axis = fk(J(REACH))
    d = [b - a for a, b in zip(tip(REACH), tip(f))]
    for i in range(3):
        assert d[i] == pytest.approx(2 * axis[i], abs=0.2)
    assert gaze(f) == pytest.approx(gaze(REACH), abs=0.5)


# ---------------------------------------------------------------- gripper

def test_gripper_values_and_wrist_roll_kept():
    for word, val in (('张开', 240), ('合上', 578), ('半开', 400)):
        cur = list(REACH)
        cur[1] = 333.0
        f = P({'steps': [{'gripper': word}]}, cur).keyframes[0].fields
        assert f[0] == val and f[1] == 333.0
        assert f[2:] == pytest.approx(cur[2:])


def test_one_step_composes_into_one_frame():
    p = P({'steps': [{'toward': '左边', 'turn': [{'dir': '下', 'amount': '一点'}], 'gripper': '合上'}]})
    assert len(p.keyframes) == 1
    f = p.keyframes[0].fields
    assert gaze(f) == pytest.approx((90, -23), abs=0.5)
    assert f[0] == 578


# ---------------------------------------------------------------- oscillate

@pytest.mark.parametrize('cur', [HOME, REACH], ids=['home', 'reach'])
@pytest.mark.parametrize('axis, idx', [('上下', 2), ('左右', 5), ('夹爪', 0)])
@pytest.mark.parametrize('times', [None, 1, 4])
def test_oscillate_frames_and_return_to_start(cur, axis, idx, times):
    osc = {'axis': axis}
    if times:
        osc['times'] = times
    p = P({'steps': [{'oscillate': osc}]}, cur)
    n = times or CFG.oscillate_times
    assert len(p.keyframes) == 2 * n + 1
    assert p.keyframes[-1].fields == pytest.approx(cur)
    for k in p.keyframes:
        assert_valid(k.fields)
        # 只动该轴；点头腕到限位时允许肩/肘(p4/p5)补足，但底座/夹爪/腕旋不动
        moving = {idx, 3, 4} if axis == '上下' else {idx}
        for i in range(6):
            if i not in moving:
                assert k.fields[i] == pytest.approx(cur[i])
    vals = [k.fields[idx] for k in p.keyframes[:-1]]
    if axis == '夹爪':                                          # 开/合交替，第一下先往相反状态走
        assert set(vals) == {240.0, 578.0}
        assert vals[0] != cur[0]
    else:
        assert all(abs(v - cur[idx]) > 1 for v in vals)        # 每个来回帧真的动了
    assert all(a != pytest.approx(b) for a, b in zip(vals, vals[1:]))   # 去、回交替


def test_nod_amplitude():
    p = P({'steps': [{'oscillate': {'axis': '上下', 'amount': '一点'}}]}, REACH)
    els = [gaze(k.fields)[1] for k in p.keyframes]
    assert els[0] == pytest.approx(-68, abs=0.5) and els[1] == pytest.approx(-52, abs=0.5)
    assert not p.partial


def test_nod_at_wrist_limit_uses_shoulder_full_amplitude():
    """2026-10-31 实车：home 腕 -76.8，只动腕往下只有 13°（实车 7°，用户没看出点头）。
    现在腕到限位由肩/肘补足 ⇒ 上下都满 15°、不 partial。"""
    p = P({'steps': [{'oscillate': {'axis': '上下'}}]})
    el0 = gaze(HOME)[1]
    els = [gaze(k.fields)[1] for k in p.keyframes]
    assert els[0] == pytest.approx(el0 - 15, abs=0.5) and els[1] == pytest.approx(el0 + 15, abs=0.5)
    assert p.keyframes[0].fields[4] != pytest.approx(HOME[4])     # 肩动了
    assert not p.partial


def test_nod_clipped_at_el_limit_is_reported():
    """视线已经接近 −90：往下的那一半到不了，要报。"""
    start = P({'steps': [{'turn': [{'dir': '下', 'degrees': 32}]}]}).keyframes[0].fields   # el ≈ −87
    p = P({'steps': [{'oscillate': {'axis': '上下'}}]}, start)
    assert p.partial and any('点头' in n and '往下' in n for n in p.notes)


# ---------------------------------------------------------------- 时长

def test_duration_formula_and_speed_order():
    t = {}
    for sp in ('慢', '正常', '快'):
        p = P({'steps': [{'toward': '左边'}], 'speed': sp})
        t[sp] = p.keyframes[0].t_ms
    # 底座 0 → 90° 是最大关节变化
    sp = CFG.speeds
    assert t == {k: math.ceil(90.0 / sp[k] * 1000) for k in ('慢', '正常', '快')}
    assert t['慢'] > t['正常'] > t['快'] >= 250


def test_duration_has_floor():
    p = P({'steps': [{'turn': [{'dir': '下', 'degrees': 1}]}], 'speed': '快'})
    assert p.keyframes[0].t_ms == 250
    for k in P({'steps': [{'oscillate': {'axis': '上下', 'amount': '一点'}}]}, REACH).keyframes:
        assert k.t_ms >= 250


def test_default_speed_normal_and_oscillate_fast():
    big = {'steps': [{'turn': [{'dir': '左', 'degrees': 60}]}]}
    assert P(big).keyframes[0].t_ms == math.ceil(60.0 / CFG.speeds['正常'] * 1000)   # 默认「正常」
    osc = {'oscillate': {'axis': '左右', 'amount': '很多', 'times': 1}}
    p = P({'steps': [osc]})
    assert p.keyframes[1].t_ms == math.ceil(50 / CFG.speeds['快'] * 1000)      # 快；第 2 帧从 +25 摆到 -25
    p = P({'steps': [osc], 'speed': '慢'})
    assert p.keyframes[1].t_ms == math.ceil(50 / CFG.speeds['慢'] * 1000)


# ---------------------------------------------------------------- repeat / 杂项

def test_repeat_accumulates_relative_steps():
    p = P({'steps': [{'turn': [{'dir': '下', 'amount': '一点'}]}], 'repeat': 2}, REACH)
    assert len(p.keyframes) == 2
    assert gaze(p.keyframes[-1].fields)[1] == pytest.approx(gaze(REACH)[1] - 16, abs=0.5)


def test_multi_step_sequence():
    p = P({'steps': [{'turn': [{'dir': '下', 'amount': '一些'}]}, {'gripper': '合上'}]})
    assert len(p.keyframes) == 2
    assert p.keyframes[1].fields[2:] == pytest.approx(p.keyframes[0].fields[2:])
    assert p.keyframes[0].fields[0] == 240 and p.keyframes[1].fields[0] == 578


def test_readback_slightly_out_of_range_is_tolerated():
    cur = list(HOME)
    cur[2] = 123.0                  # 腕回读略低于 125
    p = P({'steps': [{'turn': [{'dir': '上', 'amount': '一点'}]}]}, cur)
    assert_valid(p.keyframes[0].fields)
    # 只动夹爪的帧也不能把越界的回读原样发回去
    for c in (cur, [240.0, 496.0, 300.0, 123.0, 415.0, 0.0], [240.0, 496.0, 300.0, 300.0, 877.0, 0.0]):
        p = P({'steps': [{'gripper': '合上'}]}, c)
        assert_valid(p.keyframes[0].fields)


def test_to_spec_samples_plan_from_both_poses():
    import importlib.util
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_motion_spec.py')
    sp = importlib.util.spec_from_file_location('_motion_spec_samples', path)
    mod = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(mod)
    TO_SPEC_SAMPLES = mod.TO_SPEC_SAMPLES
    for cur in (HOME, REACH):
        for js in TO_SPEC_SAMPLES:
            p = plan(js, cur, [list(HOME)], CFG)
            assert p.keyframes
            for k in p.keyframes:
                assert_valid(k.fields)
                assert k.t_ms >= 250
