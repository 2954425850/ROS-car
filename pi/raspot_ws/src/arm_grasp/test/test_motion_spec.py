# -*- coding: utf-8 -*-
"""motion_spec：配置加载、spec 解析、档位换算。"""
import json
import os

import pytest

from arm_grasp import motion_spec as ms
from arm_grasp.motion_spec import MAX, SpecError, load_config, parse_spec, resolve_amount

CFG = load_config()

# 语音侧 `pi/voice-chatbot/tools/arm_schema.py` 的 `to_spec(name, args)` 在 2026-10-31（定稿形状 B）
# 实际产出的 spec JSON（两个包不在同一目录、不能互相 import，所以抄成固定样例；注释是产生它的调用）。
TO_SPEC_SAMPLES = [
    # to_spec('arm_act', {"turn": [{"dir": "下", "amount": "一点"}]})
    '{"steps": [{"turn": [{"dir": "下", "amount": "一点"}]}]}',
    # to_spec('arm_act', {"toward": "左边"})
    '{"steps": [{"toward": "左边"}]}',
    # to_spec('arm_act', {"toward": "上一个姿态"})
    '{"steps": [{"toward": "上一个姿态"}]}',
    # to_spec('arm_act', {"turn": [{"dir": "左", "degrees": 30}]})
    '{"steps": [{"turn": [{"dir": "左", "degrees": 30}]}]}',
    # to_spec('arm_act', {"shift": [{"dir": "前", "cm": 5}]})
    '{"steps": [{"shift": [{"dir": "前", "cm": 5}]}]}',
    # to_spec('arm_act', {"turn": [{"dir": "下", "amount": "一些"}], "approach": {"dir": "靠近", "amount": "一点"}})
    '{"steps": [{"turn": [{"dir": "下", "amount": "一些"}], "approach": {"dir": "靠近", "amount": "一点"}}]}',
    # to_spec('arm_act', {"gripper": "张开", "shift": [{"dir": "前", "amount": "一点"}]})
    '{"steps": [{"shift": [{"dir": "前", "amount": "一点"}], "gripper": "张开"}]}',
    # to_spec('arm_act', {"oscillate": {"axis": "上下", "times": 3}, "speed": "快"})
    '{"speed": "快", "steps": [{"oscillate": {"axis": "上下", "times": 3}}]}',
    # to_spec('arm_act', {"oscillate": {"axis": "左右"}})
    '{"steps": [{"oscillate": {"axis": "左右"}}]}',
    # to_spec('arm_act', {"steps": [{"turn": [{"dir": "下", "amount": "一些"}]}, {"gripper": "张开"}]})
    '{"steps": [{"turn": [{"dir": "下", "amount": "一些"}]}, {"gripper": "张开"}]}',
    # to_spec('arm_act', {"turn": [{"dir": "左", "amount": "一点"}, {"dir": "下", "amount": "一点"}], "repeat": 2, "speed": "慢"})
    '{"speed": "慢", "repeat": 2, "steps": [{"turn": [{"dir": "左", "amount": "一点"}, {"dir": "下", "amount": "一点"}]}]}',
    # to_spec('arm_act', {"toward": "地上", "steps": [{"gripper": "合上"}]})
    '{"steps": [{"toward": "地上"}, {"gripper": "合上"}]}',
    # to_spec('arm_move', {"oscillate": {"axis": "夹爪", "amount": "一点", "times": 2}})
    '{"steps": [{"oscillate": {"axis": "夹爪", "amount": "一点", "times": 2}}]}',
    # to_spec('arm_look', {"toward": "后方", "speed": "慢"})
    '{"speed": "慢", "steps": [{"toward": "后方"}]}',
]


# ---------------------------------------------------------------- 配置

def test_config_values_match_design():
    assert CFG.base_limit_deg == 270
    assert CFG.min_frame_ms == 250
    assert CFG.speeds == {'慢': 45, '正常': 100, '快': 160}
    assert CFG.oscillate_speed == '快'
    assert CFG.amounts['turn'] == {'一点': 8, '一些': 20, '很多': 40}
    assert CFG.amounts['shift'] == {'一点': 2, '一些': 5, '很多': 10}
    assert CFG.amounts['oscillate'] == {'一点': 8, '一些': 15, '很多': 25}
    assert CFG.gripper == {'张开': 240, '合上': 578, '半开': 400}
    assert CFG.directions['后方'] == (180, -15)
    assert CFG.directions['地上'] == CFG.directions['脚下'] == (0, -75)
    assert set(CFG.directions) == set(ms.TOWARD_NAMES)


def test_default_path_falls_back_to_source_tree():
    # Windows/无 ROS 环境没有 share 目录 ⇒ 退回源码树 config/motion.yaml
    p = ms.default_config_path()
    assert os.path.isfile(p) and p.replace('\\', '/').endswith('config/motion.yaml')


def test_config_missing_direction_is_rejected(tmp_path):
    src = open(ms._src_config_path(), encoding='utf-8').read()
    assert '  地上远处: [0, -40]\n' in src
    bad = tmp_path / 'motion.yaml'
    bad.write_text(src.replace('  地上远处: [0, -40]\n', ''), encoding='utf-8')
    with pytest.raises(ValueError, match='地上远处'):
        load_config(str(bad))


# ---------------------------------------------------------------- resolve_amount

def test_resolve_amount_table_number_default_and_max():
    assert resolve_amount('turn', '一点', None, CFG) == 8
    assert resolve_amount('shift', '很多', None, CFG) == 10
    assert resolve_amount('approach', '一些', None, CFG) == 5
    assert resolve_amount('oscillate', '很多', None, CFG) == 25
    assert resolve_amount('turn', '一点', 30, CFG) == 30          # 给了数就原样用
    assert resolve_amount('turn', None, None, CFG) == 20          # 默认「一些」
    assert resolve_amount('shift', '到底', None, CFG) == MAX
    with pytest.raises(SpecError):
        resolve_amount('turn', '一丢丢', None, CFG)


# ---------------------------------------------------------------- parse_spec

@pytest.mark.parametrize('js', TO_SPEC_SAMPLES)
def test_to_spec_samples_parse(js):
    spec = parse_spec(js)
    obj = json.loads(js)
    assert len(spec.steps) == len(obj['steps'])
    assert spec.repeat == obj.get('repeat', 1)
    assert spec.speed == obj.get('speed')


def test_parse_fields():
    st = parse_spec(TO_SPEC_SAMPLES[5]).steps[0]
    assert st.turn[0].dir == '下' and st.turn[0].amount == '一些'
    assert st.approach.dir == '靠近' and st.approach.amount == '一点'
    st = parse_spec(TO_SPEC_SAMPLES[3]).steps[0]
    assert st.turn[0].number == 30.0 and st.turn[0].amount is None
    st = parse_spec(TO_SPEC_SAMPLES[7]).steps[0]
    assert st.oscillate.axis == '上下' and st.oscillate.times == 3


BAD_SPECS = [
    ({'steps': [{'toward': '天上'}]}, '方向'),                                    # 未知命名方向
    ({'steps': [{'turn': [{'dir': '前'}]}]}, 'turn方向'),                          # turn 没有「前」
    ({'steps': [{'shift': [{'dir': '斜'}]}]}, 'shift方向'),
    ({'steps': [{'turn': [{'dir': '下', 'amount': '一丢丢'}]}]}, '程度'),          # 未知档位
    ({'steps': [{'oscillate': {'axis': '上下', 'amount': '到底'}}]}, '幅度'),
    ({'steps': [{'oscillate': {'axis': '上下'}, 'gripper': '张开'}]}, '同一步'),    # oscillate 混其它字段
    ({'steps': [{'oscillate': {'axis': '左右'}, 'turn': [{'dir': '下'}]}]}, '同一步'),
    ({'steps': [{'dir': '下', 'amount': '一些'}]}, '不认识的键'),                  # 拍平到步上
    ({'steps': []}, '没有任何动作'),
    ({'steps': [{}]}, '空的'),
    ({'steps': [{'turn': [{'dir': '下'}]}], 'repeat': 0}, 'repeat'),
    ({'steps': [{'turn': [{'dir': '下', 'degrees': -5}]}]}, '正数'),
    ({'steps': [{'gripper': '捏紧'}]}, '夹爪'),
    ({'steps': [{'turn': [{'dir': '下'}]}], 'speed': '飞快'}, '速度'),
    ({'turn': [{'dir': '下'}]}, '不认识的键'),
]


@pytest.mark.parametrize('bad, why', BAD_SPECS, ids=[str(i) for i in range(len(BAD_SPECS))])
def test_parse_rejects(bad, why):
    with pytest.raises(SpecError, match=why):
        parse_spec(json.dumps(bad, ensure_ascii=False))


def test_parse_rejects_bad_json():
    with pytest.raises(SpecError):
        parse_spec('{"steps": [')


def test_turn_single_object_is_tolerated():
    s = parse_spec({'steps': [{'turn': {'dir': '下', 'amount': '一点'}}]})
    assert [m.dir for m in s.steps[0].turn] == ['下']
