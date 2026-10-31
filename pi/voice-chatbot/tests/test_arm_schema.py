"""机械臂工具 schema 与评测语料的结构测试（Task 1）。

不联网、不 import rclpy。只验：
  - SCHEMAS_A / SCHEMAS_B 是合法的 OpenAI tools 形状，枚举与设计 §5/§5.1 一致，required 正确；
  - corpus 每条 expect 引用的工具 / 字段 / 枚举值在 schema 里存在；
  - 评测脚本的判定器（judge）与 spec 互转在几条典型输入上给出预期结论。
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml

from tools import arm_schema as S

ROOT = Path(__file__).resolve().parent.parent
CORPUS = yaml.safe_load((ROOT / "tests" / "arm_eval" / "corpus.yaml").read_text(encoding="utf-8"))

_spec = importlib.util.spec_from_file_location("eval_arm_tools", ROOT / "scripts" / "eval_arm_tools.py")
EV = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(EV)

# 设计 §5 / §5.1 的原文（抄在这里，不从 arm_schema 取 —— 否则改错了两边一起错，测不出来）
DESIGN = {
    "toward": {"前方", "左边", "右边", "左前", "右前", "上方", "地上", "脚下", "地上远处", "后方", "上一个姿态"},
    "turn_dir": {"上", "下", "左", "右"},
    "shift_dir": {"前", "后", "左", "右", "上", "下"},
    "approach_dir": {"靠近", "远离"},
    "amount": {"一点", "一些", "很多", "到底"},
    "gripper": {"张开", "合上", "半开"},
    "axis": {"上下", "左右", "夹爪"},
    "speed": {"慢", "正常", "快"},
}

# 现有工具名（corpus 里混淆对照会引用）
EXISTING = {"ask_user", "get_current_time", "play_music", "stop_music", "car_move", "car_turn", "car_stop",
            "car_status", "look", "follow_person", "follow_stop", "remember_person", "list_known_people",
            "leave_message", "forget_person", "track_object", "grasp_object",
            "mcp__weather__get_current_weather", "mcp__weather__get_forecast"}


def _by_name(schemas):
    return {s["function"]["name"]: s for s in schemas}


def _params(variant, name):
    return _by_name(S.schemas(variant))[name]["function"]["parameters"]


def _step_props(variant):
    """拿到变体里"一步"的 properties（A 取 arm_move.steps.items，B 取 arm_act.steps.items）。"""
    tool = "arm_move" if variant == "A" else "arm_act"
    return _params(variant, tool)["properties"]["steps"]["items"]["properties"]


# ---------------------------------------------------------------- schema 形状

@pytest.mark.parametrize("variant", ["A", "B"])
def test_openai_tool_shape(variant):
    seen = set()
    for s in S.schemas(variant):
        assert s["type"] == "function"
        fn = s["function"]
        assert set(fn) == {"name", "description", "parameters"}
        assert fn["name"] not in seen
        seen.add(fn["name"])
        assert fn["name"] not in EXISTING, "臂工具名不能和现有工具撞"
        assert len(fn["description"]) >= 20
        p = fn["parameters"]
        assert p["type"] == "object"
        assert isinstance(p["properties"], dict)
        assert set(p["required"]) <= set(p["properties"])


def test_variant_tool_sets():
    assert set(_by_name(S.SCHEMAS_A)) == {"arm_look", "arm_move", "arm_state", "arm_home"}
    assert set(_by_name(S.SCHEMAS_B)) == {"arm_act", "arm_state", "arm_home"}


def test_required_fields():
    assert _params("B", "arm_act")["required"] == ["steps"]
    assert _params("B", "arm_act")["properties"]["steps"]["minItems"] == 1
    for name in ("arm_look", "arm_move"):
        assert _params("A", name)["required"] == []
    for v in "AB":
        assert _params(v, "arm_state")["properties"] == {}
        assert _params(v, "arm_home")["properties"] == {}


def test_split_fields_A():
    assert set(_params("A", "arm_look")["properties"]) == {"toward", "turn", "speed"}
    assert set(_params("A", "arm_move")["properties"]) == {
        "shift", "approach", "gripper", "oscillate", "steps", "repeat", "speed"}


@pytest.mark.parametrize("variant", ["A", "B"])
def test_enums_match_design(variant):
    st = _step_props(variant)
    assert set(st) == {"toward", "turn", "shift", "approach", "gripper", "oscillate"}
    assert set(st["toward"]["enum"]) == DESIGN["toward"]
    turn = st["turn"]["items"]
    assert set(turn["properties"]["dir"]["enum"]) == DESIGN["turn_dir"]
    assert set(turn["properties"]["amount"]["enum"]) == DESIGN["amount"]
    assert turn["properties"]["degrees"]["type"] == "number"
    assert turn["required"] == ["dir"]
    shift = st["shift"]["items"]
    assert set(shift["properties"]["dir"]["enum"]) == DESIGN["shift_dir"]
    assert set(shift["properties"]["amount"]["enum"]) == DESIGN["amount"]
    assert shift["properties"]["cm"]["type"] == "number"
    assert shift["required"] == ["dir"]
    ap = st["approach"]
    assert set(ap["properties"]["dir"]["enum"]) == DESIGN["approach_dir"]
    assert ap["required"] == ["dir"]
    assert set(st["gripper"]["enum"]) == DESIGN["gripper"]
    osc = st["oscillate"]
    assert set(osc["properties"]["axis"]["enum"]) == DESIGN["axis"]
    assert set(osc["properties"]["amount"]["enum"]) == DESIGN["amount"] - {"到底"}
    assert osc["properties"]["times"]["type"] == "integer"
    assert osc["required"] == ["axis"]
    top = _params(variant, "arm_move" if variant == "A" else "arm_act")["properties"]
    assert set(top["speed"]["enum"]) == DESIGN["speed"]


def test_A_top_level_fields_same_as_step():
    """A 的 arm_look/arm_move 顶层字段必须和 step 里的同名字段一模一样（否则两条路径行为漂移）。"""
    st = _step_props("A")
    look = _params("A", "arm_look")["properties"]
    move = _params("A", "arm_move")["properties"]
    for f in S.LOOK_FIELDS:
        assert look[f] == st[f]
    for f in S.MOVE_FIELDS:
        assert move[f] == st[f]


@pytest.mark.parametrize("variant", ["A", "B"])
def test_descriptions_mark_car_boundary(variant):
    """动作工具的 description 必须写清"车不是我"——这是车/臂混淆为 0 的唯一手段。"""
    for name, s in _by_name(S.schemas(variant)).items():
        if name in ("arm_look", "arm_move", "arm_act"):
            d = s["function"]["description"]
            for w in ("car_turn", "car_move", "grasp_object"):
                assert w in d, f"{name} 的描述没提 {w}"


# ---------------------------------------------------------------- spec 互转

def test_to_spec_from_spec_roundtrip():
    specs = [
        {"steps": [{"turn": [{"dir": "下", "amount": "一点"}]}]},
        {"steps": [{"toward": "左边"}], "speed": "慢"},
        {"steps": [{"shift": [{"dir": "前", "cm": 5}]}]},
        {"steps": [{"oscillate": {"axis": "上下", "times": 2}}]},
        {"steps": [{"turn": [{"dir": "下", "amount": "一些"}]}, {"gripper": "张开"}]},
        {"steps": [{"toward": "前方", "gripper": "合上"}]},
    ]
    for v in "AB":
        for spec in specs:
            calls = S.from_spec(v, spec)
            assert len(calls) == 1
            name, args = calls[0]
            assert name in _by_name(S.schemas(v))
            assert EV.validate(args, _params(v, name)) == [], (v, spec, args)
            assert S.to_spec(name, args) == spec, (v, spec, name, args)


def test_to_spec_non_motion_is_none():
    assert S.to_spec("arm_state", {}) is None
    assert S.to_spec("car_turn", {"angle_deg": 90}) is None
    with pytest.raises(ValueError):
        S.to_spec("arm_act", {})


# ---------------------------------------------------------------- corpus 引用

def _alts(expect):
    return expect.get("any") or [expect]


def test_corpus_size_and_ids():
    ids = [c["id"] for c in CORPUS]
    assert len(ids) >= 50
    assert len(ids) == len(set(ids))
    cats = {i[0] for i in ids}
    assert cats == set(EV.CATEGORIES)
    for c in CORPUS:
        assert isinstance(c["say"], str) and c["say"].strip()
        assert "expect" in c


def test_corpus_covers_required_confusions():
    says = {c["say"]: c for c in CORPUS}
    for s in ("左转", "往前走", "掉个头", "抓那个杯子", "盯着那个人", "还有多少电"):
        assert s in says, f"缺混淆对照「{s}」"
    assert any("天气" in s for s in says)


_KIND_FIELDS = {
    "toward": {"to"}, "turn": {"dir", "amount", "degrees"}, "shift": {"dir", "amount", "cm"},
    "approach": {"dir", "amount", "cm"}, "gripper": {"value"}, "oscillate": {"axis", "amount", "times"},
}


def _enum_for(kind, field):
    st = _step_props("B")
    if kind == "toward":
        return set(st["toward"]["enum"])
    if kind == "gripper":
        return set(st["gripper"]["enum"])
    if kind in ("turn", "shift"):
        props = st[kind]["items"]["properties"]
    else:
        props = st[kind]["properties"]
    return set(props[field]["enum"]) if "enum" in props[field] else None


@pytest.mark.parametrize("item", CORPUS, ids=lambda c: c["id"])
def test_corpus_expect_references_exist(item):
    arm_tools = set(_by_name(S.SCHEMAS_A)) | set(_by_name(S.SCHEMAS_B))
    known = EXISTING | arm_tools
    exp = item["expect"]
    assert set(exp) <= {"any", "arm", "ordered", "speed", "extra_ok", "tool", "tools_any", "forbid"}
    for n in exp.get("forbid") or []:
        assert n in known
    for alt in _alts(exp):
        if "tool" in alt:
            assert alt["tool"] == "none" or alt["tool"] in known, alt
        for n in alt.get("tools_any") or []:
            assert n in known
        if "speed" in alt:
            assert alt["speed"] in DESIGN["speed"]
        for pat in alt.get("arm") or []:
            kind = pat["kind"]
            assert kind in _KIND_FIELDS, pat
            for f, v in pat.items():
                if f == "kind":
                    continue
                assert f in _KIND_FIELDS[kind], (kind, f)
                enum = _enum_for(kind, "dir" if f == "dir" else f) if f in ("dir", "amount", "axis") \
                    else (_enum_for(kind, f) if f in ("to", "value") else None)
                if enum is not None:
                    for x in (v if isinstance(v, list) else [v]):
                        assert x in enum, (item["id"], kind, f, x)
    # 多轮历史：spec 必须能在两个变体下渲染成合法调用
    for h in item.get("history") or []:
        assert "user" in h and "result" in h
        if "spec" in h:
            for v in "AB":
                for name, args in S.from_spec(v, h["spec"]):
                    assert EV.validate(args, _params(v, name)) == [], (item["id"], v, args)
        else:
            assert h["call"]["name"] in known


# ---------------------------------------------------------------- 判定器

_SCH_B = _by_name(S.SCHEMAS_B)
_SCH_A = _by_name(S.SCHEMAS_A)


def _j(expect, calls, sch=_SCH_B):
    return EV.judge(expect, calls, sch)


def test_judge_arm_pass_and_amount_mismatch():
    exp = {"arm": [{"kind": "turn", "dir": "下", "amount": ["一点", "一些"]}]}
    ok = _j(exp, [{"name": "arm_act", "args": {"steps": [{"turn": [{"dir": "下", "amount": "一点"}]}]}}])
    assert ok["pass"] and not ok["confusion"]
    bad = _j(exp, [{"name": "arm_act", "args": {"steps": [{"turn": [{"dir": "下", "amount": "到底"}]}]}}])
    assert not bad["pass"]


def test_judge_car_instead_of_arm_is_confusion():
    exp = {"arm": [{"kind": "toward", "to": "左边"}]}
    r = _j(exp, [{"name": "car_turn", "args": {"angle_deg": 90}}])
    assert not r["pass"] and r["confusion"]


def test_judge_arm_instead_of_car_is_confusion():
    r = _j({"tool": "car_turn"}, [{"name": "arm_act", "args": {"steps": [{"turn": [{"dir": "左"}]}]}}])
    assert not r["pass"] and r["confusion"]


def test_judge_extra_atom_fails_and_order():
    exp = {"arm": [{"kind": "turn", "dir": "下"}, {"kind": "gripper", "value": "张开"}], "ordered": True}
    good = {"steps": [{"turn": [{"dir": "下"}]}, {"gripper": "张开"}]}
    rev = {"steps": [{"gripper": "张开"}, {"turn": [{"dir": "下"}]}]}
    extra = {"steps": [{"turn": [{"dir": "下"}]}, {"gripper": "张开"}, {"toward": "前方"}]}
    assert _j(exp, [{"name": "arm_act", "args": good}])["pass"]
    assert not _j(exp, [{"name": "arm_act", "args": rev}])["pass"]
    assert not _j(exp, [{"name": "arm_act", "args": extra}])["pass"]


def test_judge_schema_violation_fails():
    exp = {"arm": [{"kind": "turn", "dir": "下"}]}
    r = _j(exp, [{"name": "arm_act", "args": {"steps": [{"turn": [{"dir": "下", "amount": "稍微"}]}]}}])
    assert not r["pass"] and "schema" in r["reason"]
    osc = {"steps": [{"oscillate": {"axis": "上下"}, "gripper": "张开"}]}
    assert not _j({"arm": [{"kind": "oscillate", "axis": "上下"}], "extra_ok": True},
                  [{"name": "arm_act", "args": osc}])["pass"]


def test_judge_numbers_and_speed():
    exp = {"arm": [{"kind": "turn", "dir": "左", "degrees": 30}]}
    assert _j(exp, [{"name": "arm_look", "args": {"turn": [{"dir": "左", "degrees": 30}]}}], _SCH_A)["pass"]
    assert not _j(exp, [{"name": "arm_look", "args": {"turn": [{"dir": "左", "degrees": 45}]}}], _SCH_A)["pass"]
    rng = {"arm": [{"kind": "turn", "dir": "下", "degrees": {"lte": 15}}]}
    assert _j(rng, [{"name": "arm_look", "args": {"turn": [{"dir": "下", "degrees": 10}]}}], _SCH_A)["pass"]
    assert not _j(rng, [{"name": "arm_look", "args": {"turn": [{"dir": "下", "degrees": 30}]}}], _SCH_A)["pass"]
    sp = {"arm": [{"kind": "turn", "dir": "下"}], "speed": "慢"}
    assert _j(sp, [{"name": "arm_look", "args": {"turn": [{"dir": "下"}], "speed": "慢"}}], _SCH_A)["pass"]
    assert not _j(sp, [{"name": "arm_look", "args": {"turn": [{"dir": "下"}]}}], _SCH_A)["pass"]


def test_judge_none_and_stray_arm():
    assert _j({"tool": "none"}, [])["pass"]
    assert _j({"tool": "none"}, [{"name": "get_current_time", "args": {}}])["pass"]
    r = _j({"tool": "none"}, [{"name": "arm_act", "args": {"steps": [{"oscillate": {"axis": "上下"}}]}}])
    assert not r["pass"] and r["stray_arm"]
    r = _j({"tool": "look"}, [{"name": "arm_look", "args": {"toward": "前方"}}, {"name": "look", "args": {}}],
           _SCH_A)
    assert not r["pass"]


def test_judge_any_alternatives():
    exp = {"any": [{"arm": [{"kind": "shift", "dir": "后"}]}, {"tool": "arm_home"}]}
    assert _j(exp, [{"name": "arm_home", "args": {}}])["pass"]
    assert _j(exp, [{"name": "arm_act", "args": {"steps": [{"shift": [{"dir": "后"}]}]}}])["pass"]
    assert not _j(exp, [{"name": "arm_act", "args": {"steps": [{"shift": [{"dir": "前"}]}]}}])["pass"]
