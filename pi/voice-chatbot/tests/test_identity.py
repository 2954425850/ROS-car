"""身份 v1：认人判据、档案、记忆解析、编排（全部离线，不碰 ROS / 板子）。"""

from __future__ import annotations

import json
import time

import pytest

from identity import memory
from identity.manager import IdentityManager
from identity.profiles import ProfileStore
from identity.speaker import SpeakerConfig, guess_speaker
from tools.identity import register_identity_tools
from tools.registry import ToolRegistry
from vision.k230_state import FaceSighting, FaceWindow


def _f(t, who=None, match=None, live=0.9, ok=True, area=0.05, n=1):
    return FaceSighting(t=t, who=who, match=match, live=live, ok=ok, area=area,
                        box=(0.4, 0.3, 0.6, 0.6), n_faces=n)


# ---------------------------------------------------------------- 认人判据

def test_known_needs_repeated_high_score():
    g = guess_speaker([_f(1.0, "u001", 0.90), _f(1.2, "u001", 0.88), _f(1.4, "u001", 0.91)])
    assert g.status == "known" and g.pid == "u001"


def test_single_hit_is_not_enough():
    g = guess_speaker([_f(1.0, "u001", 0.95), _f(1.2), _f(1.4)])
    assert g.status == "unsure"


def test_score_in_fuzzy_zone_is_unsure_not_stranger():
    # 板子认出了（≥0.78）但不到叫名门槛 —— 既不叫名，也绝不能当新人去录
    g = guess_speaker([_f(t, "u001", 0.80) for t in (1.0, 1.2, 1.4, 1.6)])
    assert g.status == "unsure"


def test_stranger_requires_clear_live_faces():
    assert guess_speaker([_f(t) for t in (1.0, 1.2, 1.4)]).status == "stranger"
    assert guess_speaker([_f(t, live=0.1) for t in (1.0, 1.2, 1.4)]).status == "none"
    assert guess_speaker([_f(t, ok=False) for t in (1.0, 1.2, 1.4)]).status == "none"
    assert guess_speaker([_f(1.0), _f(1.2)]).status == "none"       # 只看清两帧


def test_none_detail_separates_no_face_quality_and_liveness():
    """「不认人」的四个病因必须能分开，否则没法判断该调站位还是调门限。"""
    assert "没有检测到人脸" in guess_speaker([]).detail
    d = guess_speaker([_f(1.0, ok=False), _f(1.2, ok=False)]).detail
    assert "质量" in d and "2 张" in d and "没有检测到人脸" not in d
    d = guess_speaker([_f(1.0, live=0.1), _f(1.2, live=0.2)]).detail
    assert "活体" in d and "2 张" in d


def test_stranger_detail_reports_best_score():
    g = guess_speaker([_f(t, match=0.61) for t in (1.0, 1.2, 1.4)])
    assert g.status == "stranger" and "0.61" in g.detail


def test_crowd_refuses_to_guess():
    s = []
    for t in (1.0, 1.2, 1.4):
        s += [_f(t, "u001", 0.95, area=0.08, n=2), _f(t, "u002", 0.95, area=0.02, n=2)]
    assert guess_speaker(s).status == "crowd"


def test_largest_face_per_frame_wins():
    # 背景里偶尔闪过的小脸不该算进来（同帧取最大）
    cfg = SpeakerConfig(crowd_frac=0.5)
    s = [_f(1.0, "u001", 0.9, area=0.08), _f(1.2, "u001", 0.9, area=0.08, n=2),
         _f(1.2, "u002", 0.95, area=0.01, n=2), _f(1.4, "u001", 0.9, area=0.08)]
    g = guess_speaker(s, cfg)
    assert g.status == "known" and g.pid == "u001"


def test_face_window_feed_and_since():
    w = FaceWindow(keep_s=5.0)
    w.feed({"stamp_mono": 10.0, "faces": [{"who": "u1", "match": 0.9, "live": 0.8, "ok": True,
                                            "box": [0.1, 0.1, 0.3, 0.4]}],
            "objs": [{"cls": "cup", "score": 0.7, "box": [0, 0, 0.1, 0.1]}]}, now=10.0)
    w.feed({"stamp_mono": 20.0, "faces": []}, now=20.0)
    assert w.faces_since(0) == []                      # 10s 那条出窗了
    assert w.objects(now=20.5) == []                   # 最新一帧没物体


# ---------------------------------------------------------------- 档案

def test_profiles_seed_name_and_facts(tmp_path):
    people = tmp_path / "people.json"
    people.write_text(json.dumps({"id1": "老王"}), encoding="utf-8")
    st = ProfileStore(tmp_path / "p", seed_people_json=str(people))
    assert st.find_by_name("老王") == "id1"
    assert st.next_auto_id() == "u001"
    st.ensure("u001", auto=True)
    assert st.next_auto_id() == "u002"
    st.set_name("u001", "张三")
    st.add_facts("u001", {"喜欢": "周杰伦"})
    st2 = ProfileStore(tmp_path / "p")                 # 重新加载
    assert st2.get("u001")["name"] == "张三"
    assert st2.get("u001")["facts"] == {"喜欢": "周杰伦"}
    assert st2.find_by_name("张三同学") == "u001"


def test_profiles_reject_unsafe_ids(tmp_path):
    st = ProfileStore(tmp_path)
    with pytest.raises(ValueError):
        st.ensure("../x")


# ---------------------------------------------------------------- 记忆解析

def test_parse_extraction_tolerates_wrapping_and_filters():
    name, facts = memory.parse_extraction(
        '好的：```json\n{"name": "张三", "facts": {"职业": "学生", "银行卡": "6222"}}\n```')
    assert name == "张三" and facts == {"职业": "学生"}
    assert memory.parse_extraction("不是 json") == (None, {})
    assert memory.parse_extraction('{"name": "我", "facts": []}') == (None, {})


# ---------------------------------------------------------------- 编排

class _Cfg:
    def __init__(self, d):
        self.d = d

    def get(self, k, default=None):
        return self.d.get(k, default)


class _State:
    enabled = True

    def __init__(self):
        self.window = FaceWindow()

    def fresh(self, now=None, max_age=3.0):
        return True


class _LLM:
    def __init__(self, out):
        self.out = out

    def complete(self, messages, max_tokens=None):
        return self.out


class _Target:
    def __init__(self):
        self.enrolled = []
        self.forgot = []

    def enroll(self, pid, timeout=10.0):
        self.enrolled.append(pid)
        return "enroll ok"

    def forget(self, pid):
        self.forgot.append(pid)
        return "ok"


def _mgr(tmp_path, llm_out='{"name": null, "facts": {}}', **cfg):
    c = {"identity.dir": str(tmp_path), "identity.enroll_cooldown_s": 0.0}
    c.update(cfg)
    st, tg = _State(), _Target()
    m = IdentityManager(_Cfg(c), _LLM(llm_out), st, tg)
    return m, st, tg


def _show(st, who, match, n=4):
    now = time.monotonic()
    for i in range(n):
        st.window.feed({"stamp_mono": now - 0.1 * i,
                        "faces": [{"who": who, "match": match, "live": 0.9, "ok": True,
                                   "box": [0.4, 0.3, 0.6, 0.6]}]}, now)


def _wait(cond, t=2.0):
    end = time.monotonic() + t
    while time.monotonic() < end and not cond():
        time.sleep(0.01)
    return cond()


def test_known_speaker_gets_named_context(tmp_path):
    m, st, _ = _mgr(tmp_path)
    m.profiles.ensure("u001", name="张三")
    m.profiles.add_facts("u001", {"喜欢": "周杰伦"})
    m.on_wake()
    _show(st, "u001", 0.92)
    text = m.begin_turn()
    assert m.current_pid == "u001"
    assert "张三" in text and "周杰伦" in text


def test_stranger_is_auto_enrolled_then_named_from_chat(tmp_path):
    m, st, tg = _mgr(tmp_path, llm_out='{"name": "李四", "facts": {"职业": "老师"}}')
    m.on_wake()
    _show(st, None, None)
    text = m.begin_turn()
    assert "认不出" in text
    assert _wait(lambda: tg.enrolled == ["u001"] and m.current_pid == "u001")
    m.end_turn("我叫李四，是个老师", "你好李四老师")
    assert _wait(lambda: (m.profiles.get("u001") or {}).get("name") == "李四")
    assert m.profiles.get("u001")["facts"] == {"职业": "老师"}


def test_fuzzy_face_is_never_enrolled(tmp_path):
    m, st, tg = _mgr(tmp_path)
    m.on_wake()
    _show(st, "u001", 0.80)
    m.begin_turn()
    time.sleep(0.05)
    assert tg.enrolled == [] and m.current_pid is None


def test_enroll_cooldown_distinguishes_success_from_failure(tmp_path):
    """成功后才进入长冷却；失败只等短重试间隔。"""
    m, _, _ = _mgr(tmp_path, **{"identity.enroll_cooldown_s": 120.0,
                                "identity.enroll_retry_s": 20.0})
    now = 1000.0
    m._last_enroll_ok_t = -1e9
    m._last_enroll_try_t = now - 21
    assert m._enroll_allowed(now) is True          # 失败 21s 后可重试
    m._last_enroll_try_t = now - 5
    assert m._enroll_allowed(now) is False         # 刚失败，5s 内不重试
    m._last_enroll_ok_t = now - 60
    m._last_enroll_try_t = now - 1000
    assert m._enroll_allowed(now) is False         # 成功冷却仍按 120s 算
    m._last_enroll_ok_t = now - 121
    assert m._enroll_allowed(now) is True


class _FailTarget(_Target):
    def enroll(self, pid, timeout=10.0):
        raise RuntimeError("k230ctl enroll 失败：enroll %s: 等结果超时（10s）" % pid)


class _AlreadyTarget(_Target):
    def enroll(self, pid, timeout=10.0):
        raise RuntimeError("k230ctl enroll 失败：enroll %s: fail n=8 already known as id1" % pid)


def test_failed_enroll_does_not_burn_long_cooldown(tmp_path):
    m, st, _ = _mgr(tmp_path, **{"identity.enroll_cooldown_s": 120.0,
                                 "identity.enroll_retry_s": 20.0})
    m._target = _FailTarget()
    m.on_wake()
    _show(st, None, None)
    m.begin_turn()
    assert _wait(lambda: m._last_enroll_try_t > 0)
    assert _wait(lambda: not m._enrolling)         # 等后台线程收尾
    assert m._last_enroll_ok_t < 0                 # 失败不写成功时间


def test_message_relay_delivered_once(tmp_path):
    m, st, _ = _mgr(tmp_path)
    m.profiles.ensure("u001", name="张三")
    m.profiles.ensure("u002", name="李四")
    reg = ToolRegistry()
    register_identity_tools(reg, m)
    m.current_pid = "u001"
    out = reg.dispatch("leave_message", {"to": "李四", "text": "晚上七点开会"}).result
    assert "记下了" in out
    m.on_wake()
    _show(st, "u002", 0.93)
    text = m.begin_turn()
    assert "晚上七点开会" in text and "张三" in text
    m.end_turn("在吗", "李四，张三让我转告你晚上七点开会")
    assert m.messages.peek("u002") == []


def _leave_message(m, to, text, sender_pid="u001"):
    m.profiles.ensure("u001", name="张三")
    m.profiles.ensure("u002", name="李四")
    reg = ToolRegistry()
    register_identity_tools(reg, m)
    prev = m.current_pid
    m.current_pid = sender_pid
    try:
        return reg.dispatch("leave_message", {"to": to, "text": text}).result
    finally:
        m.current_pid = prev


def test_message_kept_when_reply_ignores_it(tmp_path):
    """只说了「张三给你留了话」不算转告 —— 不能把内容删掉。"""
    m, st, _ = _mgr(tmp_path)
    _leave_message(m, "李四", "晚上七点开会")
    m.on_wake()
    _show(st, "u002", 0.93)
    m.begin_turn()
    m.end_turn("在吗", "你好呀，今天想做什么？")
    assert len(m.messages.peek("u002")) == 1
    # 下一轮仍然能拿到这条留言
    assert "晚上七点开会" in m.begin_turn()


def test_message_paraphrase_counts_as_delivered(tmp_path):
    m, st, _ = _mgr(tmp_path)
    _leave_message(m, "李四", "晚上七点开会")
    m.on_wake()
    _show(st, "u002", 0.93)
    m.begin_turn()
    m.end_turn("在吗", "李四，张三说晚上七点要开会")
    assert m.messages.peek("u002") == []


def test_only_conveyed_message_is_removed(tmp_path):
    m, st, _ = _mgr(tmp_path)
    _leave_message(m, "李四", "晚上七点开会")
    _leave_message(m, "李四", "记得带充电器")
    assert len(m.messages.peek("u002")) == 2
    m.on_wake()
    _show(st, "u002", 0.93)
    m.begin_turn()
    m.end_turn("在吗", "李四，张三说晚上七点要开会")
    left = [x["text"] for x in m.messages.peek("u002")]
    assert left == ["记得带充电器"]


def test_reply_conveys_ignores_punctuation():
    from identity.manager import reply_conveys
    assert reply_conveys("好的，晚上 7 点开会。", "晚上7点开会")
    assert not reply_conveys("张三给你留了话", "晚上七点开会")


def test_forget_self_removes_profile_and_board(tmp_path):
    m, _, tg = _mgr(tmp_path)
    m.profiles.ensure("u001", name="张三")
    m.profiles.ensure("u002", name="李四")
    reg = ToolRegistry()
    register_identity_tools(reg, m)
    m.current_pid = "u001"
    assert "只能删除你自己" in reg.dispatch("forget_person", {"name": "李四"}).result
    reg.dispatch("forget_person", {"name": "我"})
    assert m.profiles.get("u001") is None and tg.forgot == ["u001"]


def test_speaker_change_is_flagged(tmp_path):
    m, st, _ = _mgr(tmp_path)
    m.on_wake()
    _show(st, "u001", 0.92)
    m.begin_turn()
    assert not m.speaker_changed()
    st.window = FaceWindow()
    m.on_wake()
    _show(st, "u002", 0.92)
    m.begin_turn()
    assert m.speaker_changed()


def test_seed_placeholder_name_is_not_a_name(tmp_path):
    people = tmp_path / "people.json"
    people.write_text(json.dumps({"id1": "id1"}), encoding="utf-8")
    st = ProfileStore(tmp_path / "p", seed_people_json=str(people))
    assert st.get("id1")["name"] == ""


# ---------------------------------------------------------------- 显式「记住我」

def test_remember_person_when_known_updates_name(tmp_path):
    m, _, tg = _mgr(tmp_path)
    m.profiles.ensure("u001")
    m.current_pid = "u001"
    out = m.remember("高岩")
    assert "已经记住" in out and "高岩" in out
    assert m.profiles.get("u001")["name"] == "高岩"
    assert tg.enrolled == []


def test_remember_person_enrolls_and_reports_success(tmp_path):
    m, st, tg = _mgr(tmp_path, **{"identity.auto_enroll": False})
    m.on_wake()
    _show(st, None, None)
    m.begin_turn()
    out = m.remember("高岩")
    assert tg.enrolled == ["u001"]
    assert "记住了" in out and "高岩" in out
    assert m.current_pid == "u001"
    assert m.profiles.get("u001")["name"] == "高岩"
    assert m._last_enroll_ok_t > 0


def test_remember_failure_is_reported_not_pretended(tmp_path):
    m, st, _ = _mgr(tmp_path, **{"identity.auto_enroll": False})
    m._target = _FailTarget()
    m.on_wake()
    _show(st, None, None)
    m.begin_turn()
    out = m.remember("高岩")
    assert "没记住" in out and "没在时限内" in out
    assert m.profiles.get("u001") is None
    assert m.current_pid is None
    # 下一轮的身份段落要带上失败原因，模型才能如实回答
    m.on_wake()
    _show(st, None, None)
    text = m.begin_turn()
    assert "上一次建档没成功" in text


def test_remember_without_face_refuses(tmp_path):
    m, st, tg = _mgr(tmp_path, **{"identity.auto_enroll": False})
    m.on_wake()
    now = time.monotonic()
    st.window.feed({"stamp_mono": now, "faces": []}, now)
    m.begin_turn()
    out = m.remember("高岩")
    assert "没看清" in out and tg.enrolled == []


def test_remember_resolves_when_board_already_knows(tmp_path):
    m, st, _ = _mgr(tmp_path, **{"identity.auto_enroll": False})
    m._target = _AlreadyTarget()
    m.on_wake()
    _show(st, None, None)
    m.begin_turn()
    out = m.remember("高岩")
    assert "已经记住" in out
    assert m.current_pid == "id1"
    assert m.profiles.get("id1")["name"] == "高岩"


def test_remember_pronoun_is_not_stored_as_name(tmp_path):
    m, _, _ = _mgr(tmp_path)
    m.profiles.ensure("u001")
    m.current_pid = "u001"
    m.remember("我")
    assert m.profiles.get("u001")["name"] == ""


def test_people_summary_lists_profiles(tmp_path):
    m, _, _ = _mgr(tmp_path)
    assert "空的" in m.people_summary()
    m.profiles.ensure("u001", name="张三")
    m.profiles.add_facts("u001", {"职业": "学生"})
    m.current_pid = "u001"
    out = m.people_summary()
    assert "张三" in out and "档案 u001" in out and "跟你说话的人" in out


def test_stranger_context_tells_model_archive_exists(tmp_path):
    m, st, _ = _mgr(tmp_path)
    m.on_wake()
    _show(st, None, None)
    text = m.begin_turn()
    assert "人脸档案" in text and "只能记住这次对话" in text


def test_identity_tools_include_remember_and_list(tmp_path):
    m, _, _ = _mgr(tmp_path)
    reg = ToolRegistry()
    register_identity_tools(reg, m)
    assert "remember_person" in reg and "list_known_people" in reg
