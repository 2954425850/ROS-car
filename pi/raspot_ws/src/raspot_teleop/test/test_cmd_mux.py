# -*- coding: utf-8 -*-
"""cmd_mux.MuxCore 测试（不需要 ROS）：
    cd pi/raspot_ws/src && python -m pytest raspot_teleop/test/test_cmd_mux.py -q
"""
from raspot_teleop.cmd_mux import MuxCore


def test_nobody_talking_is_zero():
    assert MuxCore().select(10.0) == (None, 0.0, 0.0)


def test_teleop_overrides_voice():
    m = MuxCore()
    m.feed('voice', 0.3, 0.0, 10.0)
    m.feed('teleop', -0.1, 0.5, 10.0)
    assert m.select(10.1) == ('teleop', -0.1, 0.5)


def test_voice_takes_over_after_teleop_goes_quiet():
    m = MuxCore()
    m.feed('teleop', 0.0, 0.0, 10.0)          # PS2 松杆零速尾巴的最后一帧
    m.feed('voice', 0.3, 0.0, 10.55)
    assert m.select(10.4)[0] == 'teleop'
    assert m.select(10.6) == ('voice', 0.3, 0.0)


def test_stale_source_stops_car():
    m = MuxCore()
    m.feed('voice', 0.3, 0.0, 10.0)
    assert m.select(10.5) == ('voice', 0.3, 0.0)
    assert m.select(10.51) == (None, 0.0, 0.0)


def test_legacy_cloud_beats_voice_but_not_teleop():
    m = MuxCore()
    m.feed('voice', 0.3, 0.0, 10.0)
    m.feed('legacy', 0.2, 0.0, 10.0)
    assert m.select(10.0)[0] == 'legacy'
    m.feed('teleop', 0.1, 0.0, 10.0)
    assert m.select(10.0)[0] == 'teleop'


def test_nav_is_lowest():
    m = MuxCore()
    m.feed('nav', 0.2, 0.1, 10.0)
    assert m.select(10.0)[0] == 'nav'
    m.feed('voice', 0.0, 0.0, 10.0)
    assert m.select(10.0)[0] == 'voice'
