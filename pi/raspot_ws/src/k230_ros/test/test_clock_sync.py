# -*- coding: utf-8 -*-
"""clock_sync.TickClock 测试（不需要 ROS）：
    cd pi/raspot_ws/src/k230_ros && python -m pytest test -q
"""
import random

import pytest

from k230_ros.clock_sync import TickClock

TRUE_OFF = 1234.5     # Pi mono = tick/1000 + 1234.5


def _stream(clock, seconds, hz=10.0, lat=lambda: 0.005, t0_tick=100000):
    for i in range(int(seconds * hz)):
        tick = t0_tick + int(i * 1000 / hz)
        clock.feed(tick, tick / 1000.0 + TRUE_OFF + lat())
    return tick


def test_min_of_jittery_latency_recovers_offset():
    random.seed(1)
    c = TickClock()
    # 最小延迟 3ms，之上叠 0~300ms 的 WiFi 抖动
    _stream(c, 10, lat=lambda: 0.003 + random.random() ** 3 * 0.3)
    assert c.offset() == pytest.approx(TRUE_OFF + 0.003, abs=0.002)


def test_mean_would_be_wrong_so_min_matters():
    # 变异锚点：如果实现成"取平均"，这条必红
    c = TickClock()
    lats = iter([0.003] + [0.2] * 49)
    _stream(c, 5, lat=lambda: next(lats))
    assert c.offset() == pytest.approx(TRUE_OFF + 0.003, abs=1e-6)


def test_old_samples_leave_window():
    c = TickClock(window_s=5.0)
    lats = iter([0.001] + [0.050] * 200)
    _stream(c, 20, lat=lambda: next(lats))
    # 那条 1ms 的快样本早已出窗，现在的最小值是 50ms
    assert c.offset() == pytest.approx(TRUE_OFF + 0.050, abs=1e-6)


def test_board_reboot_resets_window():
    c = TickClock()
    last = _stream(c, 5)
    # 重启：tick 从小值重新数，真实偏差跟着变
    c.feed(500, 600.0)
    assert c.resets == 1
    assert c.offset() == pytest.approx(600.0 - 0.5)
    assert last > 500


def test_capture_uses_cap_tick_and_falls_back():
    c = TickClock()
    _stream(c, 3)
    off = c.offset()
    rec = {'tick': 200000, 'cap_tick': 199920, '_rx_mono': 200.0 + TRUE_OFF + 0.005}
    mono, src = c.capture_mono(rec)
    assert src == 'cap_tick'
    assert mono == pytest.approx(199.920 + off, abs=1e-6)

    mono, src = c.capture_mono({'tick': 200100, '_rx_mono': 200.1 + TRUE_OFF + 0.005})
    assert src == 'tick'

    # cap_tick 比 tick 晚（回绕/乱序）或早得离谱 ⇒ 不信，退到 tick
    assert c.capture_mono({'tick': 200200, 'cap_tick': 200300,
                           '_rx_mono': 200.2 + TRUE_OFF})[1] == 'tick'
    assert c.capture_mono({'tick': 200300, 'cap_tick': 190000,
                           '_rx_mono': 200.3 + TRUE_OFF})[1] == 'tick'


def test_no_tick_uses_receive_time():
    c = TickClock()
    assert c.capture_mono({'_rx_mono': 5.0}) == (5.0, 'rx')


def test_min_in_middle_of_window():
    # 最快的样本既不是第一条也不是最后一条（变异锚点：只取队首/队尾的实现必红）
    c = TickClock()
    lats = iter([0.2] * 10 + [0.004] + [0.2] * 39)
    _stream(c, 5, lat=lambda: next(lats))
    assert c.offset() == pytest.approx(TRUE_OFF + 0.004, abs=1e-6)


def test_detections_from_record_pixels():
    from k230_ros.records import detections_from_record
    rec = {'w': 1280, 'h': 720, 'objs': [
        {'cls': 'cup', 'score': 0.8, 'box': [0.25, 0.5, 0.75, 1.0]},
        {'cls': 'obj', 'score': 1.0, 'box': [0.0, 0.0, 0.1, 0.1], 'src': 'track', 'track_id': 1},
        {'cls': 'bad', 'box': [0.1]},
    ]}
    d = detections_from_record(rec)
    assert len(d) == 2
    assert d[0] == ('cup', 0.8, 640.0, 540.0, 640.0, 360.0, '')
    assert d[1][-1] == 'track'
