# -*- coding: utf-8 -*-
"""odometry.py + protocol 0x55 解析的测试。不需要 ROS：
    cd pi/raspot_ws/src && python -m pytest raspot_teleop/test/test_odometry.py -q
"""
import math
import struct

import pytest

from raspot_teleop import protocol
from raspot_teleop.odometry import OdomIntegrator


def _feed(od, wheels, gz, seconds, hz=50.0, **kw):
    n = int(round(seconds * hz))
    t0 = od._last_t if od._last_t is not None else 0.0
    if od._last_t is None:
        od.update(wheels, gz, t0, **kw)
    for i in range(1, n + 1):
        od.update(wheels, gz, t0 + i / hz, **kw)


def test_straight_line_one_meter():
    od = OdomIntegrator()
    _feed(od, (0.5, 0.5, 0.5, 0.5), 0.0, 2.0)
    assert od.x == pytest.approx(1.0, abs=1e-9)
    assert od.y == pytest.approx(0.0, abs=1e-9)
    assert od.theta == pytest.approx(0.0, abs=1e-12)


def test_spin_in_place_uses_gyro_not_wheels():
    # 左轮后退、右轮前进（原地左转），轮速差按 track 1.0 会给 0.6 rad/s，
    # 陀螺只测到 0.3 —— 滑移转向的实情。航向必须跟陀螺走。
    od = OdomIntegrator()
    _feed(od, (-0.3, -0.3, 0.3, 0.3), 0.3, 2.0)
    assert od.theta == pytest.approx(0.6, abs=1e-9)
    assert od.x == pytest.approx(0.0, abs=1e-9)


def test_wheel_yaw_source_for_comparison():
    od = OdomIntegrator()
    _feed(od, (-0.3, -0.3, 0.3, 0.3), 0.3, 2.0, yaw_source='wheel')
    assert od.theta == pytest.approx(1.2, abs=1e-9)


def test_quarter_circle_arc_endpoint():
    # v=0.5, wz=0.5 → 半径 1 m；π 秒走完 1/4 圆，终点 (1, 1)
    od = OdomIntegrator()
    _feed(od, (0.25, 0.25, 0.75, 0.75), 0.5, math.pi, hz=500.0)
    assert od.theta == pytest.approx(math.pi / 2, abs=2e-3)
    assert od.x == pytest.approx(1.0, abs=5e-3)
    assert od.y == pytest.approx(1.0, abs=5e-3)


def test_still_gate_ignores_gyro_bias():
    od = OdomIntegrator()
    _feed(od, (0.0, 0.002, -0.001, 0.0), 0.01, 60.0)
    assert od.theta == 0.0 and od.x == 0.0


def test_gap_longer_than_max_dt_is_not_integrated():
    od = OdomIntegrator()
    od.update((0.5,) * 4, 0.0, 0.0)
    od.update((0.5,) * 4, 0.0, 3.0)           # 3 s 断流
    assert od.x == 0.0
    od.update((0.5,) * 4, 0.0, 3.02)
    assert od.x == pytest.approx(0.01)


def test_parse_t55_full_layout():
    body = bytes((0x55, 28)) + struct.pack(
        '<IhhhhhhhhHBB', 123456, 100, -200, 300, -400, 10, -20, 31415, -250,
        12345, 0x08, 7)
    frame = body + struct.pack('<H', protocol.crc16(body))
    buf = bytearray(frame)
    (kind, d), = protocol.UplinkParser.feed(buf)
    assert kind == 't55'
    assert d['tick_ms'] == 123456
    assert d['wheels'] == (0.1, -0.2, 0.3, -0.4)
    assert d['roll'] == pytest.approx(0.001)
    assert d['pitch'] == pytest.approx(-0.002)
    assert d['yaw'] == pytest.approx(3.1415)
    assert d['gz'] == pytest.approx(-0.25)
    assert d['voltage_mv'] == 12345
    assert d['faults'] == 0x08
    assert d['seq'] == 7
