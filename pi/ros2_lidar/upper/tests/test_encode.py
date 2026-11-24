import base64
import math
import struct
from types import SimpleNamespace

from encode import (occupancy_payload, yaw_from_quat, laser_origin,
                    scan_to_points, pack_points)


def _grid(data, w=2, h=1):
    return SimpleNamespace(
        info=SimpleNamespace(width=w, height=h, resolution=0.05,
                             origin=SimpleNamespace(
                                 position=SimpleNamespace(x=-1.0, y=-2.0))),
        data=data)


def test_occupancy_payload_keeps_unknown_as_0xff():
    p = occupancy_payload(_grid([-1, 100]))
    assert p["w"] == 2 and p["h"] == 1
    assert p["res"] == 0.05
    assert (p["ox"], p["oy"]) == (-1.0, -2.0)
    assert base64.b64decode(p["data"]) == bytes([0xFF, 100])


def test_yaw_from_quat_identity_and_quarter_turn():
    assert yaw_from_quat(0.0, 0.0, 0.0, 1.0) == 0.0
    s = math.sin(math.pi / 4)
    assert abs(yaw_from_quat(0.0, 0.0, s, s) - math.pi / 2) < 1e-9


def test_laser_origin_respects_yaw():
    x, y = laser_origin(10.0, 20.0, 0.0)
    assert abs(x - 9.744) < 1e-9 and abs(y - 20.0065) < 1e-9
    x, y = laser_origin(10.0, 20.0, math.pi / 2)
    assert abs(x - 9.9935) < 1e-9 and abs(y - 19.744) < 1e-9


def test_scan_to_points_drops_bad_and_puts_range_along_angle():
    ranges = [2.0, float("nan"), 0.1, float("inf")]
    pts = scan_to_points(ranges, 0.0, math.pi / 2, range_min=0.15,
                         range_max=12.0, tx=1.0, ty=2.0, yaw=0.0)
    assert len(pts) == 1
    assert abs(pts[0][0] - (1.0 - 0.256 + 2.0)) < 1e-9
    assert abs(pts[0][1] - (2.0 + 0.0065)) < 1e-9


def test_scan_to_points_uses_yaw_in_angle():
    # 车头朝 +y 时，0 度射线应指向 +y
    pts = scan_to_points([1.0], 0.0, 0.0, 0.0, 12.0,
                         tx=0.0, ty=0.0, yaw=math.pi / 2)
    assert abs(pts[0][0] - (-0.0065)) < 1e-9
    assert abs(pts[0][1] - (-0.256 + 1.0)) < 1e-9


def test_pack_points_roundtrip():
    s = pack_points([(1.5, -2.0), (0.0, 3.0)])
    raw = base64.b64decode(s)
    assert len(raw) == 16
    assert struct.unpack("<ff", raw[:8]) == (1.5, -2.0)
