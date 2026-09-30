import pytest
from arm_grasp import ltrace

FB_A = [578.0, 496.0, 126.0, 213.0, 483.0, -277.0]
FB_B = [578.0, 496.0, 226.0, 313.0, 583.0, -177.0]


def test_interpolates_halfway():
    tr = ltrace.Trace()
    tr.add(10.0, FB_A, FB_A)
    tr.add(10.4, FB_B, FB_B)
    got = tr.at(10.2)
    assert got == pytest.approx([(a + b) / 2 for a, b in zip(FB_A, FB_B)], abs=1e-6)


def test_clamps_out_of_range():
    tr = ltrace.Trace()
    tr.add(10.0, FB_A, FB_A)
    tr.add(10.4, FB_B, FB_B)
    assert tr.at(9.0) == pytest.approx(FB_A)
    assert tr.at(99.0) == pytest.approx(FB_B)


def test_falls_back_to_cmd_when_fb_has_zero():
    """回读里 p3/p4/p5 出现 0 = 该拍没读到；那一刻必须退回指令值。"""
    tr = ltrace.Trace()
    bad = [578.0, 496.0, 0.0, 0.0, 0.0, -277.0]
    tr.add(10.0, FB_A, bad)
    tr.add(10.4, FB_A, bad)
    assert tr.at(10.2) == pytest.approx(FB_A)          # auto：用 cmd
    with pytest.raises(ValueError):
        tr.at(10.2, source='fb')                       # 强制 fb 就该报错


def test_empty_raises():
    with pytest.raises(ValueError):
        ltrace.Trace().at(1.0)
