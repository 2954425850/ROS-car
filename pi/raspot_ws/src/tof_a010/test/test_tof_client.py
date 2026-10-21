# -*- coding: utf-8 -*-
"""`tof_client` 的取数与基准偏置。**不起 ROS、不开串口**（直接喂内存里的帧）。"""
import numpy as np
import pytest

from tof_a010.tof_client import TofClient


def _cfg(**kw):
    c = dict(unit_mm_per_count=4.0, pad=0.0, min_mm=150.0, val_max=254,
             map={"ax": 1.0, "bx": 0.0, "ay": 1.0, "by": 0.0},
             exclude=[], offset_mm=0.0, max_age_s=0.25)
    c.update(kw)
    return c


def _client(mm_value, **kw):
    """`mm_value` 是**毫米**（和 `/tof/depth` 的 16UC1 一致，不是计数）。"""
    c = TofClient(cfg=_cfg(**kw))
    c._mm = np.full(100 * 100, mm_value, dtype=np.uint16)
    c._rows = c._cols = 100
    c._t = None                      # 不判时效（这里只测换算）
    return c


BOX = [0.4, 0.4, 0.6, 0.6]


# --------------------------------------------------------------------------
# 基准偏置
# --------------------------------------------------------------------------

def test_no_offset_passes_the_raw_reading_through():
    d, info = _client(256).depth_for_box(BOX)
    assert d == 256
    assert info["raw_mm"] == 256 and info["offset_mm"] == 0.0


def test_positive_offset_pushes_the_target_further_away():
    """★ +20 就是这次要做的：目标点 z 更低 ⇒ 臂扎得更深（现在停在上面 2cm）。"""
    d, info = _client(256, offset_mm=20.0).depth_for_box(BOX)
    assert d == 276
    assert info["raw_mm"] == 256 and info["offset_mm"] == 20.0


def test_negative_offset_is_symmetric():
    d, _ = _client(256, offset_mm=-20.0).depth_for_box(BOX)
    assert d == 236


def test_offset_is_applied_AFTER_the_near_field_floor():
    """★ 关键：150mm 下限判的是**传感器读数**。

    读数 40mm（模块自身的近场假回波）即使加 +20 也是 60mm，
    **仍然必须被拒**；如果实现成"先加偏置再判下限"，160 > 150 就会被放行 ——
    那等于把近场防线整体平移，抓取会去够一个根本不存在的近点。
    """
    d, _ = _client(40, offset_mm=20.0).depth_for_box(BOX)
    assert d is None


def test_offset_is_applied_after_the_saturation_ceiling():
    """254 计数 = 1016mm 是有效上限；255 计数 = 1020mm 已被判无效。"""
    assert _client(1016, offset_mm=20.0).depth_for_box(BOX)[0] == 1036
    assert _client(1020, offset_mm=20.0).depth_for_box(BOX)[0] is None


# --------------------------------------------------------------------------
# 那条"拿不到就 None"的契约
# --------------------------------------------------------------------------

def test_no_frame_yet_returns_none_with_a_reason():
    c = TofClient(cfg=_cfg())
    d, info = c.depth_for_box(BOX)
    assert d is None
    assert "还没有" in info["why"]


def test_stale_frame_is_refused():
    c = _client(256)
    c._t = 0.0                       # 很久以前
    d, info = c.depth_for_box(BOX)
    assert d is None and "太旧" in info["why"]


def test_box_outside_the_view_returns_none():
    d, _ = _client(256).depth_for_box([1.2, 0.4, 1.5, 0.6])
    assert d is None


# --------------------------------------------------------------------------
# 按拍摄时刻对齐（at_mono，2026-10-21）
# --------------------------------------------------------------------------

def _ring_client(frames, **kw):
    """frames = [(t_mono, mm_value), ...]，按时间顺序进环形缓冲。"""
    c = TofClient(cfg=_cfg(**kw))
    for t, v in frames:
        mm = np.full(100 * 100, v, dtype=np.uint16)
        c._ring.append((t, mm, 100, 100))
        c._mm, c._rows, c._cols, c._t = mm, 100, 100, t
    return c


def test_at_mono_picks_the_frame_nearest_the_capture_time():
    c = _ring_client([(10.00, 300), (10.05, 310), (10.10, 320), (10.15, 330)])
    d, info = c.depth_for_box(BOX, at_mono=10.06)
    assert d == 310                       # 不是最新的 330
    assert info["dt_s"] == pytest.approx(-0.01)


def test_at_mono_too_far_from_any_frame_is_refused():
    c = _ring_client([(10.00, 300), (10.05, 310)])
    d, info = c.depth_for_box(BOX, at_mono=10.40)
    assert d is None and "拍摄时刻" in info["why"]


def test_at_mono_ignores_age_of_latest_frame():
    # 拍摄时刻对得上就行，不看"最新一帧多旧"（离线回放/处理延迟都不该误拒）
    c = _ring_client([(10.00, 300)])
    d, _ = c.depth_for_box(BOX, at_mono=10.02)
    assert d == 300


def test_without_at_mono_behaviour_is_unchanged_latest_frame():
    c = _ring_client([(10.00, 300), (10.05, 310)])
    c._t = None                           # 不判时效
    d, _ = c.depth_for_box(BOX)
    assert d == 310
