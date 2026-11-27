import math
import os

import pytest

from motion_scan import (arc_samples, corridor_relevant, filter_ranges,
                         filter_ranges_union, lookup_tf_or_none,
                         motion_active, point_relevant, should_filter_cmd)

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HALF_L, HALF_W = 0.30, 0.20
PAD_L, PAD_W = 0.02, 0.05


def test_motion_active_dead_band():
    assert not motion_active(0.0, 0.0)
    assert not motion_active(0.01, 0.01)
    assert motion_active(0.1, 0.0)
    assert motion_active(0.0, 0.2)


def test_should_filter_cmd_fail_open_branches():
    # Disabled, missing command, stale command, or a stationary chassis all
    # mean "pass the full scan through" (conservative).
    assert not should_filter_cmd(False, 0.0, 0.5, 0.2, 0.0)
    assert not should_filter_cmd(True, None, 0.5, 0.2, 0.0)
    assert not should_filter_cmd(True, 0.6, 0.5, 0.2, 0.0)
    assert not should_filter_cmd(True, 0.1, 0.5, 0.0, 0.0)
    assert should_filter_cmd(True, 0.1, 0.5, 0.2, 0.0)


def test_lookup_tf_failure_is_fail_open():
    class Boom:
        def lookup_transform(self, *args, **kwargs):
            raise RuntimeError("extrapolation")

    class Ok:
        def lookup_transform(self, *args, **kwargs):
            return "tf"

    assert lookup_tf_or_none(Boom(), "base", "laser", None, 0.1) is None
    assert lookup_tf_or_none(Ok(), "base", "laser", None, 0.1) == "tf"


def test_arc_samples_straight_and_reverse():
    straight = arc_samples(0.2, 0.0, horizon=1.0, step=0.1)
    assert math.isclose(straight[-1][0], 0.2, abs_tol=1e-9)
    assert all(abs(y) < 1e-9 for _, y, _, _ in straight)

    reverse = arc_samples(-0.2, 0.0, horizon=1.0, step=0.1)
    assert math.isclose(reverse[-1][0], -0.2, abs_tol=1e-9)


def test_forward_keeps_front_only():
    samples = arc_samples(0.2, 0.0)
    assert point_relevant(0.45, 0.0, samples, HALF_L, HALF_W,
                          PAD_L, PAD_W, v_sign=1)
    assert not point_relevant(-0.36, 0.0, samples, HALF_L, HALF_W,
                              PAD_L, PAD_W, v_sign=1)
    assert not point_relevant(0.0, 0.26, samples, HALF_L, HALF_W,
                              PAD_L, PAD_W, v_sign=1)


def test_reverse_keeps_rear_only():
    samples = arc_samples(-0.2, 0.0)
    assert point_relevant(-0.45, 0.0, samples, HALF_L, HALF_W,
                          PAD_L, PAD_W, v_sign=-1)
    assert not point_relevant(0.45, 0.0, samples, HALF_L, HALF_W,
                              PAD_L, PAD_W, v_sign=-1)


def test_side_band_is_5cm_not_zero():
    # scan_self_filter already removes <= 2 cm.  The gate keeps the 2-5 cm
    # band so a real scrape still stops; 6 cm and beyond is ignored.
    assert corridor_relevant(0.0, 0.23, 1, HALF_L, HALF_W, PAD_L, PAD_W)
    assert corridor_relevant(0.0, 0.245, 1, HALF_L, HALF_W, PAD_L, PAD_W)
    assert not corridor_relevant(0.0, 0.26, 1, HALF_L, HALF_W, PAD_L, PAD_W)


def test_heading_lane_ignores_speed_and_side_wall():
    # Speed-independent lane: a far point straight ahead stays relevant...
    assert corridor_relevant(3.0, 0.10, 1, HALF_L, HALF_W, PAD_L, PAD_W)
    # ...but a wall 6 cm off the side does not, even alongside the body.
    assert not corridor_relevant(0.0, 0.26, 1, HALF_L, HALF_W, PAD_L, PAD_W)
    assert not corridor_relevant(-0.5, 0.10, 1, HALF_L, HALF_W, PAD_L, PAD_W)
    # Pure rotation has no lane.
    assert not corridor_relevant(3.0, 0.0, 0, HALF_L, HALF_W, PAD_L, PAD_W)


def test_pivot_sweeps_side_but_not_far_rear():
    samples = arc_samples(0.0, 0.35)
    assert point_relevant(0.0, 0.23, samples, HALF_L, HALF_W, PAD_L, PAD_W)
    assert not point_relevant(0.0, -0.60, samples, HALF_L, HALF_W,
                              PAD_L, PAD_W)


def test_filter_ranges_lane_works_at_crawl_speed():
    # v=0.05 m/s only sweeps 5 cm in 1 s, but the heading lane must still
    # keep an obstacle 3 m straight ahead and drop one behind.
    out, kept, checked = filter_ranges(
        [3.0, 0.5], angle_min=0.0, angle_increment=math.pi,
        samples=arc_samples(0.05, 0.0),
        half_length=HALF_L, half_width=HALF_W,
        pad_long=PAD_L, pad_width=PAD_W, v_sign=1)
    assert (kept, checked) == (1, 2)
    assert out[0] == 3.0
    assert math.isinf(out[1])


def test_filter_ranges_masks_unrelated_rays():
    ranges = [0.5, 0.30, 0.5]
    out, kept, checked = filter_ranges(
        ranges, angle_min=0.0, angle_increment=math.pi / 2,
        samples=arc_samples(0.2, 0.0),
        half_length=HALF_L, half_width=HALF_W,
        pad_long=PAD_L, pad_width=PAD_W)
    assert (kept, checked) == (1, 3)
    assert out[0] == 0.5
    assert math.isinf(out[1]) and math.isinf(out[2])


def test_filter_ranges_only_skips_nonfinite_samples():
    # 2026-11-27: 低于 range_min 的有限回波也是真障碍，必须按走廊处理。
    # 之前把它原样留下，导致车后 0.12 m 的箱子永远过滤不掉。
    ranges = [float('inf'), float('nan'), 0.01, 0.5]
    out, kept, checked = filter_ranges(
        ranges, angle_min=0.0, angle_increment=math.pi / 2,
        samples=arc_samples(0.2, 0.0),
        half_length=HALF_L, half_width=HALF_W,
        pad_long=PAD_L, pad_width=PAD_W,
        range_min=0.05, range_max=9.0)
    assert math.isinf(out[0]) and math.isnan(out[1])
    assert out[2] == 0.01          # below range_min but inside footprint -> kept
    assert math.isinf(out[3])      # -y ray is outside the forward sweep
    assert checked == 2 and kept == 1


def test_union_keeps_old_or_new_motion():
    # Reversal hold: the old forward mask and the new reverse mask must both
    # stay in force, not replace each other.
    out, kept, checked = filter_ranges_union(
        [0.5, 0.26, 0.5], angle_min=0.0, angle_increment=math.pi / 2,
        motions=[(arc_samples(0.2, 0.0), 1),
                 (arc_samples(-0.2, 0.0), -1)],
        half_length=HALF_L, half_width=HALF_W,
        pad_long=PAD_L, pad_width=PAD_W)
    assert (kept, checked) == (2, 3)
    assert out[0] == 0.5           # old forward
    assert out[2] == 0.5           # new reverse
    assert math.isinf(out[1])      # side, neither motion


def test_both_nav_configs_point_collision_monitor_at_the_gate():
    yaml = pytest.importorskip("yaml")
    for name in ("nav2_params.yaml", "nav2_params_fusion.yaml"):
        with open(os.path.join(HERE, "config", name), encoding="utf-8") as f:
            d = yaml.safe_load(f)
        src = d["collision_monitor"]["ros__parameters"]["scan"]
        assert src["topic"] == "scan_motion_guard", name


def test_both_modes_treat_global_unknown_as_free():
    # 2026-11-27: both nav and fusion global costmaps treat unknown as free.
    # Fusion relies on the local costmap / collision_monitor for real near
    # obstacles; camera + ToF fusion is the future 3D complement.
    yaml = pytest.importorskip("yaml")

    def load(name):
        with open(os.path.join(HERE, "config", name), encoding="utf-8") as f:
            return yaml.safe_load(f)

    nav = load("nav2_params.yaml")["global_costmap"]["global_costmap"][
        "ros__parameters"]
    fus = load("nav2_params_fusion.yaml")["global_costmap"]["global_costmap"][
        "ros__parameters"]
    assert nav["track_unknown_space"] is False
    assert fus["track_unknown_space"] is False


def test_near_return_below_range_min_is_still_filtered():
    # 2026-11-27 实车：车后箱子回波 0.117~0.149 m，小于 A1M8 的 range_min
    # 0.150。旧逻辑直接 continue（不置 inf），箱子永远留在 scan_motion_guard，
    # collision_monitor 的 stop_zone 在前进时也一直停车。
    # 雷达在 base_link 后方 0.256 m：后方 0.12 m 的回波对应 base x=-0.376，
    # 在前进走廊（尾端 -0.32）之外，必须被过滤。
    out, kept, checked = filter_ranges_union(
        [0.12, 0.12], angle_min=0.0, angle_increment=math.pi,
        motions=[(arc_samples(0.2, 0.0), 1)],
        half_length=HALF_L, half_width=HALF_W,
        pad_long=PAD_L, pad_width=PAD_W,
        laser_x=-0.256, laser_y=0.0,
        range_min=0.15, range_max=9.0)
    assert (kept, checked) == (1, 2)
    assert math.isfinite(out[0])    # 侧/前：在足迹内，保留
    assert math.isinf(out[1])       # 后方箱子：必须丢弃
