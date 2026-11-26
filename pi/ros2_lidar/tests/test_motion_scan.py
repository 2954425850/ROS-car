import math
import os

import pytest

from motion_scan import (arc_samples, corridor_relevant, filter_ranges,
                         motion_active, point_relevant)

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HALF_L, HALF_W, PAD = 0.30, 0.20, 0.02


def test_motion_active_dead_band():
    assert not motion_active(0.0, 0.0)
    assert not motion_active(0.01, 0.01)
    assert motion_active(0.1, 0.0)
    assert motion_active(0.0, 0.2)


def test_arc_samples_straight_and_reverse():
    straight = arc_samples(0.2, 0.0, horizon=1.0, step=0.1)
    assert math.isclose(straight[-1][0], 0.2, abs_tol=1e-9)
    assert all(abs(y) < 1e-9 for _, y, _, _ in straight)

    reverse = arc_samples(-0.2, 0.0, horizon=1.0, step=0.1)
    assert math.isclose(reverse[-1][0], -0.2, abs_tol=1e-9)


def test_forward_keeps_front_only():
    samples = arc_samples(0.2, 0.0)
    assert point_relevant(0.45, 0.0, samples, HALF_L, HALF_W, PAD, v_sign=1)
    assert not point_relevant(-0.36, 0.0, samples, HALF_L, HALF_W, PAD,
                              v_sign=1)
    assert not point_relevant(0.0, 0.25, samples, HALF_L, HALF_W, PAD,
                              v_sign=1)


def test_reverse_keeps_rear_only():
    samples = arc_samples(-0.2, 0.0)
    assert point_relevant(-0.45, 0.0, samples, HALF_L, HALF_W, PAD,
                          v_sign=-1)
    assert not point_relevant(0.45, 0.0, samples, HALF_L, HALF_W, PAD,
                              v_sign=-1)


def test_heading_lane_ignores_speed_and_side_wall():
    # Speed-independent lane: a far point straight ahead stays relevant...
    assert corridor_relevant(3.0, 0.10, 1, HALF_L, HALF_W, PAD)
    # ...but a wall 3 cm off the side does not, even alongside the body.
    assert not corridor_relevant(0.0, 0.23, 1, HALF_L, HALF_W, PAD)
    assert not corridor_relevant(-0.5, 0.10, 1, HALF_L, HALF_W, PAD)
    # Pure rotation has no lane.
    assert not corridor_relevant(3.0, 0.0, 0, HALF_L, HALF_W, PAD)


def test_pivot_sweeps_side_but_not_far_rear():
    samples = arc_samples(0.0, 0.35)
    assert point_relevant(0.0, 0.23, samples, HALF_L, HALF_W, PAD)
    assert not point_relevant(0.0, -0.60, samples, HALF_L, HALF_W, PAD)


def test_filter_ranges_lane_works_at_crawl_speed():
    # v=0.05 m/s only sweeps 5 cm in 1 s, but the heading lane must still
    # keep an obstacle 3 m straight ahead and drop one behind.
    out, kept, checked = filter_ranges(
        [3.0, 0.5], angle_min=0.0, angle_increment=math.pi,
        samples=arc_samples(0.05, 0.0),
        half_length=HALF_L, half_width=HALF_W, pad=PAD, v_sign=1)
    assert (kept, checked) == (1, 2)
    assert out[0] == 3.0
    assert math.isinf(out[1])


def test_filter_ranges_masks_unrelated_rays():
    ranges = [0.5, 0.25, 0.5]
    out, kept, checked = filter_ranges(
        ranges, angle_min=0.0, angle_increment=math.pi / 2,
        samples=arc_samples(0.2, 0.0),
        half_length=HALF_L, half_width=HALF_W, pad=PAD)
    assert (kept, checked) == (1, 3)
    assert out[0] == 0.5
    assert math.isinf(out[1]) and math.isinf(out[2])


def test_filter_ranges_leaves_invalid_samples_alone():
    ranges = [float('inf'), float('nan'), 0.01, 0.5]
    out, kept, checked = filter_ranges(
        ranges, angle_min=0.0, angle_increment=math.pi / 2,
        samples=arc_samples(0.2, 0.0),
        half_length=HALF_L, half_width=HALF_W, pad=PAD,
        range_min=0.05, range_max=9.0)
    assert math.isinf(out[0]) and math.isnan(out[1])
    assert out[2] == 0.01          # below range_min -> untouched
    assert math.isinf(out[3])      # -y ray is outside the forward sweep
    assert checked == 1 and kept == 0


def test_both_nav_configs_point_collision_monitor_at_the_gate():
    yaml = pytest.importorskip("yaml")
    for name in ("nav2_params.yaml", "nav2_params_fusion.yaml"):
        with open(os.path.join(HERE, "config", name), encoding="utf-8") as f:
            d = yaml.safe_load(f)
        src = d["collision_monitor"]["ros__parameters"]["scan"]
        assert src["topic"] == "scan_motion_guard", name
