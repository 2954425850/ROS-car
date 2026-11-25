import math

from fusion_plan import (Boundary, Grid, line_free, clearance_ok,
                         goal_status, segment_intersects)


def make_grid(w=40, h=40, res=0.1, ox=0.0, oy=0.0, fill=0):
    return Grid(w, h, res, ox, oy, [fill] * (w * h))


def set_rect(g, x0, y0, x1, y1, value):
    for iy in range(y0, y1):
        for ix in range(x0, x1):
            g.data[iy * g.width + ix] = value


def test_classify_free_unknown_occupied_outside():
    g = make_grid(w=10, h=10, res=0.1)
    set_rect(g, 2, 2, 4, 4, -1)
    set_rect(g, 6, 6, 8, 8, 100)
    assert goal_status(g, 0.05, 0.05) == "free"
    assert goal_status(g, 0.25, 0.25) == "unknown"
    assert goal_status(g, 0.65, 0.65) == "occupied"
    assert goal_status(g, -1.0, 0.0) == "outside"


def test_world_cell_roundtrip_and_negative_origin():
    g = make_grid(w=10, h=10, res=0.1, ox=-1.0, oy=-2.0)
    ix, iy = g.world_to_cell(-0.95, -1.95)
    assert (ix, iy) == (0, 0)
    x, y = g.cell_to_world(0, 0)
    assert math.isclose(x, -0.95) and math.isclose(y, -1.95)


def test_line_free_blocks_on_occupied():
    g = make_grid(w=20, h=20, res=0.1)
    assert line_free(g, (0.1, 0.1), (1.5, 0.1))
    set_rect(g, 7, 0, 8, 20, 100)
    assert not line_free(g, (0.1, 0.1), (1.5, 0.1))


def test_line_free_blocks_on_unknown():
    g = make_grid(w=20, h=20, res=0.1)
    set_rect(g, 7, 0, 8, 20, -1)
    assert not line_free(g, (0.1, 0.1), (1.5, 0.1))


def test_clearance_sees_obstacle_in_ring():
    g = make_grid(w=40, h=40, res=0.1)
    assert clearance_ok(g, 2.0, 2.0, 0.5)
    set_rect(g, 24, 20, 25, 21, 100)   # (2.45, 2.05)
    assert not clearance_ok(g, 2.0, 2.0, 0.5)


def test_segment_intersects_crossing_and_touching():
    assert segment_intersects((0, 0), (2, 0), (1, -1), (1, 1))
    assert not segment_intersects((0, 0), (2, 0), (3, -1), (3, 1))
    assert segment_intersects((0, 0), (2, 0), (2, 0), (3, 1))  # 端点接触


def test_boundary_crosses_line():
    b = Boundary(anchor=(0.0, 0.0),
                 segments=[(2.0, -5.0, 2.0, 5.0)])
    assert b.crosses((3.0, 0.0))
    assert not b.crosses((1.0, 0.0))
    assert not b.crosses((0.0, 0.0))


def test_boundary_from_json_rejects_garbage():
    assert Boundary.from_json(None) is None
    assert Boundary.from_json({"anchor": [0, 0], "segments": []}) is None
    assert Boundary.from_json({"anchor": [0, 0],
                               "segments": [[0, 0, 1, 1]]}) is not None


from fusion_plan import (Budget, choose_intermediate, decide_target)


def _unknown_half_grid():
    # 左 half free，右 half unknown；机器人 (1,1)，目标 (5,1)
    g = make_grid(w=80, h=40, res=0.1)
    set_rect(g, 20, 0, 80, 40, -1)
    return g


def test_choose_intermediate_advances_into_free_edge():
    g = _unknown_half_grid()
    goal = choose_intermediate(g, (1.0, 1.0), (5.0, 1.0))
    assert goal is not None
    assert 1.0 < goal[0] < 2.0
    assert abs(goal[1] - 1.0) < 0.2


def test_decide_target_unknown_is_staged():
    g = _unknown_half_grid()
    d = decide_target(g, (1.0, 1.0), (5.0, 1.0))
    assert d.kind == "staged"
    assert d.goal is not None


def test_decide_target_free_is_direct():
    g = make_grid(w=80, h=40, res=0.1)
    d = decide_target(g, (1.0, 1.0), (2.0, 1.0))
    assert d.kind == "direct"
    assert d.goal == (2.0, 1.0)


def test_decide_target_occupied_rejected():
    g = make_grid(w=80, h=40, res=0.1)
    set_rect(g, 30, 10, 31, 11, 100)
    d = decide_target(g, (1.0, 1.0), (3.05, 1.05))
    assert d.kind == "reject"
    assert d.reason == "occupied"


def test_decide_target_outside_boundary_rejected():
    g = make_grid(w=80, h=40, res=0.1)
    b = Boundary((1.0, 1.0), [(2.0, -5.0, 2.0, 5.0)])
    d = decide_target(g, (1.0, 1.0), (3.0, 1.0), boundary=b)
    assert d.kind == "reject"
    assert d.reason == "outside_boundary"


def test_decide_target_no_free_cell_near_unknown_rejected():
    g = make_grid(w=40, h=40, res=0.1)
    set_rect(g, 0, 0, 40, 40, -1)        # 全未知
    set_rect(g, 19, 19, 21, 21, 0)       # 机器人所在 2x2 格 free
    d = decide_target(g, (2.0, 2.0), (3.0, 2.0))
    assert d.kind == "reject"
    assert d.reason == "no_reachable_frontier"


def test_budget_exhausts_by_legs_and_distance():
    b = Budget((0.0, 0.0), (10.0, 0.0), max_legs=2)
    assert not b.exceeded()
    b.add_leg((0, 0), (1, 0))
    b.add_leg((1, 0), (2, 0))
    assert b.exceeded()

    b2 = Budget((0.0, 0.0), (1.0, 0.0), max_distance=5.0)
    b2.add_leg((0, 0), (6, 0))
    assert b2.exceeded()


def test_choose_intermediate_honours_blacklist():
    g = _unknown_half_grid()
    # 把机器人前方 3 m 内的 free 区整片拉黑
    blacklist = [(x * 0.1, y * 0.1, 0.6)
                 for x in range(0, 25) for y in range(0, 40)]
    assert choose_intermediate(g, (1.0, 1.0), (5.0, 1.0),
                                blacklist=blacklist) is None


from fusion_plan import Frontier, find_frontiers, choose_frontier_goal


def _room_with_frontier():
    # 3 m x 3 m 已知房间，右侧是 unknown，其余是墙；机器人 (2,2)
    g = make_grid(w=60, h=60, res=0.1)
    set_rect(g, 0, 0, 60, 60, 100)
    set_rect(g, 10, 10, 40, 40, 0)
    set_rect(g, 40, 10, 60, 40, -1)
    return g


def test_find_frontiers_finds_opening():
    g = _room_with_frontier()
    fs = find_frontiers(g, (2.0, 2.0))
    assert fs
    assert any(abs(f.centroid[0] - 4.0) < 0.3 for f in fs)


def test_find_frontiers_filters_small_cluster():
    g = make_grid(w=60, h=60, res=0.1)
    set_rect(g, 0, 0, 60, 60, -1)
    set_rect(g, 10, 10, 12, 12, 0)      # 4 格 free，簇 < 20
    assert find_frontiers(g, (1.1, 1.1)) == []


def test_find_frontiers_filters_distance():
    g = _room_with_frontier()
    assert find_frontiers(g, (2.0, 2.0), max_frontier_dist=0.5) == []


def test_find_frontiers_filters_boundary():
    g = _room_with_frontier()
    b = Boundary((2.0, 2.0), [(3.5, -10.0, 3.5, 10.0)])
    fs = find_frontiers(g, (2.0, 2.0), boundary=b)
    assert all(not b.crosses(f.centroid) for f in fs)


def test_find_frontiers_filters_blacklist():
    g = _room_with_frontier()
    fs = find_frontiers(g, (2.0, 2.0))
    assert fs
    c = fs[0].centroid
    assert find_frontiers(g, (2.0, 2.0),
                          blacklist=[(c[0], c[1], 1.0)]) == []


def test_choose_frontier_goal_is_free_with_clearance():
    g = _room_with_frontier()
    fs = find_frontiers(g, (2.0, 2.0))
    goal = choose_frontier_goal(g, (2.0, 2.0), fs)
    assert goal is not None
    assert goal_status(g, goal[0], goal[1]) == "free"
    assert clearance_ok(g, goal[0], goal[1], 0.6)


from fusion_plan import Navigator


def _navigator_grid():
    g = make_grid(w=80, h=40, res=0.1)
    set_rect(g, 20, 0, 80, 40, -1)
    return g


def test_navigator_target_unknown_uses_staged_leg():
    g = _navigator_grid()
    n = Navigator()
    n.set_target((5.0, 1.0), g, (1.0, 1.0))
    assert n.state == "following"
    assert n.leg_kind == "staged"
    assert n.leg is not None


def test_navigator_direct_target_and_arrival():
    g = make_grid(w=80, h=40, res=0.1)
    n = Navigator()
    n.set_target((2.0, 1.0), g, (1.0, 1.0))
    assert n.leg_kind == "direct"
    n.on_leg_started((1.0, 1.0))
    n.on_leg_succeeded(g, (2.0, 1.0))
    assert n.state == "idle"
    assert n.reason == "arrived"
    assert n.final is None


def test_navigator_direct_success_is_arrival_even_if_offset():
    g = make_grid(w=80, h=40, res=0.1)
    n = Navigator()
    n.set_target((2.0, 1.0), g, (1.0, 1.0))
    n.on_leg_started((1.0, 1.0))
    n.on_leg_succeeded(g, (2.3, 1.0))   # nav2 的 0.25 m 容差附近
    assert n.state == "idle"
    assert n.reason == "arrived"
    assert n.final is None


def test_navigator_counts_legs_done():
    g = make_grid(w=80, h=40, res=0.1)
    n = Navigator()
    n.set_target((2.0, 1.0), g, (1.0, 1.0))
    n.on_leg_started((1.0, 1.0))
    assert n.status()["legs"] == 1
    n.on_leg_succeeded(g, (2.0, 1.0))
    assert n.status()["legs"] == 1


def test_navigator_staged_progress_then_arrival():
    g = _navigator_grid()
    n = Navigator()
    n.set_target((5.0, 1.0), g, (1.0, 1.0))
    assert n.leg_kind == "staged"
    first = n.leg
    n.on_leg_started((1.0, 1.0))
    # 推进后地图把目标附近变成 free
    set_rect(g, 0, 0, 80, 40, 0)
    n.on_leg_succeeded(g, first)
    assert n.leg is not None
    assert n.leg_kind == "direct"
    n.on_leg_started(first)
    n.on_leg_succeeded(g, (5.0, 1.0))
    assert n.state == "idle"


def test_navigator_failure_retries_then_blocks():
    g = _navigator_grid()
    n = Navigator()
    n.set_target((5.0, 1.0), g, (1.0, 1.0))
    assert n.leg_kind == "staged"
    n.on_leg_started((1.0, 1.0))
    n.on_leg_failed(g, (1.0, 1.0))
    assert n.state == "following"
    assert n.leg_kind == "staged"
    assert n.blacklist
    n.on_leg_started((1.0, 1.0))
    n.on_leg_failed(g, (1.0, 1.0))
    assert n.state == "blocked"
    assert n.reason == "leg_failed"


def test_navigator_explore_and_stop():
    g = _room_with_frontier()
    n = Navigator()
    n.set_explore(True, g, (2.0, 2.0))
    assert n.state == "exploring"
    assert n.leg_kind == "explore"
    n.set_explore(False, g, (2.0, 2.0))
    assert n.state == "idle"
    assert n.leg is None


def test_navigator_manual_target_preempts_explore():
    g = _room_with_frontier()
    n = Navigator()
    n.set_explore(True, g, (2.0, 2.0))
    n.set_target((2.5, 2.5), g, (2.0, 2.0))
    assert n.explore is True
    assert n.final == (2.5, 2.5)
    assert n.leg_kind == "direct"


def test_navigator_cancel_clears_target():
    g = _navigator_grid()
    n = Navigator()
    n.set_target((5.0, 1.0), g, (1.0, 1.0))
    n.cancel()
    assert n.final is None
    assert n.leg is None
    assert n.state == "idle"


def test_navigator_budget_blocks_runaway():
    g = _navigator_grid()
    n = Navigator(max_legs=1)
    n.set_target((5.0, 1.0), g, (1.0, 1.0))
    n.on_leg_started((1.0, 1.0))
    n.on_leg_succeeded(g, (2.0, 1.0))   # 地图未变，距离没推进 -> stall
    # 第二次规划时 budget 已用掉 1 腿
    assert n.state in ("blocked", "following")
    n.on_leg_failed(g, (2.0, 1.0))
    assert n.reason in ("budget_exceeded", "leg_failed")
