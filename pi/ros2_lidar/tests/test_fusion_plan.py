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


def test_choose_intermediate_snaps_start_out_of_unknown_footprint():
    g = make_grid(w=80, h=40, res=0.1)
    set_rect(g, 20, 0, 80, 40, -1)      # 右侧 unknown
    set_rect(g, 9, 9, 12, 12, -1)       # 自车 footprint 是 unknown
    goal = choose_intermediate(g, (1.0, 1.0), (5.0, 1.0))
    assert goal is not None
    assert 1.0 < goal[0] < 2.0


def test_choose_intermediate_respects_inflation_clearance():
    g = make_grid(w=100, h=40, res=0.1)
    set_rect(g, 40, 0, 100, 40, -1)
    # 障碍物紧贴自由区边缘（x=4.05, y=1.55）；旧 clearance=0.35 会选到
    # 贴边点，在 master 上是 253（内切区）。
    g.data[15 * 100 + 40] = 100
    goal = choose_intermediate(g, (1.0, 1.0), (8.0, 1.0))
    assert goal is not None
    assert clearance_ok(g, goal[0], goal[1], 0.6)
    assert unknown_distance(g, goal[0], goal[1], 0.8) >= 0.45


def test_choose_intermediate_target_next_to_wall_in_unknown():
    # 用户点在 unknown 里、紧挨着已知墙外侧：goal 格本身贴墙不能算不可达，
    # 路线要能穿过 unknown 指过去，中间点仍落在带余量的 free 格里。
    g = make_grid(w=100, h=40, res=0.1)
    set_rect(g, 40, 0, 100, 40, -1)
    g.data[10 * 100 + 40] = 100        # unknown 中、紧挨目标的墙格
    goal = choose_intermediate(g, (1.0, 1.0), (4.15, 1.05))
    assert goal is not None
    assert goal_status(g, goal[0], goal[1]) == "free"
    assert clearance_ok(g, goal[0], goal[1], 0.6)


def test_choose_intermediate_honours_blacklist():
    g = _unknown_half_grid()
    # 把机器人前方 3 m 内的 free 区整片拉黑
    blacklist = [(x * 0.1, y * 0.1, 0.6)
                 for x in range(0, 25) for y in range(0, 40)]
    assert choose_intermediate(g, (1.0, 1.0), (5.0, 1.0),
                                blacklist=blacklist) is None


from fusion_plan import (Frontier, find_frontiers, choose_frontier_goal,
                         unknown_distance)


def _deep_free_grid():
    # free x<4.0，右侧 unknown；比 min_leg 深，能同时满足推进和余量
    g = make_grid(w=100, h=40, res=0.1)
    set_rect(g, 40, 0, 100, 40, -1)
    return g


def test_choose_intermediate_keeps_unknown_margin():
    # 旧实现会选到贴 unknown 边缘的 free 格；Smac 的 footprint 压到
    # unknown（track_unknown_space）就 no valid path。中间点要留出余量。
    g = _deep_free_grid()
    goal = choose_intermediate(g, (1.0, 1.0), (8.0, 1.0))
    assert goal is not None
    assert unknown_distance(g, goal[0], goal[1], 0.8) >= 0.45
    assert goal[0] > 2.0                # 同时确实往前推进了


def test_choose_intermediate_straight_when_corridor_clear():
    # 前方直通时不能无缘无故拐到侧面
    g = _deep_free_grid()
    goal = choose_intermediate(g, (1.0, 1.0), (8.0, 2.0))
    assert goal is not None
    want = math.atan2(2.0 - 1.0, 8.0 - 1.0)
    got = math.atan2(goal[1] - 1.0, goal[0] - 1.0)
    assert abs(got - want) < math.radians(15.0)


def test_choose_intermediate_routes_around_wall_outside_old_cone():
    # 墙从 y=0 一直挡到 y=3.5：旧实现的 60° 锥内没有一条能绕过墙的
    # 直线，只能报 no_reachable_frontier；新实现应该沿 A* 路线先去墙顶。
    g = make_grid(w=80, h=60, res=0.1)
    set_rect(g, 40, 0, 80, 60, -1)      # x >= 4.0 未知，目标在未知里
    set_rect(g, 25, 0, 27, 36, 100)     # x 2.5~2.7，y 0~3.5 的墙
    goal = choose_intermediate(g, (1.0, 1.0), (5.0, 1.0))
    assert goal is not None
    assert goal[1] > 3.5                # 必须绕到墙顶以上
    assert clearance_ok(g, goal[0], goal[1], 0.6)


def test_choose_intermediate_falls_back_in_narrow_corridor():
    # 0.8 m 宽走廊两侧都是 unknown：0.5 m 余量不可能，但不能因此不出腿
    g = make_grid(w=80, h=60, res=0.1)
    set_rect(g, 0, 0, 80, 60, -1)
    set_rect(g, 0, 25, 60, 33, 0)       # y 2.5~3.3 的走廊，x 0~6.0 已建图
    goal = choose_intermediate(g, (1.0, 2.9), (7.0, 2.9))
    assert goal is not None


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


def test_choose_frontier_goal_keeps_unknown_clearance():
    g = _room_with_frontier()
    fs = find_frontiers(g, (2.0, 2.0))
    goal = choose_frontier_goal(g, (2.0, 2.0), fs)
    assert goal is not None
    # 旧实现选前沿边缘，紧贴 unknown，master 上是 253；新实现要退到
    # unknown 间隙 >= 0.6 m 的 free 格。
    assert unknown_distance(g, goal[0], goal[1], 0.8) >= 0.5


def test_choose_frontier_goal_falls_back_in_tight_map():
    g = make_grid(w=40, h=40, res=0.1)
    set_rect(g, 0, 0, 40, 40, -1)
    set_rect(g, 10, 14, 30, 26, 0)      # 1.2 m 宽走廊，unknown 在两侧
    fs = find_frontiers(g, (2.0, 2.0))
    assert fs
    goal = choose_frontier_goal(g, (2.0, 2.0), fs)
    assert goal is not None


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


def test_navigator_route_growth_is_not_no_progress():
    # 刚看到墙/门时路线会变长，这是地图信息增加，不能判成卡住
    g = _navigator_grid()
    n = Navigator()
    n.final = (5.0, 1.0)
    n.remaining = 0.5
    n.stalls = 2
    n.replan(g, (1.0, 1.0))
    assert n.state == "following"
    assert n.stalls == 0


def test_navigator_flat_remaining_still_trips_no_progress():
    g = _navigator_grid()
    n = Navigator()
    n.final = (5.0, 1.0)
    n.remaining = 10.0
    n.replan(g, (1.0, 1.0))          # 得到本次的剩余里程
    assert n.state == "following"
    rem = n.remaining
    n.leg = None                     # 模拟腿结束但原地绕圈
    n.remaining = rem + 0.1
    n.stalls = 2
    n.replan(g, (1.0, 1.0))
    assert n.state == "blocked"
    assert n.reason == "no_progress"


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
