"""融合模式纯算法：分步追目标、前沿探索、边界判定。

不 import ROS，不碰文件系统；坐标系为 map 系，单位米。
OccupancyGrid 约定：-1 未知，0..49 可通行，>=50 障碍
（与 tools/save_map_snapshot.py 的阈值一致）。
"""
import heapq
import math
from dataclasses import dataclass

UNKNOWN = -1
FREE_MAX = 49
OCC_MIN = 50


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: list

    def value(self, ix, iy):
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return self.data[iy * self.width + ix]
        return None

    def world_to_cell(self, x, y):
        return (math.floor((x - self.origin_x) / self.resolution),
                math.floor((y - self.origin_y) / self.resolution))

    def cell_to_world(self, ix, iy):
        return (self.origin_x + (ix + 0.5) * self.resolution,
                self.origin_y + (iy + 0.5) * self.resolution)

    def classify(self, x, y):
        ix, iy = self.world_to_cell(x, y)
        v = self.value(ix, iy)
        if v is None:
            return "outside"
        if v < 0:
            return "unknown"
        if v >= OCC_MIN:
            return "occupied"
        return "free"

    def is_free(self, ix, iy):
        v = self.value(ix, iy)
        return v is not None and 0 <= v < OCC_MIN

    def free_area(self):
        return sum(1 for v in self.data if 0 <= v < OCC_MIN) * \
            self.resolution ** 2


def goal_status(grid, x, y):
    return grid.classify(x, y)


def line_free(grid, a, b, step=None):
    step = step or grid.resolution * 0.5
    dx, dy = b[0] - a[0], b[1] - a[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return grid.classify(a[0], a[1]) == "free"
    n = int(math.ceil(dist / step))
    for i in range(n + 1):
        t = i / n
        if grid.classify(a[0] + dx * t, a[1] + dy * t) != "free":
            return False
    return True


def clearance_ok(grid, x, y, radius=0.35):
    if radius <= 0:
        return True
    steps = max(2, int(math.ceil(radius / (grid.resolution * 0.5))))
    # 24 条射线：8 条在 0.6 m 处相邻射线间隔 0.47 m，墙端/噪点会从
    # 两条射线之间漏过去，选出的点看着有 0.6 m 余量、实际贴墙。
    for k in range(24):
        a = k * math.pi / 12.0
        c, s = math.cos(a), math.sin(a)
        for i in range(1, steps + 1):
            r = radius * i / steps
            if grid.classify(x + c * r, y + s * r) == "occupied":
                return False
    return True


def obstacle_distance(grid, x, y, max_radius=0.8):
    step = grid.resolution * 0.5
    best = max_radius
    for k in range(16):
        a = 2.0 * math.pi * k / 16.0
        c, s = math.cos(a), math.sin(a)
        r = step
        while r <= max_radius:
            if grid.classify(x + c * r, y + s * r) == "occupied":
                best = min(best, r)
                break
            r += step
    return best


def nearest_free_point(grid, point, max_radius=0.5):
    """把点吸附到最近的 free 格；自车所在格常被 scan_self_filter 留成 unknown。"""
    cx, cy = grid.world_to_cell(point[0], point[1])
    if grid.is_free(cx, cy):
        return (float(point[0]), float(point[1]))
    r = int(math.ceil(max_radius / grid.resolution))
    best = None
    best_d = None
    for iy in range(cy - r, cy + r + 1):
        for ix in range(cx - r, cx + r + 1):
            if not grid.is_free(ix, iy):
                continue
            wx, wy = grid.cell_to_world(ix, iy)
            d = math.hypot(wx - point[0], wy - point[1])
            if d > max_radius:
                continue
            if best_d is None or d < best_d:
                best_d = d
                best = (wx, wy)
    return best


def _orient(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) -
            (b[1] - a[1]) * (c[0] - a[0]))


def _on_seg(a, b, c):
    return (min(a[0], b[0]) - 1e-9 <= c[0] <= max(a[0], b[0]) + 1e-9 and
            min(a[1], b[1]) - 1e-9 <= c[1] <= max(a[1], b[1]) + 1e-9)


def segment_intersects(p1, p2, q1, q2):
    o1 = _orient(p1, p2, q1)
    o2 = _orient(p1, p2, q2)
    o3 = _orient(q1, q2, p1)
    o4 = _orient(q1, q2, p2)
    eps = 1e-9
    if (((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and
            ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps))):
        return True
    if abs(o1) <= eps and _on_seg(p1, p2, q1):
        return True
    if abs(o2) <= eps and _on_seg(p1, p2, q2):
        return True
    if abs(o3) <= eps and _on_seg(q1, q2, p1):
        return True
    if abs(o4) <= eps and _on_seg(q1, q2, p2):
        return True
    return False


@dataclass
class Boundary:
    anchor: tuple
    segments: list

    @classmethod
    def from_json(cls, obj):
        if not isinstance(obj, dict):
            return None
        try:
            ax, ay = (float(v) for v in obj["anchor"])
            segs = [tuple(float(v) for v in s)
                    for s in obj.get("segments", [])]
        except Exception:
            return None
        segs = [s for s in segs if len(s) == 4]
        if not segs:
            return None
        return cls((ax, ay), segs)

    def crosses(self, point):
        if not self.segments:
            return False
        p = (float(point[0]), float(point[1]))
        if math.hypot(p[0] - self.anchor[0],
                      p[1] - self.anchor[1]) < 1e-6:
            return False
        for x1, y1, x2, y2 in self.segments:
            if segment_intersects(self.anchor, p, (x1, y1), (x2, y2)):
                return True
        return False


@dataclass
class Budget:
    start: tuple
    final: tuple
    max_legs: int = 20
    max_distance: float = 0.0
    legs: int = 0
    travelled: float = 0.0

    def __post_init__(self):
        if self.max_distance <= 0:
            d = math.hypot(self.final[0] - self.start[0],
                           self.final[1] - self.start[1])
            self.max_distance = max(3.0 * d, 10.0)

    def add_leg(self, a, b):
        self.legs += 1
        self.travelled += math.hypot(b[0] - a[0], b[1] - a[1])

    def exceeded(self):
        return self.legs >= self.max_legs or self.travelled >= self.max_distance


@dataclass
class Decision:
    kind: str
    goal: tuple = None
    reason: str = ""
    # 沿路线从本腿终点到最终目标的剩余里程。staged 绕墙时欧氏距离会
    # 先变大，用它判断“有没有在往前推”才不会误判 no_progress。
    remaining: float = 0.0


_EIGHT = ((1, 0), (-1, 0), (0, 1), (0, -1),
          (1, 1), (1, -1), (-1, 1), (-1, -1))


def _blacklisted(point, blacklist):
    return any(math.hypot(point[0] - bx, point[1] - by) <= br
               for bx, by, br in blacklist)


def _chamfer_distance(grid, is_source):
    """每格到最近 source 的距离（单位：格），8 邻域两步 chamfer。"""
    w, h = grid.width, grid.height
    big = float(w + h + 1)
    dist = [big] * (w * h)
    data = grid.data
    for i in range(w * h):
        if is_source(data[i]):
            dist[i] = 0.0
    diag = math.sqrt(2.0)
    for y in range(h):
        base = y * w
        for x in range(w):
            i = base + x
            if dist[i] == 0.0:
                continue
            best = dist[i]
            if x > 0 and dist[i - 1] + 1.0 < best:
                best = dist[i - 1] + 1.0
            if y > 0:
                if dist[i - w] + 1.0 < best:
                    best = dist[i - w] + 1.0
                if x > 0 and dist[i - w - 1] + diag < best:
                    best = dist[i - w - 1] + diag
                if x + 1 < w and dist[i - w + 1] + diag < best:
                    best = dist[i - w + 1] + diag
            dist[i] = best
    for y in range(h - 1, -1, -1):
        base = y * w
        for x in range(w - 1, -1, -1):
            i = base + x
            if dist[i] == 0.0:
                continue
            best = dist[i]
            if x + 1 < w and dist[i + 1] + 1.0 < best:
                best = dist[i + 1] + 1.0
            if y + 1 < h:
                if dist[i + w] + 1.0 < best:
                    best = dist[i + w] + 1.0
                if x + 1 < w and dist[i + w + 1] + diag < best:
                    best = dist[i + w + 1] + diag
                if x > 0 and dist[i + w - 1] + diag < best:
                    best = dist[i + w - 1] + diag
            dist[i] = best
    return dist


def plan_route(grid, start, goal, blacklist=(), *, hard_clearance=0.22,
               clear_soft=1.0, clear_weight=3.0, unknown_penalty=4.0,
               escape_radius=0.6):
    """在 free+unknown 上做 A*，返回 start→goal 的格中心折线。

    unknown 可通行但代价高：路线可以指向未知区，同时在有已知走廊时不会
    为了抄近路压墙。只用来决定“往哪走”，中间点仍必须是带余量的 free 格。
    goal 落在障碍/界外/黑名单里时返回 None。
    """
    w, h = grid.width, grid.height
    res = grid.resolution
    sx, sy = grid.world_to_cell(start[0], start[1])
    gx, gy = grid.world_to_cell(goal[0], goal[1])
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return None
    data = grid.data
    start_i = sy * w + sx
    goal_i = gy * w + gx
    if data[start_i] >= OCC_MIN or data[goal_i] >= OCC_MIN:
        return None
    if _blacklisted(goal, blacklist):
        return None

    obs = _chamfer_distance(grid, lambda v: v >= OCC_MIN)
    escape_r = max(1, int(math.ceil(escape_radius / res)))
    cost = [0.0] * (w * h)
    passable = bytearray(w * h)
    for iy in range(h):
        base = iy * w
        for ix in range(w):
            i = base + ix
            v = data[i]
            if v >= OCC_MIN:
                continue
            d_m = obs[i] * res
            # 自车贴墙时（self filter 也可能把贴墙格留成 unknown）留出
            # 逃逸空间，否则第一步就无路可走。
            near_start = (abs(ix - sx) <= escape_r and
                          abs(iy - sy) <= escape_r)
            # 未知格不做硬性贴墙限制：目标是用户点在 unknown 里的，
            # 常紧贴墙外侧；路线可以穿过 unknown 探过去。
            if v >= 0 and d_m < hard_clearance and not near_start:
                continue
            pen = 0.0
            if d_m < clear_soft:
                ratio = (clear_soft - d_m) / clear_soft
                pen = clear_weight * ratio * ratio
            cost[i] = 1.0 + pen + (unknown_penalty if v < 0 else 0.0)
            passable[i] = 1

    # 黑名单格当作不可通行；只豁免起点自身，避免腿失败后原地死锁。
    for bx, by, br in blacklist:
        ix0, iy0 = grid.world_to_cell(bx - br, by - br)
        ix1, iy1 = grid.world_to_cell(bx + br, by + br)
        for iy in range(max(0, iy0), min(h - 1, iy1) + 1):
            base = iy * w
            for ix in range(max(0, ix0), min(w - 1, ix1) + 1):
                i = base + ix
                if i == start_i:
                    continue
                wx, wy = grid.cell_to_world(ix, iy)
                if math.hypot(wx - bx, wy - by) <= br:
                    passable[i] = 0

    gx_m, gy_m = goal[0], goal[1]
    diag = math.sqrt(2.0)
    best = [float("inf")] * (w * h)
    best[start_i] = 0.0
    parent = [-1] * (w * h)
    openq = [(math.hypot(start[0] - gx_m, start[1] - gy_m), 0.0, start_i)]
    visited = bytearray(w * h)
    found = False
    while openq:
        _, gc, i = heapq.heappop(openq)
        if visited[i]:
            continue
        visited[i] = 1
        if i == goal_i:
            found = True
            break
        ix, iy = i % w, i // w
        for dx, dy in _EIGHT:
            nx, ny = ix + dx, iy + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            j = ny * w + nx
            if not passable[j] or visited[j]:
                continue
            if dx and dy:
                if not passable[i + dx] or not passable[i + dy * w]:
                    continue
            step = (diag if dx and dy else 1.0) * res
            ng = gc + step * cost[j]
            if ng < best[j] - 1e-12:
                best[j] = ng
                parent[j] = i
                hx, hy = grid.cell_to_world(nx, ny)
                heapq.heappush(openq, (ng + math.hypot(hx - gx_m, hy - gy_m),
                                       ng, j))
    if not found:
        return None
    route = []
    i = goal_i
    while i != -1:
        ix, iy = i % w, i // w
        route.append(grid.cell_to_world(ix, iy))
        if i == start_i:
            break
        i = parent[i]
    route.reverse()
    return route


def _pick_waypoint(grid, pose, route, boundary, blacklist, *, min_leg,
                   max_leg, clearance, unknown_clearance):
    """沿路线取 leg 终点：free、离墙/未知区有余量、不越 unknown。

    返回 (waypoint, 剩余里程)；剩余里程沿 A* 路线从 waypoint 到目标。
    """
    pose_to_start = math.hypot(route[0][0] - pose[0],
                               route[0][1] - pose[1])
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(route, route[1:]))
    acc = 0.0
    prefix = []
    prev = route[0]
    for point in route[1:]:
        acc += math.hypot(point[0] - prev[0], point[1] - prev[1])
        prev = point
        if acc + pose_to_start > max_leg + 1e-9:
            break
        if grid.classify(point[0], point[1]) != "free":
            break            # 不能先穿过 unknown 去取后面的点
        if acc + pose_to_start < min_leg - 1e-9:
            continue
        prefix.append((acc, point))
    tiers = [unknown_clearance]
    if 0.35 < unknown_clearance:
        tiers.append(0.35)
    if unknown_clearance > 0.0:
        tiers.append(0.0)
    for margin in tiers:
        best = None
        for point_acc, point in prefix:
            if boundary is not None and boundary.crosses(point):
                continue
            if _blacklisted(point, blacklist):
                continue
            if not clearance_ok(grid, point[0], point[1], clearance):
                continue
            if margin > 0 and unknown_distance(
                    grid, point[0], point[1], 0.8) < margin:
                continue
            best = ((float(point[0]), float(point[1])), total - point_acc)
        if best is not None:
            return best
    return None, None


def _choose_intermediate(grid, pose, final, boundary=None, blacklist=(),
                         *, min_leg=0.9, max_leg=3.0, clearance=0.6,
                         unknown_clearance=0.5, unknown_penalty=4.0):
    """给 unknown 目标选中间腿：先规划一条 free+unknown 的 A* 路线，
    再取路线前段（≤ max_leg）最后一个满足墙体/未知区余量的 free 格。

    旧实现只在 60° 锥内找欧氏距离最近的点：会贴墙、会为抄近路拐大弯，
    且贴在 unknown 边界上的点会让 Smac 因 footprint 压 unknown 而失败。
    返回 (waypoint, 剩余路线里程) 或 (None, None)。
    """
    px, py = float(pose[0]), float(pose[1])
    gx, gy = float(final[0]), float(final[1])
    if math.hypot(gx - px, gy - py) < 1e-6:
        return None, None
    start = nearest_free_point(grid, (px, py), 0.5)
    if start is None:
        return None, None
    route = plan_route(grid, start, (gx, gy), blacklist,
                       unknown_penalty=unknown_penalty)
    if route is None:
        return None, None
    return _pick_waypoint(grid, (px, py), route, boundary, blacklist,
                          min_leg=min_leg, max_leg=max_leg,
                          clearance=clearance,
                          unknown_clearance=unknown_clearance)


def choose_intermediate(grid, pose, final, boundary=None, blacklist=(),
                        *, min_leg=0.9, max_leg=3.0, clearance=0.6,
                        unknown_clearance=0.5, unknown_penalty=4.0):
    goal, _ = _choose_intermediate(
        grid, pose, final, boundary, blacklist, min_leg=min_leg,
        max_leg=max_leg, clearance=clearance,
        unknown_clearance=unknown_clearance,
        unknown_penalty=unknown_penalty)
    return goal


def decide_target(grid, pose, final, boundary=None, budget=None,
                  blacklist=(), *, min_leg=0.9, max_leg=3.0,
                  clearance=0.6, unknown_clearance=0.5):
    if final is None:
        return Decision("reject", None, "no_target")
    if boundary is not None and boundary.crosses(final):
        return Decision("reject", None, "outside_boundary")
    status = goal_status(grid, final[0], final[1])
    if status == "outside":
        return Decision("reject", None, "outside_map")
    if status == "occupied":
        return Decision("reject", None, "occupied")
    if budget is not None and budget.exceeded():
        return Decision("reject", None, "budget_exceeded")
    if status == "free":
        return Decision("direct", (float(final[0]), float(final[1])), "",
                        math.hypot(final[0] - pose[0],
                                   final[1] - pose[1]))
    goal, remaining = _choose_intermediate(
        grid, pose, final, boundary, blacklist,
        min_leg=min_leg, max_leg=max_leg, clearance=clearance,
        unknown_clearance=unknown_clearance)
    if goal is None:
        return Decision("reject", None, "no_reachable_frontier")
    return Decision("staged", goal, "", remaining)


@dataclass
class Frontier:
    cells: list
    size: int
    centroid: tuple


def find_frontiers(grid, pose, boundary=None, blacklist=(),
                   *, min_cluster_cells=20, max_frontier_dist=8.0):
    w, h = grid.width, grid.height
    flag = bytearray(w * h)
    data = grid.data
    for iy in range(h):
        base = iy * w
        for ix in range(w):
            v = data[base + ix]
            if v < 0 or v >= OCC_MIN:
                continue
            adj_unknown = False
            if ix + 1 < w and data[base + ix + 1] < 0:
                adj_unknown = True
            elif ix > 0 and data[base + ix - 1] < 0:
                adj_unknown = True
            elif iy + 1 < h and data[(iy + 1) * w + ix] < 0:
                adj_unknown = True
            elif iy > 0 and data[(iy - 1) * w + ix] < 0:
                adj_unknown = True
            if adj_unknown:
                flag[base + ix] = 1

    seen = bytearray(w * h)
    out = []
    px, py = pose[0], pose[1]
    for start in range(w * h):
        if not flag[start] or seen[start]:
            continue
        stack = [start]
        seen[start] = 1
        cells = []
        while stack:
            i = stack.pop()
            cells.append(i)
            x, y = i % w, i // w
            for nx, ny in ((x + 1, y), (x - 1, y),
                           (x, y + 1), (x, y - 1)):
                if 0 <= nx < w and 0 <= ny < h:
                    j = ny * w + nx
                    if flag[j] and not seen[j]:
                        seen[j] = 1
                        stack.append(j)
        if len(cells) < min_cluster_cells:
            continue
        sx = sum(i % w for i in cells) / len(cells)
        sy = sum(i // w for i in cells) / len(cells)
        cx, cy = grid.cell_to_world(sx, sy)
        if math.hypot(cx - px, cy - py) > max_frontier_dist:
            continue
        if boundary is not None and boundary.crosses((cx, cy)):
            continue
        if any(math.hypot(cx - bx, cy - by) <= br
               for bx, by, br in blacklist):
            continue
        out.append(Frontier(cells, len(cells), (cx, cy)))
    return out


def unknown_distance(grid, x, y, max_radius=0.8):
    step = grid.resolution * 0.5
    best = max_radius
    for k in range(16):
        a = 2.0 * math.pi * k / 16.0
        c, s = math.cos(a), math.sin(a)
        r = step
        while r <= max_radius:
            if grid.classify(x + c * r, y + s * r) == "unknown":
                best = min(best, r)
                break
            r += step
    return best


def choose_frontier_goal(grid, pose, frontiers, *, clearance=0.6,
                         unknown_clearance=0.6, w_gain=1.0, w_dist=0.6,
                         w_clear=1.5, gain_cap=200):
    px, py = pose[0], pose[1]
    start = nearest_free_point(grid, (px, py), 0.5)
    if start is None:
        return None
    best = None
    best_score = None
    for f in frontiers:
        dist = math.hypot(f.centroid[0] - px, f.centroid[1] - py)
        goal = None
        # 不能直接选前沿边缘：unknown 的膨胀会把边缘点压到 master 253，
        # Smac 直接 no valid path。从前沿质心朝机器人退 0.6~1.2 m，
        # 取第一个 unknown/障碍都 >= 0.6 m 且直线可达的 free 格。
        for back in (0.6, 0.8, 1.0, 1.2):
            if dist < 1e-6:
                break          # 质心与机器人重合，跳过 approach，走兜底
            t = back / dist
            if t >= 1.0:
                break
            gx = f.centroid[0] + (px - f.centroid[0]) * t
            gy = f.centroid[1] + (py - f.centroid[1]) * t
            if goal_status(grid, gx, gy) != "free":
                continue
            if unknown_distance(grid, gx, gy, 0.8) < unknown_clearance:
                continue
            if obstacle_distance(grid, gx, gy, 0.8) < clearance:
                continue
            if not line_free(grid, start, (gx, gy)):
                continue
            goal = (gx, gy)
            break
        if goal is None:
            # 紧凑场地/角落：approach point 找不到时退回簇内自由格。
            # 边缘点可能被 planner 拒一次，Navigator 会黑名单换下一个。
            for min_clear in (clearance, max(0.3, clearance * 0.6)):
                goal_d = None
                for i in f.cells:
                    ix, iy = i % grid.width, i // grid.width
                    wx, wy = grid.cell_to_world(ix, iy)
                    d = math.hypot(wx - px, wy - py)
                    if goal_d is not None and d >= goal_d:
                        continue
                    if not clearance_ok(grid, wx, wy, min_clear):
                        continue
                    if not line_free(grid, start, (wx, wy)):
                        continue
                    goal, goal_d = (wx, wy), d
                if goal is not None:
                    break
        if goal is None:
            continue
        gain = min(f.size, gain_cap)
        obs = obstacle_distance(grid, goal[0], goal[1], 0.8)
        score = w_gain * gain - w_dist * dist + w_clear * min(obs, 0.6)
        if best_score is None or score > best_score:
            best_score = score
            best = goal
    return best


class Navigator:
    """不碰 ROS 的状态机；外部在目标/腿结束/定时调用。"""

    def __init__(self, *, min_leg=0.9, max_leg=3.0, clearance=0.6,
                 unknown_clearance=0.5, max_legs=20, stall_limit=3,
                 blacklist_radius=0.5, frontier_clearance=0.6):
        self.min_leg = min_leg
        self.max_leg = max_leg
        self.clearance = clearance
        self.unknown_clearance = unknown_clearance
        self.max_legs = max_legs
        self.stall_limit = stall_limit
        self.blacklist_radius = blacklist_radius
        self.frontier_clearance = frontier_clearance
        self.reset()

    def reset(self):
        self.final = None
        self.budget = None
        self.leg = None
        self.leg_kind = None
        self.leg_start = None
        self.state = "idle"
        self.reason = ""
        self.explore = False
        self.blacklist = []
        self.stalls = 0
        self.failures = 0
        self.remaining = None
        self.frontier_count = 0
        self.legs_done = 0

    def status(self):
        return {
            "state": self.state,
            "explore": self.explore,
            "final": self.final,
            "leg": self.leg,
            "leg_kind": self.leg_kind,
            "reason": self.reason,
            "remaining": (None if self.remaining is None
                          else round(self.remaining, 2)),
            "legs": self.legs_done,
            "frontiers": self.frontier_count,
        }

    def set_explore(self, on, grid, pose, boundary=None):
        self.explore = bool(on)
        if not on and self.leg_kind == "explore":
            self.leg = None
            self.leg_kind = None
            self.state = "idle"
            self.reason = "explore_off"
        elif on and self.final is None and self.leg is None:
            self._plan_explore(grid, pose, boundary)

    def set_target(self, final, grid, pose, boundary=None):
        self.final = (float(final[0]), float(final[1]))
        self.budget = Budget((pose[0], pose[1]), self.final,
                             max_legs=self.max_legs)
        self.leg = None
        self.leg_kind = None
        self.leg_start = None
        self.stalls = 0
        self.failures = 0
        self.remaining = None
        self.blacklist.clear()
        self.legs_done = 0
        self._plan(grid, pose, boundary)

    def cancel(self):
        self.final = None
        self.budget = None
        self.leg = None
        self.leg_kind = None
        self.leg_start = None
        self.state = "idle"
        self.reason = "canceled"

    def replan(self, grid, pose, boundary=None):
        if self.leg is not None:
            return
        if self.state in ("blocked", "done"):
            return
        if self.final is not None:
            self._plan(grid, pose, boundary)
        elif self.explore:
            self._plan_explore(grid, pose, boundary)

    def _set_leg(self, goal, kind):
        self.leg = (float(goal[0]), float(goal[1]))
        self.leg_kind = kind
        self.leg_start = None
        self.state = "exploring" if kind == "explore" else "following"
        self.reason = ""

    def _plan(self, grid, pose, boundary):
        dec = decide_target(
            grid, pose, self.final, boundary, self.budget,
            blacklist=self.blacklist,
            min_leg=self.min_leg, max_leg=self.max_leg,
            clearance=self.clearance,
            unknown_clearance=self.unknown_clearance)
        if dec.kind == "reject":
            self.state = "blocked"
            self.reason = dec.reason
            self.leg = None
            self.leg_kind = None
            return
        # 用“沿路线的剩余里程”判进度：绕墙时欧氏距离会先变大，
        # 按欧氏判会把正常绕行误判成 no_progress。
        # 剩余里程明显变长（>0.5 m）说明新地图把原路线作废了（刚看到
        # 墙/门），这是探索信息增加，不是卡住，计数清零。
        if dec.kind == "staged" and self.remaining is not None:
            delta = self.remaining - dec.remaining
            if delta < -0.5:
                self.stalls = 0
            elif delta < 0.2:
                self.stalls += 1
            else:
                self.stalls = 0
            if self.stalls >= self.stall_limit:
                self.state = "blocked"
                self.reason = "no_progress"
                self.leg = None
                self.leg_kind = None
                return
        self.remaining = dec.remaining
        self._set_leg(dec.goal, dec.kind)

    def _plan_explore(self, grid, pose, boundary):
        if not self.explore:
            self.state = "idle"
            return
        fs = find_frontiers(grid, pose, boundary, self.blacklist)
        self.frontier_count = len(fs)
        goal = choose_frontier_goal(
            grid, pose, fs, clearance=self.frontier_clearance)
        if goal is None:
            self.leg = None
            self.leg_kind = None
            self.state = "idle"
            self.reason = "no_frontier"
            return
        self._set_leg(goal, "explore")

    def on_leg_started(self, pose):
        self.leg_start = (float(pose[0]), float(pose[1]))
        self.legs_done += 1

    def on_leg_succeeded(self, grid, pose, boundary=None):
        if self.leg is None:
            return
        start = self.leg_start or (pose[0], pose[1])
        if self.budget is not None:
            self.budget.add_leg(start, (pose[0], pose[1]))
        kind = self.leg_kind
        self.leg = None
        self.leg_kind = None
        self.leg_start = None
        self.failures = 0
        if kind == "direct":
            # nav2 的 action 成功就是到达；不要再按自己的 0.25 m 阈值重算，
            # 否则车会在目标附近反复生成同一个 direct 腿、来回磨。
            self.final = None
            self.budget = None
            self.state = "idle"
            self.reason = "arrived"
            if self.explore:
                self._plan_explore(grid, pose, boundary)
            return
        if kind == "explore":
            self._plan_explore(grid, pose, boundary)
            return
        # staged：中间点到了，继续朝最终目标推进
        if self.final is not None:
            d = math.hypot(self.final[0] - pose[0],
                           self.final[1] - pose[1])
            if d <= 0.25:
                self.final = None
                self.budget = None
                self.state = "idle"
                self.reason = "arrived"
                if self.explore:
                    self._plan_explore(grid, pose, boundary)
                return
        self._plan(grid, pose, boundary)

    def on_leg_failed(self, grid, pose, boundary=None):
        if self.leg is None:
            return
        self.blacklist.append(
            (self.leg[0], self.leg[1], self.blacklist_radius))
        kind = self.leg_kind
        self.leg = None
        self.leg_kind = None
        self.leg_start = None
        self.failures += 1
        if kind == "explore":
            if self.failures < self.stall_limit:
                self._plan_explore(grid, pose, boundary)
                return
        elif self.final is not None and self.failures < 2:
            goal, remaining = _choose_intermediate(
                grid, pose, self.final, boundary, self.blacklist,
                min_leg=self.min_leg, max_leg=self.max_leg,
                clearance=self.clearance,
                unknown_clearance=self.unknown_clearance)
            if goal is not None:
                self.remaining = remaining
                self._set_leg(goal, "staged")
                self.reason = "retry_staged"
                return
        self.state = "blocked"
        self.reason = "leg_failed"
