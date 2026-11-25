"""融合模式纯算法：分步追目标、前沿探索、边界判定。

不 import ROS，不碰文件系统；坐标系为 map 系，单位米。
OccupancyGrid 约定：-1 未知，0..49 可通行，>=50 障碍
（与 tools/save_map_snapshot.py 的阈值一致）。
"""
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
    for k in range(8):
        a = k * math.pi / 4.0
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


def choose_intermediate(grid, pose, final, boundary=None, blacklist=(),
                        *, min_leg=0.9, max_leg=3.0, clearance=0.35,
                        cone_deg=60.0, step_cells=1):
    # step_cells=1：自由边缘可能落在任意格子，粗采样会错过最靠前的那一格，
    # 导致中间点偏到侧面。3 m 内约 3.7k 个候选，单次规划 <0.2 s，可接受。
    px, py = float(pose[0]), float(pose[1])
    gx, gy = float(final[0]), float(final[1])
    dx, dy = gx - px, gy - py
    gdist = math.hypot(dx, dy)
    if gdist < 1e-6:
        return None
    cos_cone = math.cos(math.radians(cone_deg))
    cx, cy = grid.world_to_cell(px, py)
    n = int(math.ceil(max_leg / (grid.resolution * step_cells)))
    best = None
    best_score = None
    for ky in range(-n, n + 1):
        iy = cy + ky * step_cells
        for kx in range(-n, n + 1):
            ix = cx + kx * step_cells
            if not grid.is_free(ix, iy):
                continue
            wx, wy = grid.cell_to_world(ix, iy)
            vx, vy = wx - px, wy - py
            d = math.hypot(vx, vy)
            if d < min_leg or d > max_leg:
                continue
            if (vx * dx + vy * dy) / (d * gdist) < cos_cone:
                continue
            if boundary is not None and boundary.crosses((wx, wy)):
                continue
            if any(math.hypot(wx - bx, wy - by) <= br
                   for bx, by, br in blacklist):
                continue
            if not clearance_ok(grid, wx, wy, clearance):
                continue
            if not line_free(grid, (px, py), (wx, wy)):
                continue
            score = math.hypot(gx - wx, gy - wy)
            if best_score is None or score < best_score - 1e-9:
                best_score = score
                best = (wx, wy)
    return best


def decide_target(grid, pose, final, boundary=None, budget=None,
                  blacklist=(), *, min_leg=0.9, max_leg=3.0,
                  clearance=0.35):
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
        return Decision("direct", (float(final[0]), float(final[1])), "")
    goal = choose_intermediate(
        grid, pose, final, boundary, blacklist,
        min_leg=min_leg, max_leg=max_leg, clearance=clearance)
    if goal is None:
        return Decision("reject", None, "no_reachable_frontier")
    return Decision("staged", goal, "")


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


def choose_frontier_goal(grid, pose, frontiers, *, clearance=0.6,
                         w_gain=1.0, w_dist=0.6, w_clear=1.5,
                         gain_cap=200):
    px, py = pose[0], pose[1]
    best = None
    best_score = None
    for f in frontiers:
        goal = None
        goal_d = None
        for i in f.cells:
            ix, iy = i % grid.width, i // grid.width
            wx, wy = grid.cell_to_world(ix, iy)
            d = math.hypot(wx - px, wy - py)
            if goal_d is not None and d >= goal_d:
                continue
            if not clearance_ok(grid, wx, wy, clearance):
                continue
            goal, goal_d = (wx, wy), d
        if goal is None:
            continue
        gain = min(f.size, gain_cap)
        dist = math.hypot(f.centroid[0] - px, f.centroid[1] - py)
        obs = obstacle_distance(grid, goal[0], goal[1], 0.8)
        score = w_gain * gain - w_dist * dist + w_clear * min(obs, 0.6)
        if best_score is None or score > best_score:
            best_score = score
            best = goal
    return best


class Navigator:
    """不碰 ROS 的状态机；外部在目标/腿结束/定时调用。"""

    def __init__(self, *, min_leg=0.9, max_leg=3.0, clearance=0.35,
                 max_legs=20, stall_limit=3, blacklist_radius=0.5,
                 frontier_clearance=0.6):
        self.min_leg = min_leg
        self.max_leg = max_leg
        self.clearance = clearance
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
        self.last_dist = None
        self.frontier_count = 0

    def status(self):
        return {
            "state": self.state,
            "explore": self.explore,
            "final": self.final,
            "leg": self.leg,
            "leg_kind": self.leg_kind,
            "reason": self.reason,
            "legs": self.budget.legs if self.budget else 0,
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
        self.blacklist.clear()
        self.last_dist = math.hypot(self.final[0] - pose[0],
                                    self.final[1] - pose[1])
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
            clearance=self.clearance)
        if dec.kind == "reject":
            self.state = "blocked"
            self.reason = dec.reason
            self.leg = None
            self.leg_kind = None
            return
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
        if kind == "explore":
            self._plan_explore(grid, pose, boundary)
            return
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
            if self.last_dist is not None and d > self.last_dist - 0.2:
                self.stalls += 1
            self.last_dist = d
            if self.stalls >= self.stall_limit:
                self.state = "blocked"
                self.reason = "no_progress"
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
            goal = choose_intermediate(
                grid, pose, self.final, boundary, self.blacklist,
                min_leg=self.min_leg, max_leg=self.max_leg,
                clearance=self.clearance)
            if goal is not None:
                self._set_leg(goal, "staged")
                self.reason = "retry_staged"
                return
        self.state = "blocked"
        self.reason = "leg_failed"
