# 融合建图导航（边建边航）+ 自主探索 — 实施计划

> **For agentic workers:** 按任务顺序执行：每个任务先写失败测试，确认它真的会红，再写最小实现，最后提交。
> **设计依据**：`docs/plans/2026-11-25-fusion-exploration-design.md`。
> **执行环境**：Pi 上 ROS 2 Jazzy；纯算法测试可在 Windows 上跑。

**Goal:** 增加第三种模式 `fusion`——slam_toolbox 提供 `map→odom`，nav2 只跑规划/控制；人点和 frontier 探索共用一条"分步追目标"链，支持点未知区逐步推进和走廊边界线。

**Architecture:** 新增纯算法模块 `fusion_plan.py`（可单测）和一个薄 ROS 节点 `fusion_nav.py`；upper 网页只负责起停、转发目标、画小人和边界线。融合启动 = 雷达 + TF + slam_toolbox + nav2 navigation（不含 AMCL/map_server）。

**Tech Stack:** Python 3.12、rclpy、nav2_msgs、tf2_ros、stdlib HTTP/SSE、原生 JS canvas、pytest。

---

## Global Constraints

- 只新增/修改：`pi/ros2_lidar/tools/fusion_plan.py`、`tools/fusion_nav.py`、
  `launch/fusion.launch.py`、`config/slam_params_fusion.yaml`、
  `config/nav2_params_fusion.yaml`、`tests/`、`upper/server.py`、`upper/index.html`、
  `upper/tests/`、`NAV.md`、`upper/README.md`。
- **不改** `config/nav2_params.yaml`、`config/slam_params.yaml`、`slam.service`、`nav.service`、
  `mapping.launch.py`、`nav.launch.py`。
- 不新增 systemd 单元；融合 launch 由 `upper` 以子进程组管理。
- 融合 launch 默认 `use_composition:=False`；`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。
- `allow_unknown` 在融合参数里**保持 false**；未知区由分步器推进。
- 速度链路（`cmd_vel_raw → smoother → no_pivot_guard → cmd_vel_guard → collision_monitor →
  cmd_vel_nav → cmd_mux`）不动。
- 页面零外部依赖（无 CDN）；面向用户的文案用中文。
- 纯算法模块不 import ROS、不读写文件。
- 每个任务结束提交一次；提交前测试必须全绿。

---

## 文件结构

| 文件 | 职责 |
|---|---|
| `pi/ros2_lidar/tools/fusion_plan.py` | 纯算法：Grid、Boundary、分步追目标、前沿探索、Navigator 状态机 |
| `pi/ros2_lidar/tools/fusion_nav.py` | ROS 适配层：/map、TF、action、服务、状态发布 |
| `pi/ros2_lidar/launch/fusion.launch.py` | 融合启动组合 |
| `pi/ros2_lidar/config/slam_params_fusion.yaml` | 融合建图参数（1 s 地图更新） |
| `pi/ros2_lidar/config/nav2_params_fusion.yaml` | 融合 nav2 参数（全局无动态障碍层、4×4 局部图） |
| `pi/ros2_lidar/tests/conftest.py` | 把 `tools/` 加入 `sys.path` |
| `pi/ros2_lidar/tests/test_fusion_plan.py` | 纯算法测试 |
| `pi/ros2_lidar/tests/test_fusion_config.py` | 参数集静态检查 |
| `pi/ros2_lidar/upper/server.py` | 三态模式探测、fusion 起停、目标转发、探索/边界 API、存图写位姿 |
| `pi/ros2_lidar/upper/index.html` | 第三模式、建图中点图、中间腿、探索开关、画边界线 |
| `pi/ros2_lidar/upper/tests/test_server.py` | 后端测试扩展 |
| `pi/ros2_lidar/upper/tests/test_index.py` | 页面关键控件静态检查 |

---

### Task 1: `fusion_plan.py` — 网格、边界、可达判定

**Files:**
- Create: `pi/ros2_lidar/tools/fusion_plan.py`
- Create: `pi/ros2_lidar/tests/conftest.py`
- Test: `pi/ros2_lidar/tests/test_fusion_plan.py`

**Interfaces:**
- Produces:
  - `Grid(width, height, resolution, origin_x, origin_y, data)`
    - `value(ix, iy) -> int | None`
    - `world_to_cell(x, y) -> (int, int)`
    - `cell_to_world(ix, iy) -> (float, float)`
    - `classify(x, y) -> "free" | "unknown" | "occupied" | "outside"`
    - `is_free(ix, iy) -> bool`
    - `free_area() -> float`
  - `goal_status(grid, x, y) -> str`
  - `line_free(grid, a, b, step=None) -> bool`
  - `clearance_ok(grid, x, y, radius=0.35) -> bool`
  - `obstacle_distance(grid, x, y, max_radius=0.8) -> float`
  - `segment_intersects(p1, p2, q1, q2) -> bool`
  - `Boundary(anchor, segments)`；`Boundary.from_json(obj) -> Boundary | None`；`crosses(point) -> bool`
  - 常量 `UNKNOWN = -1`、`OCC_MIN = 50`

- [ ] **Step 1: 写失败测试**

`pi/ros2_lidar/tests/conftest.py`：

```python
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
```

`pi/ros2_lidar/tests/test_fusion_plan.py`：

```python
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
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cd pi/ros2_lidar && python -m pytest tests/test_fusion_plan.py -v
```

Expected: `ModuleNotFoundError: No module named 'fusion_plan'`。

- [ ] **Step 3: 写最小实现**

`pi/ros2_lidar/tools/fusion_plan.py`：

```python
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
```

- [ ] **Step 4: 运行测试，确认通过**

```bash
cd pi/ros2_lidar && python -m pytest tests/test_fusion_plan.py -v
```

Expected: 全部 PASS。

- [ ] **Step 5: 变异自检**

把 `OCC_MIN = 50` 临时改成 `OCC_MIN = 1000`，重跑测试，确认
`test_classify_free_unknown_occupied_outside` 与
`test_line_free_blocks_on_occupied` 变红；改回 50，重新全绿。
**不会红的测试不算证据。**

- [ ] **Step 6: 提交**

```bash
git add pi/ros2_lidar/tools/fusion_plan.py pi/ros2_lidar/tests/conftest.py \
        pi/ros2_lidar/tests/test_fusion_plan.py
git commit -m "feat(fusion): grid, boundary and reachability primitives"
```

---

### Task 2: `fusion_plan.py` — 分步追目标与预算

**Files:**
- Modify: `pi/ros2_lidar/tools/fusion_plan.py`
- Test: `pi/ros2_lidar/tests/test_fusion_plan.py`

**Interfaces:**
- Consumes: Task 1 的 `Grid`、`Boundary`、`line_free`、`clearance_ok`。
- Produces:
  - `Budget(start, final, max_legs=20, max_distance=0.0)`
    - `add_leg(a, b) -> None`；`exceeded() -> bool`
  - `Decision(kind, goal=None, reason="")`；`kind ∈ {"direct","staged","reject"}`
  - `choose_intermediate(grid, pose, final, boundary=None, blacklist=(), *, min_leg=0.9, max_leg=3.0, clearance=0.35, cone_deg=60.0, step_cells=1) -> (x,y) | None`
  - `decide_target(grid, pose, final, boundary=None, budget=None, blacklist=(), *, min_leg=0.9, max_leg=3.0, clearance=0.35) -> Decision`

- [ ] **Step 1: 写失败测试（追加到 `tests/test_fusion_plan.py`）**

```python
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
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cd pi/ros2_lidar && python -m pytest tests/test_fusion_plan.py -v
```

Expected: `ImportError: cannot import name 'Budget'`。

- [ ] **Step 3: 实现**

追加到 `tools/fusion_plan.py`：

```python
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
```

- [ ] **Step 4: 运行测试**

```bash
cd pi/ros2_lidar && python -m pytest tests/test_fusion_plan.py -v
```

Expected: 全部 PASS。若 `test_decide_target_no_free_cell_near_unknown_rejected`
失败，检查测试网格：机器人 (2,2) 在 cell (20,20)，周围 2×2 格 free，
到任何 free 格都 < `min_leg=0.9`，应返回 None。

- [ ] **Step 5: 变异自检**

把 `choose_intermediate` 里的 `if d < min_leg or d > max_leg:` 临时改成
`if d > max_leg:`，重跑，确认 `test_choose_intermediate_advances_into_free_edge`
或 `no_free_cell...` 变红；改回。

- [ ] **Step 6: 提交**

```bash
git add pi/ros2_lidar/tools/fusion_plan.py pi/ros2_lidar/tests/test_fusion_plan.py
git commit -m "feat(fusion): staged target pursuit and leg budget"
```

---

### Task 3: `fusion_plan.py` — 前沿探索

**Files:**
- Modify: `pi/ros2_lidar/tools/fusion_plan.py`
- Test: `pi/ros2_lidar/tests/test_fusion_plan.py`

**Interfaces:**
- Produces:
  - `Frontier(cells: list[int], size: int, centroid: tuple)`
  - `find_frontiers(grid, pose, boundary=None, blacklist=(), *, min_cluster_cells=20, max_frontier_dist=8.0) -> list[Frontier]`
  - `choose_frontier_goal(grid, pose, frontiers, *, clearance=0.6, w_gain=1.0, w_dist=0.6, w_clear=1.5, gain_cap=200) -> (x,y) | None`

- [ ] **Step 1: 写失败测试（追加）**

```python
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
```

- [ ] **Step 2: 运行，确认失败**

```bash
cd pi/ros2_lidar && python -m pytest tests/test_fusion_plan.py -v
```

Expected: `ImportError: cannot import name 'find_frontiers'`。

- [ ] **Step 3: 实现**

追加到 `tools/fusion_plan.py`：

```python
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
```

- [ ] **Step 4: 运行测试**

```bash
cd pi/ros2_lidar && python -m pytest tests/test_fusion_plan.py -v
```

Expected: 全部 PASS。若 `test_find_frontiers_finds_opening` 找不到右侧
前沿，检查房间 free 区域是否 `set_rect(g, 10, 10, 40, 40, 0)`（cell
x=10..39，对应世界 1.0..4.0 m）。

- [ ] **Step 5: 变异自检**

把 `if len(cells) < min_cluster_cells:` 临时改成 `if len(cells) < 1:`，
确认 `test_find_frontiers_filters_small_cluster` 变红；改回。
再把 `if boundary is not None and boundary.crosses(...)` 注释掉，
确认 boundary 测试变红；改回。

- [ ] **Step 6: 提交**

```bash
git add pi/ros2_lidar/tools/fusion_plan.py pi/ros2_lidar/tests/test_fusion_plan.py
git commit -m "feat(fusion): frontier detection, clustering and scoring"
```

---

### Task 4: `fusion_plan.py` — Navigator 状态机

**Files:**
- Modify: `pi/ros2_lidar/tools/fusion_plan.py`
- Test: `pi/ros2_lidar/tests/test_fusion_plan.py`

**Interfaces:**
- Consumes: Task 1–3 的全部函数。
- Produces: `Navigator(...)`
  - `set_target(final, grid, pose, boundary=None)`
  - `set_explore(on, grid, pose, boundary=None)`
  - `replan(grid, pose, boundary=None)`
  - `cancel()`
  - `on_leg_started(pose)`
  - `on_leg_succeeded(grid, pose, boundary=None)`
  - `on_leg_failed(grid, pose, boundary=None)`
  - `status() -> dict`
  - 属性：`final`、`leg`、`leg_kind`、`state`、`reason`、`explore`、`blacklist`、`frontier_count`

- [ ] **Step 1: 写失败测试（追加）**

```python
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
```

- [ ] **Step 2: 运行，确认失败**

```bash
cd pi/ros2_lidar && python -m pytest tests/test_fusion_plan.py -v
```

Expected: `ImportError: cannot import name 'Navigator'`。

- [ ] **Step 3: 实现**

追加到 `tools/fusion_plan.py`：

```python
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
```

- [ ] **Step 4: 运行测试**

```bash
cd pi/ros2_lidar && python -m pytest tests/test_fusion_plan.py -v
```

Expected: 全部 PASS。若 `test_navigator_budget_blocks_runaway` 的状态
不符合，按测试里的 `in ("blocked", "following")` 判定，只要求最终
`reason` 是预算或失败。

- [ ] **Step 5: 变异自检**

把 `on_leg_succeeded` 里 `if d <= 0.25:` 的 0.25 改成 `1e-9`，确认
`test_navigator_direct_target_and_arrival` 变红；改回。
把 `replan` 的 `if self.leg is not None: return` 删掉，确认
`test_navigator_manual_target_preempts_explore` 仍绿但手动跑会出现重复腿
（这个变异不会被单测抓住，记录为已知盲区，实车看日志确认不会重复发腿）。

- [ ] **Step 6: 提交**

```bash
git add pi/ros2_lidar/tools/fusion_plan.py pi/ros2_lidar/tests/test_fusion_plan.py
git commit -m "feat(fusion): navigator state machine for staged and explore goals"
```

---

### Task 5: `fusion_nav.py` — ROS 适配节点

**Files:**
- Create: `pi/ros2_lidar/tools/fusion_nav.py`
- Test: `python -m py_compile`（ROS 依赖只能在 Pi 上跑）

**Interfaces:**
- Consumes: `fusion_plan.Navigator`、`Grid`、`Boundary`。
- Produces（ROS 面）：
  - 订阅 `/map`（transient_local）、`/fusion/target`（`PoseStamped`）、
    `/fusion/boundary`（`String`，transient_local）
  - 服务 `/fusion/explore`（`SetBool`）、`/fusion/cancel`（`Trigger`）
  - 发布 `/fusion/status`（`String` JSON）、`/fusion/leg`（`PoseStamped`）
  - action client `navigate_to_pose`（`NavigateToPose`）

- [ ] **Step 1: 写节点**

`pi/ros2_lidar/tools/fusion_nav.py`：

```python
#!/usr/bin/env python3
"""融合模式执行节点：分步追目标 + 前沿探索。

只做导航目标，不管地图保存和模式起停（那是 upper 的事）。
用法：python3 fusion_nav.py（由 fusion.launch.py 拉起）
"""
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fusion_plan import Boundary, Grid, Navigator  # noqa: E402


def grid_from_msg(msg):
    return Grid(msg.info.width, msg.info.height, msg.info.resolution,
                msg.info.origin.position.x, msg.info.origin.position.y,
                list(msg.data))


class FusionNav(Node):
    def __init__(self):
        super().__init__("fusion_nav")
        self.declare_parameter("explore_max_s", 1800.0)
        self.declare_parameter("explore_max_m", 200.0)
        self.declare_parameter("stall_area_m2", 0.5)
        self.declare_parameter("stall_window_s", 60.0)

        self.grid = None
        self.area = 0.0
        self.pose = None
        self.boundary = None
        self.nav = Navigator()
        self.action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.leg_handle = None
        self.leg_goal = None
        self.leg_sending = False
        self.leg_seq = 0
        self.explore_start_t = None
        self.last_area = 0.0
        self.last_area_t = None
        self.distance = 0.0
        self.last_pose_xy = None
        self.last_status_t = 0.0
        self.blacklist_clear_t = time.monotonic()

        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        boundary_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                  reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(PoseStamped, "/fusion/target",
                                 self.on_target, 10)
        self.create_subscription(String, "/fusion/boundary",
                                 self.on_boundary, boundary_qos)
        self.create_service(SetBool, "/fusion/explore", self.on_explore)
        self.create_service(Trigger, "/fusion/cancel", self.on_cancel)
        self.status_pub = self.create_publisher(String, "/fusion/status", 10)
        self.leg_pub = self.create_publisher(PoseStamped, "/fusion/leg", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.plan_tick)
        self.get_logger().info("fusion_nav: 等待 /map 与 TF")

    # ---------------------------------------------------------------- 输入
    def on_map(self, msg):
        self.grid = grid_from_msg(msg)
        self.area = self.grid.free_area()

    def on_boundary(self, msg):
        try:
            obj = json.loads(msg.data) if msg.data else None
        except Exception:
            obj = None
        self.boundary = Boundary.from_json(obj)
        self.get_logger().info("fusion_nav: 边界%s" % (
            "已设置" if self.boundary else "已清除"))

    def on_target(self, msg):
        if self.grid is None or self.pose is None:
            self.get_logger().warn("fusion_nav: 没有地图/位姿，忽略目标")
            return
        p = msg.pose.position
        self.nav.set_target((p.x, p.y), self.grid, self.pose, self.boundary)
        self.explore_start_t = None
        self.sync_action()

    def on_explore(self, req, resp):
        if self.grid is None or self.pose is None:
            resp.success = False
            resp.message = "没有地图/位姿"
            return resp
        self.nav.set_explore(req.data, self.grid, self.pose, self.boundary)
        if req.data and self.explore_start_t is None:
            self.explore_start_t = time.monotonic()
            self.last_area = self.area
            self.last_area_t = self.explore_start_t
            self.distance = 0.0
        resp.success = True
        resp.message = "探索已开启" if req.data else "探索已关闭"
        self.sync_action()
        return resp

    def on_cancel(self, req, resp):
        self.nav.cancel()
        self.explore_start_t = None
        self.cancel_action()
        resp.success = True
        resp.message = "已取消"
        return resp

    # ---------------------------------------------------------------- 周期
    def tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose = (t.x, t.y, yaw)
            if self.last_pose_xy is not None:
                self.distance += math.hypot(
                    t.x - self.last_pose_xy[0],
                    t.y - self.last_pose_xy[1])
            self.last_pose_xy = (t.x, t.y)
        except Exception:
            self.pose = None
            self.last_pose_xy = None

        now = time.monotonic()
        if now - self.last_status_t >= 0.5:
            self.last_status_t = now
            self.publish_status()
        if now - self.blacklist_clear_t >= 600.0:
            self.blacklist_clear_t = now
            self.nav.blacklist.clear()

    def plan_tick(self):
        if self.grid is None or self.pose is None:
            return
        if self.nav.explore:
            if self.explore_start_t is None:
                self.explore_start_t = time.monotonic()
                self.last_area = self.area
                self.last_area_t = self.explore_start_t
            elapsed = time.monotonic() - self.explore_start_t
            if elapsed > float(self.get_parameter("explore_max_s").value):
                self.finish_explore("time_limit")
                return
            if self.distance > float(self.get_parameter("explore_max_m").value):
                self.finish_explore("distance_limit")
                return
            if self.area < self.last_area + float(
                    self.get_parameter("stall_area_m2").value):
                if time.monotonic() - self.last_area_t > float(
                        self.get_parameter("stall_window_s").value):
                    self.finish_explore("area_stalled")
                    return
            else:
                self.last_area = self.area
                self.last_area_t = time.monotonic()
        self.nav.replan(self.grid, self.pose, self.boundary)
        self.sync_action()

    def finish_explore(self, reason):
        self.nav.set_explore(False, self.grid, self.pose, self.boundary)
        self.nav.state = "done"
        self.nav.reason = reason
        self.explore_start_t = None
        self.cancel_action()
        self.get_logger().info("fusion_nav: 探索结束（%s）" % reason)

    # ---------------------------------------------------------------- 输出
    def publish_status(self):
        st = self.nav.status()
        st.update({
            "distance_m": round(self.distance, 2),
            "covered_m2": round(self.area, 2),
            "elapsed_s": (round(time.monotonic() - self.explore_start_t, 1)
                          if self.explore_start_t else 0.0),
            "pose": self.pose,
        })
        msg = String()
        msg.data = json.dumps(st, ensure_ascii=False,
                              separators=(",", ":"))
        self.status_pub.publish(msg)
        if self.nav.leg is not None and self.pose is not None:
            leg = PoseStamped()
            leg.header.frame_id = "map"
            leg.header.stamp = self.get_clock().now().to_msg()
            leg.pose.position.x = float(self.nav.leg[0])
            leg.pose.position.y = float(self.nav.leg[1])
            yaw = math.atan2(self.nav.leg[1] - self.pose[1],
                             self.nav.leg[0] - self.pose[0])
            leg.pose.orientation.z = math.sin(yaw / 2.0)
            leg.pose.orientation.w = math.cos(yaw / 2.0)
            self.leg_pub.publish(leg)

    # ---------------------------------------------------------------- action
    def sync_action(self):
        if self.nav.leg is None:
            self.cancel_action()
            return
        if self.leg_goal == self.nav.leg:
            return
        self.cancel_action()
        self.send_leg(self.nav.leg)

    def send_leg(self, goal):
        if not self.action.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("fusion_nav: nav2 action 不在")
            return
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(goal[0])
        g.pose.pose.position.y = float(goal[1])
        yaw = 0.0
        if self.pose is not None:
            yaw = math.atan2(goal[1] - self.pose[1],
                             goal[0] - self.pose[0])
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.leg_seq += 1
        seq = self.leg_seq
        self.leg_goal = tuple(goal)
        self.leg_sending = True
        self.nav.on_leg_started(self.pose)
        fut = self.action.send_goal_async(g)
        fut.add_done_callback(lambda f: self.on_goal_response(f, seq))

    def on_goal_response(self, fut, seq):
        if seq != self.leg_seq:
            return
        try:
            gh = fut.result()
        except Exception:
            gh = None
        if gh is None or not gh.accepted:
            self.leg_sending = False
            self.leg_goal = None
            self.on_leg_done(False)
            return
        self.leg_handle = gh
        self.leg_sending = False
        gh.get_result_async().add_done_callback(
            lambda f: self.on_goal_result(f, seq))

    def on_goal_result(self, fut, seq):
        if seq != self.leg_seq:
            return
        self.leg_handle = None
        self.leg_goal = None
        self.leg_sending = False
        try:
            status = fut.result().status
        except Exception:
            status = GoalStatus.STATUS_ABORTED
        self.on_leg_done(status == GoalStatus.STATUS_SUCCEEDED)

    def on_leg_done(self, ok):
        if self.grid is None or self.pose is None:
            return
        if ok:
            self.nav.on_leg_succeeded(self.grid, self.pose, self.boundary)
        else:
            self.nav.on_leg_failed(self.grid, self.pose, self.boundary)
        self.publish_status()
        self.sync_action()

    def cancel_action(self):
        gh = self.leg_handle
        self.leg_seq += 1
        self.leg_handle = None
        self.leg_goal = None
        self.leg_sending = False
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass


def main():
    rclpy.init()
    node = FusionNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 语法与导入检查（Windows 可跑）**

```bash
python -m py_compile pi/ros2_lidar/tools/fusion_nav.py pi/ros2_lidar/tools/fusion_plan.py
```

Expected: 无输出。

- [ ] **Step 3: Pi 上做一次"无动作"冒烟**

```bash
# Pi，先不启动 nav2，只启动节点
source /opt/ros/jazzy/setup.bash
python3 ~/ROS-car/pi/ros2_lidar/tools/fusion_nav.py
# 另开一个终端：
ros2 topic echo /fusion/status --once
ros2 service call /fusion/explore std_srvs/srv/SetBool "{data: false}"
```

Expected：节点不崩；`/fusion/status` 有 JSON（没有地图时 `state=idle`）；
服务返回 `success: true, message: 没有地图/位姿`。按 Ctrl-C 退出。

- [ ] **Step 4: 提交**

```bash
git add pi/ros2_lidar/tools/fusion_nav.py
git commit -m "feat(fusion): fusion_nav ROS adapter"
```

---

### Task 6: 融合启动与参数集

**Files:**
- Create: `pi/ros2_lidar/launch/fusion.launch.py`
- Create: `pi/ros2_lidar/config/slam_params_fusion.yaml`
- Create: `pi/ros2_lidar/config/nav2_params_fusion.yaml`
- Test: `pi/ros2_lidar/tests/test_fusion_config.py`

**Interfaces:**
- Consumes: `tools/fusion_nav.py`、`launch/nav2_navigation_launch.py`。
- Produces: `ros2 launch .../fusion.launch.py` 一套可跑的融合栈。

- [ ] **Step 1: 写失败测试**

`pi/ros2_lidar/tests/test_fusion_config.py`：

```python
import os

import pytest

yaml = pytest.importorskip("yaml")

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(name):
    with open(os.path.join(HERE, "config", name), encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_nav2_fusion_global_plugins_have_no_obstacle_layer():
    d = load("nav2_params_fusion.yaml")
    p = d["global_costmap"]["global_costmap"]["ros__parameters"]
    assert p["plugins"] == ["static_layer", "inflation_layer"]


def test_nav2_fusion_keeps_unknown_blocked():
    d = load("nav2_params_fusion.yaml")
    p = d["planner_server"]["ros__parameters"]["GridBased"]
    assert p["allow_unknown"] is False


def test_nav2_fusion_local_costmap_is_4m():
    d = load("nav2_params_fusion.yaml")
    p = d["local_costmap"]["local_costmap"]["ros__parameters"]
    assert p["width"] == 4 and p["height"] == 4


def test_nav2_fusion_loosens_goal_checker():
    d = load("nav2_params_fusion.yaml")
    p = d["controller_server"]["ros__parameters"]["general_goal_checker"]
    assert p["xy_goal_tolerance"] == 0.25
    assert p["yaw_goal_tolerance"] == 0.45


def test_slam_fusion_updates_faster_and_keeps_scan_filter():
    d = load("slam_params_fusion.yaml")
    p = d["slam_toolbox"]["ros__parameters"]
    assert p["map_update_interval"] == 1.0
    assert p["scan_topic"] == "/scan_filtered"
    assert p["max_laser_range"] == 12.0
```

- [ ] **Step 2: 运行，确认失败**

```bash
cd pi/ros2_lidar && python -m pytest tests/test_fusion_config.py -v
```

Expected: `FileNotFoundError: config/nav2_params_fusion.yaml`。

- [ ] **Step 3: 生成 `slam_params_fusion.yaml`**

```bash
cd pi/ros2_lidar
cp config/slam_params.yaml config/slam_params_fusion.yaml
python - <<'PY'
p = "config/slam_params_fusion.yaml"
s = open(p, encoding="utf-8").read()
s = s.replace("map_update_interval: 5.0", "map_update_interval: 1.0")
s = s.replace("minimum_time_interval: 0.5", "minimum_time_interval: 0.3")
open(p, "w", encoding="utf-8").write(s)
PY
```

确认文件头注释仍保留（不要用 yaml.safe_load + dump，会丢注释）。

- [ ] **Step 4: 生成 `nav2_params_fusion.yaml`**

```bash
cd pi/ros2_lidar
cp config/nav2_params.yaml config/nav2_params_fusion.yaml
```

然后做四处精确修改：

1) 全局代价图去掉 `obstacle_layer`。把：

```yaml
      plugins:
      - static_layer
      - obstacle_layer
      - inflation_layer
      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan_filtered
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_max_range: 12.0
          raytrace_min_range: 0.15
          obstacle_max_range: 12.0
          obstacle_min_range: 0.15
      static_layer:
```

改成：

```yaml
      plugins:
      - static_layer
      - inflation_layer
      static_layer:
```

2) 局部代价图 3 m → 4 m：

```yaml
      width: 3
      height: 3
```

改成：

```yaml
      width: 4
      height: 4
```

3) 目标容差（`general_goal_checker`）：

```yaml
      xy_goal_tolerance: 0.05
      yaw_goal_tolerance: 0.2
```

改成：

```yaml
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.45
```

4) RPP 的终点容差（`FollowPath` 段里的 `xy_goal_tolerance: 0.05`）：

```yaml
      xy_goal_tolerance: 0.05
```

改成：

```yaml
      xy_goal_tolerance: 0.2
```

**注意**：第 3 条和第 4 条都含 `xy_goal_tolerance: 0.05`，用上下文
区分；不要用全局替换把 `general_goal_checker` 也改成 0.2。

- [ ] **Step 5: 写 `fusion.launch.py`**

`pi/ros2_lidar/launch/fusion.launch.py`：

```python
"""融合模式：slam_toolbox 提供 map->odom，nav2 只跑规划/控制。

    ros2 launch /home/cy/ros2_lidar/launch/fusion.launch.py

不启动 map_server / AMCL / pose_persist / auto_localize。
速度链路与 nav.launch.py 相同（no_pivot_guard + cmd_mux 不变）。
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
SLAM_PARAMS = os.path.join(HERE, "config", "slam_params_fusion.yaml")
NAV_PARAMS = os.path.join(HERE, "config", "nav2_params_fusion.yaml")
URDF = os.path.join(HERE, "urdf", "robot.urdf")


def generate_launch_description():
    ports = sorted(glob.glob("/dev/serial/by-id/*CP2102*"))
    if not ports:
        raise RuntimeError(
            "No CP2102 lidar adapter found under /dev/serial/by-id/ -- "
            "is the RPLIDAR plugged in?")

    with open(URDF, "r", encoding="utf-8") as f:
        robot_description = f.read()

    reset = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "tools", "lidar_reset.py"),
             ports[0]],
        output="screen")

    lidar = Node(
        package="rplidar_ros", executable="rplidar_composition",
        name="rplidar_composition", output="screen",
        parameters=[{
            "serial_port": ports[0], "serial_baudrate": 115200,
            "frame_id": "laser", "inverted": False,
            "angle_compensate": True, "auto_standby": False,
        }],
    )

    scan_filter = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "tools", "scan_self_filter.py")],
        output="screen")

    pivot_guard = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "tools", "no_pivot_guard.py")],
        output="screen")

    fusion_nav = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "tools", "fusion_nav.py")],
        output="screen")

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_toolbox"),
            "launch", "online_async_launch.py")),
        launch_arguments={
            "slam_params_file": SLAM_PARAMS,
            "use_sim_time": "false",
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            HERE, "launch", "nav2_navigation_launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": NAV_PARAMS,
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        reset,
        rsp,
        TimerAction(period=3.0, actions=[lidar]),
        TimerAction(period=5.0, actions=[scan_filter]),
        TimerAction(period=6.0, actions=[pivot_guard]),
        TimerAction(period=6.0, actions=[slam]),
        navigation,
        TimerAction(period=12.0, actions=[fusion_nav]),
    ])
```

- [ ] **Step 6: 运行配置测试 + 语法检查**

```bash
cd pi/ros2_lidar
python -m pytest tests/test_fusion_config.py -v
python -m py_compile launch/fusion.launch.py
```

Expected: 5 个测试 PASS，py_compile 无输出。

- [ ] **Step 7: 提交**

```bash
git add pi/ros2_lidar/launch/fusion.launch.py \
        pi/ros2_lidar/config/slam_params_fusion.yaml \
        pi/ros2_lidar/config/nav2_params_fusion.yaml \
        pi/ros2_lidar/tests/test_fusion_config.py
git commit -m "feat(fusion): launch combination and parameter sets"
```

---

### Task 7: upper 后端 — 三态模式、fusion 目标、探索/边界 API、存图写位姿

**Files:**
- Modify: `pi/ros2_lidar/upper/server.py`
- Test: `pi/ros2_lidar/upper/tests/test_server.py`

**Interfaces:**
- Consumes: `fusion.launch.py`、`/fusion/*` 话题与服务。
- Produces（HTTP）：
  - `POST /api/mode {"mode":"fusion"}`（`slam|nav|fusion|idle`）
  - `POST /api/explore {"enabled": bool}`
  - `POST /api/boundary {"anchor":[x,y],"segments":[[x1,y1,x2,y2],...]}`；
    `anchor:null, segments:[]` 表示清除
  - `GET /api/boundary`
  - SSE `status` 事件新增 `fusion`、`leg`
- 模式值新增 `fusion`；`detect_mode()` 返回
  `fusion | mapping | navigating | idle`。

- [ ] **Step 1: 写失败测试（追加到 `upper/tests/test_server.py`）**

```python
def test_detect_mode_three_states(monkeypatch):
    import subprocess
    import server as srv_mod

    def fake_run(cmd, **kw):
        class R:
            returncode = 0 if cmd[2] in alive else 1
        return R()

    alive = {"slam_toolbox", "nav2_controller"}
    monkeypatch.setattr(subprocess, "run", fake_run)
    assert srv_mod.detect_mode() == "fusion"
    alive = {"slam_toolbox"}
    assert srv_mod.detect_mode() == "mapping"
    alive = {"nav2_controller"}
    assert srv_mod.detect_mode() == "navigating"
    alive = set()
    assert srv_mod.detect_mode() == "idle"


class FusionController(FakeController):
    def __init__(self):
        super().__init__()
        self.boundary = None

    def set_explore(self, enabled):
        self.calls.append(("explore", enabled))
        return True, "explore %s" % enabled

    def set_boundary(self, obj):
        self.calls.append(("boundary", obj))
        self.boundary = obj
        return True, "boundary ok"


def test_explore_endpoint(tmp_path):
    ctl = FusionController()
    srv, base = _serve_ctl(tmp_path, ctl)
    try:
        body = json.loads(_post(base, "/api/explore",
                                {"enabled": True}).read())
        assert body["ok"] is True
        assert ctl.calls[-1] == ("explore", True)
    finally:
        srv.shutdown()


def test_boundary_post_and_get(tmp_path):
    ctl = FusionController()
    srv, base = _serve_ctl(tmp_path, ctl)
    try:
        obj = {"anchor": [0.0, 0.0],
               "segments": [[2.0, -1.0, 2.0, 1.0]]}
        assert json.loads(_post(base, "/api/boundary", obj).read())["ok"]
        got = json.loads(urllib.request.urlopen(base + "/api/boundary").read())
        assert got["boundary"] == obj
    finally:
        srv.shutdown()


def test_mode_accepts_fusion(tmp_path):
    ctl = FakeController()
    srv, base = _serve_ctl(tmp_path, ctl)
    try:
        body = json.loads(_post(base, "/api/mode",
                                {"mode": "fusion"}).read())
        assert body["ok"] is True
        assert ctl.calls[-1] == ("start", "fusion", None)
    finally:
        srv.shutdown()


def test_save_map_writes_pose(tmp_path, monkeypatch):
    import os
    import server as srv_mod
    from server import ModeController, State

    tools = tmp_path / "tools"
    tools.mkdir()
    (tools / "save_map_snapshot.py").write_text("")
    launch = tmp_path / "launch"
    launch.mkdir()
    maps = tmp_path / "maps"
    maps.mkdir()
    state = State()
    state.set_pose(1.25, -0.5, 0.75)
    ctl = ModeController(state, None, str(maps), str(launch))

    def fake_run(cmd, **kw):
        class R:
            returncode = 0
            stdout = "saved"
            stderr = ""
        return R()

    monkeypatch.setattr(srv_mod.subprocess, "run", fake_run)
    monkeypatch.setattr(srv_mod, "detect_mode", lambda: "fusion")
    ok, msg = ctl.save_map("trace")
    assert ok, msg
    import json as _json
    with open(maps / "trace_last_pose.json") as f:
        d = _json.load(f)
    assert d == {"x": 1.25, "y": -0.5, "yaw": 0.75}
```

- [ ] **Step 2: 运行，确认失败**

```bash
cd pi/ros2_lidar/upper && python -m pytest tests/test_server.py -v
```

Expected: `detect_mode` 仍返回 `mapping`（不是 fusion），`/api/explore` 404。

- [ ] **Step 3: 修改 `detect_mode()`**

把现有实现替换为：

```python
def detect_mode():
    """只读探测：mapping / navigating / fusion / idle。"""
    import subprocess

    def alive(pat):
        try:
            return subprocess.run(["pgrep", "-f", pat],
                                  capture_output=True).returncode == 0
        except OSError:
            return False

    slam = alive("slam_toolbox")
    nav = alive("nav2_controller")
    if slam and nav:
        return "fusion"
    if slam:
        return "mapping"
    if nav:
        return "navigating"
    return "idle"
```

- [ ] **Step 4: `ModeController` 支持 fusion**

`__init__` 增加：

```python
        self.boundary = None
        self.boundary_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "boundary.json")
```

`_external()` 的提示改为：

```python
                return False, ("检测到外部实例（%s）。先 sudo systemctl stop slam/nav"
                               % ("建图" if ext == "mapping" else
                                  "导航" if ext == "navigating" else "融合"))
```

`start()` 里的分支改成三路（必须是 `if / elif / else`，不能把
`elif` 放在 `else` 后面）：

```python
            if mode == "nav":
                if (not map_name or not NAME_RE.match(map_name)
                        or not map_name.endswith(".yaml")):
                    return False, "请选择地图（.yaml）"
                if not os.path.isfile(os.path.join(self.maps_dir, map_name)):
                    return False, "地图不存在：%s" % map_name
                launch = os.path.join(self.launch_dir, "nav.launch.py")
                cmd = ["ros2", "launch", launch, "use_composition:=False",
                       "map:=" + os.path.join(self.maps_dir, map_name)]
            elif mode == "fusion":
                launch = os.path.join(self.launch_dir, "fusion.launch.py")
                cmd = ["ros2", "launch", launch]
            else:
                launch = os.path.join(self.launch_dir, "mapping.launch.py")
                cmd = ["ros2", "launch", launch]
```

并把状态设置改成：

```python
            self.state.set_status(
                mode=("mapping" if mode == "slam" else
                      "fusion" if mode == "fusion" else "navigating"),
                owned=True, goal=None, note="")
            return True, "已启动%s（日志 %s）" % (
                {"slam": "建图", "nav": "导航", "fusion": "边建边航"}[mode],
                log_path)
```

`save_map()` 的模式门改成：

```python
        owned = self.mode if self._alive() else None
        mode = owned or detect_mode()
        if mode not in ("slam", "mapping", "fusion"):
            return False, "当前不在建图/融合模式，没有可保存的地图"
```

并在存图成功后写位姿：

```python
        pose = self.state.snapshot()["pose"]
        pose_note = ""
        if pose:
            path = os.path.join(self.maps_dir, name + "_last_pose.json")
            try:
                tmp = path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump({"x": pose["x"], "y": pose["y"],
                               "yaw": pose["yaw"]}, f)
                os.replace(tmp, path)
                pose_note = "，位姿已写入"
            except Exception as e:
                pose_note = "，位姿写入失败：%s" % e
        else:
            pose_note = "，位姿未写入（没有 TF）"
        return True, "已保存 %s.yaml%s" % (name, pose_note)
```

`send_goal()` 改成：

```python
    def send_goal(self, x, y, yaw):
        owned = self.mode if self._alive() else None
        mode = owned or detect_mode()
        if self.ros is None:
            return False, "ROS 未就绪"
        if mode == "fusion":
            return self.ros.send_target(x, y, yaw)
        if mode in ("nav", "navigating"):
            return self.ros.send_goal(x, y, yaw)
        return False, "当前不在导航/融合模式"
```

`cancel_goal()` 改成：

```python
    def cancel_goal(self):
        if self.ros is None:
            return False, "ROS 未就绪"
        owned = self.mode if self._alive() else None
        mode = owned or detect_mode()
        if mode == "fusion":
            return self.ros.cancel_target()
        return self.ros.cancel_goal()
```

新增方法：

```python
    def set_explore(self, enabled):
        owned = self.mode if self._alive() else None
        mode = owned or detect_mode()
        if mode != "fusion":
            return False, "只有融合模式能开自动探索"
        if self.ros is None:
            return False, "ROS 未就绪"
        return self.ros.set_explore(bool(enabled))

    def set_boundary(self, obj):
        if obj is not None:
            if obj.get("anchor") is None and not obj.get("segments"):
                obj = None          # 页面用 anchor:null, segments:[] 表示清除
            else:
                try:
                    anchor = [float(v) for v in obj.get("anchor", [])]
                    segs = [[float(v) for v in s]
                            for s in obj.get("segments", [])]
                except Exception:
                    return False, "边界格式不对"
                if len(anchor) != 2 or not segs or len(segs) > 200:
                    return False, "边界需要 1~200 段，锚点两个数"
                if any(len(s) != 4 for s in segs):
                    return False, "每段需要 x1,y1,x2,y2"
                obj = {"anchor": anchor, "segments": segs}
        self.boundary = obj
        try:
            tmp = self.boundary_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(obj, f)
            os.replace(tmp, self.boundary_path)
        except Exception as e:
            return False, "边界保存失败：%s" % e
        if self.ros is not None:
            self.ros.publish_boundary(obj)
        return True, "边界已更新" if obj else "边界已清除"
```

- [ ] **Step 5: `RosSource` 增加 fusion 通路**

在 `start()` 的 import 块里增加：

```python
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import String
            from std_srvs.srv import SetBool, Trigger
```

在 `_Node.__init__` 末尾增加：

```python
                boundary_qos = QoSProfile(
                    depth=1, history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self._target_pub = self.create_publisher(
                    PoseStamped, "/fusion/target", 10)
                self._boundary_pub = self.create_publisher(
                    String, "/fusion/boundary", boundary_qos)
                self.create_subscription(
                    String, "/fusion/status", self._on_fusion_status, 10)
                self.create_subscription(
                    PoseStamped, "/fusion/leg", self._on_fusion_leg, 10)
                self._explore_cli = self.create_client(
                    SetBool, "/fusion/explore")
                self._cancel_cli = self.create_client(
                    Trigger, "/fusion/cancel")
```

在 `_Node` 里增加：

```python
            def _on_fusion_status(self, msg):
                try:
                    obj = json.loads(msg.data)
                except Exception:
                    return
                state.set_status(fusion=obj)

            def _on_fusion_leg(self, msg):
                p = msg.pose
                yaw = math.atan2(
                    2.0 * (p.orientation.w * p.orientation.z),
                    1.0 - 2.0 * p.orientation.z * p.orientation.z)
                state.set_status(leg={"x": p.position.x,
                                      "y": p.position.y, "yaw": yaw})

            def send_target(self, x, y, yaw):
                if self._target_pub.get_subscription_count() == 0:
                    return False, "fusion_nav 不在（等融合启动完）"
                msg = PoseStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.x = float(x)
                msg.pose.position.y = float(y)
                msg.pose.orientation.z = math.sin(float(yaw) / 2.0)
                msg.pose.orientation.w = math.cos(float(yaw) / 2.0)
                self._target_pub.publish(msg)
                return True, "目标已发送（分步执行）"

            def cancel_target(self):
                if not self._cancel_cli.wait_for_service(timeout_sec=1.0):
                    return False, "fusion_nav 不在"
                self._cancel_cli.call_async(Trigger.Request())
                return True, "已请求取消"

            def set_explore(self, enabled):
                if not self._explore_cli.wait_for_service(timeout_sec=1.0):
                    return False, "fusion_nav 不在"
                req = SetBool.Request()
                req.data = bool(enabled)
                self._explore_cli.call_async(req)
                return True, "探索已开启" if enabled else "探索已关闭"

            def publish_boundary(self, obj):
                msg = String()
                msg.data = json.dumps(obj) if obj else ""
                self._boundary_pub.publish(msg)
```

`RosSource` 公开方法追加：

```python
    def send_target(self, x, y, yaw):
        if self._node is None:
            return False, "ROS 未就绪"
        return self._node.send_target(x, y, yaw)

    def cancel_target(self):
        if self._node is None:
            return False, "ROS 未就绪"
        return self._node.cancel_target()

    def set_explore(self, enabled):
        if self._node is None:
            return False, "ROS 未就绪"
        return self._node.set_explore(enabled)

    def publish_boundary(self, obj):
        if self._node is None:
            return False, "ROS 未就绪"
        self._node.publish_boundary(obj)
        return True, "边界已下发"
```

- [ ] **Step 6: HTTP 路由**

`do_GET` 增加：

```python
        elif path == "/api/boundary":
            ctl = self.server.controller
            self._json(200, {"boundary": ctl.boundary if ctl else None})
```

`do_POST` 的 `/api/mode` 分支改成：

```python
            elif mode in ("slam", "nav", "fusion"):
                ok, msg = ctl.start(mode, body.get("map"))
            else:
                self._json(400, {"ok": False,
                                 "error": "mode 只能是 slam/nav/fusion/idle"})
                return
```

新增分支：

```python
        elif path == "/api/explore":
            ok, msg = ctl.set_explore(bool(body.get("enabled")))
        elif path == "/api/boundary":
            ok, msg = ctl.set_boundary(body)
```

SSE status 事件增加两个字段：

```python
                        "fusion": snap["status"].get("fusion"),
                        "leg": snap["status"].get("leg"),
```

- [ ] **Step 7: 启动时恢复边界**

`main()` 里创建 controller 之后：

```python
        if controller is not None:
            bpath = os.path.join(here, "boundary.json")
            if os.path.isfile(bpath):
                try:
                    with open(bpath, encoding="utf-8") as f:
                        controller.set_boundary(json.load(f))
                except Exception:
                    pass
```

- [ ] **Step 8: 运行测试**

```bash
cd pi/ros2_lidar/upper && python -m pytest tests/test_server.py -v
```

Expected: 全部 PASS。若 `test_save_map_writes_pose` 失败，检查
`ModeController.save_map` 是否在 `save_map_snapshot` 成功后才写位姿。

- [ ] **Step 9: 提交**

```bash
git add pi/ros2_lidar/upper/server.py pi/ros2_lidar/upper/tests/test_server.py
git commit -m "feat(upper): fusion mode, staged goals, explore and boundary APIs"
```

---

### Task 8: upper 页面 — 融合按钮、建图中点图、中间腿、探索与边界线

**Files:**
- Modify: `pi/ros2_lidar/upper/index.html`
- Test: `pi/ros2_lidar/upper/tests/test_index.py`

**Interfaces:**
- Consumes: Task 7 的 HTTP/SSE 字段（`mode=fusion`、`fusion`、`leg`、`/api/explore`、`/api/boundary`）。
- Produces: 页面上的"边建边航""自动探索""画边界线"控件与绘制。

- [ ] **Step 1: 写失败测试**

`pi/ros2_lidar/upper/tests/test_index.py`：

```python
import os

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_fusion_controls_present():
    html = open(os.path.join(HERE, "index.html"), encoding="utf-8").read()
    for token in ('id="btn-fusion"', 'id="btn-explore"',
                  'id="btn-boundary"', 'id="btn-clear-boundary"',
                  'id="boundary-bar"'):
        assert token in html, token


def test_no_external_assets():
    html = open(os.path.join(HERE, "index.html"), encoding="utf-8").read()
    assert "http://" not in html
    assert "https://" not in html
```

- [ ] **Step 2: 运行，确认失败**

```bash
cd pi/ros2_lidar/upper && python -m pytest tests/test_index.py -v
```

Expected: `AssertionError: id="btn-fusion"`。

- [ ] **Step 3: 面板 HTML**

在 `<div class="row">`（导航那一行）之后插入：

```html
    <div class="row">
      <button id="btn-fusion" class="primary">边建边航</button>
      <button id="btn-explore">自动探索：关</button>
    </div>
    <div class="row">
      <button id="btn-boundary">画边界线</button>
      <button id="btn-clear-boundary">清除边界</button>
    </div>
```

在 `#pending` 之后插入：

```html
<div id="boundary-bar" hidden>
  <span id="boundary-text">点地图加线段，完成后点“应用”</span>
  <button class="go" id="boundary-apply">应用</button>
  <button id="boundary-cancel">取消</button>
</div>
```

- [ ] **Step 4: CSS**

把：

```css
  #pending {
```

改成：

```css
  #pending, #boundary-bar {
```

把 `#pending button {` 改成 `#pending button, #boundary-bar button {`；
把 `#pending button.go {` 改成 `#pending button.go, #boundary-bar button.go {`。

- [ ] **Step 5: JS 状态与元素**

在元素引用区增加：

```js
  const elBoundaryBar = document.getElementById("boundary-bar");
  const elBoundaryText = document.getElementById("boundary-text");
```

在第二层状态区增加：

```js
  let boundaryMode = false;
  let boundaryPoints = [];
  let boundaryAnchor = null;
  let boundary = null;
  let fusion = null;
```

`MODE_TEXT` 增加：

```js
    fusion: ["建图+导航中", "nav"],
```

- [ ] **Step 6: 绘制**

在 `render()` 里 `drawMap();` 之后插入：

```js
    drawBoundary();
```

在 `render()` 末尾增加：

```js
    drawBoundaryDraft();
    drawLeg();
```

新增函数（放在 `drawRobot()` 之后）：

```js
  function drawBoundary() {
    if (!boundary || !boundary.segments) return;
    ctx.save();
    ctx.strokeStyle = "rgba(255,95,86,.9)";
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 6]);
    for (const s of boundary.segments) {
      const [x1, y1] = w2s(s[0], s[1]);
      const [x2, y2] = w2s(s[2], s[3]);
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawBoundaryDraft() {
    if (!boundaryMode || boundaryPoints.length < 2) return;
    ctx.save();
    ctx.strokeStyle = "#ffb347";
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    for (let i = 0; i < boundaryPoints.length; i++) {
      const [sx, sy] = w2s(boundaryPoints[i][0], boundaryPoints[i][1]);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    }
    ctx.stroke();
    ctx.restore();
  }

  function drawLeg() {
    const leg = status.leg || (status.fusion && status.fusion.leg);
    if (!leg) return;
    const x = Array.isArray(leg) ? leg[0] : leg.x;
    const y = Array.isArray(leg) ? leg[1] : leg.y;
    if (x === undefined || y === undefined) return;
    const [sx, sy] = w2s(x, y);
    ctx.save();
    ctx.strokeStyle = "#ffb347";
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.arc(sx, sy, 8, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
  }
```

- [ ] **Step 7: 点图与边界交互**

在 `setGoalMode` 里把目标模式判定改成：

```js
    if (on) {
      if (!(status.mode === "navigating" || status.mode === "fusion")) {
        toast("先开始导航或边建边航，再点图发目标", true);
      } else {
        toast("在地图上点一下选目标");
      }
    } else {
      clearPending();
    }
```

把 `pointerup` 里的判定改成：

```js
  cv.addEventListener("pointerup", (e) => {
    if (press && !press.moved) {
      const [wx, wy] = s2w(e.clientX, e.clientY);
      if (boundaryMode) {
        boundaryPoints.push([wx, wy]);
        elBoundaryText.textContent =
          "已加 " + boundaryPoints.length + " 个点；至少两个点后点“应用”";
        dirty = true;
      } else if (goalMode &&
                 (status.mode === "navigating" || status.mode === "fusion")) {
        setPending(wx, wy);
      }
    }
    press = null;
    drag = null;
  });
```

新增边界函数：

```js
  function setBoundaryMode(on) {
    boundaryMode = on;
    document.getElementById("btn-boundary").classList.toggle("active", on);
    cv.style.cursor = on ? "crosshair" : "";
    if (on) {
      if (!pose) {
        toast("还没有位姿，不能画边界", true);
        setBoundaryMode(false);
        return;
      }
      boundaryAnchor = [pose.x, pose.y];
      boundaryPoints = [];
      elBoundaryBar.hidden = false;
      elBoundaryText.textContent = "点地图加线段（至少两个点），完成后点“应用”";
      toast("画在“不能去”的一侧；线的另一侧会被禁止");
    } else {
      elBoundaryBar.hidden = true;
      boundaryPoints = [];
    }
    dirty = true;
  }

  async function applyBoundary() {
    if (!boundaryAnchor || boundaryPoints.length < 2) {
      toast("至少要点两个点", true);
      return;
    }
    const segments = [];
    for (let i = 1; i < boundaryPoints.length; i++) {
      segments.push([boundaryPoints[i - 1][0], boundaryPoints[i - 1][1],
                     boundaryPoints[i][0], boundaryPoints[i][1]]);
    }
    try {
      const j = await api("/api/boundary",
                          { anchor: boundaryAnchor, segments });
      boundary = { anchor: boundaryAnchor, segments };
      toast(j.message || "边界已应用");
      setBoundaryMode(false);
    } catch (e) {
      toast(e.message, true);
    }
  }

  async function clearBoundary() {
    try {
      const j = await api("/api/boundary", { anchor: null, segments: [] });
      boundary = null;
      toast(j.message || "边界已清除");
      dirty = true;
    } catch (e) {
      toast(e.message, true);
    }
  }

  async function loadBoundary() {
    try {
      const r = await fetch("/api/boundary");
      if (!r.ok) return;
      const j = await r.json();
      boundary = j.boundary || null;
      dirty = true;
    } catch (e) { /* 断线忽略 */ }
  }
```

- [ ] **Step 8: 目标状态与探索按钮**

把 `updateGoalUI()` 替换为：

```js
  function updateGoalUI() {
    const f = status.fusion;
    if (status.mode === "fusion" && f) {
      fusion = f;
      const texts = {
        idle: "空闲",
        following: "分步推进中",
        exploring: "探索中",
        blocked: "已停止：" + (f.reason || ""),
        done: "探索结束：" + (f.reason || ""),
      };
      elGoalChip.textContent = texts[f.state] || f.state;
      elGoalChip.hidden = !f.state || f.state === "idle";
      elGoalRow.hidden = !(f.state === "following" ||
                           f.state === "exploring");
      if (f.leg) {
        activeGoal = { x: f.leg[0], y: f.leg[1], state: "executing" };
      } else {
        activeGoal = null;
      }
      const ex = document.getElementById("btn-explore");
      ex.textContent = f.explore ? "自动探索：开" : "自动探索：关";
      ex.classList.toggle("active", !!f.explore);
      dirty = true;
      return;
    }
    fusion = null;
    const g = status.goal;
    if (g && g.state && g.x !== undefined) {
      activeGoal = { x: g.x, y: g.y, yaw: g.yaw, state: g.state };
      const texts = {
        sending: "目标：发送中…",
        executing: (g.remaining !== undefined && g.remaining !== null)
          ? "目标：还剩 " + Number(g.remaining).toFixed(1) + " m"
          : "目标：执行中",
        succeeded: "已到达目标",
        aborted: "导航失败/中止",
        canceled: "已取消",
        rejected: "目标被拒绝",
        failed: "导航出错",
      };
      elGoalChip.textContent = texts[g.state] || g.state;
      elGoalChip.hidden = false;
      elGoalRow.hidden = !(g.state === "sending" ||
                           g.state === "executing");
    } else {
      activeGoal = null;
      elGoalChip.hidden = true;
      elGoalRow.hidden = true;
    }
    const ex = document.getElementById("btn-explore");
    ex.textContent = "自动探索：关";
    ex.classList.remove("active");
    if ((goalMode) &&
        !(status.mode === "navigating" || status.mode === "fusion")) {
      goalMode = false;
      document.getElementById("btn-goal").classList.remove("active");
      cv.style.cursor = "";
      clearPending();
    }
    dirty = true;
  }
```

- [ ] **Step 9: 事件绑定与启动**

在操作面板绑定区增加：

```js
  document.getElementById("btn-fusion").onclick = () => doMode("fusion");
  document.getElementById("btn-explore").onclick = async () => {
    if (status.mode !== "fusion") {
      toast("先开始边建边航", true);
      return;
    }
    const on = !(status.fusion && status.fusion.explore);
    try {
      const j = await api("/api/explore", { enabled: on });
      toast(j.message || "已切换");
    } catch (e) {
      toast(e.message, true);
    }
  };
  document.getElementById("btn-boundary").onclick =
    () => setBoundaryMode(!boundaryMode);
  document.getElementById("btn-clear-boundary").onclick = clearBoundary;
  document.getElementById("boundary-apply").onclick = applyBoundary;
  document.getElementById("boundary-cancel").onclick =
    () => setBoundaryMode(false);
```

`setControlEnabled` 的 id 列表增加：

```js
    for (const id of ["btn-slam", "btn-stop", "btn-nav", "btn-fusion",
                      "btn-explore", "btn-boundary", "btn-clear-boundary",
                      "btn-save", "btn-goal", "btn-cancel-goal"]) {
```

启动区增加 `loadBoundary();`（放在 `loadMaps();` 之后）。

- [ ] **Step 10: 运行页面测试**

```bash
cd pi/ros2_lidar/upper && python -m pytest tests/test_index.py -v
```

Expected: 2 个测试 PASS。

- [ ] **Step 11: 演示模式肉眼检查**

```bash
# Pi 上（或没有 ROS 的电脑）
python3 pi/ros2_lidar/upper/server.py --demo --port 8851
```

浏览器开 `http://127.0.0.1:8851`：确认"边建边航""自动探索""画边界线"
可见；画线模式下点击能加检查点；自动探索按钮在非融合模式提示先启动
（demo 模式不可操作，属预期）。

- [ ] **Step 12: 提交**

```bash
git add pi/ros2_lidar/upper/index.html pi/ros2_lidar/upper/tests/test_index.py
git commit -m "feat(upper): fusion mode, staged leg, explore toggle and boundary drawing"
```

---

### Task 9: M0 + M1 实车验收

**前置：** Pi 上电、场地清空、电池 ≥ 12.3 V、雷达/底盘串口灯正常。
**不在这一步做探索和边界。**

- [ ] **Step 1: 同步代码**

```bash
cd ~/ROS-car && git pull --ff-only
git log --oneline -3
bash ~/ROS-car/pi/ros2_lidar/upper/install.sh   # 已装可跳过
```

- [ ] **Step 2: 确认外部模式没在跑**

```bash
sudo systemctl stop slam nav
systemctl is-active slam nav || true
```

Expected：都 `inactive`。

- [ ] **Step 3: 起 upper**

```bash
sudo systemctl restart upper
curl -s http://127.0.0.1:8850/api/health
```

Expected：`ros: "ok"`，`mode: "idle"`。

- [ ] **Step 4: 页面起"边建边航"**

浏览器 `http://<Pi>:8850` → 点"边建边航"。观察：

```bash
ros2 topic hz /scan_filtered        # ~7 Hz
ros2 topic hz /map                  # ~1 Hz
ros2 run tf2_ros tf2_echo map base_footprint
ros2 node list | grep -E "slam|fusion_nav|controller"
```

Expected：slam_toolbox、fusion_nav、controller_server 都在；`/map` 持续增长。
页面状态显示"建图+导航中"。

- [ ] **Step 5: 手动去 3 个已知 free 点**

在页面上点"点图发目标"，依次点 3 个已经建出来的空地（离墙 ≥ 0.6 m），
每个点等到达或超时。记录：

- 是否到达、停车误差（目测/卷尺）；
- 页面是否显示"分步推进中"或直接到达；
- `/fusion/status` 的 `legs`、`reason`；
- 倒车段方向是否正确（特别是有没有"倒车但转向反了"）。

Expected：3/3 到达；没有原地转；`reason` 为空或 `arrived`。

- [ ] **Step 6: 点一个未知区**

在页面点地图上灰色未知区（距离 1~3 m）。Expected：

- 机器人先走向已知 free 的中间目标，地图跟着长；
- 页面画橙色虚线中间腿；
- 到中间点后重新选腿，直到到达或 `blocked`；
- 不允许出现"直线穿墙"的规划。

记录 `legs` 和总耗时。若第一次就 `no_reachable_frontier`，检查点击点是否
离已知 free 边缘超过 3 m（`max_leg` 默认 3 m；这是预期行为，不是 bug）。

- [ ] **Step 7: 存图并检查位姿文件**

页面上"保存地图"，名字用 `fusion_YYYYMMDD`。Expected：

```bash
ls -l ~/ros2_lidar/maps/fusion_*.yaml ~/ros2_lidar/maps/fusion_*_last_pose.json
cat ~/ros2_lidar/maps/fusion_*_last_pose.json   # {"x":..,"y":..,"yaw":..}
```

- [ ] **Step 8: 回归旧导航**

停止融合 → 选一张旧图 → 开始导航，然后：

```bash
python3 ~/ros2_lidar/tools/nav_accept.py --rounds 20
```

Expected：`rounds_succeeded >= 18/20`、`in_place_rotation_samples == 0`、
`stop_error_max_m <= 0.10`。若回归失败，先别继续 M2，回到参数集检查。

- [ ] **Step 9: 记录结果**

把 Step 5–8 的数字写进 `docs/plans/2026-11-25-fusion-m0-m1-results.md`
（新建），至少包含：日期、地图名、3 个目标是否到达、未知区推进腿数、
存图位姿、20 次回归四项指标。

---

### Task 10: M2 自主探索实车 + M3 边界线实车

**前置：** Task 9 全绿。

- [ ] **Step 1: 起融合，确认在已知区域**

页面"边建边航"，等 `/map` 稳定、位姿正常。

- [ ] **Step 2: 开自动探索**

点"自动探索：关"→"开"。观察 10 分钟，记录：

- `covered_m2` 增长曲线（每 30 s 记一次）；
- `frontiers` 数量变化；
- 卡住次数（`state=blocked` 或雷达停车）；
- 假前沿（明显是玻璃/反光/12 m 量程边缘的目标）次数；
- 是否出现原地转（应始终为 0）。

Expected：能自行走完 27×7 m 场地的主要区域；结束原因应为
`no_frontier`、`area_stalled` 或人工关闭之一，不能无限跑。

- [ ] **Step 3: 人工打断**

探索中在页面点一个已知 free 点。Expected：当前探索腿被取消，机器人去
人点目标；到达后（`explore` 仍为开）恢复探索。记录恢复是否正常。

- [ ] **Step 4: 画边界线**

页面"画边界线" → 在走廊中间横着点两个点（线要横跨走廊、两端都碰到墙
或超出可通行区）→ "应用"。Expected：

- 页面显示红色虚线；
- 探索不再越过线；
- 在线外点目标被拒，页面显示 `outside_boundary`；
- 线内点目标正常。

- [ ] **Step 5: 清除边界**

点"清除边界"，确认探索恢复可越线（先别真的让它越，观察 `frontiers`
是否重新包含线外簇即可）。

- [ ] **Step 6: 记录结果并更新文档**

新建 `docs/plans/2026-11-25-fusion-m2-m3-results.md`，记录覆盖率、耗时、
里程、卡住次数、假前沿、边界线测试结果。更新：

- `pi/ros2_lidar/NAV.md`：加"融合模式（边建边航）"一节，写启动方式、
  三个模式互斥、`fusion_nav` 的话题/服务、验收数字。
- `pi/ros2_lidar/upper/README.md`：加第三模式、自动探索、画边界线的
  页面操作和排障。
- `docs/plans/2026-11-25-fusion-exploration-design.md` 顶部状态从
  "未实施"改成实际状态。

- [ ] **Step 7: 提交**

```bash
git add pi/ros2_lidar/NAV.md pi/ros2_lidar/upper/README.md \
        docs/plans/2026-11-25-fusion-exploration-design.md \
        docs/plans/2026-11-25-fusion-m0-m1-results.md \
        docs/plans/2026-11-25-fusion-m2-m3-results.md
git commit -m "docs(fusion): on-car acceptance results and runbook"
```

---

## 自检

**Spec coverage：**

| 设计条目 | 任务 |
|---|---|
| D1 真融合 launch | Task 6 |
| D2 分步追目标、不用 allow_unknown | Task 2、5 |
| D3 原地转先硬禁、倒车 | Task 6 参数不动；Task 9 Step 5 实测 |
| D4 边界 L1 进主线、L2 以后 | Task 1、7、8、10 |
| D5 全局去掉 obstacle_layer | Task 6 |
| D6 存图写位姿 | Task 7 |
| D7 独立 fusion_nav 节点 | Task 5 |
| D8 长走廊结束条件 | Task 5 `plan_tick` 的时间/里程/面积停滞 |
| D9 M1 验收点未知区 | Task 9 Step 6 |
| D10 工期翻倍 | 计划按 M0–M3 分阶段，不压时间 |
| 地图 5 s → 1 s | Task 6 |
| 回环失败重试 | Task 4 `on_leg_failed`、Task 5 |
| 低电/返航 | 本计划不含（M4） |
| L2 KeepoutFilter | 本计划不含（M4） |

**已知盲区（实车看，不靠推断）：**

- `StaticLayer` 是否随 slam_toolbox 的 `/map` 全量更新：Task 9 Step 4
  用 `ros2 topic hz /map` + 页面地图增长验证；若全局代价图不更新，
  停下来记录，不在本计划里静默绕。
- 局部图 4×4 是否够：Task 9 Step 5 看是否频繁 `blocked`。
- `fusion_nav` 在 640k 格地图上的 `find_frontiers` 耗时：Task 10 Step 2
  看 `plan_tick` 是否积压（日志/CPU）。
- 倒车转弯方向：Task 9 Step 5 专门记录。
