# 建图/导航可视化入口（第一层：纯看）实现计划

> **For agentic workers:** 按任务顺序执行，每个任务结束时运行测试并提交。
> 设计依据：`docs/plans/2026-11-24-upper-visualization-design.md`。

**Goal:** 在 Pi 上手动起一个网页服务，浏览器实时看到 `/map`、`/scan_filtered`、`map→base_footprint` 位姿；只读，不启停任何进程。

**Architecture:** 一个进程 = 后台线程里的 rclpy 节点（订阅 + TF）+ 主线程里的 stdlib `ThreadingHTTPServer`（静态页 + SSE）。节点把最新帧写进线程安全的 `State`，HTTP 每 50 ms 检查版本号并通过 SSE 推给浏览器。纯函数（编码/坐标变换）单独放 `encode.py`，不 import ROS，可在 Windows 上直接测。

**Tech Stack:** Python 3.11 stdlib（`http.server`、`base64`、`struct`）、rclpy（仅服务端）、原生 JS + canvas（页面零依赖）、pytest。

## Global Constraints

- 新增代码**只**放在 `pi/ros2_lidar/upper/`；不新增 systemd 单元、sudoers、symlink。
- 不修改 `slam.service`、`nav.service`、`rosbridge.service`、`slam_params.yaml`、`nav2_params.yaml`。
- 第一层**不允许**启动/停止任何 ROS 进程；`server.py` 里不得出现 `subprocess` 启停 launch。
- 端口 8850；监听 `0.0.0.0`。
- 页面不得引用任何外部 URL（无 CDN）。
- 所有面向用户的文案用中文。
- 删除方式必须保持为 `rm -rf pi/ros2_lidar/upper`。

---

## 文件结构

| 文件 | 职责 |
|---|---|
| `pi/ros2_lidar/upper/encode.py` | 纯函数：OccupancyGrid→payload、四元数→yaw、雷达点→map 系、打包 |
| `pi/ros2_lidar/upper/server.py` | `State`、HTTP 路由、SSE、`RosSource`、`DemoSource`、`main()` |
| `pi/ros2_lidar/upper/index.html` | 页面（canvas 渲染 + SSE 客户端） |
| `pi/ros2_lidar/upper/run.sh` | source ROS 后启动 server |
| `pi/ros2_lidar/upper/README.md` | 起停、排障、删除 |
| `pi/ros2_lidar/upper/tests/test_encode.py` | 纯函数测试 |
| `pi/ros2_lidar/upper/tests/test_server.py` | HTTP/SSE 测试（假 State，不需要 ROS） |

---

### Task 1: 纯函数层 `encode.py`

**Files:**
- Create: `pi/ros2_lidar/upper/encode.py`
- Test: `pi/ros2_lidar/upper/tests/test_encode.py`

**Interfaces:**
- Produces:
  - `occupancy_payload(msg) -> dict`，键 `w,h,res,ox,oy,data`（`data` 为 base64 的 int8 原始字节，`-1` 存成 `0xFF`）
  - `yaw_from_quat(x, y, z, w) -> float`
  - `laser_origin(tx, ty, yaw) -> (float, float)`，用常量 `LASER_X=-0.256, LASER_Y=0.0065`
  - `scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, tx, ty, yaw, step=1) -> list[(x,y)]`
  - `pack_points(points) -> str`（`<ff` 小端 float32 点对，base64）
  - `b64_bytes(raw: bytes) -> str`

- [ ] **Step 1: 写失败测试**

```python
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


def test_pack_points_roundtrip():
    s = pack_points([(1.5, -2.0), (0.0, 3.0)])
    raw = base64.b64decode(s)
    assert len(raw) == 16
    assert struct.unpack("<ff", raw[:8]) == (1.5, -2.0)
```

- [ ] **Step 2: 运行确认失败**

Run: `python -m pytest pi/ros2_lidar/upper/tests/test_encode.py -q`
Expected: FAIL（`ModuleNotFoundError: No module named 'encode'`）

- [ ] **Step 3: 实现**

```python
"""纯函数：不 import rclpy，Windows 上可直接测。"""
import base64
import math
import struct

# urdf/robot.urdf: laser 在 base_link 后方 25.6 cm、左 0.65 cm
LASER_X = -0.256
LASER_Y = 0.0065


def b64_bytes(raw):
    return base64.b64encode(raw).decode("ascii")


def occupancy_payload(msg):
    info = msg.info
    raw = bytes((int(v) & 0xFF) for v in msg.data)
    return {
        "w": int(info.width),
        "h": int(info.height),
        "res": float(info.resolution),
        "ox": float(info.origin.position.x),
        "oy": float(info.origin.position.y),
        "data": b64_bytes(raw),
    }


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def laser_origin(tx, ty, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * LASER_X - s * LASER_Y, ty + s * LASER_X + c * LASER_Y


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max,
                   tx, ty, yaw, step=1):
    ox, oy = laser_origin(tx, ty, yaw)
    out = []
    for i in range(0, len(ranges), step):
        r = ranges[i]
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = yaw + angle_min + i * angle_increment
        out.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    return out


def pack_points(points):
    buf = bytearray()
    for x, y in points:
        buf += struct.pack("<ff", x, y)
    return b64_bytes(bytes(buf))
```

- [ ] **Step 4: 运行测试**

Run: `python -m pytest pi/ros2_lidar/upper/tests/test_encode.py -q`
Expected: 5 passed

- [ ] **Step 5: 提交**

```bash
git add pi/ros2_lidar/upper/encode.py pi/ros2_lidar/upper/tests/test_encode.py
git commit -m "feat(upper): 可视化用纯函数层（编码/TF 投影）"
```

---

### Task 2: HTTP + SSE 服务（假 State，不依赖 ROS）

**Files:**
- Create: `pi/ros2_lidar/upper/server.py`（本任务只写 `State` / `build_server` / `main` 的 HTTP 部分）
- Test: `pi/ros2_lidar/upper/tests/test_server.py`

**Interfaces:**
- Consumes: `encode.py` 的同目录导入
- Produces:
  - `State`：`set_map(payload)`、`set_scan(payload)`、`set_pose(x, y, yaw)`、`set_status(**kw)`、`snapshot() -> dict`（含 `map_v`/`scan_v`/`pose_v` 版本号）
  - `build_server(host, port, state, index_path) -> ThreadingHTTPServer`
  - `sse_event(name, obj) -> bytes`
  - HTTP：`GET /`、`GET /events`、`GET /api/health`，其余 404

- [ ] **Step 1: 写失败测试**

```python
import json
import threading
import time
import urllib.request

from server import State, build_server


def _serve(tmp_path, state):
    index = tmp_path / "index.html"
    index.write_text("<html>ok</html>", encoding="utf-8")
    srv = build_server("127.0.0.1", 0, state, str(index))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, "http://127.0.0.1:%d" % srv.server_address[1]


def test_index_and_health(tmp_path):
    state = State()
    state.set_status(mode="mapping", tf_ok=True, ros="ok")
    srv, base = _serve(tmp_path, state)
    try:
        assert urllib.request.urlopen(base + "/").read() == b"<html>ok</html>"
        body = json.loads(urllib.request.urlopen(base + "/api/health").read())
        assert body["mode"] == "mapping"
        assert body["tf_ok"] is True
        assert body["map"] is None
    finally:
        srv.shutdown()


def test_unknown_path_is_404(tmp_path):
    srv, base = _serve(tmp_path, State())
    try:
        try:
            urllib.request.urlopen(base + "/nope")
            assert False, "expected 404"
        except urllib.error.HTTPError as e:
            assert e.code == 404
    finally:
        srv.shutdown()


def test_sse_pushes_status_and_map(tmp_path):
    state = State()
    state.set_map({"w": 2, "h": 1, "res": 0.05, "ox": 0.0, "oy": 0.0,
                   "data": "AAA="})
    srv, base = _serve(tmp_path, state)
    try:
        resp = urllib.request.urlopen(base + "/events", timeout=5)
        assert resp.headers["Content-Type"].startswith("text/event-stream")
        buf = b""
        deadline = time.time() + 3
        while time.time() < deadline and b"event: map" not in buf:
            buf += resp.read(1)
        assert b"event: map" in buf
    finally:
        srv.shutdown()
```

- [ ] **Step 2: 运行确认失败**

Run: `python -m pytest pi/ros2_lidar/upper/tests/test_server.py -q`
Expected: FAIL（`ImportError`）

- [ ] **Step 3: 实现 `State` + HTTP**

```python
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class State:
    def __init__(self):
        self._lock = threading.Lock()
        self._map = None
        self._scan = None
        self._pose = None
        self._status = {"mode": "unknown", "tf_ok": False, "ros": "starting",
                        "note": ""}
        self._v = 0
        self.clients = 0

    def set_map(self, payload):
        with self._lock:
            self._map = payload
            self._v += 1

    def set_scan(self, payload):
        with self._lock:
            self._scan = payload
            self._v += 1

    def set_pose(self, x, y, yaw):
        with self._lock:
            self._pose = {"x": x, "y": y, "yaw": yaw}
            self._v += 1

    def set_status(self, **kw):
        with self._lock:
            self._status.update(kw)

    def snapshot(self):
        with self._lock:
            return {"map": self._map, "scan": self._scan, "pose": self._pose,
                    "status": dict(self._status), "v": self._v,
                    "clients": self.clients}
```

**注意**：`set_map`/`set_scan`/`set_pose` 共用一个 `v` 会导致「地图变了但页面以为没变」时重复推？不会——重复推是安全的，漏推才危险。但 SSE 每帧应记各自的版本；实现里用 `snapshot()` 返回的对象计数：`map_v` 等改为在 snapshot 里按字段身份计数不可行。**采用**：分别为 `map_v/scan_v/pose_v`：

```python
    def set_map(self, payload):
        with self._lock:
            self._map, self.map_v = payload, self.map_v + 1
```

`State.__init__` 里初始化 `self.map_v = self.scan_v = self.pose_v = 0`，`snapshot()` 返回这三个版本号。

SSE 帧格式：

```python
def sse_event(name, obj):
    data = json.dumps(obj, separators=(",", ":"), ensure_ascii=False)
    return ("event: %s\ndata: %s\n\n" % (name, data)).encode("utf-8")
```

处理器（关键部分）：

```python
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    def _json(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        state = self.server.state
        if path == "/":
            body = open(self.server.index_path, "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/api/health":
            snap = state.snapshot()
            m = snap["map"]
            self._json(200, {
                "ros": snap["status"].get("ros"), "mode": snap["status"].get("mode"),
                "tf_ok": snap["status"].get("tf_ok"),
                "note": snap["status"].get("note", ""),
                "map": None if m is None else {"w": m["w"], "h": m["h"]},
                "scan_points": snap["scan"]["n"] if snap["scan"] else 0,
                "clients": state.clients})
        elif path == "/events":
            self._sse()
        else:
            self._json(404, {"error": "not found"})

    def _chunk(self, data):
        self.wfile.write(b"%x\r\n" % len(data) + data + b"\r\n")
        self.wfile.flush()

    def _sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Transfer-Encoding", "chunked")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        state = self.server.state
        state.clients += 1
        last = {"map": -1, "scan": -1, "pose": -1}
        last_status = 0.0
        try:
            while True:
                snap = state.snapshot()
                if snap["map_v"] != last["map"]:
                    last["map"] = snap["map_v"]
                    self._chunk(sse_event("map", snap["map"]))
                if snap["scan_v"] != last["scan"]:
                    last["scan"] = snap["scan_v"]
                    self._chunk(sse_event("scan", snap["scan"]))
                if snap["pose_v"] != last["pose"]:
                    last["pose"] = snap["pose_v"]
                    self._chunk(sse_event("pose", snap["pose"]))
                now = time.time()
                if now - last_status >= 1.0:
                    last_status = now
                    m = snap["map"]
                    self._chunk(sse_event("status", {
                        "mode": snap["status"].get("mode"),
                        "tf_ok": snap["status"].get("tf_ok"),
                        "ros": snap["status"].get("ros"),
                        "note": snap["status"].get("note", ""),
                        "map_w": m["w"] if m else 0,
                        "map_h": m["h"] if m else 0,
                        "map_res": m["res"] if m else 0,
                        "scan_n": snap["scan"]["n"] if snap["scan"] else 0,
                        "clients": state.clients}))
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            state.clients -= 1
            try:
                self.close_connection = True
            except Exception:
                pass


def build_server(host, port, state, index_path):
    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    srv.state = state
    srv.index_path = index_path
    return srv
```

- [ ] **Step 4: 运行测试**

Run: `python -m pytest pi/ros2_lidar/upper/tests/test_server.py -q`
Expected: 3 passed（SSE 测试若因缓冲卡住，把 `resp.read(1)` 改成 `resp.readline()` 逐行读到空行）

- [ ] **Step 5: 提交**

```bash
git add pi/ros2_lidar/upper/server.py pi/ros2_lidar/upper/tests/test_server.py
git commit -m "feat(upper): HTTP + SSE 推送服务（可脱离 ROS 测试）"
```

---

### Task 3: ROS 数据源 `RosSource`

**Files:**
- Modify: `pi/ros2_lidar/upper/server.py`（追加 `RosSource`、`detect_mode`、`main`）

**Interfaces:**
- Consumes: `State`、`encode.*`
- Produces: `RosSource(state).start() -> bool`；后台线程；失败时 `state.set_status(ros="unavailable", note=...)`，HTTP 仍可服务

- [ ] **Step 1: 实现**

要点（完整代码写进 `server.py`）：

```python
class RosSource:
    def __init__(self, state):
        self.state = state
        self._stop = threading.Event()

    def start(self):
        try:
            import rclpy
            from nav_msgs.msg import OccupancyGrid
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                                   ReliabilityPolicy)
            from sensor_msgs.msg import LaserScan
            from tf2_ros import Buffer, TransformListener
        except Exception as e:          # 没 source ROS 环境
            self.state.set_status(ros="unavailable", note=str(e))
            return False

        rclpy.init(args=[])
        state = self.state

        class _Node(Node):
            def __init__(self):
                super().__init__("upper_visual")
                self.buf = Buffer()
                self.listener = TransformListener(self.buf, self)
                self.pose = None
                map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                     reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                scan_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                                      reliability=ReliabilityPolicy.BEST_EFFORT)
                self.create_subscription(OccupancyGrid, "/map", self._on_map,
                                         map_qos)
                self.create_subscription(LaserScan, "/scan_filtered",
                                         self._on_scan, scan_qos)
                self.create_timer(0.1, self._tick)

            def _on_map(self, msg):
                from encode import occupancy_payload
                state.set_map(occupancy_payload(msg))

            def _on_scan(self, msg):
                from encode import scan_to_points, pack_points
                if self.pose is None:
                    return
                x, y, yaw = self.pose
                pts = scan_to_points(msg.ranges, msg.angle_min,
                                     msg.angle_increment, msg.range_min,
                                     msg.range_max, x, y, yaw, step=1)
                state.set_scan({"n": len(pts), "pts": pack_points(pts)})

            def _tick(self):
                from encode import yaw_from_quat
                try:
                    tf = self.buf.lookup_transform("map", "base_footprint",
                                                   rclpy.time.Time())
                except Exception as e:
                    if state.snapshot()["status"].get("tf_ok"):
                        state.set_status(tf_ok=False, note="等待 map→base_footprint")
                    state.set_status(tf_ok=False, note="等待 map→base_footprint")
                    return
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
                self.pose = (t.x, t.y, yaw)
                state.set_pose(t.x, t.y, yaw)
                state.set_status(tf_ok=True, note="")

        node = _Node()
        state.set_status(ros="ok")
        threading.Thread(target=lambda: self._spin(node, rclpy),
                         daemon=True, name="upper-ros").start()
        self._node = node
        return True

    def _spin(self, node, rclpy):
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        while not self._stop.is_set():
            ex.spin_once(timeout_sec=0.1)
```

`detect_mode()`（每 2 s 调一次，只读）：

```python
def detect_mode():
    import subprocess
    for pat, name in (("slam_toolbox", "mapping"),
                      ("nav2_controller", "navigating")):
        if subprocess.run(["pgrep", "-f", pat], capture_output=True).returncode == 0:
            return name
    return "idle"
```

`main()`：

```python
def main():
    import argparse
    import os
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8850)
    parser.add_argument("--demo", action="store_true",
                        help="不连 ROS，用合成数据自检页面")
    args = parser.parse_args()

    state = State()
    if args.demo:
        start_demo(state)
    else:
        source = RosSource(state)
        if source.start():
            threading.Thread(target=_mode_loop, args=(state,),
                             daemon=True).start()

    srv = build_server(args.host, args.port, state,
                       os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "index.html"))
    print("upper: http://%s:%d" % (args.host, args.port), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
```

- [ ] **Step 2: 语法与导入检查（Windows 上无 ROS，只能编到 import 为止）**

Run: `python -m py_compile pi/ros2_lidar/upper/server.py`
Expected: 无输出

- [ ] **Step 3: 提交**

```bash
git add pi/ros2_lidar/upper/server.py
git commit -m "feat(upper): ROS 订阅与 TF 数据源"
```

---

### Task 4: `--demo` 合成数据（Pi 关机时也能验收页面）

**Files:**
- Modify: `pi/ros2_lidar/upper/server.py`

**Interfaces:**
- Produces: `start_demo(state)`；10×8 m 的合成房间，机器人绕圈，雷达射线打在四壁上

- [ ] **Step 1: 实现 `start_demo`**

要点：
- 地图 240×200 @0.05（12×10 m），房间 `x∈[-4,4]`、`y∈[-3,3]`，四周未知；柱子 (±1.2, ∓0.8)。
- 占用数据：四壁、两根柱子为 100；房间内 0；房间外 -1。
- 位姿：椭圆轨迹（半轴 1.6/1.0 m，0.25 rad/s），yaw 指切向。
- 雷达：360 束，从激光原点向各角度发射，与四壁/柱子求交（房间尺寸保证回波 ≤ 12 m），产出 ranges 后走 `scan_to_points`/`pack_points`。
- 直接构造 `SimpleNamespace(info=..., data=...)` 调 `occupancy_payload`。

- [ ] **Step 2: 本地跑起来看**

Run: `python pi/ros2_lidar/upper/server.py --demo --port 8850`
打开 `http://127.0.0.1:8850`，确认：地图画出、雷达点贴墙、箭头绕圈、状态条 1 s 刷新。

- [ ] **Step 3: 提交**

```bash
git add pi/ros2_lidar/upper/server.py
git commit -m "feat(upper): --demo 合成数据自检"
```

---

### Task 5: 页面 `index.html`

**Files:**
- Create: `pi/ros2_lidar/upper/index.html`

**Interfaces:**
- Consumes: SSE 事件 `map`/`scan`/`pose`/`status`（格式见设计文档 §3.3）

页面结构：

```
<header>  左侧标题「建图 / 导航」+ 模式徽标；右侧：连接状态、地图尺寸、缩放
<canvas id="view">  全屏
<footer>  提示：滚轮缩放 · 拖拽平移 · 双击复位
```

关键实现（写进页面）：

```js
function b64Bytes(s) {
  const bin = atob(s), u = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i);
  return u;
}
// map: v = u[i] > 127 ? u[i] - 256 : u[i]；-1 未知，>=50 占用
// 离屏 canvas 尺寸 w×h，行 y 翻转：putImageData 前写 row = h-1-y
// 世界→屏幕（y 向上）：sx = W/2 + (x-camX)*scale; sy = H/2 - (y-camY)*scale
// 地图绘制：左上角 = w2s(ox, oy + h*res)，宽 w*res*scale，高 h*res*scale，
//           imageSmoothingEnabled = false
// 雷达：Float32Array(u.buffer)，两两成对，1–2 px 红点
// 机器人：三角形 + 中线，旋转角 = -yaw
// 交互：wheel 以光标为中心缩放；pointerdown/move 平移；dblclick fit
// fit()：按地图包围盒缩放到 90% 视口并居中
// 状态条：mode → 建图中/导航中/空闲；tf_ok=false 时显示黄色「等待定位」
// EventSource('/events')，onerror 显示「连接断开，正在重连」
```

视觉：深色背景 `#0e1116`；空闲区 `#2a303b`；占用 `#e8ecf2`；未知 `#141820`；雷达 `#ff5f56`；机器人 `#4cc9f0`。文字用系统字体栈，`ui-monospace` 显示数字。

- [ ] **Step 1: 写页面**
- [ ] **Step 2: 用 demo 服务实看**

Run: `python pi/ros2_lidar/upper/server.py --demo`
用浏览器截图检查（Playwright 可用）：地图/雷达/箭头/状态条齐全，缩放拖动正常，无控制台报错。

- [ ] **Step 3: 提交**

```bash
git add pi/ros2_lidar/upper/index.html
git commit -m "feat(upper): 单页 canvas 建图可视化"
```

---

### Task 6: 启动脚本与说明

**Files:**
- Create: `pi/ros2_lidar/upper/run.sh`
- Create: `pi/ros2_lidar/upper/README.md`

`run.sh`：

```bash
#!/usr/bin/env bash
# 建图/导航可视化（临时件）。删除：rm -rf ~/ROS-car/pi/ros2_lidar/upper
set -e
source /opt/ros/jazzy/setup.bash
exec python3 "$(dirname "$0")/server.py" "$@"
```

`README.md` 写明：起停、端口、模式判定是只读的、如何删除、`--demo`、常见问题
（TF 一直不出现 = 没起 slam/nav；`ros: unavailable` = 没用 run.sh 起）。

- [ ] **Step 1: 写文件并 `chmod +x run.sh`**
- [ ] **Step 2: 提交**

```bash
git add pi/ros2_lidar/upper/run.sh pi/ros2_lidar/upper/README.md
git commit -m "chore(upper): 启动脚本与使用说明"
```

---

### Task 7: Pi 实车验收（**需要上电**）

- [ ] `cd ~/ROS-car && git pull`（或 `bash ~/ROS-car/pi/ros2_lidar/deploy_nav.sh` 同步到 `~/ros2_lidar`）
- [ ] `bash ~/ROS-car/pi/ros2_lidar/upper/run.sh`；笔记本打开 `http://10.64.35.225:8850`
- [ ] `sudo systemctl start slam`；看地图增长、雷达与墙对齐、箭头朝向
- [ ] 刷新页面：地图立即完整（transient_local）
- [ ] `sudo systemctl stop slam; sudo systemctl start nav`：地图/AMCL 位姿正常
- [ ] `curl -s http://127.0.0.1:8850/api/health`
- [ ] Ctrl-C 退出；确认 `ss -lnt | grep 8850` 为空、无残留 python 进程
- [ ] 对照：不跑本服务时，slam/nav/语音行为不变

---

## Self-Review

- 覆盖设计 §3.1（订阅/TF/模式）、§3.2（HTTP）、§3.3（SSE）、§3.4（页面）、§5（降级：ROS 不可用仍服务、TF 缺失标黄、断线重连）、§6（验收 1–7）。
- 未覆盖：第二层（起停/存图/目标点）——按设计明确不做。
- 类型一致：`State.set_map/set_scan/set_pose`、`snapshot()` 的 `map_v/scan_v/pose_v`、SSE 事件名与页面一致。
