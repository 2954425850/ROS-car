#!/usr/bin/env python3
"""建图/导航可视化入口（临时件，第一层：只读）。

一个进程里两件事：
  * 后台线程：rclpy 节点订阅 /map、/scan_filtered，查 TF map->base_footprint
  * 主线程：stdlib ThreadingHTTPServer，提供页面 + SSE 推送

只读。不启动/停止任何 ROS 进程，不写任何文件。
删除：rm -rf pi/ros2_lidar/upper（Pi 上没有 systemd 单元、sudoers、symlink）。

    bash upper/run.sh                 # 连 ROS
    python3 upper/server.py --demo    # 不连 ROS，合成数据自检页面
"""
import base64
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from encode import occupancy_payload, pack_points, scan_to_points, yaw_from_quat

DEFAULT_PORT = 8850


class State:
    """ROS 线程写、HTTP 线程读。所有字段在锁内读写。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._map = None
        self._scan = None
        self._pose = None
        self._status = {"mode": "unknown", "tf_ok": False, "ros": "starting",
                        "note": ""}
        self.map_v = 0
        self.scan_v = 0
        self.pose_v = 0
        self.clients = 0

    def set_map(self, payload):
        with self._lock:
            self._map = payload
            self.map_v += 1

    def set_scan(self, payload):
        with self._lock:
            self._scan = payload
            self.scan_v += 1

    def set_pose(self, x, y, yaw):
        with self._lock:
            self._pose = {"x": x, "y": y, "yaw": yaw}
            self.pose_v += 1

    def set_status(self, **kw):
        with self._lock:
            self._status.update(kw)

    def snapshot(self):
        with self._lock:
            return {"map": self._map, "scan": self._scan, "pose": self._pose,
                    "status": dict(self._status),
                    "map_v": self.map_v, "scan_v": self.scan_v,
                    "pose_v": self.pose_v, "clients": self.clients}


def sse_event(name, obj):
    data = json.dumps(obj, separators=(",", ":"), ensure_ascii=False)
    return ("event: %s\ndata: %s\n\n" % (name, data)).encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    def _json(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        state = self.server.state
        if path == "/":
            with open(self.server.index_path, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)
        elif path == "/api/health":
            snap = state.snapshot()
            m = snap["map"]
            self._json(200, {
                "ros": snap["status"].get("ros"),
                "mode": snap["status"].get("mode"),
                "tf_ok": snap["status"].get("tf_ok"),
                "note": snap["status"].get("note", ""),
                "map": None if m is None else {"w": m["w"], "h": m["h"]},
                "scan_points": snap["scan"]["n"] if snap["scan"] else 0,
                "clients": state.clients,
            })
        elif path == "/events":
            self._sse()
        elif path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
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
                if snap["map_v"] != last["map"] and snap["map"] is not None:
                    last["map"] = snap["map_v"]
                    self._chunk(sse_event("map", snap["map"]))
                if snap["scan_v"] != last["scan"] and snap["scan"] is not None:
                    last["scan"] = snap["scan_v"]
                    self._chunk(sse_event("scan", snap["scan"]))
                if snap["pose_v"] != last["pose"] and snap["pose"] is not None:
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
                        "clients": state.clients,
                    }))
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            state.clients -= 1
            self.close_connection = True


def build_server(host, port, state, index_path):
    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    srv.state = state
    srv.index_path = index_path
    return srv


# ---------------------------------------------------------------------------
# ROS 数据源
# ---------------------------------------------------------------------------

def detect_mode():
    """只读探测当前跑的是建图还是导航（外部 systemd 起的也算）。"""
    import subprocess
    for pat, name in (("slam_toolbox", "mapping"),
                      ("nav2_controller", "navigating")):
        try:
            if subprocess.run(["pgrep", "-f", pat],
                              capture_output=True).returncode == 0:
                return name
        except OSError:
            break
    return "idle"


def _mode_loop(state, stop):
    while not stop.is_set():
        mode = detect_mode()
        if state.snapshot()["status"].get("mode") != mode:
            state.set_status(mode=mode)
        stop.wait(2.0)


class RosSource:
    def __init__(self, state):
        self.state = state
        self._stop = threading.Event()
        self._node = None

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
        except Exception as e:  # 没 source ROS 环境
            self.state.set_status(ros="unavailable",
                                  note="ROS 环境不可用：%s" % e)
            return False

        state = self.state

        rclpy.init(args=[])

        class _Node(Node):
            def __init__(self):
                super().__init__("upper_visual")
                self.buf = Buffer()
                self.listener = TransformListener(self.buf, self)
                self.pose = None
                self._tf_ok = False
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
                state.set_map(occupancy_payload(msg))

            def _on_scan(self, msg):
                if self.pose is None:
                    return
                x, y, yaw = self.pose
                pts = scan_to_points(msg.ranges, msg.angle_min,
                                     msg.angle_increment, msg.range_min,
                                     msg.range_max, x, y, yaw, step=1)
                state.set_scan({"n": len(pts), "pts": pack_points(pts)})

            def _tick(self):
                try:
                    tf = self.buf.lookup_transform("map", "base_footprint",
                                                   rclpy.time.Time())
                except Exception:
                    if self._tf_ok:
                        self._tf_ok = False
                        state.set_status(tf_ok=False, note="等待 map→base_footprint")
                    return
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
                self.pose = (t.x, t.y, yaw)
                state.set_pose(t.x, t.y, yaw)
                if not self._tf_ok:
                    self._tf_ok = True
                    state.set_status(tf_ok=True, note="")

        node = _Node()
        self._node = node
        state.set_status(ros="ok")
        threading.Thread(target=self._spin, args=(node, rclpy),
                         daemon=True, name="upper-ros").start()
        return True

    def _spin(self, node, rclpy):
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        while not self._stop.is_set():
            try:
                ex.spin_once(timeout_sec=0.1)
            except Exception:
                time.sleep(0.1)


# ---------------------------------------------------------------------------
# --demo：不连 ROS 的合成数据，用来在 Pi 关机时验收页面
# ---------------------------------------------------------------------------

DEMO_RES = 0.05
DEMO_OX, DEMO_OY = -6.0, -5.0
DEMO_W, DEMO_H = 240, 200          # 12 m x 10 m（含四周未知区）
ROOM_X, ROOM_Y = 4.0, 3.0          # 房间半宽 / 半高，保证任意射线 <= 12 m
PILLARS = ((-1.2, 0.8, 0.15), (1.2, -0.8, 0.15))


def _demo_map():
    data = []
    for gy in range(DEMO_H):
        cy = DEMO_OY + (gy + 0.5) * DEMO_RES
        for gx in range(DEMO_W):
            cx = DEMO_OX + (gx + 0.5) * DEMO_RES
            if abs(cx) > ROOM_X + DEMO_RES or abs(cy) > ROOM_Y + DEMO_RES:
                data.append(-1)
            elif abs(cx) >= ROOM_X or abs(cy) >= ROOM_Y:
                data.append(100)
            elif any((cx - px) ** 2 + (cy - py) ** 2 <= pr * pr
                     for px, py, pr in PILLARS):
                data.append(100)
            else:
                data.append(0)
    return data


def _ray_wall(ox, oy, a):
    """返回射线到房间四壁的最近正距离。"""
    best = float("inf")
    c, s = math.cos(a), math.sin(a)
    if abs(c) > 1e-9:
        for wx in (ROOM_X, -ROOM_X):
            t = (wx - ox) / c
            if t > 0 and abs(oy + t * s) <= ROOM_Y:
                best = min(best, t)
    if abs(s) > 1e-9:
        for wy in (ROOM_Y, -ROOM_Y):
            t = (wy - oy) / s
            if t > 0 and abs(ox + t * c) <= ROOM_X:
                best = min(best, t)
    for px, py, pr in PILLARS:
        dx, dy = ox - px, oy - py
        b = dx * c + dy * s
        disc = b * b - (dx * dx + dy * dy - pr * pr)
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if t > 0:
                best = min(best, t)
    return best


class DemoSource:
    def __init__(self, state):
        self.state = state
        self._stop = threading.Event()

    def start(self):
        state = self.state
        grid = _demo_map()
        state.set_status(ros="demo", mode="demo", tf_ok=True, note="合成数据")
        state.set_map({
            "w": DEMO_W, "h": DEMO_H, "res": DEMO_RES,
            "ox": DEMO_OX, "oy": DEMO_OY,
            "data": base64.b64encode(
                bytes(v & 0xFF for v in grid)).decode("ascii"),
        })
        threading.Thread(target=self._loop, daemon=True, name="upper-demo").start()
        return True

    def _pose_at(self, t):
        # 椭圆轨迹：x 半轴 1.6 m、y 半轴 1.0 m
        x = 1.6 * math.cos(0.25 * t)
        y = 1.0 * math.sin(0.25 * t)
        yaw = math.atan2(1.0 * 0.25 * math.cos(0.25 * t),
                         -1.6 * 0.25 * math.sin(0.25 * t))
        return x, y, yaw

    def _loop(self):
        state = self.state
        t0 = time.monotonic()
        while not self._stop.is_set():
            t = time.monotonic() - t0
            x, y, yaw = self._pose_at(t)
            state.set_pose(x, y, yaw)
            state.set_status(tf_ok=True, mode="demo")

            from encode import laser_origin
            ox, oy = laser_origin(x, y, yaw)
            n = 360
            step = 2 * math.pi / n
            ranges = [_ray_wall(ox, oy, yaw + i * step) for i in range(n)]
            pts = scan_to_points(ranges, 0.0, step, 0.15, 12.0, x, y, yaw)
            state.set_scan({"n": len(pts), "pts": pack_points(pts)})
            time.sleep(0.14)


# ---------------------------------------------------------------------------

def main(argv=None):
    import argparse
    import os

    parser = argparse.ArgumentParser(description="建图/导航可视化（只读）")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--demo", action="store_true",
                        help="不连 ROS，用合成数据自检页面")
    args = parser.parse_args(argv)

    state = State()
    stop = threading.Event()
    if args.demo:
        DemoSource(state).start()
    else:
        source = RosSource(state)
        if source.start():
            threading.Thread(target=_mode_loop, args=(state, stop),
                             daemon=True, name="upper-mode").start()

    here = os.path.dirname(os.path.abspath(__file__))
    srv = build_server(args.host, args.port, state,
                       os.path.join(here, "index.html"))
    print("upper: http://%s:%d" % (args.host, args.port), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        srv.shutdown()


if __name__ == "__main__":
    main()
