#!/usr/bin/env python3
"""建图/导航可视化入口（临时件，两层）。

一个进程里三件事：
  * 后台线程：rclpy 节点订阅 /map、/scan_filtered，查 TF map->base_footprint
  * 后台线程：ModeController 以子进程组起停 mapping/nav launch，存图、发目标
  * 主线程：stdlib ThreadingHTTPServer，提供页面 + SSE 推送

进程控制只动**自己拉起的子进程**；检测到 systemd 的 slam/nav 在跑就拒绝，
不抢、不 kill 别人的实例。删除：rm -rf pi/ros2_lidar/upper。

    bash upper/run.sh                 # 连 ROS
    python3 upper/server.py --demo    # 不连 ROS，合成数据自检页面（不能操作）
"""
import base64
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from encode import occupancy_payload, pack_points, scan_to_points, yaw_from_quat

DEFAULT_PORT = 8850
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")


class State:
    """ROS 线程写、HTTP 线程读。所有字段在锁内读写。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._map = None
        self._scan = None
        self._pose = None
        self._status = {"mode": "unknown", "tf_ok": False, "ros": "starting",
                        "note": "", "goal": None, "owned": False}
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
                "goal": snap["status"].get("goal"),
                "owned": snap["status"].get("owned", False),
                "control": self.server.controller is not None,
            })
        elif path == "/api/maps":
            ctl = self.server.controller
            if ctl is None:
                self._json(503, {"error": "演示模式不可操作"})
            else:
                self._json(200, {"maps": ctl.list_maps(),
                                 "maps_dir": ctl.maps_dir})
        elif path == "/events":
            self._sse()
        elif path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        ctl = self.server.controller
        if ctl is None:
            self._json(503, {"ok": False, "error": "演示模式不可操作"})
            return
        try:
            n = int(self.headers.get("Content-Length", "0"))
            if n <= 0 or n > 65536:
                raise ValueError("bad length")
            body = json.loads(self.rfile.read(n).decode("utf-8"))
            if not isinstance(body, dict):
                raise ValueError("bad body")
        except Exception:
            self._json(400, {"ok": False, "error": "请求格式不对"})
            return

        if path == "/api/mode":
            mode = body.get("mode")
            if mode == "idle":
                ok, msg = ctl.stop()
            elif mode in ("slam", "nav"):
                ok, msg = ctl.start(mode, body.get("map"))
            else:
                self._json(400, {"ok": False, "error": "mode 只能是 slam/nav/idle"})
                return
        elif path == "/api/map/save":
            name = body.get("name") or ""
            ok, msg = ctl.save_map(name)
        elif path == "/api/goal":
            try:
                x = float(body["x"])
                y = float(body["y"])
                yaw = float(body.get("yaw", 0.0))
            except Exception:
                self._json(400, {"ok": False, "error": "目标点格式不对"})
                return
            ok, msg = ctl.send_goal(x, y, yaw)
        elif path == "/api/goal/cancel":
            ok, msg = ctl.cancel_goal()
        else:
            self._json(404, {"error": "not found"})
            return

        if ok:
            self._json(200, {"ok": True, "message": msg})
        else:
            self._json(409, {"ok": False, "error": msg})

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
                        "goal": snap["status"].get("goal"),
                        "owned": snap["status"].get("owned", False),
                        "control": self.server.controller is not None,
                    }))
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            state.clients -= 1
            self.close_connection = True


def build_server(host, port, state, index_path, controller=None):
    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    srv.state = state
    srv.index_path = index_path
    srv.controller = controller
    return srv


# ---------------------------------------------------------------------------
# 进程控制（第二层）
# ---------------------------------------------------------------------------

class ModeController:
    """起停自己拉起的 launch 子进程；存图；转发导航目标。

    只动自己的进程组。看到 systemd 的 slam/nav（或别处起的实例）就拒绝，
    不抢、不 kill 别人的进程。
    """

    def __init__(self, state, ros, maps_dir, launch_dir):
        self.state = state
        self.ros = ros                # RosSource；没有就发不了目标
        self.maps_dir = maps_dir
        self.launch_dir = launch_dir
        self.tools_dir = os.path.join(os.path.dirname(launch_dir), "tools")
        self.proc = None
        self.mode = None
        self.lock = threading.Lock()
        threading.Thread(target=self._watch, daemon=True,
                         name="upper-watch").start()

    # ---------------------------------------------------------------- 内部
    def _watch(self):
        """子进程自己退出（崩溃/被 kill）时把状态收回 idle。"""
        while True:
            time.sleep(1.0)
            with self.lock:
                if self.proc is not None and self.proc.poll() is not None:
                    code = self.proc.returncode
                    self.proc = None
                    self.mode = None
                    self.state.set_status(mode="idle", owned=False,
                                          goal=None,
                                          note="进程已退出（rc=%s）" % code)

    def _alive(self):
        return self.proc is not None and self.proc.poll() is None

    def _external(self):
        for unit, name in (("slam", "mapping"), ("nav", "navigating")):
            try:
                r = subprocess.run(["systemctl", "is-active", unit],
                                   capture_output=True, text=True, timeout=3)
            except Exception:
                continue
            if r.stdout.strip() == "active":
                return name
        if self.proc is None:
            m = detect_mode()
            if m != "idle":
                return m
        return None

    def _kill(self):
        if self.proc is None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except OSError:
                pass
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        except OSError:
            pass
        self.proc = None
        self.mode = None
        self.state.set_status(owned=False, goal=None)

    # ---------------------------------------------------------------- 调色板
    def list_maps(self):
        out = []
        try:
            for fn in os.listdir(self.maps_dir):
                if not fn.endswith(".yaml"):
                    continue
                p = os.path.join(self.maps_dir, fn)
                try:
                    st = os.stat(p)
                except OSError:
                    continue
                out.append({"name": fn, "size": st.st_size,
                            "mtime": int(st.st_mtime)})
        except OSError:
            pass
        out.sort(key=lambda m: -m["mtime"])
        return out

    def start(self, mode, map_name=None):
        with self.lock:
            if self._alive():
                if self.mode == mode:
                    return True, "已经在运行"
                return False, "另一个模式正在运行，先停止"
            ext = self._external()
            if ext:
                return False, ("检测到外部实例（%s）。先 sudo systemctl stop slam/nav"
                               % ("建图" if ext == "mapping" else "导航"))
            if mode == "nav":
                if (not map_name or not NAME_RE.match(map_name)
                        or not map_name.endswith(".yaml")):
                    return False, "请选择地图（.yaml）"
                if not os.path.isfile(os.path.join(self.maps_dir, map_name)):
                    return False, "地图不存在：%s" % map_name
                launch = os.path.join(self.launch_dir, "nav.launch.py")
                cmd = ["ros2", "launch", launch, "use_composition:=False",
                       "map:=" + os.path.join(self.maps_dir, map_name)]
            else:
                launch = os.path.join(self.launch_dir, "mapping.launch.py")
                cmd = ["ros2", "launch", launch]
            log_path = "/tmp/upper_%s.log" % mode
            try:
                log = open(log_path, "ab")
                self.proc = subprocess.Popen(
                    cmd, stdout=log, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, start_new_session=True)
            except Exception as e:
                self.proc = None
                return False, "启动失败：%s" % e
            time.sleep(2.0)
            if self.proc.poll() is not None:
                self.proc = None
                return False, "启动失败，看 %s" % log_path
            self.mode = mode
            self.state.set_status(
                mode="mapping" if mode == "slam" else "navigating",
                owned=True, goal=None, note="")
            return True, "已启动%s（日志 %s）" % (
                "建图" if mode == "slam" else "导航", log_path)

    def stop(self):
        with self.lock:
            if not self._alive():
                self.proc = None
                self.mode = None
                self.state.set_status(owned=False)
                return True, "没有由本页面启动的进程"
            self._kill()
            return True, "已停止"

    def save_map(self, name):
        name = (name or "").strip()
        for ext in (".yaml", ".pgm", ".png"):
            if name.endswith(ext):
                name = name[:-len(ext)]
        if not NAME_RE.match(name):
            return False, "名字只能用字母/数字/下划线/点/横线（≤64 字）"
        if not (self._alive() and self.mode == "slam") and detect_mode() != "mapping":
            return False, "当前不在建图，没有可保存的地图"
        base = os.path.join(self.maps_dir, name)
        snapshot = os.path.join(self.tools_dir, "save_map_snapshot.py")
        try:
            r = subprocess.run([sys.executable, snapshot, base],
                               capture_output=True, text=True, timeout=25)
        except subprocess.TimeoutExpired:
            return False, "存图超时（没有收到 /map）"
        if r.returncode != 0:
            return False, (r.stderr or r.stdout or "存图失败").strip()[-300:]
        return True, "已保存 %s.yaml" % name

    def send_goal(self, x, y, yaw):
        if not (self._alive() and self.mode == "nav") and detect_mode() != "navigating":
            return False, "当前不在导航模式"
        if self.ros is None:
            return False, "ROS 未就绪"
        return self.ros.send_goal(x, y, yaw)

    def cancel_goal(self):
        if self.ros is None:
            return False, "ROS 未就绪"
        return self.ros.cancel_goal()


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
    """页面自己拉起的进程由 ModeController 负责状态；这里只检测外部实例。"""
    while not stop.is_set():
        st = state.snapshot()["status"]
        if not st.get("owned"):
            mode = detect_mode()
            if st.get("mode") != mode:
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
            try:
                from action_msgs.msg import GoalStatus
                from nav2_msgs.action import NavigateToPose
                from rclpy.action import ActionClient
                have_nav2 = True
            except Exception:
                have_nav2 = False
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
                self._ac = (ActionClient(self, NavigateToPose,
                                         "navigate_to_pose")
                            if have_nav2 else None)
                self._goal_handle = None
                self._goal = None

            # ---- 导航目标 ----
            def _set_goal(self, **kw):
                g = dict(self._goal or {})
                g.update(kw)
                self._goal = g
                state.set_status(goal=dict(g))

            def send_goal(self, x, y, yaw):
                if self._ac is None:
                    return False, "nav2_msgs 不可用"
                if not self._ac.wait_for_server(timeout_sec=2.0):
                    return False, "导航服务不在（nav2 没起？）"
                if self._goal_handle is not None:
                    try:
                        self._goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                    self._goal_handle = None
                g = NavigateToPose.Goal()
                g.pose.header.frame_id = "map"
                g.pose.header.stamp = self.get_clock().now().to_msg()
                g.pose.pose.position.x = float(x)
                g.pose.pose.position.y = float(y)
                g.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
                g.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
                self._set_goal(state="sending", x=float(x), y=float(y),
                               yaw=float(yaw))
                fut = self._ac.send_goal_async(
                    g, feedback_callback=self._on_feedback)
                fut.add_done_callback(self._on_goal_response)
                return True, "目标已发送"

            def _on_feedback(self, msg):
                d = getattr(msg.feedback, "distance_remaining", None)
                if d is not None:
                    self._set_goal(state="executing", remaining=float(d))

            def _on_goal_response(self, fut):
                try:
                    gh = fut.result()
                except Exception as e:
                    self._set_goal(state="failed", error=str(e))
                    return
                if not gh.accepted:
                    self._set_goal(state="rejected")
                    return
                self._goal_handle = gh
                self._set_goal(state="executing")
                gh.get_result_async().add_done_callback(self._on_goal_result)

            def _on_goal_result(self, fut):
                self._goal_handle = None
                try:
                    st = fut.result().status
                except Exception as e:
                    self._set_goal(state="failed", error=str(e))
                    return
                names = {GoalStatus.STATUS_SUCCEEDED: "succeeded",
                         GoalStatus.STATUS_CANCELED: "canceled",
                         GoalStatus.STATUS_ABORTED: "aborted"}
                self._set_goal(state=names.get(st, "failed"))

            def cancel_goal(self):
                if self._goal_handle is None:
                    return False, "没有正在执行的导航目标"
                self._goal_handle.cancel_goal_async()
                return True, "已请求取消"

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

    def send_goal(self, x, y, yaw):
        if self._node is None:
            return False, "ROS 未就绪"
        return self._node.send_goal(x, y, yaw)

    def cancel_goal(self):
        if self._node is None:
            return False, "ROS 未就绪"
        return self._node.cancel_goal()


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

    here = os.path.dirname(os.path.abspath(__file__))
    maps_default = "/home/cy/ros2_lidar/maps"
    if not os.path.isdir(maps_default):
        maps_default = os.path.normpath(os.path.join(here, "..", "maps"))

    parser = argparse.ArgumentParser(description="建图/导航入口（可视化 + 操作）")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--demo", action="store_true",
                        help="不连 ROS，用合成数据自检页面（不能操作）")
    parser.add_argument("--maps-dir", default=maps_default,
                        help="地图目录（默认 %s）" % maps_default)
    args = parser.parse_args(argv)

    state = State()
    stop = threading.Event()
    controller = None
    if args.demo:
        DemoSource(state).start()
    else:
        source = RosSource(state)
        ok = source.start()
        controller = ModeController(
            state, source if ok else None, args.maps_dir,
            os.path.normpath(os.path.join(here, "..", "launch")))
        if ok:
            threading.Thread(target=_mode_loop, args=(state, stop),
                             daemon=True, name="upper-mode").start()

    srv = build_server(args.host, args.port, state,
                       os.path.join(here, "index.html"), controller)
    print("upper: http://%s:%d  maps=%s" % (args.host, args.port,
                                            args.maps_dir), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if controller is not None:
            controller.stop()
        srv.shutdown()


if __name__ == "__main__":
    main()
