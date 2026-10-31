# -*- coding: utf-8 -*-
"""抓取链路侧的 ToF 取数：`/tof/depth` → 「框 → 毫米」。

给 `arm_grasp` 用。设计要求（用户定）：**抓取路径保持能降级** ——
rclpy 起不来、话题没人发、帧太旧，一律返回 `None`，**绝不抛异常、绝不回落成常数**。
下游 `grasp.require_depth` 那道闸会因此拒绝下扎，这正是我们要的。

## ★ 为什么是"自己的执行器 + 后台线程"，不是 `rclpy.spin_once`

先说走过的弯路：一开始照 `collect.ArmIO` 的写法用
`rclpy.spin_once(node, timeout_sec=0.0)`（全局执行器）**"拉"**消息。
结果**收不到帧**（实测：连等 3 秒、`msgs==0`；同一时刻一个正经 `rclpy.spin()`
的订阅者能稳稳收到 40 帧）。原因是 `spin_once(timeout_sec=0)` 走的是全局执行器，
而它**每次都把节点加进去、跑一次、再从 `finally` 里摘出来**
（`rclpy/__init__.py:64-72`）—— 零超时的等待和 DDS 的投递是赛跑，
99% 的情况它先返回，消息永远轮不上。

**所以这里换成**：自己建一个 `SingleThreadedExecutor`（**不是全局那个**），
加进 ToF 节点，在一条 daemon 线程里一直 `spin()`。
`ArmIO` 那份仍然用全局执行器 —— 两个执行器是**不同对象**，
所以不违反 rclpy 那条"全局执行器不许跨线程共用"的规矩，互不干扰。

好处是这条路的**新鲜度**也更好：线程一直在转，`depth_for_box()` 直接从内存里
读最新一帧，不用等主循环来"拉"。
"""
from __future__ import annotations

import threading
import time
from collections import deque

from . import a010, config as tof_config


class TofClient(object):

    def __init__(self, cfg=None, topic="/tof/depth", node_name="tof_a010_client"):
        self.cfg = cfg or tof_config.load()
        self.topic = topic
        self.node_name = node_name

        self.node = None
        self.executor = None
        self.thread = None
        self.enabled = False
        self.reason = "还没 start()"
        self.n_msgs = 0
        self.n_null = 0

        self._mm = None            # 最新一帧的毫米数组（一维，0 = 无效）
        self._rows = self._cols = 0
        self._t = None
        # 最近 ~1.5s 的帧 (t_mono, mm, rows, cols)：按**拍摄时刻**对齐用（depth_for_box 的 at_mono）
        self._ring = deque(maxlen=int(self.cfg.get("ring_frames", 30)))

    # -- 生命周期 -----------------------------------------------------------

    def start(self):
        """建节点 + 订阅 + 后台 spin 线程。**任何失败都只是降级**，不抛。"""
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from sensor_msgs.msg import Image
        except ImportError as e:
            self.reason = "没有 rclpy（%s）" % e
            return False
        try:
            # ArmIO 可能已经 init 过了；重复 init 会抛，吞掉即可
            try:
                rclpy.init(args=None)
            except Exception:                       # noqa: BLE001
                pass
            self.node = Node(self.node_name)
            # best-effort：和发布端一致。用默认的 RELIABLE 会和 best-effort 发布端
            # 匹配不上（QoS 不兼容 ⇒ **一句错都不报，只是永远收不到**）。
            qos = _sensor_qos()
            self.node.create_subscription(Image, self.topic, self._on_depth, qos)
            # ★ 自己的执行器 —— 不是 `rclpy.get_global_executor()`，
            #   这样它和 ArmIO 那份互不干扰（见模块头）。
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin,
                                           daemon=True, name="tof-client-spin")
            self.thread.start()
        except Exception as e:                      # noqa: BLE001
            self.reason = "建订阅失败：%r" % (e,)
            self.node = None
            return False
        self.enabled = True
        self.reason = "ok（%s）" % self.topic
        return True

    def stop(self):
        """收尾。**顺序不能乱**：先让 `spin()` 返回、**join 线程**，再 destroy 节点。

        ⚠️ 2026-10-20 实测：执行器 `shutdown()` 之后线程要过一会儿才退出；
        如果紧接着 `destroy_node()`，rclpy 的 C 层会和还在跑的 spin 撞上，
        进程直接 `terminate called without an active exception` **core dump**。
        这条会在**每次抓取收尾**触发（`grasp_once` 的 finally 里就调它）。
        """
        if self.executor is not None:
            try:
                self.executor.shutdown(timeout_sec=0.5)
            except Exception:                       # noqa: BLE001
                pass
        if self.thread is not None:
            self.thread.join(timeout=3.0)           # ★ 必须等它真的退出
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:                       # noqa: BLE001
                pass
        self.executor = None
        self.thread = None
        self.node = None
        self.enabled = False

    def _on_depth(self, msg):
        try:
            import numpy as np
        except ImportError:
            return
        if msg.encoding != "16UC1":
            self.reason = "encoding 不是 16UC1 而是 %s" % msg.encoding
            return
        arr = np.frombuffer(msg.data, dtype=np.uint16)
        if arr.size < msg.height * msg.width:
            return
        now = time.monotonic()
        t = now
        # header.stamp（tof_node 发布时的 ROS 时钟）→ Pi 单调时刻。同一台 Pi 上 ROS 时钟
        # 就是系统墙钟，和 monotonic 只差常数；按"现在"两边各取一次做差换算。
        st = msg.header.stamp
        if self.node is not None and (st.sec or st.nanosec):
            now_ros_ns = self.node.get_clock().now().nanoseconds
            lag = (now_ros_ns - (st.sec * 1_000_000_000 + st.nanosec)) / 1e9
            if 0.0 <= lag < 1.0:
                t = now - lag
        self._mm = arr[:msg.height * msg.width]
        self._rows, self._cols = msg.height, msg.width
        self._t = t
        self._ring.append((t, self._mm, self._rows, self._cols))
        self.n_msgs += 1

    # -- 取数 ---------------------------------------------------------------

    def wait_for_first(self, timeout=3.0, poll=0.02):
        """等到**第一帧**深度（或超时）。返回是否等到。

        ★ 必须真的 `sleep`：DDS 的**发现期**是墙上时间（subscriber 与 publisher
          配对要几百 ms），不是 CPU 时间。因此"紧循环查几十次"是无效的。
        """
        if not self.enabled:
            return False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._mm is not None:
                return True
            time.sleep(poll)
        return self._mm is not None

    def age(self):
        return None if self._t is None else time.monotonic() - self._t

    def _frame_near(self, at_mono):
        """环形缓冲里离 `at_mono` 最近的一帧 → (t, mm, rows, cols)；缓冲空返回 None。"""
        best = None
        for fr in self._ring:
            if best is None or abs(fr[0] - at_mono) < abs(best[0] - at_mono):
                best = fr
        return best

    def depth_for_box(self, box_norm, at_mono=None):
        """`box_norm`（归一化 l,t,r,b，1280x720 推流系）→ `（毫米, 说明）`。

        `at_mono`（可选）：K230 那帧的**拍摄时刻**（Pi `time.monotonic()` 秒，
        即 `/k230/result` 的 `stamp_mono`）。给了就从环形缓冲里取**同一时刻**的 ToF 帧，
        两者相差超过 `max_dt_s`（默认 0.15s）当没拿到；不给 = 老行为，取最新一帧。
        手臂在动的时候（视觉追踪）必须给 —— 框是 ~100ms 前拍的，最新一帧 ToF
        对应的是臂现在的位置，乘错射线。

        ★ **超龄当没拿到**（`max_age_s`，默认 250ms）：深度必须对应这一拍的手臂姿态，
          拿个 1 秒前的深度去乘射线的意义不大 —— 那时臂还在别处。
        """
        if self._mm is None:
            self.n_null += 1
            return None, {"why": "还没有任何深度帧", "src": "tof"}
        mm, rows, cols = self._mm, self._rows, self._cols
        dt = None
        if at_mono is not None:
            fr = self._frame_near(at_mono)
            if fr is None:
                fr = (self._t, mm, rows, cols)
            max_dt = float(self.cfg.get("max_dt_s", 0.15))
            if fr[0] is None:
                self.n_null += 1
                return None, {"why": "帧没有时间戳，无法按拍摄时刻对齐", "src": "tof"}
            dt = fr[0] - at_mono
            if abs(dt) > max_dt:
                self.n_null += 1
                return None, {"why": "和拍摄时刻差 %.0fms > %.0fms" % (dt * 1000, max_dt * 1000),
                              "src": "tof", "dt_s": dt}
            _t, mm, rows, cols = fr
        age = self.age()
        max_age = float(self.cfg.get("max_age_s", 0.25))
        if at_mono is None and age is not None and age > max_age:
            self.n_null += 1
            return None, {"why": "帧太旧 %.0fms > %.0fms" % (age * 1000, max_age * 1000),
                          "src": "tof", "age_s": age}
        d = a010.box_depth_mm(mm, rows, cols, list(box_norm),
                              unit_mm=float(self.cfg["unit_mm_per_count"]),
                              map_cfg=self.cfg.get("map"),
                              pad=float(self.cfg["pad"]),
                              min_mm=float(self.cfg["min_mm"]),
                              val_max=int(self.cfg["val_max"]),
                              exclude=self.cfg.get("exclude") or None,
                              kth=int(self.cfg.get("depth_kth", 1)))
        if d is None:
            self.n_null += 1
            return None, {"why": "框内没有落在 [%.0f, %.0f]mm 的有效回波"
                                  % (float(self.cfg["min_mm"]),
                                     int(self.cfg["val_max"]) * float(self.cfg["unit_mm_per_count"])),
                          "src": "tof", "age_s": age}
        # ★ 基准偏置**必须加在阈值之后**：`min_mm`（150 量程下限）和
        #   `val_max*unit`（饱和）说的是**传感器的读数**，加完偏移再去比会把
        #   那道近场假回波的防线整体平移掉。所以先按原始读数判定，最后才换算到
        #   "相机光心轴向深度"这个 `geom` 要的基准上。
        raw = int(d)
        off = float(self.cfg.get("offset_mm", 0.0))
        # 同一帧、同一区域里的桌面深度（原始读数系，和 raw 相减即物体高，offset 抵消）
        table = None
        try:
            region = a010.region_bounds(list(box_norm), map_cfg=self.cfg.get("map"),
                                        pad=float(self.cfg["pad"]), rows=rows, cols=cols)
            if region is not None:
                table = a010.region_table_mm(
                    mm, region, rows, cols, top_mm=raw, lo=float(self.cfg["min_mm"]),
                    hi=int(self.cfg["val_max"]) * float(self.cfg["unit_mm_per_count"]))
        except Exception:      # noqa: BLE001 —— 量不出桌面只是不往下扎，不影响深度本身
            table = None
        return int(round(raw + off)), {"src": "tof", "age_s": age, "dt_s": dt,
                                       "n_msgs": self.n_msgs,
                                       "raw_mm": raw, "offset_mm": off,
                                       "table_mm": table}

    def info(self):
        return {"enabled": self.enabled, "reason": self.reason,
                "msgs": self.n_msgs, "null": self.n_null,
                "age_s": self.age(), "shape": (self._rows, self._cols)}


def _sensor_qos(depth=5):
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    return QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)
