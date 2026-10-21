#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A010 ToF 的 ROS2 节点：把模块的 100x100 深度发成话题。

    ros2 run tof_a010 tof_node
    ros2 run tof_a010 tof_node --ros-args -p device:=/dev/ttyUSB0 -p publish_cloud:=true

发：

    /tof/depth    sensor_msgs/Image       16UC1，**毫米**，height=rows width=cols
    /tof/points   sensor_msgs/PointCloud2 x/y/z float32（米）        （默认关）
    /tof/status   std_msgs/String         JSON：fps/连接/重连/帧龄/最后错误

## 为什么不直接抄厂商那份 C++

厂商 `sipeed_tof_ms_a010`（本机已编译）有三个问题，改不如重写：

1. **尺度错**：它把 payload 直接当**毫米**塞进 `mono8`（点云里 `z = depth/1000`），
   而模块在 `AT+UNIT=4` 下 payload 是**计数** —— 差一个 ×4。而且 mono8 装不下
   > 255mm 的值（本模块满量程 1020mm），超了就截断。
2. **阻塞**：`O_NONBLOCK` 被清掉 + `VTIME=10`，单线程执行器里一次 `read()` 最长卡 1 秒。
3. **静默失败**：握手失败时构造函数直接 `return`，`spin()` 照转，**一个 publisher 都不建**、
   一行错误都不打 —— 看起来像卡死。这是最坑的一条。
"""
from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String

from . import a010, config as tof_config
from .serial_dev import TofReader


def _sensor_qos(depth=5):
    """**best-effort**：深度流是"最新的才有用"，丢了就丢了。

    厂商那份用的是默认 RELIABLE keep_last(10) —— 一旦订阅端跟不上，
    发布端会阻塞在 DDS 重传上，把整个节点拖慢。
    """
    return QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


class TofNode(Node):

    def __init__(self):
        super().__init__("tof_a010")
        cfg = tof_config.load()
        self.cfg = cfg
        self.get_logger().info("配置：%s" % (cfg.get("_path") or "（内置默认，没找到 yaml）"))

        self.declare_parameter("device", cfg["device"])
        self.declare_parameter("unit_mm_per_count", float(cfg["unit_mm_per_count"]))
        self.declare_parameter("disp_mode", int(cfg["disp_mode"]))
        self.declare_parameter("binn", int(cfg["binn"]))
        self.declare_parameter("frame_id", cfg["frame_id"])
        self.declare_parameter("publish_cloud", bool(cfg["publish_cloud"]))
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("fx", float(cfg["fx"]))
        self.declare_parameter("fy", float(cfg["fy"]))
        self.declare_parameter("cx", float(cfg["cx"]))
        self.declare_parameter("cy", float(cfg["cy"]))

        self.gp = lambda n: self.get_parameter(n).value
        unit = float(self.gp("unit_mm_per_count"))

        self.pub_depth = self.create_publisher(Image, "/tof/depth", _sensor_qos())
        self.pub_cloud = self.create_publisher(PointCloud2, "/tof/points", _sensor_qos(1))
        self.pub_status = self.create_publisher(String, "/tof/status", 1)

        dev_path, why = tof_config.resolve_device({"device": self.gp("device")})
        self.reader = TofReader(_make_dev(dev_path),
                                unit_mm=unit, binn=int(self.gp("binn")),
                                disp=int(self.gp("disp_mode")))
        self.reader.start()

        self._last_frame_id = None
        self._t_status = 0.0
        self._logged_reconnects = 0
        self._logged_starting = False   # ★ 必须在 __init__ 里给出：
        self._ever_connected = False    #   `_maybe_status` 会先读再写，少了就是
                                        #   第一拍 AttributeError（回调里被吞掉 ⇒
                                        #   /tof/status 一条都发不出来，还不报错）
        self.create_timer(1.0 / float(self.gp("publish_rate_hz")), self.tick)
        self.get_logger().info(
            "启动：%s（%s）  unit=%smm/计数  DISP=%s  发 /tof/depth%s"
            % (dev_path, why, unit, self.gp("disp_mode"),
               " + /tof/points" if self.gp("publish_cloud") else ""))

    # -- 主循环 -------------------------------------------------------------

    def tick(self):
        fr, age = self.reader.snapshot()
        if fr is None:
            self._maybe_status(force_err="还没有任何帧（检查 DISP 位 / 接线）")
            return
        if fr.frame_id == self._last_frame_id:
            self._maybe_status()
            return
        self._last_frame_id = fr.frame_id

        unit = float(self.gp("unit_mm_per_count"))
        self.pub_depth.publish(self._to_image(fr, unit))
        if self.gp("publish_cloud"):
            self.pub_cloud.publish(self._to_cloud(fr, unit))
        self._maybe_status()

    def _stamp(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gp("frame_id")
        return msg

    def _to_image(self, fr, unit):
        mm = np.array(a010.depth_map_mm(fr, unit_mm=unit,
                                        val_max=int(self.cfg["val_max"])),
                      dtype=np.uint16).reshape(fr.rows, fr.cols)
        msg = Image()
        self._stamp(msg)
        msg.height, msg.width = fr.rows, fr.cols
        msg.encoding = "16UC1"
        msg.is_bigendian = False
        msg.step = fr.cols * 2
        msg.data = mm.tobytes()
        return msg

    def _to_cloud(self, fr, unit):
        mm = np.array(a010.depth_map_mm(fr, unit_mm=unit,
                                        val_max=int(self.cfg["val_max"])),
                      dtype=np.float32).reshape(fr.rows, fr.cols)
        z = mm / 1000.0                                  # 米
        fx, fy = float(self.gp("fx")), float(self.gp("fy"))
        cx, cy = float(self.gp("cx")), float(self.gp("cy"))
        jj, ii = np.meshgrid(np.arange(fr.cols), np.arange(fr.rows))
        x = z * (ii - cx) / fx
        y = z * (jj - cy) / fy
        pts = np.stack([x, y, z], axis=-1).astype(np.float32)
        valid = (mm > 0)                                 # 0 已经是"无效"

        msg = PointCloud2()
        self._stamp(msg)
        msg.height, msg.width = fr.rows, fr.cols
        msg.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32,
                                 count=1) for k, n in enumerate("xyz")]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * fr.cols
        msg.is_dense = bool(valid.all())
        msg.data = pts.tobytes()
        return msg

    def _maybe_status(self, force_err=None):
        now = time.monotonic()
        if now - self._t_status < 1.0:
            return
        self._t_status = now
        st = self.reader.status()
        if force_err:
            st["note"] = force_err
        # 掉线/重连要**进日志**：模块实测会枚举抖动，"没数据"和"口掉了"必须一眼分开。
        # 只放进 /tof/status 的话，排障时得先想到去 echo 那个话题才知道。
        if st["connected"]:
            self._ever_connected = True
        if st["reconnects"] != self._logged_reconnects:
            self._logged_reconnects = st["reconnects"]
            self.get_logger().warn("串口重连第 %d 次：%s（%s）"
                                   % (st["reconnects"], st["last_error"], st["device"]))
        elif self._ever_connected and not st["connected"]:
            # ★ 只在"连过又掉了"时喊。启动那 ~7s（开串口等 DTR 复位 + 走完 AT 初始化）
            #   本来就还没连上，那时每秒喊一句"未连接"是**假警报**，
            #   真掉线时反而淹没在噪声里。
            self.get_logger().warn("串口掉了：%s" % st["last_error"])
        elif not self._ever_connected and not self._logged_starting:
            self._logged_starting = True
            self.get_logger().info("串口初始化中（开串口等 DTR 复位 + AT 初始化，约 3~8s）…")
        self.pub_status.publish(String(data=json.dumps(st, ensure_ascii=False)))

    def destroy_node(self):
        try:
            self.reader.stop()
            self.reader.dev.stop_stream()
            self.reader.dev.close()
        except Exception:                     # noqa: BLE001
            pass
        return super().destroy_node()


def _make_dev(device):
    from .serial_dev import A010
    return A010(device=device)


def main(argv=None):
    rclpy.init(args=argv)
    node = TofNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
