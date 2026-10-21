# -*- coding: utf-8 -*-
"""K230 结果流 → ROS 话题。

链路：K230 --TCP 8556--> k230-resultd（写 /tmp/k230/latest-result.json，老链路不动）
                          └--UDP 127.0.0.1:18556（每条原样 + _rx_mono）--> 本节点

发布：
  /k230/result      std_msgs/String   整条记录的 JSON（含 faces、src、track_id 等全部字段），
                                      额外加：
                                        stamp_mono   拍摄时刻，Pi time.monotonic() 秒
                                        stamp_src    'cap_tick' | 'tick' | 'rx'（用的哪个量）
                                        age_ms       本节点处理时距拍摄多久
                                        clock_off_s  当前板/Pi 时钟偏差估计
  /k230/detections  vision_msgs/Detection2DArray   objs（检测框 + 跟踪框）
      header.stamp = 拍摄时刻（换算到 ROS 时钟），frame_id = 'k230_camera'
      bbox 用**像素**（推流分辨率 w×h，REP 惯例）；class_id = cls，score = score；
      id = 'track' 表示跟踪器框（src=track），检测框为 ''。

为什么 faces 只在 JSON 里：身份（who/match/live/ok）塞进 Detection2D 只能硬凑字段，
下游（人脸 v1）是 Python，直接读 JSON 更不容易用错。

时间戳：见 clock_sync.py。ToF / 手臂回读用的也是同一台 Pi 的时钟，
所以要对齐就拿 stamp_mono（或 header.stamp）去它们的缓冲里找最近的一帧。
"""
import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .clock_sync import TickClock
from .records import detections_from_record

UDP_ADDR = ('127.0.0.1', 18556)


class K230Bridge(Node):

    def __init__(self):
        super().__init__('k230_bridge')
        self.declare_parameter('frame_id', 'k230_camera')
        self.frame_id = self.get_parameter('frame_id').value
        self.pub_json = self.create_publisher(String, 'k230/result', 10)
        self.pub_det = self.create_publisher(Detection2DArray, 'k230/detections', 10)
        self.clock_sync = TickClock()
        self.n = 0
        self.bad = 0
        self.prev_resets = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(UDP_ADDR)
        self.sock.settimeout(0.5)
        self.stop = threading.Event()
        threading.Thread(target=self._rx_loop, daemon=True).start()
        self.create_timer(10.0, self._report)
        self.get_logger().info('k230_bridge 启动：udp %s:%d → /k230/result, /k230/detections'
                               % UDP_ADDR)

    def _rx_loop(self):
        while not self.stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                time.sleep(0.2)
                continue
            try:
                rec = json.loads(data.decode('utf-8', 'replace'))
                self._handle(rec)
            except Exception as e:
                self.bad += 1
                self.get_logger().warn(f'坏记录: {e}', throttle_duration_sec=5.0)

    def _handle(self, rec):
        cap_mono, src = self.clock_sync.capture_mono(rec)
        now_mono = time.monotonic()
        now_ros = self.get_clock().now()
        if cap_mono is None:
            cap_mono, src = now_mono, 'none'
        age = now_mono - cap_mono
        stamp = (now_ros - rclpy.duration.Duration(seconds=max(0.0, age))).to_msg()

        rec['stamp_mono'] = cap_mono
        rec['stamp_src'] = src
        rec['age_ms'] = round(age * 1000.0, 1)
        rec['clock_off_s'] = self.clock_sync.offset()
        m = String()
        m.data = json.dumps(rec, ensure_ascii=False)
        self.pub_json.publish(m)

        arr = Detection2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self.frame_id
        for cls, score, cx, cy, w, h, did in detections_from_record(rec):
            d = Detection2D()
            d.header = arr.header
            d.id = did
            d.bbox.center.position.x = cx
            d.bbox.center.position.y = cy
            d.bbox.size_x = w
            d.bbox.size_y = h
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cls
            hyp.hypothesis.score = score
            d.results.append(hyp)
            arr.detections.append(d)
        self.pub_det.publish(arr)
        self.n += 1
        self._last_age = age
        self._last_src = src

    def _report(self):
        if self.clock_sync.resets != self.prev_resets:
            self.get_logger().warn(f'板子时钟重置 {self.clock_sync.resets} 次（板子重启？）')
            self.prev_resets = self.clock_sync.resets
        if self.n == 0:
            self.get_logger().warn('10s 没收到任何结果（k230-resultd 在跑？板子在推？）')
            return
        self.get_logger().info(
            f'10s 内 {self.n} 帧，最近一帧距拍摄 {self._last_age * 1000:.0f}ms'
            f'（{self._last_src}），坏记录累计 {self.bad}')
        self.n = 0


def main(args=None):
    rclpy.init(args=args)
    node = K230Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
