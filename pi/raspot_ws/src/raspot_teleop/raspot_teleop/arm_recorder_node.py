#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂黑匣子记录节点：抓"动了但没人指挥"的瞬间。

记录到 ~/arm_recorder.log（满 5MB 轮转为 .1）：
  CMD   t 每条 /arm/command 的目标值
  JS    t 手柄原始事件（/dev/input/js0，多读端合法；轴值全记）
  VOLT  t 电池电压（变化 >0.15V 记一条，抓电源轨抖动）
  ESTOP t 急停状态变化
  MOVE  t 关节位移检测（相邻回读差 ≥ 阈值）：
        last_cmd_ago > 1s 的标记 !! UNCOMMANDED 并附近 2s 上下文

裁决逻辑：
  MOVE 且近期无 CMD → 指令不是我们发的（固件底座保持/烧录版差异/电气）
  MOVE 前有 CMD + JS 事件 → 手柄输入链路（看 JS 行还原具体事件）
  MOVE 前 CMD 但无 JS → teleop 自身逻辑问题
"""
import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32

LOG_CAP_BYTES = 5 * 1024 * 1024
CONTEXT_LINES = 80


class ArmRecorderNode(Node):

    def __init__(self):
        super().__init__('arm_recorder')
        self.declare_parameter('log_path', os.path.expanduser('~/arm_recorder.log'))
        self.declare_parameter('move_threshold', 4)
        self.declare_parameter('js_device', '/dev/input/js0')
        self.path = self.get_parameter('log_path').value
        self.th = int(self.get_parameter('move_threshold').value)
        js_dev = self.get_parameter('js_device').value

        self.f = open(self.path, 'a', buffering=1)
        self.bytes_written = os.path.getsize(self.path) if os.path.exists(self.path) else 0
        self.recent = []
        self.last_cmd_t = None
        self.prev = None
        self.prev_volt = None

        self.create_subscription(JointState, 'arm/feedback', self.on_fb, 10)
        self.create_subscription(JointState, 'arm/command', self.on_cmd, 10)
        self.create_subscription(Bool, 'estop', self.on_estop, 10)
        self.create_subscription(Float32, 'battery_voltage', self.on_volt, 10)
        threading.Thread(target=self.js_loop, args=(js_dev,), daemon=True).start()

        self.w(f'=== recorder 启动 threshold={self.th} js={js_dev} ===')

    # —— 落盘 ——
    def w(self, line):
        stamp = time.strftime('%H:%M:%S') + '.%03d' % (int(time.time() * 1000) % 1000)
        line = f'{stamp} {line}'
        self.recent.append(line)
        if len(self.recent) > 600:
            del self.recent[:200]
        self.f.write(line + '\n')
        self.bytes_written += len(line) + 1
        if self.bytes_written > LOG_CAP_BYTES:
            self.f.close()
            if os.path.exists(self.path + '.1'):
                os.remove(self.path + '.1')
            os.rename(self.path, self.path + '.1')
            self.f = open(self.path, 'a', buffering=1)
            self.bytes_written = 0
            self.w('=== 轮转新文件 ===')

    # —— 订阅回调 ——
    def on_cmd(self, msg):
        self.last_cmd_t = time.monotonic()
        vals = ','.join('%d' % v for v in msg.position[:6])
        self.w(f'CMD [{vals}]')

    def on_estop(self, msg):
        self.w(f'ESTOP {bool(msg.data)}')

    def on_volt(self, msg):
        v = float(msg.data)
        if self.prev_volt is not None and abs(v - self.prev_volt) > 0.15:
            self.w(f'VOLT {self.prev_volt:.2f} -> {v:.2f}')
        self.prev_volt = v

    def on_fb(self, msg):
        pos = list(msg.position)
        now = time.monotonic()
        if self.prev is not None and len(pos) == 6 and len(self.prev) == 6:
            names = ('grip', 'wrist_roll', 'wrist_pitch', 'forearm', 'shoulder', 'base')
            for i in range(6):
                a, b = self.prev[i], pos[i]
                if a != 0 and b != 0 and abs(b - a) >= self.th:
                    ago = None if self.last_cmd_t is None else round(now - self.last_cmd_t, 2)
                    tag = 'CMD-DRIVEN' if (ago is not None and ago <= 1.0) else '!! UNCOMMANDED'
                    self.w(f'MOVE {names[i]} {a:.0f}->{b:.0f} last_cmd_ago={ago}s [{tag}]')
                    if tag.startswith('!!'):
                        self.w(f'  ---- 近 2s 上下文（最后 {CONTEXT_LINES} 行）----')
                        for l in self.recent[-CONTEXT_LINES:]:
                            self.w('  | ' + l)
                        self.w('  ---- 上下文结束 ----')
        self.prev = pos

    # —— 手柄原始事件 ——
    def js_loop(self, path):
        while True:
            try:
                fd = os.open(path, os.O_RDONLY)
            except OSError:
                time.sleep(1.0)
                continue
            try:
                while True:
                    data = os.read(fd, 8)
                    if len(data) < 8:
                        continue
                    _t, val, typ, num = struct.unpack('<ihBB', data)
                    ev = typ & 0x7F
                    if ev == 0x01:
                        self.w(f'JS btn{num}={"按下" if val else "松开"}')
                    elif ev == 0x02:
                        self.w(f'JS axis{num}={val}')
            except OSError:
                pass
            finally:
                try:
                    os.close(fd)
                except OSError:
                    pass
            self.w('JS 设备消失，等待重连')
            time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArmRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
