#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""l150pro 串口驱动节点：/cmd_vel + /arm/command → STM32 固件 V2.0（机械臂版）。

订阅:
  /cmd_vel      geometry_msgs/Twist     底盘 vx/wz；超 0.5s 无消息按零速处理（上游死亡保护）
  /arm/command  sensor_msgs/JointState  position=[p1..p5 raw, base field]，每条消息即发一帧 0xAC(t_ms=40)
  /estop        std_msgs/Bool           急停锁存（true: 底盘清零+机械臂保持）
服务:
  /arm/rehome   std_srvs/Trigger        单发 0xAC flags bit0（边沿触发）。目标带当前回读位姿：
                                        固件 re-home 只重锚底座，五关节原地不动。
发布:
  /battery_voltage  std_msgs/Float32    V（1Hz）
  /chassis_faults   std_msgs/Int32      固件 fault 位（变化即发并打日志）
  /arm/feedback     sensor_msgs/JointState  0x56 实测回读（**25Hz**；0=该拍没读到）
     ⚠️ 2026-10-01 之前这里写的是 10Hz —— 那是**发布定时器**的限制，不是固件的：
     固件 `ARM_RATE_HZ=50`、0x56 每 2 拍发一次 = ~25Hz，被 10Hz 的定时器砍掉了 3/5 的帧。
  /arm/online       std_msgs/Bool       机械臂 READY（25Hz，同上）

0xAA 由本节点 25Hz 恒频发送（=链路保活，固件 200ms 看门狗兜底）。
注意: 串口独占——旧 l150pro-driver / 云端驱动节点与本节点不可同时运行。
"""
import signal
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int32
from std_srvs.srv import Trigger

from .protocol import (ARM_BASE_MAX, ARM_GRIP_MAX, ARM_JOINT_MAX, ARM_JOINT_NAMES,
                       UplinkParser, build_arm, build_chassis, fault_names)

CTRL_WAIT_MAX_S = 25.0       # 等固件报 control-live 的兜底上限（回位最长约 17s）
CMD_STALE_S = 0.5            # /cmd_vel 超时视为上游死亡 → 零速
VOLT_WARN_MV = 10500


class SerialState:
    """串口线程 → 节点线程 共享遥测。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.connected = False
        self.online = False
        self.estop_fb = False
        self.voltage_mv = 0
        self.faults = 0
        self.joints = (0,) * 5
        self.base = 0
        self.status56 = 0
        self.pot_raw = -1
        self.base_st = 0
        self.ctrl_live = False   # 0x56 byte19: 1 = 固件在 INIT_READY，正在消费 0xAC
        self.seen55 = False
        self.seen56 = False


class RxWorker(threading.Thread):
    """串口收线程：自动重连 + 解析上行帧。"""

    def __init__(self, path: str, baud: int, state: SerialState):
        super().__init__(daemon=True)
        self.path = path
        self.baud = baud
        self.state = state
        self.ser = None
        self.wlock = threading.Lock()
        self._last_open_err = 0.0

    def _try_open(self):
        try:
            self.ser = serial.Serial(self.path, self.baud, timeout=0.05)
            self.ser.reset_input_buffer()
            with self.state.lock:
                self.state.connected = True
            print(f'[l150pro] 串口已打开 {self.path}', flush=True)
        except Exception as e:
            self.ser = None
            with self.state.lock:
                self.state.connected = False
            now = time.monotonic()
            if now - self._last_open_err > 10.0:
                print(f'[l150pro] 串口打开失败: {e}（持续重试）', flush=True)
                self._last_open_err = now

    def run(self):
        buf = bytearray()
        while True:
            if self.ser is None:
                self._try_open()
                time.sleep(1.0)
                continue
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n if n > 0 else 1)
            except Exception as e:
                self._drop(f'读失败: {e}')
                continue
            if not data:
                continue
            buf.extend(data)
            try:
                for kind, d in UplinkParser.feed(buf):
                    st = self.state
                    with st.lock:
                        if kind == 't55':
                            st.voltage_mv = d['voltage_mv']
                            st.faults = d['faults']
                            st.seen55 = True
                        else:
                            st.joints = d['joints']
                            st.base = d['base']
                            st.status56 = d['status']
                            st.pot_raw = d['pot_raw']
                            st.base_st = d['base_st']
                            st.ctrl_live = bool(d['ctrl_live'])
                            st.online = bool(d['status'] & 0x01)
                            st.estop_fb = bool(d['status'] & 0x80)
                            st.seen56 = True
            except Exception as e:
                print(f'[l150pro] 解析异常: {e}', flush=True)
                buf.clear()

    def _drop(self, why: str):
        with self.wlock:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
        with self.state.lock:
            self.state.connected = False
        print(f'[l150pro] 串口断开（{why}），1 秒后重连', flush=True)

    def send(self, frame: bytes) -> bool:
        with self.wlock:
            if self.ser is None:
                return False
            try:
                self.ser.write(frame)
                return True
            except Exception as e:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                with self.state.lock:
                    self.state.connected = False
                print(f'[l150pro] 写失败: {e}', flush=True)
                return False

    def close(self):
        with self.wlock:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None


class L150ProDriverNode(Node):

    def __init__(self):
        super().__init__('l150pro_driver_node')
        self.declare_parameter('serial_path', '/dev/l150pro')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('arm_t_ms', 40)
        self.declare_parameter('require_online', True)   # 烧录版固件不置 ONLINE 位时设 false
        path = self.get_parameter('serial_path').value
        baud = int(self.get_parameter('baud').value)
        self.arm_t_ms = int(self.get_parameter('arm_t_ms').value)
        self.require_online = bool(self.get_parameter('require_online').value)

        self.pub_voltage = self.create_publisher(Float32, 'battery_voltage', 10)
        self.pub_faults = self.create_publisher(Int32, 'chassis_faults', 10)
        self.pub_fb = self.create_publisher(JointState, 'arm/feedback', 10)
        self.pub_online = self.create_publisher(Bool, 'arm/online', 10)
        self.pub_pot = self.create_publisher(Int32, 'arm/pot_raw', 10)
        self.pub_status = self.create_publisher(Int32, 'arm/status', 10)

        self.sub_twist = self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)
        self.sub_arm = self.create_subscription(JointState, 'arm/command', self.on_arm, 10)
        self.sub_estop = self.create_subscription(Bool, 'estop', self.on_estop, 10)
        self.srv_rehome = self.create_service(Trigger, 'arm/rehome', self.on_rehome)

        self.state = SerialState()
        self.rx = RxWorker(path, baud, self.state)
        self.rx.start()
        self.create_timer(1.0 / 25.0, self.chassis_tick)
        # ★ 2026-10-01：**0.1 → 1/25**。
        #   固件那边 `arm.h: ARM_RATE_HZ=50`、`arm.c: if(++arm_fb_div >= ARM_RATE_HZ/20) ros_send_arm_fb()`
        #   ⇒ 0x56 是 **~25Hz（40ms）**推上来的；而这里原来用 10Hz 定时器去发 ⇒ **每 3 帧丢掉 2 帧**，
        #   回读年龄被这个定时器顶到 100~160ms（固件 40ms + 每关节总线轮询 60ms + 这里 100ms）。
        #   对照组：同一文件的 `chassis_tick` 本来就是 25Hz。
        #   **只改发布频率，不动任何数据通路**（`feedback_tick` 只是把 RxWorker 解析好的最新状态发出去）。
        #   两个订阅方都与速率无关，已核过：`ps2_teleop.on_fb`（镜像回读进目标 + 1s 操作守卫）、
        #   `arm_recorder.on_fb`（阈值阶跃检测）。
        self.create_timer(1.0 / 25.0, self.feedback_tick)

        self.tw = Twist()
        self.tw_time = 0.0
        self.estop = False
        self.ctrl_ever   = False    # 见过固件报 control-live 吗（老固件恒不报 → 不门）
        self.ctrl_ok_at  = 0.0      # 最近一次看到 control-live 的时刻
        self.ctrl_warned = False
        self.seq_aa = 0
        self.seq_ac = 0
        self.prev_online = None
        self.prev_faults = None
        self.prev_status56 = None
        self.prev_base_st = None
        self.last_joints = None      # 最近有效五关节回读（re-home 帧的目标用它，绝不推臂）
        self.last_base = 0
        self.low_volt_warned = False
        self.no55_warned = False
        self.arm_offline_warned = False
        self.t_start = time.monotonic()
        self._last_volt_pub = 0.0

        self.get_logger().info(f'l150pro_driver_node 启动：串口 {path} @{baud}, arm_t_ms={self.arm_t_ms}')

    # —— 订阅回调 ——
    def on_twist(self, msg):
        self.tw = msg
        self.tw_time = time.monotonic()

    def on_estop(self, msg):
        new = bool(msg.data)
        if new != self.estop:
            self.get_logger().warn('急停置位：底盘清零、机械臂保持' if new else '急停解除')
        self.estop = new

    def on_arm(self, msg):
        now = time.monotonic()
        if self.estop:
            return
        with self.state.lock:
            online    = self.state.online
            ctrl_live = self.state.ctrl_live
        # 固件在 INIT_POSE / INIT_HOME 期间【不消费】0xAC：发过去只会堆在信箱里，
        # 等它进 READY 时被当成"最新一帧"套用 —— 归位就是这么被冲掉的。用固件
        # 自报的 control-live 位（0x56 byte19）按住，而不是猜一个固定秒数。
        # ctrl_ever 是给老固件兜底的：从来没报过这一位就一律不门，行为和以前一致。
        if self.ctrl_ever and not ctrl_live:
            if (now - self.ctrl_ok_at) <= CTRL_WAIT_MAX_S:
                return
            if not self.ctrl_warned:
                self.ctrl_warned = True
                self.get_logger().warn(
                    f'等 control-live 超过 {CTRL_WAIT_MAX_S:.0f}s，恢复下发 arm/command')
        if not online and self.require_online:
            if not self.arm_offline_warned:
                self.get_logger().warn('机械臂未就绪（OFFLINE），忽略 arm/command')
                self.arm_offline_warned = True
            return
        self.arm_offline_warned = False
        pos = list(msg.position)
        if len(pos) < 6:
            pos += [0.0] * (6 - len(pos))
        p1 = int(round(min(max(pos[0], 0.0), ARM_GRIP_MAX)))
        p2 = int(round(min(max(pos[1], 0.0), ARM_JOINT_MAX)))
        p3 = int(round(min(max(pos[2], 0.0), ARM_JOINT_MAX)))
        p4 = int(round(min(max(pos[3], 0.0), ARM_JOINT_MAX)))
        p5 = int(round(min(max(pos[4], 0.0), ARM_JOINT_MAX)))
        base = int(round(min(max(pos[5], -ARM_BASE_MAX), ARM_BASE_MAX)))
        self.seq_ac = (self.seq_ac + 1) & 0xFF
        self.rx.send(build_arm(p1, p2, p3, p4, p5, base, self.arm_t_ms, 0x00, self.seq_ac))

    def on_rehome(self, req, resp):
        with self.state.lock:
            online = self.state.online
        if not online and self.require_online:
            resp.success = False
            resp.message = '机械臂未就绪（OFFLINE）'
            return resp
        if self.last_joints is None:
            resp.success = False
            resp.message = '尚无机械臂回读，稍后再试'
            return resp
        # 固件跑的和上电同一套流程：五关节 → ARM_RESET_POSE，然后底座重锚 + 回中。
        # 这一帧的数值会被固件在归位后用自己的影子覆盖，所以这里发当前位姿只是把
        # "五关节原地不动"这个意图写清楚；真正决定结果的是固件那两条影子赋值。
        self.seq_ac = (self.seq_ac + 1) & 0xFF
        self.rx.send(build_arm(*self.last_joints, self.last_base, 0, 0x01, self.seq_ac))
        resp.success = True
        resp.message = 're-home 已下发（与上电同流程，约 5-15 秒）'
        self.get_logger().info('re-home 已下发（五关节回出厂姿态 + 底座重锚回中）')
        return resp

    # —— 定时器 ——
    def chassis_tick(self):
        now = time.monotonic()
        if self.estop or (now - self.tw_time) > CMD_STALE_S:
            vx = wz = 0.0
        else:
            vx = min(max(self.tw.linear.x, -0.5), 0.5)
            wz = min(max(self.tw.angular.z, -1.5), 1.5)
        self.seq_aa = (self.seq_aa + 1) & 0xFF
        self.rx.send(build_chassis(int(round(vx * 1000)), 0, int(round(wz * 1000)),
                                   0x03 if self.estop else 0x01, self.seq_aa))

    def feedback_tick(self):
        now = time.monotonic()
        with self.state.lock:
            online = self.state.online
            estop_fb = self.state.estop_fb
            voltage_mv = self.state.voltage_mv
            faults = self.state.faults
            joints = self.state.joints
            base = self.state.base
            seen55 = self.state.seen55
            seen56 = self.state.seen56
            status56 = self.state.status56
            pot_raw = self.state.pot_raw
            base_st = self.state.base_st
            ctrl_live = self.state.ctrl_live

        if ctrl_live:
            self.ctrl_ever   = True
            self.ctrl_ok_at  = now
            self.ctrl_warned = False

        if online != self.prev_online:
            if self.prev_online is not None:
                self.get_logger().info('机械臂 ONLINE，可控制' if online else '机械臂 OFFLINE')
            self.prev_online = online
        m = Bool()
        m.data = online
        self.pub_online.publish(m)

        if self.prev_faults is not None and faults != self.prev_faults:
            names = fault_names(faults)
            self.get_logger().warn(
                f'底盘故障位 0x{faults:02x}（{"、".join(names) or "无"}）')
        self.prev_faults = faults
        im = Int32()
        im.data = faults
        self.pub_faults.publish(im)

        if seen56 and (status56 != self.prev_status56 or base_st != self.prev_base_st):
            self.get_logger().info(
                f'机械臂 0x56: status=0x{status56:02x}（bit0=ONLINE bit7=ESTOP）base_st={base_st}'
                '（0未知/2正常 —— 固件只会发这两个，base_st 实际是布尔量；'
                '底座不可用请判 status 的 bit4=0x10）')
            self.prev_status56 = status56
            self.prev_base_st = base_st
        if seen56:
            im = Int32()
            im.data = status56
            self.pub_status.publish(im)
            im = Int32()
            im.data = pot_raw
            self.pub_pot.publish(im)

        if estop_fb:
            self.get_logger().warn('固件急停反馈置位（0x56 bit7）', throttle_duration_sec=5.0)

        if 0 < voltage_mv < VOLT_WARN_MV and not self.low_volt_warned:
            self.get_logger().warn(f'电压偏低 {voltage_mv} mV，注意充电')
            self.low_volt_warned = True
        elif voltage_mv >= 11000:
            self.low_volt_warned = False
        if now - self._last_volt_pub >= 1.0 and voltage_mv > 0:
            self._last_volt_pub = now
            f = Float32()
            f.data = voltage_mv / 1000.0
            self.pub_voltage.publish(f)

        if seen56:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(ARM_JOINT_NAMES)
            js.position = [float(x) for x in joints] + [float(base)]
            self.pub_fb.publish(js)
            if self.last_joints is None:
                self.last_joints = [int(x) for x in joints]
            else:
                for i in range(5):
                    if joints[i] != 0:            # 0 = 该拍没读到，保留缓存
                        self.last_joints[i] = int(joints[i])
            self.last_base = int(base)

        if not seen55 and (now - self.t_start) > 5.0 and not self.no55_warned:
            self.get_logger().warn('5 秒未收到底盘遥测 0x55（串口没通？）')
            self.no55_warned = True

    def shutdown(self):
        for _ in range(3):
            self.seq_aa = (self.seq_aa + 1) & 0xFF
            self.rx.send(build_chassis(0, 0, 0, 0x01, self.seq_aa))
            time.sleep(0.04)
        self.rx.close()


def main(args=None):
    def _on_term(*_):
        raise KeyboardInterrupt()
    rclpy.init(args=args)
    signal.signal(signal.SIGTERM, _on_term)
    node = L150ProDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
