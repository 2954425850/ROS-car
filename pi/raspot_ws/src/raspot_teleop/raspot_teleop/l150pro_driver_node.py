#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""l150pro 串口驱动节点：/cmd_vel + /arm/command → STM32 固件 V2.0（机械臂版）。

订阅:
  /cmd_vel_mux  geometry_msgs/Twist     底盘 vx/wz（cmd_mux 仲裁后的；参数 cmd_topic）；超 0.5s 无消息按零速
  /arm/command  sensor_msgs/JointState  position=[p1..p5 raw, base field]，每条消息即发一帧 0xAC(t_ms=40)
  /estop        std_msgs/Bool           急停锁存（true: 底盘清零+机械臂保持）
服务:
  /arm/rehome   std_srvs/Trigger        单发 0xAC flags bit0（边沿触发）。目标带当前回读位姿：
                                        固件 re-home 只重锚底座，五关节原地不动。
发布:
  /battery_voltage  std_msgs/Float32    V（1Hz）
  /chassis_faults   std_msgs/Int32      固件 fault 位（变化即发并打日志）
  /wheel_odom       nav_msgs/Odometry   odom→base_footprint，~50Hz（每帧 0x55 一条）；航向默认用陀螺积分
  /imu/data_raw     sensor_msgs/Imu     只有 angular_velocity.z 可信（同老驱动：不发 orientation）
  /driver_fault     std_msgs/Int16MultiArray [fault, 0, 0, dropped] —— 与老驱动同格式（语音/云端读 data[0]）
  /yaw_loop         std_msgs/Float32MultiArray [wz_ref, wz_meas, wz_out, I]（偏航闭环诊断）
  /chassis/raw55    std_msgs/Float32MultiArray [vA,vB,vC,vD, roll,pitch,yaw, gz, tick_ms]（0x55 原值，排错用）
  TF odom→base_footprint（publish_odom_tf，默认开；以后上 EKF 时关掉、由 EKF 发）
  ★ 2026-10-21：以上四个话题 + TF + 偏航闭环原来在 l150pro_ws 的老驱动里，
    老驱动不会说机械臂协议、和本节点抢串口，一直关着 ⇒ 语音的 car_move/car_turn
    一直拿不到反馈。现在并进本节点，老驱动退役。
  /arm/feedback     sensor_msgs/JointState  0x56 实测回读（**25Hz**；0=该拍没读到）
     ⚠️ 2026-10-01 之前这里写的是 10Hz —— 那是**发布定时器**的限制，不是固件的：
     固件 `ARM_RATE_HZ=50`、0x56 每 2 拍发一次 = ~25Hz，被 10Hz 的定时器砍掉了 3/5 的帧。
  /arm/online       std_msgs/Bool       机械臂 READY（25Hz，同上）
  /arm/cmd_gate     std_msgs/Bool       1 = 现在可以给臂下 0xAC（25Hz，只在收到过 0x56 后发）。
                                        固件在 INIT_* 期间不消费 0xAC（发了只会堆在信箱里、
                                        进 READY 时被当成最新一帧套用 ⇒ 归位被冲掉），
                                        所以复位窗口内一律 0。ps2_teleop 靠它把回中期间的
                                        臂输入和发布掐掉（见 arm_gate.py）。
                                        判定 = 复位窗口外 且（固件 control-live 位为 1，
                                        或固件不报这一位）。

0xAA 由本节点 25Hz 恒频发送（=链路保活，固件 200ms 看门狗兜底）。
注意: 串口独占——旧 l150pro-driver / 云端驱动节点与本节点不可同时运行。
"""
import signal
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int16MultiArray, Int32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .odometry import OdomIntegrator
from .protocol import (ARM_BASE_MAX, ARM_FLAG_KEEP_GRIPPER, ARM_FLAG_REHOME,
                       ARM_GRIP_MAX, ARM_JOINT_MAX, ARM_JOINT_NAMES,
                       UplinkParser, build_arm, build_chassis, fault_names)
from .yaw_loop import YawLoopConfig, YawRateLoop

FAULT_UNDERVOLT = 0x01
FAULT_IMU = 0x08

CTRL_WAIT_MAX_S = 25.0       # 等固件报 control-live 的兜底上限（回位最长约 17s）
# 复位窗口兜底。固件报 control-live 时窗口精确在 READY 那一刻结束，这只是上限。
# ★ 2026-10-28 现场：复位最长实测到 >25s（五关节 + 底座回中的行程决定），取 25s。
RESET_GUARD_S = 25.0
# 复位请求发出后多久还没看到固件进 HOMING（control-live=0），就认定这次请求没被执行。
# 固件收帧到置 HOMING 只需 ~60ms，2s 是极宽的余量。
# ★ 2026-10-30 现场：re-home 是**边沿**触发，而边沿只在下一帧 flags=0 的 0xAC 被消费
#   时才复位；复位窗口把期间的 0xAC 全掐了 ⇒ 复位后只要用户不碰臂，固件那位一直挂着
#   ⇒ 下一次 START 被**静默忽略**（日志：status 没有 0x0b(HOMING)、也没有 0x80(ESTOP)
#   ⇒ ArmCanRehome 是放行的，就是边沿丢了），而窗口还在傻等 25s。
RESET_ARM_S = 2.0
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
        self.on_t55 = None       # 回调 (dict, rx_monotonic)；在本线程、不持 state.lock 时调

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
                    if kind == 't55' and self.on_t55 is not None:
                        self.on_t55(d, time.monotonic())
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
        self.declare_parameter('rehome_keep_gripper', False)  # true = re-home 时夹爪保持（flags 0x03）
        path = self.get_parameter('serial_path').value
        baud = int(self.get_parameter('baud').value)
        self.arm_t_ms = int(self.get_parameter('arm_t_ms').value)
        self.require_online = bool(self.get_parameter('require_online').value)
        # ★ 2026-10-12（用户点头）：底盘速度限幅做成 ROS 参数。
        #   原来是写死的 `min(max(x, -0.5), 0.5)` / `±1.5`，那两行在冻结名单里
        #   （`chassis_tick`）。**默认值一个没动（0.5 / 1.5）⇒ 行为与冻结版逐字一致**，
        #   做成参数只是为了让台架上量传动速度上限时能
        #   `ros2 param set /l150pro_driver max_vx 1.8` —— 改完立即生效、不用重启服务、
        #   也不用再改代码（同 `l150pro_ws` 那份驱动的做法）。测完 set 回 0.5 即可。
        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_wz', 1.5)

        # —— 里程计（2026-10-21 从 l150pro_ws 老驱动并过来，参数名照旧）——
        self.declare_parameter('frame_id_odom', 'odom')
        self.declare_parameter('frame_id_base', 'base_footprint')
        self.declare_parameter('frame_id_imu', 'imu_link')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('odom_yaw_source', 'gyro')    # gyro | wheel，见 odometry.py
        self.declare_parameter('track_eff', 1.0)             # 仅 odom_yaw_source=wheel 用；未标定
        self.declare_parameter('wheel_scale', 1.0)
        self.declare_parameter('odom_vx_variance', 1.0e-2)
        self.declare_parameter('odom_wz_variance', 1.0e-2)

        # —— 陀螺偏航速率闭环（yaw_loop.py；参数与老驱动同名同值）——
        # ★ 默认**关**（老驱动默认开）：老驱动的符号验证是在旧固件上做的，V2.0 机械臂版
        #   固件的 gz 符号还没在本车上验过 —— 判反就是正反馈（见 yaw_loop.py【安全】）。
        #   另外它会改变 PS2 手感（转向从"只转到 ~50%"变成"按指令转"）。
        #   验证：yaw_loop_enable=false 时发 wz=+0.2，看 /yaw_loop 的 wz_meas 是否同号。
        self.declare_parameter('yaw_loop_enable', False)
        self.declare_parameter('yaw_loop_kp', 0.2)
        self.declare_parameter('yaw_loop_ki', 0.2)
        self.declare_parameter('yaw_loop_max_wz_out', 2.0)
        self.declare_parameter('yaw_loop_i_clamp_frac', 1.5)
        self.declare_parameter('yaw_loop_out_rate_limit', 6.0)
        self.declare_parameter('yaw_loop_min_voltage', 10.5)

        self.pub_voltage = self.create_publisher(Float32, 'battery_voltage', 10)
        self.pub_faults = self.create_publisher(Int32, 'chassis_faults', 10)
        self.pub_fb = self.create_publisher(JointState, 'arm/feedback', 10)
        self.pub_online = self.create_publisher(Bool, 'arm/online', 10)
        self.pub_pot = self.create_publisher(Int32, 'arm/pot_raw', 10)
        self.pub_status = self.create_publisher(Int32, 'arm/status', 10)
        # 现在能不能给臂下 0xAC：ps2_teleop 靠它在回中/复位期间停发臂指令
        self.pub_cmd_gate = self.create_publisher(Bool, 'arm/cmd_gate', 10)
        # Firmware 0x56 ``base`` is already the unwrapped cumulative angle
        # estimate (0.36 degree per field count).  Publish the decoded value
        # so other Pi nodes do not mistake it for a modulo potentiometer.
        self.pub_base_angle = self.create_publisher(Float32, 'arm/base_angle_deg', 10)

        # ★ 2026-10-21：默认听 cmd_mux 的输出，不再直接听 cmd_vel（那现在是 mux 的兼容输入口）。
        #   单独起本节点调试时可 `-p cmd_topic:=cmd_vel` 退回直连。
        self.declare_parameter('cmd_topic', 'cmd_vel_mux')
        self.sub_twist = self.create_subscription(
            Twist, self.get_parameter('cmd_topic').value, self.on_twist, 10)
        self.sub_arm = self.create_subscription(JointState, 'arm/command', self.on_arm, 10)
        self.sub_estop = self.create_subscription(Bool, 'estop', self.on_estop, 10)
        self.srv_rehome = self.create_service(Trigger, 'arm/rehome', self.on_rehome)
        self.srv_rehome_hold = self.create_service(Trigger, 'arm/rehome_hold', self.on_rehome_hold)

        self.pub_odom = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.pub_driver_fault = self.create_publisher(Int16MultiArray, 'driver_fault', 10)
        self.pub_yaw_loop = self.create_publisher(Float32MultiArray, 'yaw_loop', 10)
        self.pub_raw55 = self.create_publisher(Float32MultiArray, 'chassis/raw55', 10)
        self.tf_bc = TransformBroadcaster(self)

        # 串口线程（喂陀螺/积分里程计）与 chassis_tick（闭环 step）共用
        self.ctl_lock = threading.Lock()
        self.odom = OdomIntegrator()
        self.yaw_loop = YawRateLoop(self._yaw_loop_config())
        self.latest55 = None
        self.last55_tick = None
        self.dropped55 = 0
        self.yaw_trip_logged = None
        self.t55_err_at = 0.0
        self.t_start = time.monotonic()   # 串口线程一起来就会用到（odom 协方差），必须在 rx.start() 前

        self.state = SerialState()
        self.rx = RxWorker(path, baud, self.state)
        self.rx.on_t55 = self.on_t55
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
        self.reset_t0 = None         # 复位窗口起点（None = 没有进行中的复位）
        self.reset_saw_dead = False  # 窗口内见过 control-live=0 吗（决定何时提前结束）
        self.low_volt_warned = False
        self.no55_warned = False
        self.arm_offline_warned = False
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
        # ★ 2026-10-28：复位窗口（本节点刚下发过 re-home）内一律不发。固件不报
        #   control-live 位的烧录版靠这一条兜底，否则帧会堆在信箱里、READY 时被套用。
        if self.reset_t0 is not None:
            return
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
        # arm_t_ms 每帧现读：抓取服务（arm_grasp/grasp_server）跑的时候临时 set 成 100，
        # 结束再 set 回来 —— 原来它得停掉整个服务另起一个驱动才能换这个值。
        t_ms = int(self.get_parameter('arm_t_ms').value)
        self.rx.send(build_arm(p1, p2, p3, p4, p5, base, t_ms, 0x00, self.seq_ac))

    def on_rehome(self, req, resp):
        return self._rehome(req, resp, keep_gripper=False)

    def on_rehome_hold(self, req, resp):
        """复位但**夹爪保持**（flags bit1，固件 2026-10-21）。给抓取服务收尾用。"""
        return self._rehome(req, resp, keep_gripper=True)

    def _rehome(self, _req, resp, keep_gripper):
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
        # ★ 2026-10-30：先补一帧 flags=0 把固件的边沿锁存清掉，再发复位帧。
        #   固件的 re-home 是边沿触发（ArmFlags 是"最近一帧的快照"，边沿要等下一帧
        #   flags=0 被消费才复位）。复位窗口期间我们把 0xAC 全掐了，用户复位后不碰臂
        #   ⇒ 没有任何 flags=0 的帧被消费 ⇒ 固件一直认为"还按着" ⇒ 下一次 START 被
        #   静默忽略。两帧之间隔 60ms：arm 任务 50Hz（20ms 一拍），保证前清帧先被读走。
        #   清边沿帧的数值 = 当前位姿，不含运动意图。
        self.seq_ac = (self.seq_ac + 1) & 0xFF
        self.rx.send(build_arm(*self.last_joints, self.last_base, 0, 0x00, self.seq_ac))
        time.sleep(0.06)
        self.seq_ac = (self.seq_ac + 1) & 0xFF
        keep = keep_gripper or bool(self.get_parameter('rehome_keep_gripper').value)
        flags = ARM_FLAG_REHOME | (ARM_FLAG_KEEP_GRIPPER if keep else 0)
        sent = self.rx.send(build_arm(*self.last_joints, self.last_base, 0, flags, self.seq_ac))
        resp.success = sent
        if not sent:
            resp.message = '串口没开，re-home 没发出去'
            self.get_logger().error('re-home 发送失败（串口没开）')
            return resp
        # ★ 2026-10-28：开复位窗口 —— 期间不发任何 0xAC（这条路径自己的帧也算），
        #   等固件 control-live 回到 1；固件不报这一位时 RESET_GUARD_S 兜底。
        self.reset_t0 = time.monotonic()
        self.reset_saw_dead = False
        resp.message = ('re-home 已下发（%s夹爪，约 5-15 秒）' % ('保持' if keep else '也回出厂位'))
        self.get_logger().info('re-home 已下发 flags=0x%02x（%s）'
                               % (flags, '保持夹爪' if keep else '五关节回出厂 + 底座回中'))
        return resp

    # —— 0x55：里程计 / IMU / 陀螺喂闭环（串口线程里调）——
    def on_t55(self, d, rx_time):
        try:
            self._handle_t55(d, rx_time)
        except Exception as e:      # 发布出错不能把串口解析一起带崩
            if rx_time - self.t55_err_at > 5.0:
                self.t55_err_at = rx_time
                self.get_logger().error(f'0x55 处理异常: {e}')

    def _handle_t55(self, d, rx_time):
        tick = d['tick_ms']
        if self.last55_tick is not None:
            delta = (tick - self.last55_tick) & 0xFFFFFFFF
            if delta > 30:                       # 标称 20ms 一帧
                self.dropped55 += max(0, delta // 20 - 1)
        self.last55_tick = tick

        with self.ctl_lock:
            self.latest55 = d
            self.yaw_loop.update_gyro(d['gz'], rx_time)
            vx, wz = self.odom.update(
                d['wheels'], d['gz'], rx_time,
                track_eff=float(self.get_parameter('track_eff').value),
                wheel_scale=float(self.get_parameter('wheel_scale').value),
                yaw_source=str(self.get_parameter('odom_yaw_source').value))
            x, y = self.odom.x, self.odom.y
            qz, qw = self.odom.quaternion_z_w()

        # 用【接收时刻】做时间戳：固件 tick 与 Pi 时钟不同源（同老驱动）
        stamp = self.get_clock().now().to_msg()
        f_odom = self.get_parameter('frame_id_odom').value
        f_base = self.get_parameter('frame_id_base').value

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = f_odom
        od.child_frame_id = f_base
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = vx
        od.twist.twist.angular.z = wz
        od.twist.covariance[0] = float(self.get_parameter('odom_vx_variance').value)
        od.twist.covariance[7] = 1.0e3            # 差速车无横移
        od.twist.covariance[35] = float(self.get_parameter('odom_wz_variance').value)
        drift = max(1e-4, (rx_time - self.t_start) * 1e-5)
        od.pose.covariance[0] = drift
        od.pose.covariance[7] = drift
        od.pose.covariance[35] = max(drift, 1.0e-2)
        self.pub_odom.publish(od)

        if self.get_parameter('publish_odom_tf').value:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = f_odom
            tf.child_frame_id = f_base
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_bc.sendTransform(tf)

        im = Imu()
        im.header.stamp = stamp
        im.header.frame_id = self.get_parameter('frame_id_imu').value
        im.angular_velocity.z = d['gz']
        im.angular_velocity_covariance[0] = -1.0   # 协议只有 gz
        im.angular_velocity_covariance[4] = -1.0
        im.angular_velocity_covariance[8] = 1.0e-3
        im.linear_acceleration_covariance[0] = -1.0
        im.orientation_covariance[0] = -1.0        # yaw 纯陀螺积分会漂，不发姿态
        self.pub_imu.publish(im)

        raw = Float32MultiArray()
        raw.data = [float(v) for v in d['wheels']] + [
            float(d['roll']), float(d['pitch']), float(d['yaw']), float(d['gz']),
            float(d['tick_ms'])]
        self.pub_raw55.publish(raw)

        df = Int16MultiArray()
        df.data = [int(d['faults']), 0, 0, min(int(self.dropped55), 32767)]
        self.pub_driver_fault.publish(df)

    # —— 偏航闭环 ——
    def _yaw_loop_config(self):
        return YawLoopConfig(
            kp=float(self.get_parameter('yaw_loop_kp').value),
            ki=float(self.get_parameter('yaw_loop_ki').value),
            max_wz_out=float(self.get_parameter('yaw_loop_max_wz_out').value),
            i_clamp_frac=float(self.get_parameter('yaw_loop_i_clamp_frac').value),
            out_rate_limit=float(self.get_parameter('yaw_loop_out_rate_limit').value),
            dt_nominal=1.0 / 25.0,
        )

    def _yaw_loop_allowed(self):
        """**调用时必须持有 ctl_lock**。门控条件同老驱动 `_yaw_loop_allowed`。"""
        if self.estop or not self.get_parameter('yaw_loop_enable').value:
            return False
        d = self.latest55
        if d is None or d['faults'] & (FAULT_IMU | FAULT_UNDERVOLT):
            return False
        return d['voltage_mv'] / 1000.0 >= float(
            self.get_parameter('yaw_loop_min_voltage').value)

    def _log_yaw_trip(self, reason):
        if not reason:
            self.yaw_trip_logged = None
            return
        if reason == self.yaw_trip_logged:
            return
        self.yaw_trip_logged = reason
        if reason == 'stall':
            self.get_logger().error('偏航闭环：检测到【陀螺卡死】，已退回直通 wz_ref；松杆回中解除')
        elif reason == 'wrong_sign':
            self.get_logger().error('偏航闭环：检测到【陀螺符号与指令相反】（正反馈），'
                                    '已退回直通；先确认符号再用闭环！')

    # —— 定时器 ——
    def chassis_tick(self):
        now = time.monotonic()
        if self.estop or (now - self.tw_time) > CMD_STALE_S:
            vx = wz_ref = 0.0
        else:
            # 限幅值每拍现读一次 ⇒ `ros2 param set` 立刻生效（默认仍是冻结版的 0.5/1.5）
            lim_vx = float(self.get_parameter('max_vx').value)
            lim_wz = float(self.get_parameter('max_wz').value)
            vx = min(max(self.tw.linear.x, -lim_vx), lim_vx)
            wz_ref = min(max(self.tw.angular.z, -lim_wz), lim_wz)
        # 闭环在零速分支里也要 step（allowed=False ⇒ 直通并清状态），
        # 否则急停/超时期间它还攥着旧积分，解除时冲出去。
        with self.ctl_lock:
            self.yaw_loop.cfg = self._yaw_loop_config()
            wz = self.yaw_loop.step(wz_ref, now, allowed=self._yaw_loop_allowed())
            diag = (wz_ref, self.yaw_loop.wz_meas, wz, self.yaw_loop.integral)
            trip = self.yaw_loop.trip_reason
        if self.estop:
            wz = 0.0
        self._log_yaw_trip(trip)
        ym = Float32MultiArray()
        ym.data = [float(diag[0]), float('nan') if diag[1] is None else float(diag[1]),
                   float(diag[2]), float(diag[3])]
        self.pub_yaw_loop.publish(ym)
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
        # ★ 2026-10-28：复位窗口 —— 结束条件是"固件自报 control-live 回来了"（权威），
        #   固件不报这一位（ctrl_ever=False）时到 RESET_GUARD_S 兜底放行。
        if self.reset_t0 is not None:
            if self.ctrl_ever and not ctrl_live:
                self.reset_saw_dead = True
            if (self.reset_saw_dead and ctrl_live) or \
                    (now - self.reset_t0) > RESET_GUARD_S:
                self.get_logger().info(
                    f'复位窗口结束（{now - self.reset_t0:.1f}s）：恢复臂指令')
                self.reset_t0 = None
            elif self.ctrl_ever and not self.reset_saw_dead and \
                    (now - self.reset_t0) > RESET_ARM_S:
                # 固件压根没进 HOMING ⇒ 这次复位请求没被执行（边沿丢了/帧丢了），
                # 没必要继续把臂锁着 —— 固件一直在 READY，指令本来就会被正常消费。
                self.get_logger().warn(
                    f'复位请求 {now - self.reset_t0:.1f}s 内没被执行'
                    '（固件没进 HOMING，臂没动），提前放行')
                self.reset_t0 = None
        # 现在能不能给臂下 0xAC。只在收到过 0x56 之后发：没数据 ≠ 固件在复位
        # （否则 driver 刚起那一秒 ps2_teleop 会把臂门住）。
        gate = (self.reset_t0 is None
                and not (self.ctrl_ever and not ctrl_live
                         and (now - self.ctrl_ok_at) <= CTRL_WAIT_MAX_S))
        if seen56:
            m = Bool()
            m.data = gate
            self.pub_cmd_gate.publish(m)

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
            angle = Float32()
            angle.data = float(base) * (360.0 / 1000.0)
            self.pub_base_angle.publish(angle)

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
