"""L150Pro C30D(2.0) ROS 2 驱动节点。

职责边界（重要，别越界）：
  STM32 侧   : 电机闭环、轮速测量、IMU 原始量、看门狗、舵机 PWM
  本节点侧   : 协议收发、里程计积分、ROS 话题发布/订阅
  EKF 融合   : 交给 robot_localization 的 ekf_node（见 config/ekf.yaml）

本节点不做传感器融合——那是 ekf_node 的活。

关于 IMU 的一个硬约束：
  下位机的姿态是"陀螺仪 + 加速度计"互补滤波，**磁力计从未参与**。
  因此 roll/pitch 有重力参考（绝对、不漂），但 **yaw 是纯陀螺仪积分、
  会持续漂移、无绝对参考**。
  所以本节点默认【不发布 IMU 的 orientation】，只发角速度和线加速度，
  航向由 ekf_node 从轮式里程计观测。详见 imu 相关参数。
"""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, Int16MultiArray

try:
    import serial
    from serial.serialutil import SerialException
except ImportError:            # 允许在没有 pyserial 的环境下 import 做静态检查
    serial = None
    SerialException = Exception

from .protocol import (
    FAULT_ENCODER, FAULT_ESTOP, FAULT_IMU, FAULT_OVERVOLT,
    FAULT_RX_TIMEOUT, FAULT_UNDERVOLT,
    FLAG_ENABLE, FLAG_ESTOP,
    SERVO_CENTER_US, SERVO_CHANNELS, SERVO_MAX_US, SERVO_MIN_US,
    TRACK_EFF, Uplink, UplinkParser,
    build_motion_frame, build_servo_frame, servo_command_to_pulse,
)


class L150ProDriver(Node):

    def __init__(self):
        super().__init__("l150pro_driver")

        # ---------------- 参数 ----------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id_odom", "odom")
        self.declare_parameter("frame_id_base", "base_footprint")
        self.declare_parameter("frame_id_imu", "imu_link")

        # 控制频率：运动帧 20Hz（固件看门狗 200ms，必须持续发）
        self.declare_parameter("cmd_rate_hz", 20.0)
        # cmd_vel 限幅。
        # 默认取得很保守（0.5 m/s）。
        # 注意 2026-09 换车后，固件轮径已由 0.065 改成实测的 0.125，指令速度
        # 与实车速度现在是 1:1；此前固件轮径写小，实车约是指令的 1.92 倍。
        # 所以同样发 0.5 会比换车前体感更慢，那是修正对了，不是回归。
        # 标定完成后可以放心调大。
        self.declare_parameter("max_vx", 0.5)
        self.declare_parameter("max_vy", 0.5)
        self.declare_parameter("max_wz", 1.5)

        # ---------------- 标定参数（装车实测后改，不用改代码）----------------
        #
        # track_eff: 有效全轮距，用于从左右轮速反解角速度。
        #   默认 1.000 = 固件里的 W+A = 0.460+0.540（2026-09 换车后实测）。
        #   注意是【全】轮距不是半轮距——用半轮距会让角速度大一倍。
        #   这是外推初值不是标定值：滑移转向的等效轮距含侧滑修正，
        #   需原地转标定（monitor.py 按 c 键）。标定前 odom 的 yaw 不要信。
        self.declare_parameter("track_eff", TRACK_EFF)
        #
        # wheel_scale: 轮速比例修正，补偿固件里轮径配置与实际不符。
        #   固件用 Encoder_precision 和 Wheel_perimeter 把编码器计数换算成 m/s，
        #   轮径配错会让上报速度整体等比缩放。
        #   标定方法：指令车走已知距离 D_指令，量实际距离 D_实际，
        #             则 wheel_scale = D_实际 / D_指令
        #   （注意：这只修正【上报值】，不改变车的实际速度——那要在固件里改轮径）
        #   2026-09 换车后固件轮径已改对，此项应保持 1.0，只吸收直线标定的
        #   残余误差（滚动半径/打滑，预计 1~2%），超过 5% 才值得动。
        self.declare_parameter("wheel_scale", 1.0)

        # IMU orientation 发布策略，见文件头说明
        #   False(默认) = 不发布 orientation，只发角速度+线加速度
        #   True        = 发布 orientation（roll/pitch 准，yaw 按下方协方差加权）
        self.declare_parameter("publish_imu_orientation", False)
        # yaw 方差（仅在上一条为 True 时生效）。单位 rad^2。
        #
        # 实测漂移 0.17 度/分钟（冷机、静止、电机不转）。但这是最好的情况：
        # 电机振动会让陀螺产生整流误差，热机后温漂也会改变零偏。
        # 所以【不要贴着实测值设】，按 1~2 度/分钟量级留余量。
        #   1e-2  -> 标准差约 5.7 度，明显悲观但让 EKF 仍能参考
        #   1e-3  -> 标准差约 1.8 度，接近实测，偏乐观
        #   1e3   -> 等效于忽略（不要用这个，那就直接设
        #            publish_imu_orientation=false 更清晰）
        self.declare_parameter("yaw_variance", 1.0e-2)

        # 轮式里程计协方差（粗略，够 EKF 用；要精细请实测标定）
        self.declare_parameter("odom_vx_variance", 1.0e-2)
        self.declare_parameter("odom_wz_variance", 1.0e-2)

        self.declare_parameter("publish_diagnostics", True)

        # 云台水平通道（ch5）方向。
        #   true(默认) = 物理舵机是反装的，出线前翻一次，使 ch5 值增大 = 往右
        #   false      = 直通（把舵机装正、或换了一颗不反装的之后用这个）
        # 见 protocol.servo_command_to_pulse() 的契约说明。
        self.declare_parameter("pan_invert", True)

        # ---------------- 状态 ----------------
        self._parser = UplinkParser()
        self._latest: Optional[Uplink] = None
        self._last_rx_time: Optional[float] = None

        # 里程计累积位姿
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_odom_time: Optional[float] = None
        self._odom_start = time.monotonic()

        # 待发送的控制量（由 cmd_vel 回调更新，由定时器发出）
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._cmd_estop = False
        self._seq = 0

        self._servo = [SERVO_CENTER_US] * SERVO_CHANNELS
        self._servo_dirty = True
        self._servo_seq = 0

        self._lock = threading.Lock()
        self._serial = None
        self._reader_stop = threading.Event()

        # ---------------- 串口 ----------------
        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        if serial is None:
            raise RuntimeError("未安装 pyserial：pip install pyserial")

        # exclusive=True 走 TIOCEXCL：独占打开，第二个实例会【立刻失败】而不是
        # 静默地无限报 "device reports readiness to read but returned no data"。
        # CDC-ACM 设备允许多进程同时 open，但读取会互相干扰，症状极难定位——
        # 这个坑我们踩过，所以宁可启动就报错。
        # 某些平台/驱动不支持 TIOCEXCL，失败时退回非独占并给出提示。
        self._serial = None
        for exclusive in (True, False):
            try:
                self._serial = serial.Serial(port, baud, timeout=0.05,
                                             exclusive=exclusive)
                break
            except (SerialException, OSError, ValueError) as e:
                if exclusive:
                    self.get_logger().warn(
                        f"独占打开 {port} 失败（可能已被别的进程占用）：{e}；"
                        f"退回非独占模式，若读到 EIO 请检查是否有另一个实例在跑")
                    continue
                raise RuntimeError(f"打开串口 {port} 失败：{e}") from e
        self.get_logger().info(f"串口已打开 {port} @ {baud}")

        # ---------------- ROS 接口 ----------------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self.pub_odom = self.create_publisher(Odometry, "wheel_odom", qos)
        self.pub_imu = self.create_publisher(Imu, "imu/data_raw", qos)
        self.pub_fault = self.create_publisher(Int16MultiArray, "driver_fault", qos)
        # 电压**不受 publish_diagnostics 控制**：它是判断供电是否够用的核心指标，
        # 欠压是这台车最常踩的故障（见 README 的供电一节），应当始终发布。
        self.pub_battery = self.create_publisher(Float32, "battery_voltage", qos)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, qos)
        self.create_subscription(TwistStamped, "cmd_vel_stamped",
                                 self._on_cmd_vel_stamped, qos)
        self.create_subscription(JointState, "servo_cmd", self._on_servo, qos)

        # ---------------- 线程与定时器 ----------------
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        rate = float(self.get_parameter("cmd_rate_hz").value)
        self.create_timer(1.0 / rate, self._tx_timer)
        self.create_timer(1.0, self._status_timer)

        # 启动时先发几帧带使能的零速帧，避免"节点起来了但车不动"的困惑
        self._priming_frames = 10

        self.get_logger().info(
            f"就绪 | 有效轮距 "
            f"{float(self.get_parameter('track_eff').value):.3f} m | "
            f"cmd_vel 限幅 vx±{self.get_parameter('max_vx').value} "
            f"wz±{self.get_parameter('max_wz').value}")

    # ======================================================================
    # 串口读取
    # ======================================================================

    def _read_loop(self):
        fails = 0
        while not self._reader_stop.is_set():
            try:
                data = self._serial.read(256)
            except Exception as e:                       # 串口拔出等
                fails += 1
                if fails == 10:
                    # 连续失败到一定次数才给一次可操作的提示，之后降噪
                    self.get_logger().error(
                        f"串口连续读取失败 {fails} 次：{e}\n"
                        f"  最常见原因是【另一个进程也打开了同一个串口】。查：\n"
                        f"    sudo fuser -v {self.get_parameter('port').value}\n"
                        f"    ps aux | grep driver_node\n"
                        f"  ModemManager 也可能抢 ttyACM 设备，见 README 的 udev 规则。")
                elif fails < 10:
                    self.get_logger().error(f"串口读取失败：{e}")
                time.sleep(0.5)
                continue
            fails = 0
            if not data:
                continue
            for up in self._parser.feed(data):
                with self._lock:
                    self._latest = up
                self._last_rx_time = time.monotonic()
                self._publish_state(up)

    # ======================================================================
    # 发布
    # ======================================================================

    def _publish_state(self, up: Uplink):
        # 用【接收时刻】做时间戳，不用固件的 tick——两者时钟不同源。
        stamp = self.get_clock().now().to_msg()
        self._publish_odom(up, stamp)
        self._publish_imu(up, stamp)

        battery = Float32()
        battery.data = float(up.voltage)
        self.pub_battery.publish(battery)

        if self.get_parameter("publish_diagnostics").value:
            self._publish_fault(up)

    def _publish_odom(self, up: Uplink, stamp):
        now = time.monotonic()
        if self._last_odom_time is None:
            dt = 0.0
        else:
            dt = now - self._last_odom_time
            if dt > 0.5:        # 长时间断流，不做跨断层的积分
                dt = 0.0
        self._last_odom_time = now

        # 用参数化的轮距与比例，而不是模块常量——装车实测后要能直接改参数
        scale = float(self.get_parameter("wheel_scale").value)
        track = float(self.get_parameter("track_eff").value)
        v_left = (up.vA + up.vB) * 0.5 * scale
        v_right = (up.vC + up.vD) * 0.5 * scale
        vx = (v_left + v_right) * 0.5
        wz = (v_right - v_left) / track

        # 四驱差速是滑移转向，转向时明显打滑，这里用手臂积分（2D 刚体）
        if dt > 0.0:
            self._theta += wz * dt
            self._x += vx * math.cos(self._theta) * dt
            self._y += vx * math.sin(self._theta) * dt

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter("frame_id_odom").value
        msg.child_frame_id = self.get_parameter("frame_id_base").value

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._theta / 2.0)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz

        # 协方差：轮式里程计对 vx/wz 的信心，对绝对位姿的信心随时间下降
        vv = float(self.get_parameter("odom_vx_variance").value)
        wv = float(self.get_parameter("odom_wz_variance").value)
        msg.twist.covariance[0] = vv          # vx
        msg.twist.covariance[7] = 1.0e3       # vy 不可观测（差速车无横移）
        msg.twist.covariance[35] = wv         # wz

        # 位姿协方差随积分时间增大；yaw 因无绝对参考，给一个较大的基值
        drift = max(1e-4, (now - self._odom_start) * 1e-5)
        msg.pose.covariance[0] = drift
        msg.pose.covariance[7] = drift
        msg.pose.covariance[35] = max(drift, 1.0e-2)

        self.pub_odom.publish(msg)

    def _publish_imu(self, up: Uplink, stamp):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter("frame_id_imu").value

        # 角速度：gz 是陀螺仪原始测量，【可信】
        msg.angular_velocity.z = up.gz
        # 另外两轴的角速度下位机没发（协议里只有 gz）。
        # 标 -1 表示"该字段本消息不可用"，符合 REP-145 约定。
        msg.angular_velocity_covariance[0] = -1.0
        msg.angular_velocity_covariance[4] = -1.0
        msg.angular_velocity_covariance[8] = 1.0e-3

        # 线加速度：协议里没有，标不可用。
        # 若以后要在 EKF 里用加速度，需要先扩展上行帧。
        msg.linear_acceleration_covariance[0] = -1.0

        if self.get_parameter("publish_imu_orientation").value:
            # 方案 B：发 orientation，但让 EKF 基本忽略 yaw
            cy, sy = math.cos(up.yaw / 2.0), math.sin(up.yaw / 2.0)
            cp, sp = math.cos(up.pitch / 2.0), math.sin(up.pitch / 2.0)
            cr, sr = math.cos(up.roll / 2.0), math.sin(up.roll / 2.0)
            msg.orientation.w = cy * cp * cr + sy * sp * sr
            msg.orientation.x = cy * cp * sr - sy * sp * cr
            msg.orientation.y = cy * sp * cr + sy * cp * sr
            msg.orientation.z = sy * cp * cr - cy * sp * sr
            msg.orientation_covariance[0] = 1.0e-3    # roll 可信（重力参考）
            msg.orientation_covariance[4] = 1.0e-3    # pitch 可信
            msg.orientation_covariance[8] = float(
                self.get_parameter("yaw_variance").value)
        else:
            # 方案 A（默认）：不发 orientation，明确告知 ROS 本消息不含姿态
            msg.orientation_covariance[0] = -1.0

        self.pub_imu.publish(msg)

    def _publish_fault(self, up: Uplink):
        msg = Int16MultiArray()
        msg.data = [up.fault, self._parser.stats.crc_err,
                    self._parser.stats.head_err, self._parser.stats.dropped]
        self.pub_fault.publish(msg)

    # ======================================================================
    # 下行
    # ======================================================================

    def _on_cmd_vel(self, msg: Twist):
        max_vx = float(self.get_parameter("max_vx").value)
        max_vy = float(self.get_parameter("max_vy").value)
        max_wz = float(self.get_parameter("max_wz").value)
        with self._lock:
            self._cmd_vx = max(-max_vx, min(max_vx, msg.linear.x))
            self._cmd_vy = max(-max_vy, min(max_vy, msg.linear.y))
            self._cmd_wz = max(-max_wz, min(max_wz, msg.angular.z))

    def _on_cmd_vel_stamped(self, msg: TwistStamped):
        self._on_cmd_vel(msg.twist)

    def _on_servo(self, msg: JointState):
        """JointState.position 按顺序映射到 ch1..ch6，单位微秒。

        值 <500 视为"该路保持不变"（固件约定 0 = 不变）。
        """
        with self._lock:
            # 每次读参数而不是缓存：这样 ros2 param set 能立刻生效，
            # 调方向时不用重启节点。
            pan_invert = bool(self.get_parameter("pan_invert").value)
            for i, p in enumerate(msg.position[:SERVO_CHANNELS]):
                if p < SERVO_MIN_US:
                    self._servo[i] = 0                     # 保持不变
                else:
                    self._servo[i] = servo_command_to_pulse(
                        i + 1, int(min(SERVO_MAX_US, p)), pan_invert)
            self._servo_dirty = True

    def _tx_timer(self):
        if self._serial is None:
            return

        # 1) 运动帧
        flags = 0
        with self._lock:
            vx, vy, wz = self._cmd_vx, self._cmd_vy, self._cmd_wz
            estop = self._cmd_estop
            primed = self._priming_frames > 0
            if primed:
                self._priming_frames -= 1

        # 前若干帧强制零速但带使能，帮固件 s_have_link 建立链路
        if primed:
            vx = vy = wz = 0.0
        if estop:
            flags |= FLAG_ESTOP
            vx = vy = wz = 0.0
        flags |= FLAG_ENABLE

        self._seq = (self._seq + 1) & 0xFF
        frame = build_motion_frame(vx, vy, wz, flags, self._seq)

        with self._lock:
            servo_dirty = self._servo_dirty
            if servo_dirty:
                servo = list(self._servo)
                self._servo_dirty = False

        try:
            self._serial.write(frame)
            # 2) 舵机帧只在有变化时发（固件侧值 0 = 保持不变）
            if servo_dirty:
                self._servo_seq = (self._servo_seq + 1) & 0xFF
                self._serial.write(build_servo_frame(servo, self._servo_seq))
        except Exception as e:
            self.get_logger().error(f"串口写入失败：{e}")

    # ======================================================================
    # 状态监控
    # ======================================================================

    def _status_timer(self):
        now = time.monotonic()
        if self._last_rx_time is None:
            self.get_logger().warn("尚未收到任何上行帧——检查接线/波特率/共地",
                                   throttle_duration_sec=5.0)
            return

        age = now - self._last_rx_time
        if age > 0.5:
            self.get_logger().error(
                f"{age:.1f}s 未收到上行帧，下位机可能已复位或掉线",
                throttle_duration_sec=2.0)
            return

        with self._lock:
            up = self._latest
        if up is None:
            return

        names = up.fault_names()
        if names:
            self.get_logger().warn(
                f"下位机故障位：{'/'.join(names)}  电池 {up.voltage:.2f}V",
                throttle_duration_sec=2.0)

        s = self._parser.stats
        if s.crc_err or s.head_err or s.dropped:
            self.get_logger().info(
                f"链路: 帧{s.frames} crc_err{s.crc_err} "
                f"head_err{s.head_err} dropped{s.dropped} "
                f"| 电池 {up.voltage:.2f}V",
                throttle_duration_sec=10.0)

        # 低压硬提醒（固件 10V 以下会禁止运动）
        if up.voltage < 10.5:
            self.get_logger().error(
                f"电池电压 {up.voltage:.2f}V —— 低于 10V 时固件会拒绝运动",
                throttle_duration_sec=5.0)

    def destroy_node(self):
        self._reader_stop.set()
        try:
            # 退出前发一帧零速，避免车带着最后速度冲出去
            if self._serial is not None:
                self._serial.write(
                    build_motion_frame(0.0, 0.0, 0.0, FLAG_ENABLE, 0))
                time.sleep(0.05)
                self._serial.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = L150ProDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
