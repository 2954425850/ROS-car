"""ROS 2 适配层 —— 全项目唯一 import rclpy 的地方。

单独拆出来的三个理由：
  1. CarController 的闭环逻辑（走两米、转九十度）能在假桥上测，不需要插着车。
  2. 将来导航/建图节点如果只想要「发速度、读里程计」，可以只用这一层。
  3. ROS 环境加载失败是**静默**的（systemd 不会 source setup.bash），
     集中在一处才好做出声的检查。
"""

from __future__ import annotations

import threading
import time

from loguru import logger

from car.types import RobotState

_SERVO_CHANNELS = 6


class RosUnavailable(RuntimeError):
    """ROS 2 环境没准备好。故意做成吵闹的异常，不要静默降级。"""


def ensure_ros_available() -> None:
    """在 import rclpy 之前给出可操作的报错。

    systemd 用户单元不会 source ROS 的 setup.bash，失败表现为
    「ModuleNotFoundError: No module named 'rclpy'」——那句话对使用者
    毫无指导意义。这里换成能照着做的提示。
    """
    try:
        import rclpy  # noqa: F401
    except ImportError as exc:
        raise RosUnavailable(
            "加载 rclpy 失败 —— ROS 2 环境没有准备好。\n"
            "  由 systemd 启动时：确认单元里的 ExecStart 指向 voice-chatbot/run.sh"
            "（它会先 source ROS 再跑 main.py）。\n"
            "  手工运行请先执行：\n"
            "    source /opt/ros/jazzy/setup.bash\n"
            "    source ~/l150pro_ws/install/setup.bash"
        ) from exc


class RosBridge:
    """对 ROS 2 的薄封装：发 cmd_vel / servo_cmd，收上行状态。

    线程安全：publish_* 只做加锁后调 publisher.publish()；state() 返回不可变
    快照。所以发帧线程和闭环逻辑可以同时用它。
    """

    def __init__(self, *, cmd_rate_hz: float = 25.0) -> None:
        ensure_ros_available()

        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Float32, Int16MultiArray

        self._rclpy = rclpy
        self._Twist = Twist
        self._JointState = JointState

        rclpy.init(args=[])
        self._node = Node("voice_chatbot_car")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        # ---- 下行 ----
        self._pub_vel = self._node.create_publisher(Twist, "cmd_vel", qos)
        self._pub_servo = self._node.create_publisher(JointState, "servo_cmd", qos)

        # ---- 上行（全部只读，供闭环与 car_status 用）----
        self._lock = threading.Lock()
        self._voltage: float | None = None
        self._fault: int | None = None
        self._wheels: tuple[float, float, float, float] | None = None
        self._yaw_rate: float | None = None
        self._traveled = 0.0
        self._last_xy: tuple[float, float] | None = None
        self._online = False

        self._node.create_subscription(Odometry, "wheel_odom", self._on_odom, qos)
        self._node.create_subscription(Imu, "imu/data_raw", self._on_imu, qos)
        self._node.create_subscription(
            Int16MultiArray, "driver_fault", self._on_fault, qos
        )
        self._node.create_subscription(
            Float32, "battery_voltage", self._on_voltage, qos
        )

        # ---- 自旋 ----
        # 用**自己的** executor，不要碰 rclpy 的进程级全局 executor。
        #
        # 裸调 `rclpy.spin_once(node)` 会落到 `get_global_executor()`，而那个
        # executor **整个进程只有一份**。桥的自旋线程先占住它之后，同进程里
        # 任何别处再裸调一次 spin_once 就会撞
        # `RuntimeError: Executor is already spinning`（已实测复现）。
        #
        # 将来导航/建图如果和助手同进程，它们也会自旋 —— 这条是给那时候留的路。
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(
            target=self._spin, name="car-ros-spin", daemon=True
        )
        self._spin_thread.start()

        # 3 秒后查一次 cmd_vel 有没有订阅者（同 teleop 的排错提示）。
        # 用定时器而不是「第一帧就查」：DDS 发现需要时间，第一帧查必然误报。
        self._link_timer = self._node.create_timer(3.0, self._check_link)
        logger.info("小车：ROS 桥已就绪（节点 voice_chatbot_car）")

    # ---------------------------------------------------------------- 发布

    def publish_velocity(self, vx: float, wz: float) -> None:
        """发一帧速度。[只取 linear.x 和 angular.z —— 差速车没有横移]"""
        msg = self._Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        with self._lock:
            self._pub_vel.publish(msg)

    def publish_servo(self, us: list[int]) -> None:
        """发一帧舵机脉宽。长度必须为 6；0 表示该路保持不动。"""
        if len(us) != _SERVO_CHANNELS:
            raise ValueError(
                f"舵机帧需要 {_SERVO_CHANNELS} 路，收到 {len(us)}"
            )
        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = [f"ch{i}" for i in range(1, _SERVO_CHANNELS + 1)]
        msg.position = [float(x) for x in us]
        with self._lock:
            self._pub_servo.publish(msg)

    # ---------------------------------------------------------------- 状态

    def state(self) -> RobotState:
        with self._lock:
            return RobotState(
                voltage=self._voltage,
                fault=self._fault,
                wheel_speeds=self._wheels,
                yaw_rate=self._yaw_rate,
                traveled=self._traveled,
                online=self._online,
            )

    # ---------------------------------------------------------------- 内部

    def _spin(self) -> None:
        while not self._spin_stop.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as exc:  # noqa: BLE001 —— 自旋不能因为一次异常就死
                logger.error(f"小车：ROS 自旋异常 —— {exc}")
                time.sleep(0.1)

    def _check_link(self) -> None:
        """只跑一次：cmd_vel 上没有订阅者 = 底盘驱动没在跑。"""
        self._node.destroy_timer(self._link_timer)
        if self._pub_vel.get_subscription_count() == 0:
            logger.warning(
                "小车：cmd_vel 上没有订阅者 —— 底盘驱动没在跑？"
                "另开终端执行：roslaunch 见 ~/l150pro_start_driver.sh"
            )

    def _on_odom(self, msg) -> None:
        p = msg.pose.pose.position
        with self._lock:
            if self._last_xy is not None:
                dx = p.x - self._last_xy[0]
                dy = p.y - self._last_xy[1]
                self._traveled += (dx * dx + dy * dy) ** 0.5
            self._last_xy = (p.x, p.y)
            self._online = True

    def _on_imu(self, msg) -> None:
        with self._lock:
            self._yaw_rate = float(msg.angular_velocity.z)
            self._online = True

    def _on_fault(self, msg) -> None:
        data = list(msg.data)
        with self._lock:
            if data:
                self._fault = int(data[0])
            self._online = True

    def _on_voltage(self, msg) -> None:
        with self._lock:
            self._voltage = float(msg.data)
            self._online = True

    # ---------------------------------------------------------------- 收尾

    def close(self) -> None:
        """关自旋线程并销毁节点。**不调 rclpy.shutdown()** ——
        它是进程级的，将来同进程还有别的 ROS 消费者时会把它们一起关掉。"""
        self._spin_stop.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        try:
            self._executor.remove_node(self._node)
            self._executor.shutdown()
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"小车：关闭 executor 失败 —— {exc}")
        try:
            self._node.destroy_node()
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"小车：销毁 ROS 节点失败 —— {exc}")
