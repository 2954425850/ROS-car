# -*- coding: utf-8 -*-
"""底盘速度仲裁（纯 Python 核心 + ROS 节点）。

多个速度源各发各的话题，本节点按优先级选一个转发到 `cmd_vel_mux`，驱动只听这一个。

| 源 | 话题 | 优先级 | 超时 |
|---|---|---|---|
| PS2 手柄 | cmd_vel_teleop | 100 | 0.5s |
| 兼容口（云端 rosbridge、没迁移的老脚本） | cmd_vel | 80 | 0.5s |
| 语音助手 | cmd_vel_voice | 50 | 0.5s |
| 导航（以后） | cmd_vel_nav | 10 | 0.5s |

规则：在超时窗口内发过消息的源里，优先级最高的那个说了算；谁都没在发 ⇒ 发零速。
**"在发"就等于"要控制权"** —— 所以各源空闲时必须**不发**（PS2 已改成松杆后只发
0.3s 零速尾巴就停），不能用一串 0 霸着通道。

为什么不用 twist_mux：Pi 上没装，而且这里要的就这么点逻辑；核心是纯函数，能离线测。
急停不在这里：`/estop` 由驱动直接锁存（底盘清零 + 机械臂保持），不经仲裁，谁也绕不过。
"""
import time

DEFAULT_SOURCES = (
    # (名字, 话题, 优先级, 超时 s)
    ('teleop', 'cmd_vel_teleop', 100, 0.5),
    ('legacy', 'cmd_vel', 80, 0.5),
    ('voice', 'cmd_vel_voice', 50, 0.5),
    ('nav', 'cmd_vel_nav', 10, 0.5),
)


class MuxCore:

    def __init__(self, sources=DEFAULT_SOURCES):
        self.sources = {name: (prio, timeout) for name, _t, prio, timeout in sources}
        self.last = {}           # name -> (t, vx, wz)

    def feed(self, name, vx, wz, t):
        self.last[name] = (t, float(vx), float(wz))

    def select(self, t):
        """返回 (源名 或 None, vx, wz)。None = 没有活跃源 ⇒ 零速。"""
        best = None
        for name, (ts, vx, wz) in self.last.items():
            prio, timeout = self.sources[name]
            if t - ts > timeout:
                continue
            if best is None or prio > best[0]:
                best = (prio, name, vx, wz)
        if best is None:
            return None, 0.0, 0.0
        return best[1], best[2], best[3]


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import String

    class CmdMuxNode(Node):

        def __init__(self):
            super().__init__('cmd_mux')
            self.declare_parameter('rate_hz', 25.0)
            self.core = MuxCore()
            self.pub = self.create_publisher(Twist, 'cmd_vel_mux', 10)
            self.pub_active = self.create_publisher(String, 'cmd_mux/active', 10)
            for name, topic, _p, _t in DEFAULT_SOURCES:
                self.create_subscription(
                    Twist, topic, lambda m, n=name: self.core.feed(
                        n, m.linear.x, m.angular.z, time.monotonic()), 10)
            self.prev = 'init'
            self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
            self.get_logger().info('cmd_mux 启动：' + ' > '.join(
                f'{t}({p})' for _n, t, p, _o in DEFAULT_SOURCES) + ' → cmd_vel_mux')

        def tick(self):
            name, vx, wz = self.core.select(time.monotonic())
            m = Twist()
            m.linear.x = vx
            m.angular.z = wz
            self.pub.publish(m)
            if name != self.prev:
                self.get_logger().info(f'速度控制权 → {name or "无（零速）"}')
                self.prev = name
                s = String()
                s.data = name or ''
                self.pub_active.publish(s)

    rclpy.init(args=args)
    node = CmdMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
