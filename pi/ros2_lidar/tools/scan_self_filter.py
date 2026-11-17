#!/usr/bin/env python3
"""Drop scan points that hit the robot itself.

The RPLIDAR sits 0.256 m behind base_link and the arm sits ~0.18 m in front of
base_link, so the arm shows up as a permanent return at 0 deg / ~0.44 m from
the laser. That is inside the robot footprint (0.60 x 0.40 m), yet nav2 sees it
as an obstacle: collision_monitor stops the robot forever and the costmaps add a
fake obstacle in front.

Points whose base-frame position is inside the footprint (+2 cm) are set to inf.
Everything else is passed through unchanged.

    publishes /scan_filtered
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

X_MIN, X_MAX = -0.32, 0.32
Y_MIN, Y_MAX = -0.22, 0.22
LASER_X, LASER_Y = -0.256, 0.0065


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__('scan_self_filter')
        self.pub = self.create_publisher(LaserScan, 'scan_filtered', 10)
        self.create_subscription(LaserScan, 'scan', self.cb, qos_profile_sensor_data)
        self.last_removed = -1

    def cb(self, msg):
        out = list(msg.ranges)
        n = 0
        for i, r in enumerate(out):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(a) + LASER_X
            y = r * math.sin(a) + LASER_Y
            if X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX:
                out[i] = float('inf')
                n += 1
        msg.ranges = out
        self.pub.publish(msg)
        if n != self.last_removed:
            self.get_logger().info('filtered %d self points per scan' % n)
            self.last_removed = n


def main():
    rclpy.init()
    node = ScanSelfFilter()
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
