#!/usr/bin/env python3
"""Hard guard against in-place rotation (2026-11-18 plan).

Sits between velocity_smoother and collision_monitor:

    controller -> cmd_vel_raw -> velocity_smoother -> cmd_vel_smoothed
      -> no_pivot_guard -> cmd_vel_guard -> collision_monitor -> cmd_vel_nav

Rule: |omega| <= |v| / R_min.  When the linear speed is ~0 the angular speed
must be ~0, so no upstream bug (DWB rotation, recovery, a stray publisher on
cmd_vel_nav) can make the chassis spin in place.

Only the navigation path goes through this node; PS2 / voice / cloud commands
are arbitrated by cmd_mux and do not pass here.

Parameters
  min_turning_radius (0.5 m)
  pivot_v_eps (0.03 m/s)      below this the robot is considered stationary
  allow_slow_pivot (False)    last-resort escape hatch: allow a slow spin
  slow_pivot_omega (0.15 rad/s)
  input_timeout (0.5 s)       stale input -> publish zero
  rate_hz (20)
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class NoPivotGuard(Node):
    def __init__(self):
        super().__init__('no_pivot_guard')
        self.declare_parameter('in_topic', 'cmd_vel_smoothed')
        self.declare_parameter('out_topic', 'cmd_vel_guard')
        self.declare_parameter('min_turning_radius', 0.5)
        self.declare_parameter('pivot_v_eps', 0.03)
        self.declare_parameter('allow_slow_pivot', False)
        self.declare_parameter('slow_pivot_omega', 0.15)
        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('rate_hz', 20.0)

        self.r_min = float(self.get_parameter('min_turning_radius').value)
        self.v_eps = float(self.get_parameter('pivot_v_eps').value)
        self.allow_pivot = bool(self.get_parameter('allow_slow_pivot').value)
        self.pivot_w = float(self.get_parameter('slow_pivot_omega').value)
        self.timeout = float(self.get_parameter('input_timeout').value)
        rate = float(self.get_parameter('rate_hz').value)

        self.pub = self.create_publisher(
            Twist, self.get_parameter('out_topic').value, 10)
        self.create_subscription(
            Twist, self.get_parameter('in_topic').value, self.on_cmd, 10)

        self.last_t = None
        self.v = 0.0
        self.w = 0.0
        self.clamped = 0
        self.pivots_zeroed = 0
        self.pivots_allowed = 0
        self.last_report = 0.0
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            'no_pivot_guard: R_min=%.2f m, v_eps=%.2f, allow_slow_pivot=%s'
            % (self.r_min, self.v_eps, self.allow_pivot))

    def on_cmd(self, msg):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)
        self.last_t = time.monotonic()

    def tick(self):
        now = time.monotonic()
        out = Twist()
        if self.last_t is None or (now - self.last_t) > self.timeout:
            self.v = 0.0
            self.w = 0.0
        else:
            v, w = self.v, self.w
            if abs(v) >= self.v_eps:
                max_w = abs(v) / max(self.r_min, 1e-3)
                if abs(w) > max_w:
                    w = math.copysign(max_w, w)
                    self.clamped += 1
            else:
                if self.allow_pivot:
                    w = max(-self.pivot_w, min(self.pivot_w, w))
                    if abs(w) > 1e-6:
                        self.pivots_allowed += 1
                else:
                    if abs(w) > 1e-6:
                        self.pivots_zeroed += 1
                    w = 0.0
            out.linear.x = v
            out.angular.z = w
        self.pub.publish(out)

        if now - self.last_report >= 2.0:
            if self.clamped or self.pivots_zeroed or self.pivots_allowed:
                self.get_logger().info(
                    'guard: clamped=%d pivot_zeroed=%d pivot_allowed=%d'
                    % (self.clamped, self.pivots_zeroed,
                       self.pivots_allowed))
            self.last_report = now


def main():
    rclpy.init()
    node = NoPivotGuard()
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
