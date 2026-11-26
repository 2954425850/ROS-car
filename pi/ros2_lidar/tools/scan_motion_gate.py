#!/usr/bin/env python3
"""Direction-aware scan gate in front of collision_monitor.

The stock Collision Monitor evaluates fixed polygons in base_footprint, so a
wall 5 cm off the side or 13 cm behind the chassis stops the robot even when
the controller only wants to drive forward.  This node republishes
/scan_filtered as /scan_motion_guard with every point outside the predicted
swept footprint set to inf.  collision_monitor then still stops/slows for real
obstacles on the intended path, but ignores rear/side clutter it cannot reach.

The side lane keeps a 5 cm pad (the old fixed box gave 8 cm, and
scan_self_filter already drops the inner 2 cm).  After a command sign flip the
old and new masks are unioned for ``union_hold`` seconds so the chassis has
time to decelerate; the mask is not swapped instantly.

Fail-safe behaviour:
  * disabled / no / stale command -> the full scan passes through
  * TF lookup fails              -> the full scan passes through
  * this node dies               -> systemd restarts it; if it stays down,
                                    collision_monitor source_timeout stops us
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from motion_scan import (arc_samples, filter_ranges_union,
                         lookup_tf_or_none, motion_active, should_filter_cmd)


class ScanMotionGate(Node):
    def __init__(self):
        super().__init__('scan_motion_gate')
        self.declare_parameter('scan_topic', 'scan_filtered')
        self.declare_parameter('out_topic', 'scan_motion_guard')
        self.declare_parameter('cmd_topic', 'cmd_vel_guard')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('half_length', 0.30)
        self.declare_parameter('half_width', 0.20)
        self.declare_parameter('pad_long', 0.02)
        self.declare_parameter('pad_width', 0.05)
        self.declare_parameter('horizon', 1.0)
        self.declare_parameter('step', 0.1)
        self.declare_parameter('union_hold', 0.3)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('v_eps', 0.02)
        self.declare_parameter('w_eps', 0.02)
        self.declare_parameter('enabled', True)
        self.declare_parameter('report_period', 5.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.half_length = float(self.get_parameter('half_length').value)
        self.half_width = float(self.get_parameter('half_width').value)
        self.pad_long = float(self.get_parameter('pad_long').value)
        self.pad_width = float(self.get_parameter('pad_width').value)
        self.horizon = float(self.get_parameter('horizon').value)
        self.step = float(self.get_parameter('step').value)
        self.union_hold = float(self.get_parameter('union_hold').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.v_eps = float(self.get_parameter('v_eps').value)
        self.w_eps = float(self.get_parameter('w_eps').value)
        self.report_period = float(self.get_parameter('report_period').value)

        # Reliable like /scan_filtered so `ros2 topic hz/echo` works without
        # extra QoS flags; collision_monitor's BEST_EFFORT scan subscription
        # is compatible with a RELIABLE publisher.
        self.pub = self.create_publisher(LaserScan, self.out_topic, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.v = 0.0
        self.w = 0.0
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.union_until = 0.0
        self.last_cmd = None
        self.last_scan = None
        self.kept = 0
        self.checked = 0
        self.last_report = time.monotonic()
        self.get_logger().info(
            'scan_motion_gate: %s -> %s, footprint %.2f x %.2f m, '
            'pads %.2f/%.2f m, horizon %.1f s'
            % (self.scan_topic, self.out_topic, 2 * self.half_length,
               2 * self.half_width, self.pad_long, self.pad_width,
               self.horizon))

    # ------------------------------------------------------------------ input
    def on_cmd(self, msg):
        was_intent = self._intent()
        old_v, old_w = self.v, self.w
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)
        self.last_cmd = self.get_clock().now()
        now_intent = self._intent()
        # A reversal / start / turn flip changes which points are relevant.
        # Re-emit at once, but union the previous mask with the new one for a
        # short hold: the chassis does not stop at the instant of the command.
        if self.last_scan is not None and now_intent != was_intent:
            self.prev_v, self.prev_w = old_v, old_w
            self.union_until = time.monotonic() + self.union_hold
            if now_intent[0] or was_intent[0]:
                self._process(self.last_scan)

    def _intent(self):
        """(moving, linear sign, angular sign) with epsilon dead bands."""
        if not self._cmd_fresh():
            return False, 0, 0
        active = motion_active(self.v, self.w, self.v_eps, self.w_eps)
        vsign = self._vsign(self.v)
        wsign = (self.w > self.w_eps) - (self.w < -self.w_eps)
        return active, vsign, wsign

    def _vsign(self, v):
        return (v > self.v_eps) - (v < -self.v_eps)

    def _cmd_age(self):
        if self.last_cmd is None:
            return None
        return (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9

    def _cmd_fresh(self):
        age = self._cmd_age()
        return age is not None and age <= self.cmd_timeout

    def _enabled(self):
        return bool(self.get_parameter('enabled').value)

    # ------------------------------------------------------------------- scan
    def on_scan(self, msg):
        self.last_scan = msg
        self._process(msg)

    def _process(self, msg):
        if not should_filter_cmd(
                self._enabled(), self._cmd_age(), self.cmd_timeout,
                self.v, self.w, self.v_eps, self.w_eps):
            self.pub.publish(msg)
            return

        # collision_monitor aligns the scan by its header stamp; use the same
        # instant instead of "latest TF", otherwise the mask is offset by up
        # to one scan period of chassis motion.
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            stamp = Time.from_msg(msg.header.stamp)
        else:
            stamp = Time()
        tf = lookup_tf_or_none(self.tf_buffer, self.base_frame,
                               msg.header.frame_id, stamp,
                               Duration(seconds=0.05))
        if tf is None:
            self.get_logger().warn(
                'no TF %s <- %s at scan stamp; passing full scan'
                % (self.base_frame, msg.header.frame_id),
                throttle_duration_sec=5.0)
            self.pub.publish(msg)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        motions = [(arc_samples(self.v, self.w, self.horizon, self.step),
                    self._vsign(self.v))]
        if time.monotonic() < self.union_until:
            motions.append(
                (arc_samples(self.prev_v, self.prev_w,
                             self.horizon, self.step),
                 self._vsign(self.prev_v)))

        out_ranges, kept, checked = filter_ranges_union(
            msg.ranges, msg.angle_min, msg.angle_increment, motions,
            self.half_length, self.half_width,
            self.pad_long, self.pad_width,
            laser_x=t.x, laser_y=t.y, laser_yaw=yaw,
            range_min=msg.range_min, range_max=msg.range_max)

        self.kept += kept
        self.checked += checked
        if kept == checked:
            self.pub.publish(msg)
        else:
            self._emit(msg, out_ranges)
        self._maybe_report()

    def _emit(self, msg, ranges):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = msg.intensities
        self.pub.publish(out)

    def _maybe_report(self):
        now = time.monotonic()
        if now - self.last_report < self.report_period:
            return
        if self.checked:
            self.get_logger().info(
                'gate: kept %d/%d points in sweep (v=%.2f w=%.2f)'
                % (self.kept, self.checked, self.v, self.w))
        self.kept = 0
        self.checked = 0
        self.last_report = now


def main():
    rclpy.init()
    node = ScanMotionGate()
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
