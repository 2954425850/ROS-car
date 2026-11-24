#!/usr/bin/env python3
"""Remember the last AMCL pose and restore it on the next start.

Why: the plan (2026-11-18) wants AMCL to keep working, but the initial pose
should not be hand-edited in nav2_params.yaml every time. This node

  * subscribers /amcl_pose and writes x/y/yaw to a small JSON file (1 Hz,
    only when the pose actually changed);
  * on startup, if AMCL has not produced a pose within a few seconds, publishes
    the saved pose on /initialpose (which is what RViz's "2D Pose Estimate"
    does), retrying a few times.

If the file is missing or the robot was moved, delete the file: AMCL then uses
the initial_pose fallback in nav2_params.yaml.

Parameters
  map_yaml       (default '')  给了就按地图名存 <地图名>_last_pose.json，
                               与 auto_localize 使用同一规则（nav_pose_file.py）
  pose_file      (default '')  显式路径优先；两者都空时退回
                               /home/cy/ros2_lidar/maps/last_pose.json
  publish_delay  (8.0 s)   wait for AMCL to start before restoring
  retry_period   (5.0 s)
  max_attempts   (3)
  map_frame      ('map')
"""
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from nav_pose_file import pose_path


class PosePersist(Node):
    def __init__(self):
        super().__init__('pose_persist')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('pose_file', '')
        self.declare_parameter('publish_delay', 8.0)
        self.declare_parameter('retry_period', 5.0)
        self.declare_parameter('max_attempts', 3)
        self.declare_parameter('map_frame', 'map')

        self.file = pose_path(self.get_parameter('map_yaml').value,
                              self.get_parameter('pose_file').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_delay = float(self.get_parameter('publish_delay').value)
        self.retry_period = float(self.get_parameter('retry_period').value)
        self.max_attempts = int(self.get_parameter('max_attempts').value)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)

        self.got_pose = False
        self.last_saved = None
        self.start_t = time.monotonic()   # delays use monotonic, not ROS time
        self.last_save_t = 0.0
        self.attempts = 0
        self.last_attempt = 0.0

        self.create_timer(0.5, self.tick)
        self.get_logger().info('pose_persist: file=%s' % self.file)

    def on_pose(self, msg):
        self.got_pose = True
        now = time.monotonic()
        if now - self.last_save_t < 1.0:
            return
        p = msg.pose.pose
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z),
                         1.0 - 2.0 * p.orientation.z * p.orientation.z)
        if self.last_saved is not None:
            lx, ly, lyaw = self.last_saved
            if (abs(p.position.x - lx) < 0.02 and abs(p.position.y - ly) < 0.02
                    and abs((yaw - lyaw + math.pi) % (2 * math.pi) - math.pi) < 0.02):
                return
        self.last_saved = (p.position.x, p.position.y, yaw)
        self.last_save_t = now
        self._write(self.last_saved)

    def _write(self, pose):
        try:
            os.makedirs(os.path.dirname(self.file), exist_ok=True)
            tmp = self.file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'x': pose[0], 'y': pose[1], 'yaw': pose[2]}, f)
            os.replace(tmp, self.file)
        except Exception as e:  # never let persistence kill navigation
            self.get_logger().warn('pose_persist write failed: %s' % e)

    def tick(self):
        if self.got_pose or self.attempts >= self.max_attempts:
            return
        now = time.monotonic()
        if self.attempts == 0:
            if now - self.start_t < self.publish_delay:
                return
        else:
            if now - self.last_attempt < self.retry_period:
                return
        if not os.path.exists(self.file):
            self.get_logger().info('pose_persist: no saved pose, leaving AMCL '
                                   'initial_pose fallback in charge')
            self.attempts = self.max_attempts
            return
        try:
            with open(self.file) as f:
                d = json.load(f)
        except Exception as e:
            self.get_logger().warn('pose_persist read failed: %s' % e)
            self.attempts = self.max_attempts
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(d['x'])
        msg.pose.pose.position.y = float(d['y'])
        msg.pose.pose.orientation.z = math.sin(float(d['yaw']) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(d['yaw']) / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.pub.publish(msg)
        self.attempts += 1
        self.last_attempt = now
        self.get_logger().info(
            'pose_persist: restored (% .3f, % .3f, % .1f deg) attempt %d/%d'
            % (d['x'], d['y'], math.degrees(d['yaw']),
               self.attempts, self.max_attempts))


def main():
    rclpy.init()
    node = PosePersist()
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
