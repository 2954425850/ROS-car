#!/usr/bin/env python3
"""融合模式执行节点：分步追目标 + 前沿探索。

只做导航目标，不管地图保存和模式起停（那是 upper 的事）。
用法：python3 fusion_nav.py（由 fusion.launch.py 拉起）
"""
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fusion_plan import Boundary, Grid, Navigator  # noqa: E402


def grid_from_msg(msg):
    return Grid(msg.info.width, msg.info.height, msg.info.resolution,
                msg.info.origin.position.x, msg.info.origin.position.y,
                list(msg.data))


class FusionNav(Node):
    def __init__(self):
        super().__init__("fusion_nav")
        self.declare_parameter("explore_max_s", 1800.0)
        self.declare_parameter("explore_max_m", 200.0)
        self.declare_parameter("stall_area_m2", 0.5)
        self.declare_parameter("stall_window_s", 60.0)

        self.grid = None
        self.area = 0.0
        self.pose = None
        self.boundary = None
        self.nav = Navigator()
        self.action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.leg_handle = None
        self.leg_goal = None
        self.leg_sending = False
        self.leg_seq = 0
        self.explore_start_t = None
        self.last_area = 0.0
        self.last_area_t = None
        self.distance = 0.0
        self.last_pose_xy = None
        self.last_status_t = 0.0
        self.blacklist_clear_t = time.monotonic()

        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        boundary_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                  reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(PoseStamped, "/fusion/target",
                                 self.on_target, 10)
        self.create_subscription(String, "/fusion/boundary",
                                 self.on_boundary, boundary_qos)
        self.create_service(SetBool, "/fusion/explore", self.on_explore)
        self.create_service(Trigger, "/fusion/cancel", self.on_cancel)
        self.status_pub = self.create_publisher(String, "/fusion/status", 10)
        self.leg_pub = self.create_publisher(PoseStamped, "/fusion/leg", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.plan_tick)
        self.get_logger().info("fusion_nav: 等待 /map 与 TF")

    # ---------------------------------------------------------------- 输入
    def on_map(self, msg):
        self.grid = grid_from_msg(msg)
        self.area = self.grid.free_area()

    def on_boundary(self, msg):
        try:
            obj = json.loads(msg.data) if msg.data else None
        except Exception:
            obj = None
        self.boundary = Boundary.from_json(obj)
        self.get_logger().info("fusion_nav: 边界%s" % (
            "已设置" if self.boundary else "已清除"))

    def on_target(self, msg):
        if self.grid is None or self.pose is None:
            self.get_logger().warn("fusion_nav: 没有地图/位姿，忽略目标")
            return
        p = msg.pose.position
        self.nav.set_target((p.x, p.y), self.grid, self.pose, self.boundary)
        self.explore_start_t = None
        self.sync_action()

    def on_explore(self, req, resp):
        if self.grid is None or self.pose is None:
            resp.success = False
            resp.message = "没有地图/位姿"
            return resp
        self.nav.set_explore(req.data, self.grid, self.pose, self.boundary)
        if req.data and self.explore_start_t is None:
            self.explore_start_t = time.monotonic()
            self.last_area = self.area
            self.last_area_t = self.explore_start_t
            self.distance = 0.0
        resp.success = True
        resp.message = "探索已开启" if req.data else "探索已关闭"
        self.sync_action()
        return resp

    def on_cancel(self, req, resp):
        self.nav.cancel()
        self.explore_start_t = None
        self.cancel_action()
        resp.success = True
        resp.message = "已取消"
        return resp

    # ---------------------------------------------------------------- 周期
    def tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose = (t.x, t.y, yaw)
            if self.last_pose_xy is not None:
                self.distance += math.hypot(
                    t.x - self.last_pose_xy[0],
                    t.y - self.last_pose_xy[1])
            self.last_pose_xy = (t.x, t.y)
        except Exception:
            self.pose = None
            self.last_pose_xy = None

        now = time.monotonic()
        if now - self.last_status_t >= 0.5:
            self.last_status_t = now
            self.publish_status()
        if now - self.blacklist_clear_t >= 600.0:
            self.blacklist_clear_t = now
            self.nav.blacklist.clear()

    def plan_tick(self):
        if self.grid is None or self.pose is None:
            return
        if self.nav.explore:
            if self.explore_start_t is None:
                self.explore_start_t = time.monotonic()
                self.last_area = self.area
                self.last_area_t = self.explore_start_t
            elapsed = time.monotonic() - self.explore_start_t
            if elapsed > float(self.get_parameter("explore_max_s").value):
                self.finish_explore("time_limit")
                return
            if self.distance > float(self.get_parameter("explore_max_m").value):
                self.finish_explore("distance_limit")
                return
            if self.area < self.last_area + float(
                    self.get_parameter("stall_area_m2").value):
                if time.monotonic() - self.last_area_t > float(
                        self.get_parameter("stall_window_s").value):
                    self.finish_explore("area_stalled")
                    return
            else:
                self.last_area = self.area
                self.last_area_t = time.monotonic()
        self.nav.replan(self.grid, self.pose, self.boundary)
        self.sync_action()

    def finish_explore(self, reason):
        self.nav.set_explore(False, self.grid, self.pose, self.boundary)
        self.nav.state = "done"
        self.nav.reason = reason
        self.explore_start_t = None
        self.cancel_action()
        self.get_logger().info("fusion_nav: 探索结束（%s）" % reason)

    # ---------------------------------------------------------------- 输出
    def publish_status(self):
        st = self.nav.status()
        st.update({
            "distance_m": round(self.distance, 2),
            "covered_m2": round(self.area, 2),
            "elapsed_s": (round(time.monotonic() - self.explore_start_t, 1)
                          if self.explore_start_t else 0.0),
            "pose": self.pose,
        })
        msg = String()
        msg.data = json.dumps(st, ensure_ascii=False,
                              separators=(",", ":"))
        self.status_pub.publish(msg)
        if self.nav.leg is not None and self.pose is not None:
            leg = PoseStamped()
            leg.header.frame_id = "map"
            leg.header.stamp = self.get_clock().now().to_msg()
            leg.pose.position.x = float(self.nav.leg[0])
            leg.pose.position.y = float(self.nav.leg[1])
            yaw = math.atan2(self.nav.leg[1] - self.pose[1],
                             self.nav.leg[0] - self.pose[0])
            leg.pose.orientation.z = math.sin(yaw / 2.0)
            leg.pose.orientation.w = math.cos(yaw / 2.0)
            self.leg_pub.publish(leg)

    # ---------------------------------------------------------------- action
    def sync_action(self):
        if self.nav.leg is None:
            self.cancel_action()
            return
        if self.leg_goal == self.nav.leg:
            return
        self.cancel_action()
        self.send_leg(self.nav.leg)

    def send_leg(self, goal):
        if not self.action.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("fusion_nav: nav2 action 不在")
            return
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(goal[0])
        g.pose.pose.position.y = float(goal[1])
        yaw = 0.0
        if self.pose is not None:
            yaw = math.atan2(goal[1] - self.pose[1],
                             goal[0] - self.pose[0])
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.leg_seq += 1
        seq = self.leg_seq
        self.leg_goal = tuple(goal)
        self.leg_sending = True
        self.nav.on_leg_started(self.pose)
        fut = self.action.send_goal_async(g)
        fut.add_done_callback(lambda f: self.on_goal_response(f, seq))

    def on_goal_response(self, fut, seq):
        if seq != self.leg_seq:
            return
        try:
            gh = fut.result()
        except Exception:
            gh = None
        if gh is None or not gh.accepted:
            self.leg_sending = False
            self.leg_goal = None
            self.on_leg_done(False)
            return
        self.leg_handle = gh
        self.leg_sending = False
        gh.get_result_async().add_done_callback(
            lambda f: self.on_goal_result(f, seq))

    def on_goal_result(self, fut, seq):
        if seq != self.leg_seq:
            return
        self.leg_handle = None
        self.leg_goal = None
        self.leg_sending = False
        try:
            status = fut.result().status
        except Exception:
            status = GoalStatus.STATUS_ABORTED
        self.on_leg_done(status == GoalStatus.STATUS_SUCCEEDED)

    def on_leg_done(self, ok):
        if self.grid is None or self.pose is None:
            return
        if ok:
            self.nav.on_leg_succeeded(self.grid, self.pose, self.boundary)
        else:
            self.nav.on_leg_failed(self.grid, self.pose, self.boundary)
        self.publish_status()
        self.sync_action()

    def cancel_action(self):
        gh = self.leg_handle
        self.leg_seq += 1
        self.leg_handle = None
        self.leg_goal = None
        self.leg_sending = False
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass


def main():
    rclpy.init()
    node = FusionNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
