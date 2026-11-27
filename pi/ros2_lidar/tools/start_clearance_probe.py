#!/usr/bin/env python3
"""只规划不动车：在已知直墙旁用 ComputePathToPose 探测起点接受范围。

在 /global_costmap/costmap_raw 里找一段竖直的致命墙（254），把车平行放在
墙的 +x 侧、车头朝 +y，使车侧边到墙面的距离依次为 --clearances（默认
0.10/0.20/0.30 m），再额外测一个「车体真的压进墙格」的位姿。

    source /opt/ros/jazzy/setup.bash
    source ~/nav2_ws/install/setup.bash
    python3 ~/ros2_lidar/tools/start_clearance_probe.py

输出每个起点的：中心 cost、车侧边 cost、planner 状态、错误码。
注意：Smac 不校验起点节点本身，只校验邻居；所以「身体压墙」不保证 ABORTED，
以实测为准（这正是放开起点的本意）。
"""
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

HALF_W = 0.20  # footprint 侧向半宽（0.60 x 0.40），见 nav2_params.yaml
CELL_HALF = 0.025


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def get_costmap(node):
    box = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Costmap, "/global_costmap/costmap_raw", box.append, qos)
    deadline = time.time() + 10.0
    while time.time() < deadline and not box:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not box:
        raise SystemExit("10s 内没收到 /global_costmap/costmap_raw")
    return box[0]


def find_wall(cm):
    md = cm.metadata
    w, h, res = md.size_x, md.size_y, md.resolution
    ox, oy = md.origin.position.x, md.origin.position.y

    def cell(i, j):
        return cm.data[j * w + i] & 0xFF

    def at(x, y):
        i = int((x - ox) / res)
        j = int((y - oy) / res)
        if 0 <= i < w and 0 <= j < h:
            return cell(i, j)
        return -1

    best = None
    for j in range(2, h - 2):
        for i in range(2, w - 2):
            if cell(i, j) < 254:
                continue
            # 竖直墙：上下各 2 格也是致命格
            if any(cell(i, j + k) < 254 for k in (-2, -1, 1, 2)):
                continue
            x = ox + (i + 0.5) * res
            y = oy + (j + 0.5) * res
            # +x 侧到 0.9 m 内不能再有致命格（253 的 inscribed 环是预期的，
            # 补丁正是要验证它不拦起点）。
            if any(at(x + d, y) >= 254 for d in [0.10 + 0.05 * k for k in range(17)]):
                continue
            if any(at(x + 0.40, y + dy) >= 254
                   for dy in [-0.45, -0.35, -0.25, 0.25, 0.35, 0.45]):
                continue
            if any(at(x + 0.40, y + 0.90 + 0.05 * k) >= 254 for k in range(9)):
                continue
            score = abs(x) + abs(y)
            if best is None or score < best[0]:
                best = (score, x, y)
    if best is None:
        raise SystemExit("代价图里找不到满足条件的竖直致命墙")
    return best[1], best[2], at


def send(node, ac, start, goal):
    g = ComputePathToPose.Goal()
    g.start.header.frame_id = "map"
    g.start.pose.position.x = float(start[0])
    g.start.pose.position.y = float(start[1])
    g.start.pose.orientation.z, g.start.pose.orientation.w = yaw_quat(start[2])[2:]
    g.goal.header.frame_id = "map"
    g.goal.pose.position.x = float(goal[0])
    g.goal.pose.position.y = float(goal[1])
    g.goal.pose.orientation.z, g.goal.pose.orientation.w = yaw_quat(goal[2])[2:]
    g.planner_id = "GridBased"
    g.use_start = True

    fut = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    if not fut.done() or fut.result() is None:
        return "NO_GOAL_HANDLE", None
    handle = fut.result()
    if not handle.accepted:
        return "REJECTED", None
    rfut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, rfut, timeout_sec=30.0)
    if not rfut.done() or rfut.result() is None:
        return "NO_RESULT", None
    wrapped = rfut.result()
    if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
        code = getattr(wrapped.result, "error_code", None)
        return "ABORTED" if wrapped.status == GoalStatus.STATUS_ABORTED else "CANCELED", code
    return "SUCCEEDED", getattr(wrapped.result, "error_code", None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clearances", nargs="+", type=float,
                    default=[0.10, 0.20, 0.30])
    ap.add_argument("--goal-len", type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("start_clearance_probe")
    cm = get_costmap(node)
    wx, wy, at = find_wall(cm)
    print(f"wall cell = ({wx:.2f}, {wy:.2f}); robot heading +y, wall on -x side")
    print(f"footprint half width {HALF_W} (0.60 x 0.40)")

    ac = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not ac.wait_for_server(timeout_sec=10.0):
        raise SystemExit("compute_path_to_pose 服务不可用")

    yaw = math.pi / 2.0
    cases = []
    for d in args.clearances:
        sx = wx + CELL_HALF + HALF_W + d
        cases.append((f"side {d:.2f} m", sx))
    cases.append(("body 0.05 m inside wall",
                  wx + CELL_HALF + HALF_W - 0.05))

    for name, sx in cases:
        start = (sx, wy, yaw)
        goal = (sx, wy + args.goal_len, yaw)
        c_center = at(sx, wy)
        c_body = at(sx - HALF_W, wy)
        status, code = send(node, ac, start, goal)
        print(f"{name:26s} start=({sx:.3f},{wy:.2f}) center_cost={c_center} "
              f"body_edge_cost={c_body} -> {status} error={code}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
