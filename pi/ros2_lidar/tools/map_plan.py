#!/usr/bin/env python3
"""Republish /map as /map_plan with unknown cells at a soft occupancy cost.

Why: the global costmaps should treat unknown as expensive-but-traversable,
not as NO_INFORMATION (blocked) and not as free (preferred).  nav2's
StaticLayer only has the trinary 0/100/-1 mapping in its default mode; in
non-trinary mode it scales OccupancyGrid values by value/100 * 254.  So a
forwarding node turns -1 into 87 (~221 cost, above analytic_expansion_max_cost
200 so Reeds-Shepp shortcuts avoid it, but still below the 253 inscribed
threshold so the A* search and the start-pose check can traverse it).

Only the global costmaps subscribe to /map_plan.  slam_toolbox, AMCL,
map_server, fusion_plan, the local costmap and collision_monitor keep using
the original /map (with -1), so frontier selection and near-field protection
are unchanged.
"""
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)

from map_plan_core import UNKNOWN_OCC, convert_values


class MapPlan(Node):
    def __init__(self):
        super().__init__('map_plan')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, 'map_plan', qos)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, qos)
        self.get_logger().info(
            'map_plan: /map -> /map_plan, unknown(-1) -> %d' % UNKNOWN_OCC)

    def on_map(self, msg):
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = convert_values(msg.data)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = MapPlan()
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
