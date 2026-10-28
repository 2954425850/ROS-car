#!/usr/bin/env python3
"""Save the current /map to <name>.pgm/.yaml/.png by subscribing to the
latched topic directly.

Why not /slam_toolbox/save_map: on this install (Jazzy, no nav2_map_server)
that service always returns 255 and writes nothing -- the node log says
"Package 'nav2_map_server' not found".

    python3 save_map_snapshot.py /home/cy/ros2_lidar/maps/map
"""
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else '/home/cy/ros2_lidar/maps/map'
    rclpy.init()
    node = Node('map_snapshot')
    qos = QoSProfile(depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    got = []
    node.create_subscription(OccupancyGrid, '/map', got.append, qos)
    while rclpy.ok() and not got:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not got:
        print('no /map message received', file=sys.stderr)
        return 1

    m = got[0]
    w, h = m.info.width, m.info.height
    # PGM rows run top-down; OccupancyGrid row 0 is the bottom.
    rows = []
    for y in range(h - 1, -1, -1):
        row = bytearray()
        for x in range(w):
            v = m.data[y * w + x]
            row.append(254 if v < 0 else (0 if v >= 50 else 255))
        rows.append(bytes(row))

    with open(name + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(b''.join(rows))

    with open(name + '.yaml', 'w') as f:
        f.write('image: %s.pgm\nresolution: %f\norigin: [%f, %f, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n' % (
                    name.rsplit('/', 1)[-1], m.info.resolution,
                    m.info.origin.position.x, m.info.origin.position.y))

    try:
        from PIL import Image
        Image.frombytes('L', (w, h), b''.join(rows)).save(name + '.png')
    except Exception as e:  # PNG is a convenience, not part of the map
        print('png skipped:', e)

    print('saved %s.pgm/.yaml (%dx%d, res %.3f)' %
          (name, w, h, m.info.resolution))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
