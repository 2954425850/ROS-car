#!/usr/bin/env python3
"""Morning acceptance run for the 2026-11-18 navigation plan.

Default route (the same one used on 2026-11-17):

    A = (-1.157, 0.556, 0 deg)      B = (4.0, 0.45, 0 deg)

Each "round" is A -> B -> A, so --rounds 20 sends 40 goals.

It checks the plan's acceptance metrics online:
  * actions with |v| < 0.03 m/s and |w| > 0.1 rad/s (must be 0)
  * round-trip success rate (target >= 18/20)
  * stop error (target <= 10 cm)
  * lateral y mean/std on the straight part of the corridor (soft metric;
    the corridor centre is around y = 0.45..0.50)

Usage
    source /opt/ros/jazzy/setup.bash
    python3 ~/ros2_lidar/tools/nav_accept.py --rounds 20
    python3 ~/ros2_lidar/tools/nav_accept.py --rounds 1 --dry-run

It must run on the Pi (same ROS graph as nav.launch.py). Stop it with Ctrl-C;
the current goal is cancelled.
"""
import argparse
import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def parse_pose(s):
    x, y, yaw_deg = [float(v) for v in s.split(',')]
    return x, y, math.radians(yaw_deg)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)


def ang_diff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class Accept(Node):
    def __init__(self):
        super().__init__('nav_accept')
        self.pose = None
        self.pose_t = 0.0
        self.rot_samples = 0
        self.rot_examples = []
        self.y_samples = []
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.on_pose, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_cmd, 10)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.pose_t = time.monotonic()
        if -0.5 <= p.position.x <= 3.5:
            self.y_samples.append(p.position.y)

    def on_cmd(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        if abs(v) < 0.03 and abs(w) > 0.1:
            self.rot_samples += 1
            if len(self.rot_examples) < 10:
                self.rot_examples.append((round(v, 3), round(w, 3)))

    def current_pose(self, timeout=2.0):
        t = time.monotonic()
        while time.monotonic() - t < timeout:
            if self.pose is not None and time.monotonic() - self.pose_t < 1.0:
                return self.pose
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose

    def wait_server(self, timeout=60.0):
        t = time.monotonic()
        while time.monotonic() - t < timeout:
            if self.client.server_is_ready():
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def go(self, x, y, yaw, timeout, dry=False):
        if dry:
            print('DRY goal (%.3f, %.3f, %.1f deg)' % (x, y, math.degrees(yaw)))
            return {'status': 'DRY', 'ok': False, 'elapsed': 0.0,
                    'final': self.current_pose(), 'err_xy': 0.0, 'err_yaw': 0.0}
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        t0 = time.monotonic()
        fut = self.client.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > 30.0:
                return {'status': 'SEND_TIMEOUT', 'ok': False,
                        'elapsed': time.monotonic() - t0,
                        'final': self.current_pose(), 'err_xy': None, 'err_yaw': None}
        gh = fut.result()
        if not gh.accepted:
            return {'status': 'REJECTED', 'ok': False,
                    'elapsed': time.monotonic() - t0,
                    'final': self.current_pose(), 'err_xy': None, 'err_yaw': None}
        rf = gh.get_result_async()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout:
                gh.cancel_goal_async()
                rclpy.spin_once(self, timeout_sec=0.5)
                return {'status': 'TIMEOUT', 'ok': False,
                        'elapsed': time.monotonic() - t0,
                        'final': self.current_pose(), 'err_xy': None, 'err_yaw': None}
        status = rf.result().status
        final = self.current_pose()
        err_xy = err_yaw = None
        if final:
            err_xy = math.hypot(final[0] - x, final[1] - y)
            err_yaw = abs(ang_diff(final[2], yaw))
        return {'status': {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                           GoalStatus.STATUS_ABORTED: 'ABORTED',
                           GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(
                               status, 'STATUS_%d' % status),
                'ok': status == GoalStatus.STATUS_SUCCEEDED,
                'elapsed': time.monotonic() - t0,
                'final': final, 'err_xy': err_xy, 'err_yaw': err_yaw}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--a', default='-1.157,0.556,0', help='x,y,yaw_deg')
    ap.add_argument('--b', default='4.0,0.45,0', help='x,y,yaw_deg')
    ap.add_argument('--rounds', type=int, default=20)
    ap.add_argument('--leg-timeout', type=float, default=120.0)
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--out', default=None)
    ap.add_argument('--max-consecutive-failures', type=int, default=3,
                    help='stop the run after this many failed legs in a row')
    args = ap.parse_args()

    A = parse_pose(args.a)
    B = parse_pose(args.b)
    rclpy.init()
    node = Accept()
    if not args.dry_run and not node.wait_server():
        print('ERROR: /navigate_to_pose action server not available')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    print('A=(%.3f,%.3f,%.1f deg)  B=(%.3f,%.3f,%.1f deg)  rounds=%d'
          % (A[0], A[1], math.degrees(A[2]), B[0], B[1], math.degrees(B[2]),
             args.rounds))
    start_pose = node.current_pose(5.0)
    legs = []
    consecutive_failures = 0
    try:
        for r in range(args.rounds):
            for name, goal in (('A->B', B), ('B->A', A)):
                res = node.go(*goal, timeout=args.leg_timeout, dry=args.dry_run)
                res['round'] = r + 1
                res['leg'] = name
                legs.append(res)
                fin = res['final']
                print('[%02d/%02d] %-4s %-10s %5.1fs  err_xy=%s  final=%s'
                      % (r + 1, args.rounds, name, res['status'], res['elapsed'],
                         ('%.3f' % res['err_xy']) if res['err_xy'] is not None else 'n/a',
                         ('(%.2f,%.2f,%.0f)' % (fin[0], fin[1], math.degrees(fin[2])))
                         if fin else 'n/a'))
                if res['status'] == 'SUCCEEDED':
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    if consecutive_failures >= args.max_consecutive_failures:
                        print('    -> stopping after %d consecutive failures'
                              % consecutive_failures)
                        raise KeyboardInterrupt
    except KeyboardInterrupt:
        print('interrupted / stopped')

    rounds_ok = 0
    for r in range(args.rounds):
        pair = [l for l in legs if l['round'] == r + 1]
        if len(pair) == 2 and all(l['ok'] for l in pair):
            rounds_ok += 1
    succeeded = sum(1 for l in legs if l['ok'])
    errs = [l['err_xy'] for l in legs if l['err_xy'] is not None]
    summary = {
        'route': {'A': A, 'B': B},
        'rounds_requested': args.rounds,
        'legs': legs,
        'legs_run': len(legs),
        'legs_succeeded': succeeded,
        'rounds_succeeded': rounds_ok,
        'in_place_rotation_samples': node.rot_samples,
        'in_place_rotation_examples': node.rot_examples,
        'stop_error_max_m': max(errs) if errs else None,
        'stop_error_mean_m': (sum(errs) / len(errs)) if errs else None,
        'lateral_y_mean': (sum(node.y_samples) / len(node.y_samples))
                          if node.y_samples else None,
        'lateral_y_std': (math.sqrt(sum((y - sum(node.y_samples) / len(node.y_samples)) ** 2
                                        for y in node.y_samples) / len(node.y_samples))
                          if node.y_samples else None),
        'lateral_samples': len(node.y_samples),
        'start_pose': start_pose,
        'generated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    out = args.out or os.path.join(
        os.path.expanduser('~/ros2_lidar/maps'),
        'nav_accept_%s.json' % time.strftime('%Y%m%d_%H%M%S'))
    with open(out, 'w') as f:
        json.dump(summary, f, indent=2)
    print('\n===== SUMMARY =====')
    print('legs run/succeeded : %d / %d' % (len(legs), succeeded))
    print('rounds succeeded   : %d / %d' % (rounds_ok, args.rounds))
    print('in-place rotations : %d  (target 0)' % node.rot_samples)
    print('stop error max/mean: %s / %s m' % (summary['stop_error_max_m'],
                                              summary['stop_error_mean_m']))
    print('lateral y mean/std : %s / %s  (centre ~0.45..0.50)'
          % (summary['lateral_y_mean'], summary['lateral_y_std']))
    print('saved: %s' % out)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
