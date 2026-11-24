#!/usr/bin/env python3
"""Automatic initial pose for AMCL (2026-11-24, A2 of the nav plan).

No operator input needed at startup:

  1. Try the pose saved by pose_persist (maps/last_pose.json).  Validate it by
     matching the current scan against the map.  Good -> publish /initialpose
     (fast path, < 1 s).
  2. If it does not match well, run a global scan-to-map search over the whole
     map (coarse 0.2 m / 5 deg, then refine 0.05 m / 1 deg) and publish the best
     pose if its score is good.
  3. If neither is good, log an error and DO NOT publish.  Navigating with a
     wrong pose is worse than refusing: the caller must move the robot back
     into the mapped area or re-map.

The score is the trimmed mean distance from the scan endpoints to the nearest
occupied map cell (1 m penalty for unknown cells, 2 m outside the map).  A pose
is accepted only if BOTH:

  * score < score_thresh, and
  * at least hit_thresh of the scan endpoints land within hit_dist of an
    occupied cell.

2026-11-24 实测（这张图）：正确位姿 score 0.005~0.02 / 命中 86%；错误位姿
score 0.08~0.12 / 命中 37%。只用距离时错误位姿也能过 0.25 的阈值，加命中率
后才能把两者分开。

此外，扫描点数不足 min_scan_points 时（雷达刚复位的第一圈常常只有 200 点左右）
不参与定位，等下一帧完整扫描。

    ros2 run-style: python3 auto_localize.py --ros-args \
        -p map_yaml:=/home/cy/ros2_lidar/maps/nav_20261124.yaml

Parameters
  map_yaml (required)   map .yaml (PGM + resolution + origin)
  scan_topic            default scan_filtered
  pose_file             default <地图名>_last_pose.json，与地图同目录
                        （不再用整个目录共用的 last_pose.json；见 nav_pose_file.py）
  map_frame             default map
  laser_x/laser_y       default -0.256 / 0.0065 (laser in base_footprint)
  score_thresh          0.25 (m)
  hit_dist              0.10 (m) 端点算作"打中"的距离
  hit_thresh            0.50     至少一半端点要打中
  min_scan_points       500      少于此数不定位
  min_clearance         0.30 m (candidate poses must be this far from walls)
  max_attempts          5 (then keep retrying every retry_period)
  retry_period          15 s
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

try:
    from scipy.ndimage import distance_transform_edt
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False

from nav_pose_file import pose_path


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5', 'expected binary PGM'
        line = f.readline()
        while line.strip().startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()  # maxval
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    return data


def chamfer_distance(occ):
    """Fallback distance-to-nearest-occupied (cells), 3x3 chamfer."""
    h, w = occ.shape
    inf = 1e9
    d = np.where(occ, 0.0, inf)
    s2 = math.sqrt(2.0)
    for y in range(h):
        row = d[y]
        prev = d[y - 1] if y > 0 else None
        for x in range(w):
            if row[x] == 0.0:
                continue
            best = row[x]
            if prev is not None:
                if x > 0:
                    best = min(best, prev[x - 1] + s2)
                best = min(best, prev[x] + 1.0)
                if x < w - 1:
                    best = min(best, prev[x + 1] + s2)
            if x > 0:
                best = min(best, row[x - 1] + 1.0)
            row[x] = best
    for y in range(h - 1, -1, -1):
        row = d[y]
        nxt = d[y + 1] if y < h - 1 else None
        for x in range(w - 1, -1, -1):
            if row[x] == 0.0:
                continue
            best = row[x]
            if nxt is not None:
                if x < w - 1:
                    best = min(best, nxt[x + 1] + s2)
                best = min(best, nxt[x] + 1.0)
                if x > 0:
                    best = min(best, nxt[x - 1] + s2)
            if x < w - 1:
                best = min(best, row[x + 1] + 1.0)
            row[x] = best
    return d


class AutoLocalize(Node):
    def __init__(self):
        super().__init__('auto_localize')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('scan_topic', 'scan_filtered')
        self.declare_parameter('pose_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('laser_x', -0.256)
        self.declare_parameter('laser_y', 0.0065)
        self.declare_parameter('score_thresh', 0.25)
        self.declare_parameter('hit_dist', 0.10)
        self.declare_parameter('hit_thresh', 0.50)
        self.declare_parameter('min_scan_points', 500)
        self.declare_parameter('min_clearance', 0.30)
        self.declare_parameter('max_attempts', 5)
        self.declare_parameter('retry_period', 15.0)

        yaml_path = self.get_parameter('map_yaml').value
        if not yaml_path:
            raise RuntimeError('auto_localize: map_yaml parameter is required')
        self.map_dir = os.path.dirname(os.path.abspath(yaml_path))
        self.pose_file = pose_path(yaml_path,
                                   self.get_parameter('pose_file').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.lx = float(self.get_parameter('laser_x').value)
        self.ly = float(self.get_parameter('laser_y').value)
        self.thresh = float(self.get_parameter('score_thresh').value)
        self.hit_dist = float(self.get_parameter('hit_dist').value)
        self.hit_thresh = float(self.get_parameter('hit_thresh').value)
        self.min_points = int(self.get_parameter('min_scan_points').value)
        self.min_clear = float(self.get_parameter('min_clearance').value)
        self.max_attempts = int(self.get_parameter('max_attempts').value)
        self.retry = float(self.get_parameter('retry_period').value)

        import yaml
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        img = meta['image']
        if not os.path.isabs(img):
            img = os.path.join(self.map_dir, img)
        a = read_pgm(img)
        self.h, self.w = a.shape
        occ = (a == 0)
        if _HAVE_SCIPY:
            self.dist = distance_transform_edt(~occ) * self.res
        else:
            self.dist = chamfer_distance(occ) * self.res
            self.get_logger().warn('scipy missing: using chamfer distance')
        self.pen = np.where(a == 205, 1.0, 0.0)
        self.score_cell = self.dist + self.pen
        # "打中"：端点离最近占用格不超过 hit_dist（距离图已在米制）
        self.hit_ok = self.dist <= self.hit_dist
        self.free = (((a == 254) | (a == 255)) &
                     (self.dist > self.min_clear))
        ys, xs = np.where(self.free)
        self.cx = self.ox + (xs + 0.5) * self.res
        self.cy = self.oy + (ys + 0.5) * self.res
        self.get_logger().info(
            'auto_localize: map %dx%d res %.2f, free candidates %d, scipy=%s'
            % (self.w, self.h, self.res, len(self.cx), _HAVE_SCIPY))

        self.scan = None
        self.scan_t = 0.0
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_timer(1.0, self.tick)

        self.done = False
        self.attempts = 0
        self.last_try = 0.0
        self.start_t = time.monotonic()

    # ---------------------------------------------------------------- scan
    def on_scan(self, msg):
        self.scan = msg
        self.scan_t = time.monotonic()

    def scan_vectors(self, max_points=120):
        m = self.scan
        r = np.asarray(m.ranges, dtype=float)
        ang = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.15) & (r < 12.0)
        r, ang = r[ok], ang[ok]
        # 雷达刚复位的第一圈常常只有 ~200 点（完整一圈约 650）。这种残帧参与
        # 匹配会把位姿带偏（2026-11-24 实测：错 4 m、147°）。等完整帧。
        if len(r) < self.min_points:
            return None
        step = max(1, len(r) // max_points)
        r, ang = r[::step], ang[::step]
        return np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)

    def score(self, bx, by, yaw, sv, trim=0.75):
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        pts = sv @ R.T + np.array([self.lx, self.ly]) @ R.T + np.array([bx, by])
        px = ((pts[:, 0] - self.ox) / self.res).astype(np.int32)
        py = (self.h - 1 - ((pts[:, 1] - self.oy) / self.res)).astype(np.int32)
        ins = (px >= 0) & (px < self.w) & (py >= 0) & (py < self.h)
        sc = np.full(len(pts), 2.0, dtype=np.float32)
        sc[ins] = self.score_cell[py[ins], px[ins]]
        k = max(1, int(len(sc) * trim))
        return float(np.sort(sc)[:k].mean())

    def hit_rate(self, bx, by, yaw, sv):
        """端点落在占用格 hit_dist 以内的比例（与 score 同一套投影）。"""
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        pts = sv @ R.T + np.array([self.lx, self.ly]) @ R.T + np.array([bx, by])
        px = ((pts[:, 0] - self.ox) / self.res).astype(np.int32)
        py = (self.h - 1 - ((pts[:, 1] - self.oy) / self.res)).astype(np.int32)
        ins = (px >= 0) & (px < self.w) & (py >= 0) & (py < self.h)
        hit = np.zeros(len(pts), dtype=bool)
        hit[ins] = self.hit_ok[py[ins], px[ins]]
        return float(hit.mean())

    def acceptable(self, score, hit):
        return score < self.thresh and hit >= self.hit_thresh

    # ------------------------------------------------------------- search
    def global_search(self, sv):
        yaws = np.deg2rad(np.arange(0, 360, 5))
        R = np.stack([np.stack([np.cos(yaws), -np.sin(yaws)], 1),
                      np.stack([np.sin(yaws), np.cos(yaws)], 1)], 1)
        LO = np.array([self.lx, self.ly])
        lo_rot = np.einsum('yij,j->yi', R, LO)
        rot_sv = np.einsum('yij,nj->yni', R, sv)
        N = len(sv)
        best = []
        t0 = time.monotonic()
        for i in range(0, len(self.cx), 4):
            bx, by = self.cx[i], self.cy[i]
            pts = rot_sv + lo_rot[:, None, :] + np.array([bx, by])
            px = ((pts[..., 0] - self.ox) / self.res).astype(np.int32)
            py = (self.h - 1 - ((pts[..., 1] - self.oy) / self.res)).astype(np.int32)
            ins = (px >= 0) & (px < self.w) & (py >= 0) & (py < self.h)
            sc = np.full(px.shape, 2.0, dtype=np.float32)
            sc[ins] = self.score_cell[py[ins], px[ins]]
            m = np.sort(sc, axis=1)[:, :int(N * 0.75)].mean(axis=1)
            j = int(np.argmin(m))
            best.append((float(m[j]), bx, by, math.degrees(yaws[j])))
        best.sort(key=lambda t: t[0])
        self.get_logger().info('auto_localize: coarse top %.3f at (%.2f,%.2f,%.0f) %.1fs'
                               % (best[0][0], best[0][1], best[0][2], best[0][3],
                                  time.monotonic() - t0))
        # refine top 5 with the full scan
        m = self.scan
        full = np.asarray(m.ranges, dtype=float)
        ang = m.angle_min + np.arange(len(full)) * m.angle_increment
        ok = np.isfinite(full) & (full > 0.15) & (full < 12.0)
        full = np.stack([full[ok] * np.cos(ang[ok]),
                         full[ok] * np.sin(ang[ok])], axis=1)
        refined = []
        for t in best[:5]:
            bb = (9e9, None)
            for dx in np.arange(-0.25, 0.26, 0.05):
                for dy in np.arange(-0.25, 0.26, 0.05):
                    for dyaw in np.arange(-8, 8.1, 2):
                        s = self.score(t[1] + dx, t[2] + dy,
                                       math.radians(t[3] + dyaw), full)
                        if s < bb[0]:
                            bb = (s, (t[1] + dx, t[2] + dy, t[3] + dyaw))
            refined.append((bb[0],) + bb[1])
        refined.sort(key=lambda t: t[0])
        return refined[0]

    # ------------------------------------------------------------- publish
    def publish_pose(self, x, y, yaw_deg, why):
        if self.count_subscribers('/initialpose') == 0:
            self.get_logger().warn('auto_localize: no /initialpose subscriber yet')
            return False
        yaw = math.radians(yaw_deg)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # tight prior: the global scan match is the best estimate; letting
        # AMCL start with a 0.5 m std lets it slide along a featureless
        # corridor (seen 2026-11-24: 1.04 -> 0.61 m).
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            time.sleep(0.15)
        self.get_logger().info(
            'auto_localize: %s -> (% .3f, % .3f, % .1f deg)' % (why, x, y, yaw_deg))
        self.done = True
        return True

    # ---------------------------------------------------------------- tick
    def tick(self):
        if self.done:
            return
        now = time.monotonic()
        if self.scan is None or now - self.scan_t > 2.0:
            if now - self.start_t > 3.0:
                self.get_logger().info('auto_localize: waiting for /scan_filtered',
                                       throttle_duration_sec=10.0)
            return
        if self.count_subscribers('/initialpose') == 0:
            return
        if self.attempts > 0 and now - self.last_try < self.retry:
            return
        sv = self.scan_vectors()
        if sv is None:
            self.get_logger().info(
                'auto_localize: waiting for a full scan (>= %d points)'
                % self.min_points, throttle_duration_sec=5.0)
            return
        self.last_try = now
        self.attempts += 1

        # 1) last saved pose
        if os.path.exists(self.pose_file):
            try:
                with open(self.pose_file) as f:
                    d = json.load(f)
                bx, by, byaw = float(d['x']), float(d['y']), float(d['yaw'])
                s = self.score(bx, by, byaw, sv)
                h = self.hit_rate(bx, by, byaw, sv)
                if self.acceptable(s, h):
                    if self.publish_pose(bx, by, math.degrees(byaw),
                                         'restored last pose score %.3f hit %.0f%%'
                                         % (s, 100 * h)):
                        return
                self.get_logger().info(
                    'auto_localize: last pose score %.3f hit %.0f%% '
                    '(need < %.2f and >= %.0f%%), global search'
                    % (s, 100 * h, self.thresh, 100 * self.hit_thresh))
            except Exception as e:
                self.get_logger().warn('auto_localize: last pose unusable: %s' % e)

        # 2) global search
        s, x, y, yaw = self.global_search(sv)
        h = self.hit_rate(x, y, math.radians(yaw), sv)
        if self.acceptable(s, h):
            self.publish_pose(x, y, yaw,
                              'global match score %.3f hit %.0f%%' % (s, 100 * h))
        else:
            self.get_logger().error(
                'auto_localize: NO match (score %.3f hit %.0f%%; need < %.2f '
                'and >= %.0f%%). Robot is probably outside the mapped area or '
                'the scene changed. NOT publishing; move it into the map or '
                're-map.' % (s, 100 * h, self.thresh, 100 * self.hit_thresh))


def main():
    rclpy.init()
    node = AutoLocalize()
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
