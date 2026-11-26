"""Pure geometry for the direction-aware scan gate (no ROS imports).

collision_monitor evaluates fixed polygons in base_footprint, so an obstacle
behind or beside the chassis blocks the robot even when it only wants to drive
forward.  A point is kept only if it is in the current-heading lane or if the
footprint could touch it during the next ``horizon`` seconds of the commanded
motion.  Motion is modelled as a constant-curvature arc (differential drive)
anchored at base_footprint, which is also the costmap footprint origin.
"""
import math


def motion_active(v, w, v_eps=0.02, w_eps=0.02):
    """True when the command would actually move the chassis."""
    return abs(v) > v_eps or abs(w) > w_eps


def arc_samples(v, w, horizon=1.0, step=0.1):
    """Poses along the predicted arc, expressed in the robot frame at t=0.

    Each item is ``(x, y, cos(theta), sin(theta))``.  For ``v == 0`` the poses
    only rotate in place, which is what a pivot turn needs.
    """
    if horizon <= 0.0 or step <= 0.0:
        return [(0.0, 0.0, 1.0, 0.0)]
    n = int(math.ceil(horizon / step))
    out = []
    for k in range(n + 1):
        t = min(k * step, horizon)
        th = w * t
        if abs(w) < 1e-6:
            x = v * t
            y = 0.0
        else:
            r = v / w
            x = r * math.sin(th)
            y = r * (1.0 - math.cos(th))
        out.append((x, y, math.cos(th), math.sin(th)))
    return out


def corridor_relevant(px, py, v_sign, half_length, half_width, pad):
    """True if (px, py) is in the current-heading lane of a straight move.

    This is deliberately speed-independent: the front of every stop/slow zone
    stays visible even when the commanded linear speed is tiny.  ``v_sign`` is
    +1 forward, -1 reverse, 0 for pure rotation (no lane).
    """
    if v_sign == 0:
        return False
    if abs(py) > half_width + pad:
        return False
    return px * v_sign >= -(half_length + pad)


def point_relevant(px, py, samples, half_length, half_width, pad, v_sign=0):
    """True if (px, py) is on the predicted swept footprint or heading lane."""
    if corridor_relevant(px, py, v_sign, half_length, half_width, pad):
        return True
    hl = half_length + pad
    hw = half_width + pad
    for x, y, c, s in samples:
        dx = px - x
        dy = py - y
        lx = dx * c + dy * s
        if lx < -hl or lx > hl:
            continue
        ly = -dx * s + dy * c
        if -hw <= ly <= hw:
            return True
    return False


def filter_ranges(ranges, angle_min, angle_increment,
                  samples, half_length, half_width, pad,
                  laser_x=0.0, laser_y=0.0, laser_yaw=0.0,
                  range_min=0.0, range_max=float('inf'), v_sign=0):
    """Set ranges outside the predicted sweep to inf.

    Returns ``(new_ranges, kept, checked)``.  Invalid and out-of-range samples
    are left untouched so the sensor's own convention is preserved.
    """
    cy = math.cos(laser_yaw)
    sy = math.sin(laser_yaw)
    out = list(ranges)
    kept = 0
    checked = 0
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        checked += 1
        a = angle_min + i * angle_increment
        px = r * math.cos(a)
        py = r * math.sin(a)
        bx = laser_x + px * cy - py * sy
        by = laser_y + px * sy + py * cy
        if point_relevant(bx, by, samples, half_length, half_width, pad,
                          v_sign):
            kept += 1
        else:
            out[i] = float('inf')
    return out, kept, checked
