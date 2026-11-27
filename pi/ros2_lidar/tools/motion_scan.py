"""Pure geometry for the direction-aware scan gate (no ROS imports).

collision_monitor evaluates fixed polygons in base_footprint, so an obstacle
behind or beside the chassis blocks the robot even when it only wants to drive
forward.  A point is kept only if it is in the current-heading lane or if the
footprint could touch it during the next ``horizon`` seconds of the commanded
motion.  Motion is modelled as a constant-curvature arc (differential drive)
anchored at base_footprint, which is also the costmap footprint origin.

The lateral pad is intentionally larger than the longitudinal one: the old
fixed stop box gave 8 cm of side margin, while scan_self_filter already
discards everything within 2 cm of the body.  Keeping the lane at
half_width + 5 cm leaves the 3-5 cm side band visible (it can still stop a
real scrape) without reintroducing the 5-8 cm false stops.
"""
import math

DEFAULT_PAD_LONG = 0.02
DEFAULT_PAD_WIDTH = 0.05


def motion_active(v, w, v_eps=0.02, w_eps=0.02):
    """True when the command would actually move the chassis."""
    return abs(v) > v_eps or abs(w) > w_eps


def should_filter_cmd(enabled, cmd_age_s, cmd_timeout_s,
                      v, w, v_eps=0.02, w_eps=0.02):
    """Safety decision shared by the node and the tests.

    Pass the full scan through when the gate is disabled, when the command is
    stale/missing, or when the chassis would not move.  Only an enabled gate
    with a fresh, moving command is allowed to discard points.
    """
    if not enabled:
        return False
    if cmd_age_s is None or cmd_age_s > cmd_timeout_s:
        return False
    return motion_active(v, w, v_eps, w_eps)


def lookup_tf_or_none(buffer, target_frame, source_frame, stamp, timeout):
    """Any TF failure means 'pass the full scan through' (conservative)."""
    try:
        return buffer.lookup_transform(target_frame, source_frame,
                                       stamp, timeout)
    except Exception:  # noqa: BLE001 - every TF failure is fail-open
        return None


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


def corridor_relevant(px, py, v_sign, half_length, half_width,
                      pad_long, pad_width):
    """True if (px, py) is in the current-heading lane of a straight move.

    This is deliberately speed-independent: the front of every stop/slow zone
    stays visible even when the commanded linear speed is tiny.  ``v_sign`` is
    +1 forward, -1 reverse, 0 for pure rotation (no lane).
    """
    if v_sign == 0:
        return False
    if abs(py) > half_width + pad_width:
        return False
    return px * v_sign >= -(half_length + pad_long)


def point_relevant(px, py, samples, half_length, half_width,
                   pad_long, pad_width, v_sign=0):
    """True if (px, py) is on the predicted swept footprint or heading lane."""
    if corridor_relevant(px, py, v_sign, half_length, half_width,
                         pad_long, pad_width):
        return True
    hl = half_length + pad_long
    hw = half_width + pad_width
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


def filter_ranges_union(ranges, angle_min, angle_increment, motions,
                        half_length, half_width, pad_long, pad_width,
                        laser_x=0.0, laser_y=0.0, laser_yaw=0.0,
                        range_min=0.0, range_max=float('inf')):
    """Keep a point if any ``(samples, v_sign)`` motion can reach it.

    The union is used for the short hold after a command sign flip: the old
    mask stays in force until the chassis has actually decelerated.
    Returns ``(new_ranges, kept, checked)``.

    ``range_min`` is deliberately NOT used to skip points: a real object closer
    than the sensor's advertised minimum (A1M8: 0.15 m) still returns a finite
    range. Skipping those points left them in the gated scan forever, so
    collision_monitor kept stopping on a box 0.12 m behind the car even when
    driving forward (2026-11-27). Only non-finite / non-positive / beyond
    ``range_max`` points are ignored.
    """
    motions = list(motions)
    cy = math.cos(laser_yaw)
    sy = math.sin(laser_yaw)
    out = list(ranges)
    kept = 0
    checked = 0
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0 or r > range_max:
            continue
        checked += 1
        a = angle_min + i * angle_increment
        px = r * math.cos(a)
        py = r * math.sin(a)
        bx = laser_x + px * cy - py * sy
        by = laser_y + px * sy + py * cy
        if any(point_relevant(bx, by, samples, half_length, half_width,
                              pad_long, pad_width, v_sign)
               for samples, v_sign in motions):
            kept += 1
        else:
            out[i] = float('inf')
    return out, kept, checked


def filter_ranges(ranges, angle_min, angle_increment,
                  samples, half_length, half_width, pad_long, pad_width,
                  laser_x=0.0, laser_y=0.0, laser_yaw=0.0,
                  range_min=0.0, range_max=float('inf'), v_sign=0):
    """Single-motion wrapper around :func:`filter_ranges_union`."""
    return filter_ranges_union(
        ranges, angle_min, angle_increment, [(samples, v_sign)],
        half_length, half_width, pad_long, pad_width,
        laser_x=laser_x, laser_y=laser_y, laser_yaw=laser_yaw,
        range_min=range_min, range_max=range_max)
