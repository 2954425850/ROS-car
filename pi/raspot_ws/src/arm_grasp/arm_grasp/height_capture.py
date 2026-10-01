"""Bounded stationary scan, with injected ROS/camera I/O and replay manifests.

This is a small motion envelope, NOT collision detection. Begin in a clear,
stationary observation pose with the target away from image/gripper borders.
Driver/service ownership belongs to the CLI, never this module.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
import time

import numpy as np

from . import geom
from .arm_kin import FIELD_HI, FIELD_LO, from_fields, to_fields
from .height import HeightRefused
from .height_vision import calibration_id, calibration_record
from .servo import ik_open_m, limit_step


@dataclass(frozen=True)
class ScanConfig:
    lateral_m: float = .025
    lift_m: float = .018
    max_tip_travel_m: float = .035
    max_joint_travel_deg: float = 12.0
    max_step_deg: float = 1.5
    min_field_margin: float = 8.0
    max_feedback_undershoot: float = 8.0
    max_feedback_drift: float = 2.0
    arrived_tolerance: float = 12.0
    stationary_seconds: float = .6
    move_timeout_s: float = 12.0
    shot_timeout_s: float = 4.0


def valid_feedback(fields):
    return (fields is not None and len(fields) == 6
            and all(math.isfinite(float(x)) for x in fields)
            and all(fields[i] != 0 for i in (2, 3, 4)))


def validate_path(start, finish, fields, config=None, floor_z=None, require_margin=True):
    cfg = config or ScanConfig()
    origin = np.array(geom.gripper_tip(start, closed=False))
    floor_z = origin[2] - .002 if floor_z is None else floor_z
    if max(abs(finish[k] - start[k]) for k in start) > cfg.max_joint_travel_deg:
        raise HeightRefused('scan joint travel too large')
    reference_fields = np.asarray(fields, dtype=float)
    if (np.min(reference_fields[2:5]) < FIELD_LO - cfg.max_feedback_undershoot
            or np.max(reference_fields[2:5]) > FIELD_HI + cfg.max_feedback_undershoot):
        raise HeightRefused('feedback far outside firmware limits')
    low = np.minimum(reference_fields[2:5], FIELD_LO)
    high = np.maximum(reference_fields[2:5], FIELD_HI)
    for t in np.linspace(0, 1, 41):
        j = {k: start[k] + float(t) * (finish[k] - start[k]) for k in start}
        f = to_fields(j, fields[0], fields[1])
        margin = cfg.min_field_margin if require_margin and t == 1 else 0.0
        if (np.any(np.array(f[2:5]) < low - 1e-7)
                or np.any(np.array(f[2:5]) > high + 1e-7)
                or abs(f[5]) > 1000):
            raise HeightRefused('scan path near firmware limit')
        if margin and (min(f[2:5]) < FIELD_LO + margin or max(f[2:5]) > FIELD_HI - margin):
            raise HeightRefused('scan endpoint near firmware limit')
        if (t == 1 and not require_margin and (min(f[2:5]) < FIELD_LO or max(f[2:5]) > FIELD_HI)
                and not np.allclose(f[2:5], reference_fields[2:5], atol=1e-6, rtol=0)):
            raise HeightRefused('out-of-range endpoint is not recorded reference pose')
        bounded = bounded_fields(f)
        # Check both measured-pose interpolation and the actual legal command
        # interpolation; a slightly low resting pot reading is not a command.
        for pose in (j, from_fields(bounded)):
            tip = np.array(geom.gripper_tip(pose, closed=False))
            if tip[2] < floor_z:
                raise HeightRefused('scan path would lower tip below observation floor')
            if np.linalg.norm(tip - origin) > cfg.max_tip_travel_m:
                raise HeightRefused('scan tip travel too large')


def plan_scan(joints, fields, config=None):
    cfg = config or ScanConfig()
    if not valid_feedback(fields) or abs(fields[1] - 496) > 8:
        raise HeightRefused('scan needs valid feedback and calibrated wrist roll')
    validate_path(joints, joints, fields, cfg, require_margin=False)
    tip = np.array(geom.gripper_tip(joints, closed=False))
    yaw = math.radians(joints['base'])
    lateral = np.array([-math.sin(yaw), math.cos(yaw), 0.])
    radial = np.array([math.cos(yaw), math.sin(yaw), 0.])
    alpha = joints['shoulder'] + joints['elbow'] + joints['wrist_pitch']
    offsets = [cfg.lateral_m * lateral, -cfg.lateral_m * lateral,
               -.01 * radial + np.array([0., 0., cfg.lift_m]),
               cfg.lateral_m * .65 * lateral + np.array([0., 0., cfg.lift_m])]
    poses = [dict(joints)]
    skipped = []
    # Every excursion returns through the reference pose: larger opposite-side
    # moves must not sneak through a 35 mm per-segment constraint.
    for delta in offsets:
        # Small pitch changes can move the elbow away from the resting limit
        # while keeping the tip at/above the observation height. Actual camera
        # translations and rotations will be measured at each stationary view.
        found = False
        for scale in (1., .9, .75, .6):
            for da in (0., -4., 4., -8., 8.):
                try:
                    candidate = ik_open_m(*(tip + delta * scale), alpha + da)
                    validate_path(joints, candidate, fields, cfg, tip[2] - .002)
                    validate_path(candidate, joints, fields, cfg, tip[2] - .002,
                                  require_margin=False)
                except ValueError as e:
                    skipped.append(str(e))
                    continue
                poses.append(candidate)
                found = True
                break
            if found:
                break
        if len(poses) == 4:
            break
    if len(poses) < 3:
        raise HeightRefused('too few bounded scan poses: %s' % '; '.join(skipped))
    centres = np.array([geom.camera_center(j) for j in poses])
    if np.max(np.linalg.norm(centres[:, None] - centres[None, :], axis=2)) < .012:
        raise HeightRefused('scan cannot produce enough camera baseline')
    return poses


def bounded_fields(fields):
    result = list(map(float, fields))
    result[2:5] = [max(FIELD_LO, min(FIELD_HI, x)) for x in result[2:5]]
    return result


def publish_fields(io, fields):
    from .collect import JOINT_NAMES
    msg = io._js()
    msg.name, msg.position = list(JOINT_NAMES), bounded_fields(fields)
    io.pub.publish(msg)


def stationary_feedback(io, hold_fields=None, config=None, timeout=None,
                        publish=publish_fields):
    cfg = config or ScanConfig()
    start, window, last_n = time.monotonic(), [], io.fb_n
    timeout = cfg.move_timeout_s if timeout is None else timeout
    while time.monotonic() - start < timeout:
        if hold_fields is not None:
            publish(io, hold_fields)
        io.spin(.05)
        if io.fb_n == last_n:
            continue
        last_n = io.fb_n
        if not valid_feedback(io.fb):
            continue
        now, fields = time.monotonic(), list(map(float, io.fb))
        if hold_fields is not None and max(abs(fields[k] - hold_fields[k])
                                           for k in range(1, 6)) > cfg.arrived_tolerance:
            window = []
            continue
        window.append((now, fields))
        window = [(t, f) for t, f in window if now - t <= cfg.stationary_seconds + .2]
        samples = np.array([f for _, f in window])
        if (len(window) >= 5 and now - window[0][0] >= cfg.stationary_seconds
                and _max_deviation(samples) <= cfg.max_feedback_drift):
            return np.median(samples, axis=0).tolist()
    raise HeightRefused('fresh stationary feedback timeout')


def move_to(io, joints, hold_fields, config=None, publish=publish_fields, floor_z=None,
            require_margin=True):
    cfg = config or ScanConfig()
    if not valid_feedback(io.fb):
        raise HeightRefused('no valid feedback before scan motion')
    current = from_fields(io.fb)
    validate_path(current, joints, hold_fields, cfg, floor_z, require_margin)
    start = time.monotonic()
    while time.monotonic() - start < cfg.move_timeout_s:
        current, _, reached = limit_step(current, joints, cfg.max_step_deg)
        publish(io, to_fields(current, hold_fields[0], hold_fields[1]))
        io.spin(.1)
        if reached:
            return stationary_feedback(io, to_fields(joints, hold_fields[0], hold_fields[1]),
                                       cfg, publish=publish)
    raise HeightRefused('scan motion timeout')


def _max_deviation(values):
    """Largest joint reading distance from the median pose that will be recorded.

    Stationary servos dither by +/-1-2 counts (seen on the real wrist pitch:
    154-157 while holding 156), so a peak-to-peak gate rejects good frames.
    A sustained move still shifts samples away from the median and is refused.
    """
    values = np.array(values, dtype=float)[:, 1:]
    return float(np.max(np.abs(values - np.median(values, axis=0))))


def _drift_diagnostics(hold_fields, values):
    values = np.array(values, dtype=float)
    return {'hold_fields': list(map(float, hold_fields)),
            'commanded_fields': bounded_fields(hold_fields),
            'feedback_span_counts': np.ptp(values, axis=0).tolist(),
            'feedback_samples': values.tolist()}


def snapshot(io, host, path, hold_fields, config=None, capture=None,
             publish=publish_fields):
    cfg = config or ScanConfig()
    if capture is None:
        from .collect import capture_frame
        capture = capture_frame
    before = stationary_feedback(io, hold_fields, cfg, publish=publish)
    values = [before]
    last_n = io.fb_n
    fresh_count, last_fresh = 0, time.monotonic()
    with ThreadPoolExecutor(max_workers=1) as pool:
        shot = pool.submit(capture, str(path), host, timeout=cfg.shot_timeout_s)
        while not shot.done():
            publish(io, hold_fields)
            io.spin(.05)
            if io.fb_n != last_n:
                last_n = io.fb_n
                if valid_feedback(io.fb):
                    fields = list(map(float, io.fb))
                    values.append(fields)
                    fresh_count += 1
                    last_fresh = time.monotonic()
                    if _max_deviation(values) > cfg.max_feedback_drift:
                        raise HeightRefused('feedback drift during exposure; image discarded',
                                            _drift_diagnostics(hold_fields, values))
            if time.monotonic() - last_fresh > .5:
                raise HeightRefused('feedback stale during exposure; image discarded')
        if not shot.result():
            raise HeightRefused('camera capture failed')
    if fresh_count < 3:
        raise HeightRefused('too few fresh feedback samples during exposure')
    after = stationary_feedback(io, hold_fields, cfg, publish=publish)
    values.append(after)
    if _max_deviation(values) > cfg.max_feedback_drift:
        raise HeightRefused('feedback changed across exposure; image discarded',
                            _drift_diagnostics(hold_fields, values))
    fields = np.median(np.array(values), axis=0).tolist()
    return {'image': Path(path).name, 'fields': fields, 'joints': from_fields(fields),
            'feedback_samples': fresh_count,
            'feedback_span_counts': np.ptp(np.array(values), axis=0).tolist()}


def capture_scan(io, host, box, out_dir, config=None, capture=None, log=print):
    cfg, start = config or ScanConfig(), time.monotonic()
    directory = Path(out_dir)
    directory.mkdir(parents=True, exist_ok=False)
    fields = stationary_feedback(io, config=cfg)
    reference = from_fields(fields)
    poses = plan_scan(reference, fields, cfg)
    floor_z = geom.gripper_tip(reference, closed=False)[2] - .002
    session = {'schema': 'arm_grasp.height_session/v1', 'coordinate_frame': 'raw_stream',
               'calibration_id': calibration_id(), 'calibration': calibration_record(),
               'reference_box': list(box), 'views': [], 'scan_config': asdict(cfg),
               'complete': False, 'reason': None}
    path = directory / 'session.json'
    try:
        for i, pose in enumerate(poses):
            if i:
                move_to(io, reference, fields, cfg, floor_z=floor_z, require_margin=False)
                hold = move_to(io, pose, fields, cfg, floor_z=floor_z)
            else:
                hold = fields
            log('测高观察 %d/%d：停稳拍摄' % (i + 1, len(poses)))
            view = snapshot(io, host, directory / ('view-%02d.jpg' % i), hold, cfg, capture)
            session['views'].append(view)
        move_to(io, reference, fields, cfg, floor_z=floor_z, require_margin=False)
        session['complete'] = True
    except BaseException as e:
        session['reason'] = str(e) or type(e).__name__
        raise
    finally:
        session['acquisition_seconds'] = time.monotonic() - start
        path.write_text(json.dumps(session, ensure_ascii=False, indent=2, allow_nan=False),
                        encoding='utf-8')
    return path
