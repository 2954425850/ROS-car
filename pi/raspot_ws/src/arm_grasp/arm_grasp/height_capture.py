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
from .arm_kin import FIELD_HI, FIELD_LO, fk, from_fields, to_fields
from .height import HeightRefused
from .height_vision import calibration_id, calibration_record
from .servo import ik_open_m, limit_step


@dataclass(frozen=True)
class ScanConfig:
    # Left/right is a pure base rotation: on the folded resting pose the tip
    # is ~8 cm from the axis, so a 25 mm tip move needs 17 deg of base, while
    # the camera (~15 cm out) already moves >20 mm for 8 deg.
    lateral_base_deg: float = 8.0
    # Forward/back at constant tip height: shoulder/elbow/wrist are accurate
    # (<2 deg on real data) and the move is across the line of sight. The old
    # lift moved mostly along it (<2 deg parallax), and the base alone stalls
    # several degrees short (real +side move gave a 7.7 mm baseline).
    radial_m: float = .02
    max_tip_travel_m: float = .035
    max_joint_travel_deg: float = 12.0
    # Planned excursions leave room for the arm not returning exactly to the
    # reference (live: base back +0.7 deg pushed an 11.3 deg plan over 12).
    plan_travel_margin_deg: float = 2.0
    max_step_deg: float = 1.5
    min_field_margin: float = 8.0
    max_feedback_undershoot: float = 8.0
    # Motion = the median pose moving; noise = single readings scattering.
    # Live stationary scatter: wrist roll 495-499, loaded shoulder 610-614.
    max_feedback_drift: float = 2.0
    max_feedback_spike: float = 6.0
    arrived_tolerance: float = 12.0
    stationary_seconds: float = .6
    move_timeout_s: float = 12.0
    shot_timeout_s: float = 4.0


def valid_feedback(fields):
    return (fields is not None and len(fields) == 6
            and all(math.isfinite(float(x)) for x in fields)
            # The five position servos (fields 0-4) are polled; 0 means that
            # servo was not read this cycle (seen live: one wrist-roll 0 among
            # steady 497 readings). The base field is the firmware's tracker
            # estimate, never an unread marker, and 0 is car front - exactly
            # where re-homing parks it.
            and all(fields[i] != 0 for i in range(5)))


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


def target_stays_in_frame(reference, candidate, box, depths=(.10, .17, .25, .35, .45)):
    """The reference box, at any plausible depth along its rays, stays inside
    the usable image of the candidate view (same limits as segmentation).

    Live: a backward view moved the image up so far that the cap was half
    cut off, which starved the top of tracks. The depth is unknown before
    measuring, so every depth in the working range must stay in frame.
    """
    w, h = geom.STREAM_W, geom.STREAM_H
    for u in (box[0], box[2]):
        for v in (box[1], box[3]):
            c, d = geom.pixel_ray(reference, *geom.to_ai(u * w, v * h))
            for t in depths:
                point = tuple(c[i] + t * d[i] for i in range(3))
                try:
                    x, y = geom.to_stream(*geom.project(candidate, point))
                except ValueError:
                    return False
                if not (8 <= x <= w - 8 and 8 <= y <= h * .86):
                    return False
    return True


def plan_scan(joints, fields, config=None, box=None):
    cfg = config or ScanConfig()
    if not valid_feedback(fields) or abs(fields[1] - 496) > 8:
        raise HeightRefused('scan needs valid feedback and calibrated wrist roll')
    validate_path(joints, joints, fields, cfg, require_margin=False)
    tip = np.array(geom.gripper_tip(joints, closed=False))
    yaw = math.radians(joints['base'])
    radial = np.array([math.cos(yaw), math.sin(yaw), 0.])
    alpha = joints['shoulder'] + joints['elbow'] + joints['wrist_pitch']
    def radial_move(sign):
        for scale in (1., .9, .75, .6):
            for da in (0., -4., 4., -8., 8.):
                yield lambda: ik_open_m(*(tip + sign * cfg.radial_m * scale * radial), alpha + da)

    def base_move(sign):
        for scale in (1., .75, .5):
            yield lambda: dict(joints, base=joints['base'] + sign * cfg.lateral_base_deg * scale)

    moves = [radial_move(1), radial_move(-1), base_move(1), base_move(-1)]
    poses = [dict(joints)]
    skipped = []
    centre0 = np.array(geom.camera_center(joints))
    axis0 = np.array(fk(joints)[1])
    def sideways(candidate):
        # Parallax comes from camera motion across the line of sight; motion
        # along it (or a pure pitch rotation) adds little.
        d = np.array(geom.camera_center(candidate)) - centre0
        return float(np.linalg.norm(d - (d @ axis0) * axis0))
    # Every excursion returns through the reference pose: larger opposite-side
    # moves must not sneak through a 35 mm per-segment constraint.
    for move in moves:
        # Small pitch changes can move the elbow away from the resting limit
        # while keeping the tip at/above the observation height. Among all
        # candidates that pass every safety check, keep the one that moves the
        # camera most across its view (a folded resting pose cannot do a pure
        # 20 mm radial move inside the joint-travel cap, and the first legal
        # candidate there mostly rotated the camera). Actual camera poses
        # are measured at each stationary view.
        best = None
        for make in move:
            try:
                candidate = make()
                if (max(abs(candidate[k] - joints[k]) for k in joints)
                        > cfg.max_joint_travel_deg - cfg.plan_travel_margin_deg):
                    raise HeightRefused('scan joint travel leaves no return margin')
                validate_path(joints, candidate, fields, cfg, tip[2] - .002)
                validate_path(candidate, joints, fields, cfg, tip[2] - .002,
                              require_margin=False)
                if box is not None and not target_stays_in_frame(joints, candidate, box):
                    raise HeightRefused('target would leave the image')
            except ValueError as e:
                skipped.append(str(e))
                continue
            if best is None or sideways(candidate) > sideways(best):
                best = candidate
        if best is not None:
            poses.append(best)
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
                and _max_deviation(samples) <= cfg.max_feedback_spike
                and _median_shift(samples[:len(samples) // 2],
                                  samples[len(samples) // 2:]) <= cfg.max_feedback_drift):
            return np.median(samples, axis=0).tolist()
    raise HeightRefused('fresh stationary feedback timeout')


def move_to(io, joints, hold_fields, config=None, publish=publish_fields, floor_z=None,
            require_margin=True):
    cfg = config or ScanConfig()
    if not valid_feedback(io.fb):
        raise HeightRefused('no valid feedback before scan motion')
    current = from_fields(io.fb)
    if floor_z is not None:
        # The floor bounds commanded descent, measured from where the arm
        # actually is: returning from a view that sagged 1 mm (live: elbow
        # 172 vs 177 commanded) dipped the joint path past the reference
        # floor, and the scan could not even go home.
        floor_z = min(floor_z, geom.gripper_tip(current, closed=False)[2] - .002)
    try:
        validate_path(current, joints, hold_fields, cfg, floor_z, require_margin)
    except HeightRefused as e:
        e.diagnostics.update(start_fields=list(map(float, io.fb)),
                             target_fields=to_fields(joints, hold_fields[0], hold_fields[1]),
                             floor_z_m=floor_z)
        raise
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
    """Largest single joint reading distance from the median pose.

    Stationary servos scatter by several counts (live: wrist pitch 153-157,
    wrist roll 495-499, loaded shoulder 610-614 around a 612.7 command), so
    single readings only bound gross glitches/jumps. Motion is judged by the
    median moving (_median_shift); the recorded pose is the median.
    """
    values = np.array(values, dtype=float)[:, 1:]
    return float(np.max(np.abs(values - np.median(values, axis=0))))


def _median_shift(a, b):
    a, b = np.array(a, dtype=float)[:, 1:], np.array(b, dtype=float)[:, 1:]
    return float(np.max(np.abs(np.median(a, axis=0) - np.median(b, axis=0))))


def _drift_diagnostics(hold_fields, values):
    values = np.array(values, dtype=float)
    return {'hold_fields': list(map(float, hold_fields)),
            'commanded_fields': bounded_fields(hold_fields),
            'feedback_span_counts': np.ptp(values, axis=0).tolist(),
            'feedback_samples': values.tolist()}


def _blank(path):
    """True for the uniform grey frame the RTSP stream can hand out on start."""
    if not Path(path).exists():
        return False  # nothing to judge; a missing image is refused on load
    import cv2
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    return image is None or float(image.std()) < 3.0


def _capture_nonblank(capture, path, host, timeout, attempts=2):
    # Live: one grab returned a flat grey frame (brightness std ~0; a real
    # desk scene is ~26). Retry once inside the monitored exposure.
    for _ in range(attempts):
        if capture(path, host, timeout=timeout) and not _blank(path):
            return True
    return False


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
        shot = pool.submit(_capture_nonblank, capture, str(path), host, cfg.shot_timeout_s)
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
                    if _max_deviation(values) > cfg.max_feedback_spike:
                        raise HeightRefused('feedback drift during exposure; image discarded',
                                            _drift_diagnostics(hold_fields, values))
            if time.monotonic() - last_fresh > .5:
                raise HeightRefused('feedback stale during exposure; image discarded')
        if not shot.result():
            raise HeightRefused('camera capture failed')
    if fresh_count < 3:
        raise HeightRefused('too few fresh feedback samples during exposure')
    after = stationary_feedback(io, hold_fields, cfg, publish=publish)
    exposure = values[1:]
    values.append(after)
    if max(_median_shift(exposure, [before]), _median_shift([after], [before])) > cfg.max_feedback_drift:
        raise HeightRefused('feedback changed across exposure; image discarded',
                            _drift_diagnostics(hold_fields, values))
    fields = np.median(np.array(values), axis=0).tolist()
    return {'image': Path(path).name, 'fields': fields, 'joints': from_fields(fields),
            'feedback_samples': fresh_count,
            'feedback_span_counts': np.ptp(np.array(values), axis=0).tolist()}


def restore_pose(io, fields, config=None, publish=publish_fields, home_timeout=40.0,
                 min_wait=3.0, move_timeout=60.0):
    """Drive back to the recorded start readings after the service restart.

    Restarting the teleop service makes the firmware re-run its power-up
    homing (five joints to the factory pose, base re-centred to car front),
    so the camera no longer sees the object the operator aimed at. The joints
    report 0 while homing, which valid_feedback rejects, so "stationary valid
    feedback" means homing has finished. The way back is streamed in small
    joint steps like the scan; gripper and wrist roll go to their start values.
    """
    cfg = config or ScanConfig()
    io.spin(min_wait)
    current = stationary_feedback(io, config=cfg, timeout=home_timeout)
    target = from_fields(fields)
    joints = from_fields(current)
    start = time.monotonic()
    while True:
        joints, _, reached = limit_step(joints, target, cfg.max_step_deg)
        publish(io, to_fields(joints, fields[0], fields[1]))
        io.spin(.1)
        if reached:
            break
        if time.monotonic() - start > move_timeout:
            raise HeightRefused('return to start pose timed out')
    return stationary_feedback(io, list(map(float, fields)), cfg, publish=publish)


def capture_scan(io, host, box, out_dir, config=None, capture=None, log=print,
                 publish=publish_fields, reference_command=None):
    cfg, start = config or ScanConfig(), time.monotonic()
    directory = Path(out_dir)
    directory.mkdir(parents=True, exist_ok=False)
    fields = stationary_feedback(io, config=cfg)
    reference = from_fields(fields)
    # Hold the reference with the command that put the arm there, not with
    # its sagged reading: re-commanding the reading lowers the target and the
    # shoulder sags again mid-exposure (live: 610 -> 612.5 on the first view).
    home_fields = list(map(float, reference_command if reference_command is not None
                           else fields))
    home = from_fields(home_fields)
    poses = plan_scan(reference, fields, cfg, box)
    floor_z = geom.gripper_tip(reference, closed=False)[2] - .002
    session = {'schema': 'arm_grasp.height_session/v1', 'coordinate_frame': 'raw_stream',
               'calibration_id': calibration_id(), 'calibration': calibration_record(),
               'reference_box': list(box), 'views': [], 'scan_config': asdict(cfg),
               'complete': False, 'reason': None}
    path = directory / 'session.json'
    try:
        for i, pose in enumerate(poses):
            if i:
                move_to(io, home, home_fields, cfg, publish=publish, floor_z=floor_z,
                        require_margin=False)
                settled = move_to(io, pose, fields, cfg, publish=publish, floor_z=floor_z)
                # Position servos keep the command that reached this pose:
                # re-commanding the sagged reading moves the target and they
                # sag again mid-exposure (live: shoulder 555 -> 559). The base
                # is the opposite: its velocity loop stalls short (live: 35 vs
                # 32.8 commanded, beyond the 0.5 deg deadband) and keeps pushing
                # until it slips mid-exposure (35 -> 28). It has no gravity sag,
                # so hold it where it settled. The camera pose is still the
                # measured feedback from snapshot.
                hold = to_fields(pose, fields[0], fields[1])
                hold[5] = settled[5]
            else:
                hold = home_fields
            log('测高观察 %d/%d：停稳拍摄' % (i + 1, len(poses)))
            view = snapshot(io, host, directory / ('view-%02d.jpg' % i), hold, cfg, capture,
                            publish=publish)
            session['views'].append(view)
        move_to(io, home, home_fields, cfg, publish=publish, floor_z=floor_z,
                require_margin=False)
        session['complete'] = True
    except BaseException as e:
        session['reason'] = str(e) or type(e).__name__
        if isinstance(e, Exception) and session['views']:
            # Leave the arm where the scan started: the service restart homes
            # the other joints but not the base, so an abandoned excursion
            # silently re-aims the next run (live: base crept 36 -> 114).
            try:
                move_to(io, home, home_fields, cfg, publish=publish, floor_z=floor_z,
                        require_margin=False)
                session['returned_to_reference'] = True
            except Exception as back:
                session['returned_to_reference'] = False
                log('测高中止后返回起始姿态失败：%s' % back)
        raise
    finally:
        session['acquisition_seconds'] = time.monotonic() - start
        path.write_text(json.dumps(session, ensure_ascii=False, indent=2, allow_nan=False),
                        encoding='utf-8')
    return path
