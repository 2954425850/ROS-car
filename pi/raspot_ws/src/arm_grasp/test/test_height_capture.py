import json
import time

import numpy as np
import pytest

from arm_grasp import geom
from arm_grasp.arm_kin import from_fields, to_fields
from arm_grasp.height import HeightRefused, measurement_target
from arm_grasp.height_capture import (ScanConfig, bounded_fields, capture_scan, move_to,
                                      plan_scan, restore_pose, snapshot, target_stays_in_frame,
                                      validate_path)
from arm_grasp.servo import ik_open_m


def observer():
    j = ik_open_m(.16, -.02, .04, -75)
    return j, to_fields(j, 240, 496)


def test_scan_has_real_baseline_preserves_height_and_limits():
    j, fields = observer()
    poses = plan_scan(j, fields)
    assert len(poses) >= 3
    centres = np.array([geom.camera_center(p) for p in poses])
    assert np.max(np.linalg.norm(centres - centres[0], axis=1)) >= .012
    floor = geom.gripper_tip(j, False)[2] - .002
    for pose in poses:
        validate_path(j, pose, fields, floor_z=floor)
        validate_path(pose, j, fields, floor_z=floor)
        assert to_fields(pose, fields[0], fields[1])[:2] == fields[:2]


def test_scan_refuses_uncalibrated_roll_and_large_motion():
    j, fields = observer()
    with pytest.raises(HeightRefused, match='roll'):
        plan_scan(j, [240, 600] + fields[2:])
    bad = dict(j, base=j['base'] + 40)
    with pytest.raises(HeightRefused, match='travel'):
        validate_path(j, bad, fields)


def test_legal_resting_pose_near_limit_can_observe_without_lowering_tip():
    from arm_grasp.arm_kin import from_fields
    fields = [240, 496, 179, 128, 413, 6]
    j = from_fields(fields)
    poses = plan_scan(j, fields)
    assert len(poses) >= 3
    floor = geom.gripper_tip(j, False)[2] - .002
    for pose in poses[1:]:
        validate_path(j, pose, fields, floor_z=floor)
        validate_path(pose, j, fields, floor_z=floor, require_margin=False)


def test_actual_pi_undershoot_never_produces_illegal_commands():
    from arm_grasp.arm_kin import from_fields
    from arm_grasp.height_capture import bounded_fields
    fields = [240, 498, 176, 121, 512, 10]
    j = from_fields(fields)
    poses = plan_scan(j, fields)
    assert len(poses) >= 3
    floor = geom.gripper_tip(j, False)[2] - .002
    for pose in poses[1:]:
        validate_path(j, pose, fields, floor_z=floor)
        validate_path(pose, j, fields, floor_z=floor, require_margin=False)
        for t in np.linspace(0, 1, 21):
            state = {k: j[k] + t * (pose[k] - j[k]) for k in j}
            command = bounded_fields(to_fields(state, fields[0], fields[1]))
            assert all(125 <= x <= 875 for x in command[2:5])
            assert geom.gripper_tip(from_fields(command), False)[2] >= floor
    bad = list(fields)
    bad[3] = 110
    with pytest.raises(HeightRefused, match='far outside'):
        plan_scan(from_fields(bad), bad)


def test_scan_tolerates_imperfect_return_to_reference():
    # Real resting pose whose 25 mm lateral plan needed 11.3 deg of base travel;
    # the arm came back slightly off and the next live move exceeded 12 deg.
    fields = [240.0, 498.0, 154.0, 149.0, 605.0, 89.0]
    j = from_fields(fields)
    poses = plan_scan(j, fields)
    assert len(poses) == 5
    floor = geom.gripper_tip(j, False)[2] - .002
    for base_error in (-1.0, 1.0):
        returned = dict(j, base=j['base'] + base_error)
        for pose in poses[1:]:
            validate_path(returned, pose, fields, floor_z=floor - .002)


def test_scan_keeps_the_target_in_frame():
    # Real state 1 and box: the backward view pushed the cap off the image.
    fields = [238.0, 497.0, 185.0, 137.0, 599.0, 47.0]
    box = [.405, .22, .525, .45]
    j = from_fields(fields)
    unconstrained = plan_scan(j, fields)
    assert not all(target_stays_in_frame(j, p, box) for p in unconstrained)
    poses = plan_scan(j, fields, box=box)
    assert len(poses) >= 4
    assert all(target_stays_in_frame(j, p, box) for p in poses)


class FakeIO:
    def __init__(self):
        self.fb = observer()[1]
        self.fb_n = 0
        self.drift = False
        self.jitter = False
        self.glitch_at = None
        self.base_pitch = self.fb[2]
        self.base_roll = self.fb[1]
        self.base_shoulder = self.fb[4]
        self.creep = False

    def spin(self, seconds):
        time.sleep(.01)
        self.fb_n += 1
        if self.drift:
            self.fb[3] += 1.0
        if self.glitch_at == self.fb_n:
            self.fb[1], self._roll = 0.0, self.fb[1]
        elif self.glitch_at is not None and self.fb[1] == 0.0:
            self.fb[1] = self._roll
        if self.jitter:
            # Real stationary wrist pitch: 154-157 counts while holding 156.
            self.fb[2] = self.base_pitch + (-1, 1, -2, 0, 1, -1)[self.fb_n % 6]
            # Real stationary wrist roll: 495-499 around 496.
            self.fb[1] = self.base_roll + (0, 0, -1, 1, 0, 3, 0, 0)[self.fb_n % 8]
            # Real loaded shoulder: 610, 612, 610, 611, 614, 610.
            self.fb[4] = self.base_shoulder + (-1, 1, -1, 0, 3, -1)[self.fb_n % 6]
        if self.creep and self.fb_n % 3 == 0:
            self.fb[4] += 1.0


def test_snapshot_records_fresh_measured_pose(tmp_path):
    io = FakeIO()
    def capture(path, host, timeout):
        time.sleep(.08)
        return True
    view = snapshot(io, 'fake', tmp_path / 'raw.jpg', list(io.fb),
                    ScanConfig(stationary_seconds=.03), capture,
                    publish=lambda *_: None)
    assert view['feedback_samples'] >= 3
    assert view['fields'] == pytest.approx(io.fb)


def test_snapshot_accepts_stationary_count_jitter(tmp_path):
    io = FakeIO()
    io.jitter = True
    def capture(path, host, timeout):
        time.sleep(.08)
        return True
    view = snapshot(io, 'fake', tmp_path / 'raw.jpg', list(io.fb),
                    ScanConfig(stationary_seconds=.03), capture,
                    publish=lambda *_: None)
    assert view['feedback_span_counts'][2] == 3
    assert view['fields'][2] == pytest.approx(io.base_pitch, abs=.5)


def test_snapshot_skips_unread_servo_sample(tmp_path):
    io = FakeIO()
    io.glitch_at = 12  # within exposure, after settling
    def capture(path, host, timeout):
        time.sleep(.08)
        return True
    view = snapshot(io, 'fake', tmp_path / 'raw.jpg', list(io.fb),
                    ScanConfig(stationary_seconds=.03), capture,
                    publish=lambda *_: None)
    assert io.fb_n > io.glitch_at
    assert max(view['feedback_span_counts'][1:]) == 0


class SaggingArm:
    """Loaded shoulder reads 4 counts past every command (arm lower, as live:
    605 commanded, 610 read); the base velocity loop
    stalls 3 counts short of a target it has to move to, and stays put when
    the target is within its deadband."""

    def __init__(self):
        self.fb = list(observer()[1])
        self.fb_n = 0
        self.capturing = False
        self.exposure_commands = []

    def publish(self, io, fields):
        command = bounded_fields(fields)
        if self.capturing:
            self.exposure_commands.append(command)
        base, error = self.fb[5], command[5] - self.fb[5]
        if abs(error) > 3.0:
            base = command[5] - 3.0 * np.sign(error)
        self.fb = command[:4] + [command[4] + 4.0, base]

    def spin(self, seconds):
        time.sleep(.002)
        self.fb_n += 1


def test_scan_holds_planned_command_not_sagged_reading(tmp_path):
    io = SaggingArm()
    exposures = []
    def capture(path, host, timeout):
        io.capturing, io.exposure_commands = True, []
        time.sleep(.05)
        io.capturing = False
        exposures.append(io.exposure_commands)
        return True
    reference_fields = list(io.fb)
    poses = plan_scan(observer()[0], reference_fields, box=[.4, .35, .6, .6])
    capture_scan(io, 'fake', [.4, .35, .6, .6], tmp_path / 'scan',
                 ScanConfig(stationary_seconds=.03), capture, log=lambda *_: None,
                 publish=io.publish)
    assert len(exposures) == len(poses)
    for pose, commands in list(zip(poses, exposures))[1:]:
        planned = bounded_fields(to_fields(pose, reference_fields[0], reference_fields[1]))
        assert commands
        # Position servos: the planned command. Base: where it stalled.
        assert all(c[:5] == pytest.approx(planned[:5]) for c in commands)
        stalled = planned[5] - 3.0 * np.sign(planned[5] - reference_fields[5])
        if abs(planned[5] - reference_fields[5]) > 3.0:
            assert all(c[5] == pytest.approx(stalled) for c in commands)


def test_first_view_holds_the_reference_command(tmp_path):
    io = SaggingArm()
    command = list(io.fb)
    command[4] -= 4.0           # the arm reads 4 counts past (lower than) it
    io.publish(io, command)
    exposures = []
    def capture(path, host, timeout):
        io.capturing, io.exposure_commands = True, []
        time.sleep(.05)
        io.capturing = False
        exposures.append(io.exposure_commands)
        return True
    capture_scan(io, 'fake', [.4, .35, .6, .6], tmp_path / 'scan',
                 ScanConfig(stationary_seconds=.03), capture, log=lambda *_: None,
                 publish=io.publish, reference_command=command)
    assert exposures[0] and all(c == pytest.approx(bounded_fields(command)) for c in exposures[0])


def test_failed_scan_returns_to_reference(tmp_path):
    io = SaggingArm()
    commands = []
    publish = lambda arm, fields: (commands.append(bounded_fields(fields)), io.publish(arm, fields))
    shots = []
    def capture(path, host, timeout):
        shots.append(path)
        time.sleep(.05)
        return len(shots) < 2
    reference_fields = list(io.fb)
    with pytest.raises(HeightRefused, match='capture failed'):
        capture_scan(io, 'fake', [.4, .35, .6, .6], tmp_path / 'scan',
                     ScanConfig(stationary_seconds=.03), capture, log=lambda *_: None,
                     publish=publish)
    assert commands[-1] == pytest.approx(bounded_fields(reference_fields))
    session = json.loads((tmp_path / 'scan' / 'session.json').read_text())
    assert session['returned_to_reference'] is True


def test_return_from_sagged_view_is_allowed():
    # Real session reference; the loaded shoulder reading 6 counts low (it
    # hunted 610-614 on another run) puts the tip 2.1 mm under it.
    reference = [238.0, 496.0, 138.0, 143.0, 588.0, 61.0]
    sagged = [238.0, 496.0, 138.0, 143.0, 594.0, 61.0]
    ref_joints = from_fields(reference)
    floor = geom.gripper_tip(ref_joints, False)[2] - .002
    with pytest.raises(HeightRefused, match='floor'):
        validate_path(from_fields(sagged), ref_joints, reference, floor_z=floor,
                      require_margin=False)
    io = SaggingArm()
    io.fb = list(sagged)
    move_to(io, ref_joints, reference, ScanConfig(stationary_seconds=.03),
            publish=io.publish, floor_z=floor, require_margin=False)


def test_blank_startup_frame_is_retried_then_refused(tmp_path):
    import cv2
    grey = np.full((720, 1280, 3), 128, np.uint8)
    scene = np.random.default_rng(0).integers(0, 255, (720, 1280, 3), dtype=np.uint8)
    frames = [grey, scene]
    def capture(path, host, timeout):
        time.sleep(.05)
        cv2.imwrite(path, frames.pop(0) if frames else grey)
        return True
    io = FakeIO()
    snapshot(io, 'fake', tmp_path / 'raw.jpg', list(io.fb),
             ScanConfig(stationary_seconds=.03), capture, publish=lambda *_: None)
    assert cv2.imread(str(tmp_path / 'raw.jpg')).std() > 3
    with pytest.raises(HeightRefused, match='capture failed'):
        snapshot(io, 'fake', tmp_path / 'raw2.jpg', list(io.fb),
                 ScanConfig(stationary_seconds=.03), capture, publish=lambda *_: None)


class HomingArm:
    """Reports 0 (unread) while the firmware homes, then follows commands."""

    def __init__(self, home, homing_spins=8):
        self.fb, self.fb_n, self.home = [0.0] * 6, 0, list(home)
        self.homing_spins, self.commands = homing_spins, []

    def publish(self, io, fields):
        self.commands.append(bounded_fields(fields))
        self.fb = bounded_fields(fields)

    def spin(self, seconds):
        time.sleep(.002)
        self.fb_n += 1
        if self.fb_n == self.homing_spins:
            self.fb = list(self.home)


def test_restore_pose_returns_to_start_after_rehome():
    start = list(observer()[1])
    home = to_fields(ik_open_m(.12, 0., .10, -80), 226, 500)
    arm = HomingArm(home)
    back = restore_pose(arm, start, ScanConfig(stationary_seconds=.03),
                        publish=arm.publish, min_wait=0.)
    assert back == pytest.approx(bounded_fields(start), abs=1e-6)
    assert arm.commands[0] != pytest.approx(bounded_fields(start))   # stepped, not jumped
    steps = [from_fields(c) for c in arm.commands]
    for a, b in zip(steps, steps[1:]):
        assert max(abs(a[k] - b[k]) for k in a) <= ScanConfig().max_step_deg + 1e-6


def test_snapshot_rejects_motion_during_capture(tmp_path):
    io = FakeIO()
    def capture(path, host, timeout):
        io.drift = True
        time.sleep(.2)
        return True
    with pytest.raises(HeightRefused, match='drift') as refused:
        snapshot(io, 'fake', tmp_path / 'raw.jpg', list(io.fb),
                 ScanConfig(stationary_seconds=.03), capture,
                 publish=lambda *_: None)
    diagnostics = refused.value.diagnostics
    assert max(diagnostics['feedback_span_counts'][1:]) > 2
    assert len(diagnostics['feedback_samples']) >= 2


def test_snapshot_rejects_slow_creep_without_spikes(tmp_path):
    io = FakeIO()
    def capture(path, host, timeout):
        io.creep = True
        time.sleep(.12)
        io.creep = False
        return True
    with pytest.raises(HeightRefused, match='changed across exposure'):
        snapshot(io, 'fake', tmp_path / 'raw.jpg', list(io.fb),
                 ScanConfig(stationary_seconds=.03), capture,
                 publish=lambda *_: None)


def test_failed_or_nonfinite_report_has_no_fallback_height():
    with pytest.raises(HeightRefused):
        measurement_target({'ok': False, 'top_z_m': .016})
    with pytest.raises(HeightRefused):
        measurement_target({'schema': 'arm_grasp.height/v1', 'ok': True,
                            'target_point_m': [0.2, 0, float('nan')],
                            'object_height_m': .03, 'support_z_m': -.07, 'top_z_m': -.04})
