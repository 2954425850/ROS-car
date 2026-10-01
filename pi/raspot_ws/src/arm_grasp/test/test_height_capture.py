import time

import numpy as np
import pytest

from arm_grasp import geom
from arm_grasp.arm_kin import to_fields
from arm_grasp.height import HeightRefused, measurement_target
from arm_grasp.height_capture import ScanConfig, plan_scan, snapshot, validate_path
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


class FakeIO:
    def __init__(self):
        self.fb = observer()[1]
        self.fb_n = 0
        self.drift = False

    def spin(self, seconds):
        time.sleep(.01)
        self.fb_n += 1
        if self.drift:
            self.fb[3] += 1.0


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


def test_snapshot_rejects_motion_during_capture(tmp_path):
    io = FakeIO()
    def capture(path, host, timeout):
        io.drift = True
        time.sleep(.08)
        return True
    with pytest.raises(HeightRefused, match='drift'):
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
