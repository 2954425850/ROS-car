"""Image-path tests use rendered scenes, not claims about physical accuracy."""
import json

import cv2
import numpy as np
import pytest

from arm_grasp import geom
from arm_grasp.arm_kin import BASE_SCALE, FACTOR, fk, from_fields, to_fields
from arm_grasp.cam_model import tool_axes
from arm_grasp.height import HeightRefused
from arm_grasp.height_vision import calibration_id, load_session, measure_session
from arm_grasp.servo import ik_open_m


def texture(seed, orange=False):
    rng = np.random.default_rng(seed)
    image = np.full((900, 900, 3), (30, 125, 230) if orange else (170, 180, 185), np.uint8)
    for _ in range(250 if orange else 2000):
        centre = tuple(int(x) for x in rng.integers(5, 895, 2))
        color = tuple(int(x) for x in rng.integers(15, 240, 3))
        radius = int(rng.integers(8, 20) if orange else rng.integers(2, 6))
        cv2.circle(image, centre, radius, color, -1)
    return image


def render(joints, background, top):
    """Ray-cast two known planes with OpenCV's independent distortion inversion."""
    k = geom.K_AI320x180_CHN2
    K = np.array([[k.fx * 4, 0, k.cx * 4], [0, k.fy * 4, k.cy * 4], [0, 0, 1.]])
    yy, xx = np.mgrid[:720, :1280]
    pixels = np.float32(np.column_stack([xx.ravel(), yy.ravel()]))
    normalized = cv2.undistortPoints(pixels[:, None], K, np.array(k.dist)).reshape(-1, 2)
    tip, axis = fk(joints)
    xh, yh, zh = np.array(tool_axes(tip, axis))
    c = np.array(geom.camera_center(joints))
    rays = normalized[:, :1] * geom.SIGMA * xh + normalized[:, 1:] * geom.SIGMA * yh + zh
    def intersect(z):
        t = (z - c[2]) / rays[:, 2]
        return c + t[:, None] * rays
    points = intersect(-.07)
    bg = cv2.remap(background, np.float32((points[:, 0] - .07) / .34 * 899).reshape(720, 1280),
                   np.float32((points[:, 1] + .16) / .32 * 899).reshape(720, 1280),
                   cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(80, 80, 80))
    p = intersect(-.04)
    on_top = ((p[:, 0] >= .202) & (p[:, 0] <= .238)
              & (p[:, 1] >= -.038) & (p[:, 1] <= -.002)).reshape(720, 1280)
    fg = cv2.remap(top, np.float32((p[:, 0] - .202) / .036 * 899).reshape(720, 1280),
                   np.float32((p[:, 1] + .038) / .036 * 899).reshape(720, 1280), cv2.INTER_LINEAR)
    bg[on_top] = fg[on_top]
    return bg


def make_session(tmp_path, textured=True, weak_support=False, reading_bias=None,
                 known_support_z=None):
    poses = [ik_open_m(*p, -75) for p in [( .16, -.02, .04), (.16, .005, .04),
                                          (.16, -.045, .04), (.15, -.02, .06)]]
    bg = texture(10) if textured else np.full((900, 900, 3), 180, np.uint8)
    if weak_support:
        bg = np.uint8(140 + .18 * (bg.astype(float) - 140))
    top = texture(20, True) if textured else np.full((900, 900, 3), (30, 125, 230), np.uint8)
    views = []
    for i, j in enumerate(poses):
        name = 'view-%d.jpg' % i
        cv2.imwrite(str(tmp_path / name), render(j, bg, top))
        fields = to_fields(j, 240, 496)
        if reading_bias:
            # Images come from the true pose; the recorded readback is off.
            base_deg, pitch_deg = reading_bias[i]
            fields[5] += base_deg * BASE_SCALE
            fields[2] += pitch_deg * FACTOR
        views.append({'image': name, 'joints': from_fields(fields), 'fields': fields})
    corners = [geom.to_stream(*geom.project(poses[0], p))
               for p in [(.202, -.038, -.04), (.238, -.038, -.04),
                         (.238, -.002, -.04), (.202, -.002, -.04)]]
    corners = np.array(corners)
    box = [(corners[:, 0].min() - 6) / 1280, (corners[:, 1].min() - 6) / 720,
           (corners[:, 0].max() + 6) / 1280, (corners[:, 1].max() + 6) / 720]
    session = {'schema': 'arm_grasp.height_session/v1', 'coordinate_frame': 'raw_stream',
               'complete': True, 'calibration_id': calibration_id(), 'reference_box': box, 'views': views}
    if known_support_z is not None:
        session['known_support_z_m'] = known_support_z
    path = tmp_path / 'session.json'
    path.write_text(json.dumps(session), encoding='utf-8')
    return path


def test_textured_images_recover_metric_height(tmp_path):
    path = make_session(tmp_path)
    report = measure_session(path)
    assert report['object_height_m'] == pytest.approx(.03, abs=.002)
    assert report['support_z_m'] == pytest.approx(-.07, abs=.002)
    assert report['top_z_m'] == pytest.approx(-.04, abs=.002)
    assert report['quality']['accuracy_verified'] is False
    assert report['quality']['top_inliers'] >= 10


# Readback errors of the size seen on the real arm (base off 3-5 deg).
REAL_BIAS = [(0., 0.), (3.4, -.4), (3.2, 0.), (5.3, -1.3)]


def test_biased_base_readback_is_refused_without_refinement(tmp_path):
    with pytest.raises(HeightRefused, match='tracks'):
        measure_session(make_session(tmp_path, reading_bias=REAL_BIAS), refine=False)


def test_pose_refinement_recovers_height_from_biased_readback(tmp_path):
    report = measure_session(make_session(tmp_path, reading_bias=REAL_BIAS))
    assert report['object_height_m'] == pytest.approx(.03, abs=.002)
    refinement = report['quality']['pose_refinement']
    for offsets, (base_deg, pitch_deg) in zip(refinement['offsets_deg'], REAL_BIAS[1:]):
        assert offsets['base'] == pytest.approx(-base_deg, abs=.3)
        assert offsets['wrist_pitch'] == pytest.approx(-pitch_deg, abs=.3)
    assert refinement['reprojection_px_median_after'] < 1


def test_pose_refinement_refuses_implausible_corrections(tmp_path):
    bias = [(0., 0.), (12., 0.), (-12., 0.), (12., 0.)]
    with pytest.raises(HeightRefused, match='beyond bounds|tracks'):
        measure_session(make_session(tmp_path, reading_bias=bias))


def test_untextured_scene_refuses_instead_of_guessing(tmp_path):
    with pytest.raises(HeightRefused, match='texture|tracks'):
        measure_session(make_session(tmp_path, textured=False))


def test_weak_real_texture_can_supply_support_evidence(tmp_path):
    report = measure_session(make_session(tmp_path, weak_support=True))
    assert report['object_height_m'] == pytest.approx(.03, abs=.002)
    assert report['quality']['support_inliers'] >= 24


def test_changed_calibration_and_upright_frames_are_refused(tmp_path):
    path = make_session(tmp_path)
    session = json.loads(path.read_text())
    session['coordinate_frame'] = 'upright_preview'
    path.write_text(json.dumps(session))
    with pytest.raises(HeightRefused, match='raw'):
        load_session(path)
    session['coordinate_frame'] = 'raw_stream'
    session['calibration_id'] = 'obsolete'
    path.write_text(json.dumps(session))
    with pytest.raises(HeightRefused, match='calibration'):
        load_session(path)


def test_wrong_image_resolution_is_refused(tmp_path):
    path = make_session(tmp_path)
    cv2.imwrite(str(tmp_path / 'view-0.jpg'), np.zeros((180, 320, 3), np.uint8))
    with pytest.raises(HeightRefused, match='dimensions'):
        load_session(path)


# Real sessions: every view's wrist pitch (the reference included) read a few
# degrees off in the same direction, on top of the base errors.
COMMON_PITCH_BIAS = [(b, p - 3.0) for b, p in REAL_BIAS]


def test_known_support_plane_fixes_absolute_top_under_common_pitch_error(tmp_path):
    report = measure_session(make_session(tmp_path, reading_bias=COMMON_PITCH_BIAS,
                                          known_support_z=-.07))
    assert report['top_z_m'] == pytest.approx(-.04, abs=.002)
    assert report['object_height_m'] == pytest.approx(.03, abs=.002)
    assert report['support_z_m'] == pytest.approx(-.07, abs=1e-9)
    assert report['quality']['support_plane_source'] == 'known'
    refinement = report['quality']['pose_refinement']
    assert refinement['common_wrist_pitch_deg'] == pytest.approx(3.0, abs=.5)


def test_common_pitch_error_biases_absolute_top_without_known_plane(tmp_path):
    # Why the known plane exists: the same images measured freely put the
    # whole scene at the wrong depth (the height itself may still look fine).
    try:
        report = measure_session(make_session(tmp_path, reading_bias=COMMON_PITCH_BIAS))
    except HeightRefused:
        return
    assert abs(report['top_z_m'] - (-.04)) > .002
