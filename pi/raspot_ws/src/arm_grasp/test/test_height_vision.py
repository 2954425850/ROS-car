"""Image-path tests use rendered scenes, not claims about physical accuracy."""
import json

import cv2
import numpy as np
import pytest

from arm_grasp import geom
from arm_grasp.arm_kin import fk, to_fields
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


def make_session(tmp_path, textured=True, weak_support=False):
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
        views.append({'image': name, 'joints': j, 'fields': to_fields(j, 240, 496)})
    corners = [geom.to_stream(*geom.project(poses[0], p))
               for p in [(.202, -.038, -.04), (.238, -.038, -.04),
                         (.238, -.002, -.04), (.202, -.002, -.04)]]
    corners = np.array(corners)
    box = [(corners[:, 0].min() - 6) / 1280, (corners[:, 1].min() - 6) / 720,
           (corners[:, 0].max() + 6) / 1280, (corners[:, 1].max() + 6) / 720]
    session = {'schema': 'arm_grasp.height_session/v1', 'coordinate_frame': 'raw_stream',
               'complete': True, 'calibration_id': calibration_id(), 'reference_box': box, 'views': views}
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
