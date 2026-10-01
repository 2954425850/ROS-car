"""Raw-image feature reconstruction and replayable scene measurement."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from . import geom, arm_kin
from .height import HeightConfig, HeightRefused, measure_cloud, triangulate


def calibration_record():
    k = geom.K_AI320x180_CHN2
    return {'intrinsics': list(k[:4]) + [list(k.dist), k.w, k.h],
            'cam_closed_m': list(geom.CAM_CLOSED), 'sigma': geom.SIGMA,
            'arm_lengths_cm': [arm_kin.L1, arm_kin.L2, arm_kin.L3, arm_kin.L4],
            'stream_size': [geom.STREAM_W, geom.STREAM_H],
            'coordinate_frame': 'raw_stream', 'wrist_roll_field': 496.0}


def calibration_id():
    raw = json.dumps(calibration_record(), sort_keys=True).encode('utf-8')
    return hashlib.sha256(raw).hexdigest()


def load_session(path):
    path = Path(path).resolve()
    with path.open(encoding='utf-8') as f:
        session = json.load(f)
    if session.get('schema') != 'arm_grasp.height_session/v1':
        raise HeightRefused('unknown session schema')
    if session.get('complete') is not True:
        raise HeightRefused('incomplete acquisition session')
    if session.get('coordinate_frame') != 'raw_stream':
        raise HeightRefused('only raw calibrated stream images are supported')
    if session.get('calibration_id') != calibration_id():
        raise HeightRefused('session calibration differs from current geometry')
    box = np.asarray(session.get('reference_box'), dtype=float)
    if (box.shape != (4,) or not np.isfinite(box).all() or np.any(box < 0)
            or np.any(box > 1) or box[0] >= box[2] or box[1] >= box[3]):
        raise HeightRefused('invalid reference target box')
    views = session.get('views', [])
    if len(views) < 3:
        raise HeightRefused('at least three stationary views are required')
    images = []
    for view in views:
        joints = view.get('joints', {})
        if any(not np.isfinite(joints.get(key, np.nan))
               for key in ('base', 'shoulder', 'elbow', 'wrist_pitch')):
            raise HeightRefused('missing finite measured joint pose')
        fields = np.asarray(view.get('fields'), dtype=float)
        if (fields.shape != (6,) or not np.isfinite(fields).all()
                or np.any(fields[2:5] == 0) or abs(fields[1] - 496) > 8):
            raise HeightRefused('invalid feedback or uncalibrated wrist roll')
        decoded = arm_kin.from_fields(fields)
        if any(abs(decoded[key] - joints[key]) > 1e-6 for key in decoded):
            raise HeightRefused('joint pose does not match recorded feedback')
        image = cv2.imread(str(path.parent / view['image']))
        if image is None:
            raise HeightRefused('cannot read image: %s' % view['image'])
        if image.shape[:2] != (geom.STREAM_H, geom.STREAM_W):
            raise HeightRefused('image dimensions differ from calibrated stream')
        images.append(image)
    return session, images


def region_masks(image, box):
    """Conservative object interior and its surrounding support ring."""
    h, w = image.shape[:2]
    x0, y0, x1, y1 = np.rint(np.array(box) * [w, h, w, h]).astype(int)
    if x0 < 8 or y0 < 8 or x1 > w - 8 or y1 > int(h * .86):
        raise HeightRefused('target clipped or overlaps unmodelled gripper image region')
    bw, bh = x1 - x0, y1 - y0
    if min(bw, bh) < 24:
        raise HeightRefused('target too small for reliable feature coverage')
    mask = np.zeros((h, w), np.uint8)
    try:
        cv2.grabCut(image, mask, (x0, y0, bw, bh), np.zeros((1, 65)),
                    np.zeros((1, 65)), 4, cv2.GC_INIT_WITH_RECT)
    except cv2.error as e:
        raise HeightRefused('object segmentation failed') from e
    foreground = np.uint8((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD)) * 255
    margin = max(2, int(min(bw, bh) * .04))
    foreground = cv2.erode(foreground, np.ones((2 * margin + 1,) * 2, np.uint8))
    if np.count_nonzero(foreground) < bw * bh * .08:
        raise HeightRefused('object segmentation insufficient; no height guessed')
    support = np.zeros((h, w), np.uint8)
    support[max(8, y0 - bh):min(int(h * .86), y1 + bh),
            max(8, x0 - bw):min(w - 8, x1 + bw)] = 255
    support[max(0, y0 - margin * 2):y1 + margin * 2,
            max(0, x0 - margin * 2):x1 + margin * 2] = 0
    return foreground, support


def mutual_matches(desc_ref, desc_other, ratio=.7):
    if desc_ref is None or desc_other is None or min(len(desc_ref), len(desc_other)) < 2:
        return {}
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    def candidates(a, b):
        return {pair[0].queryIdx: pair[0].trainIdx
                for pair in matcher.knnMatch(a, b, k=2)
                if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance}
    forward, backward = candidates(desc_ref, desc_other), candidates(desc_other, desc_ref)
    return {i: j for i, j in forward.items() if backward.get(j) == i}


def reconstruct_tracks(images, views, box, config=None):
    cfg = config or HeightConfig()
    cv2.setRNGSeed(cfg.seed)
    foreground, support_mask = region_masks(images[0], box)
    mask = cv2.bitwise_or(foreground, support_mask)
    sift = cv2.SIFT_create(nfeatures=3500, contrastThreshold=.025)
    keypoints, descriptors = [], []
    for i, image in enumerate(images):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        kp, desc = sift.detectAndCompute(gray, mask if i == 0 else None)
        keypoints.append(kp)
        descriptors.append(desc)
    if descriptors[0] is None:
        raise HeightRefused('insufficient texture in object/support region')
    mappings = [mutual_matches(descriptors[0], other) for other in descriptors[1:]]
    tracks, rejected, used = [], 0, set()
    for index, kp in enumerate(keypoints[0]):
        cell = tuple(int(q // 4) for q in kp.pt)
        if cell in used:
            continue  # SIFT may assign several orientations to one physical corner.
        visible = [(0, kp.pt)] + [(j + 1, keypoints[j + 1][mapping[index]].pt)
                                 for j, mapping in enumerate(mappings) if index in mapping]
        if len(visible) < 3:
            continue
        centres, directions = [], []
        for j, pixel in visible:
            c, d = geom.pixel_ray(views[j]['joints'], *geom.to_ai(*pixel))
            centres.append(c)
            directions.append(d)
        try:
            result = triangulate(centres, directions, cfg)
            errors = []
            for j, pixel in visible:
                predicted = geom.to_stream(*geom.project(views[j]['joints'], result['point_m']))
                errors.append(float(np.linalg.norm(np.array(predicted) - pixel)))
            if max(errors) > cfg.max_reprojection_px:
                raise HeightRefused('reprojection error')
            # Each pair is a separate depth estimate, sharing only the reference ray.
            split_a = triangulate([centres[0], centres[1]], [directions[0], directions[1]], cfg)
            split_b = triangulate([centres[0], centres[-1]], [directions[0], directions[-1]], cfg)
            if np.linalg.norm(np.array(split_a['point_m']) - split_b['point_m']) > cfg.max_split_point_m:
                raise HeightRefused('split-view point disagreement')
        except (HeightRefused, ValueError):
            rejected += 1
            continue
        u, v = map(lambda q: int(round(q)), kp.pt)
        kind = 'target' if foreground[v, u] else 'support'
        used.add(cell)
        tracks.append({'kind': kind, 'pixel': list(kp.pt), 'point_m': result['point_m'],
                       'split_a': split_a['point_m'], 'split_b': split_b['point_m'],
                       'views': [j for j, _ in visible], 'reprojection_px': max(errors),
                       'parallax_deg': result['parallax_deg'], 'ray_rms_m': result['ray_rms_m']})
    return tracks, {'reference_features': len(keypoints[0]),
                    'pair_matches': [len(m) for m in mappings],
                    'rejected_geometric_tracks': rejected}


def _measure_tracks(tracks, cfg, field='point_m'):
    support = [t[field] for t in tracks if t['kind'] == 'support']
    target = [t[field] for t in tracks if t['kind'] == 'target']
    if len(support) < cfg.min_support_points:
        raise HeightRefused('support has too few reliable multiview tracks (%d)' % len(support))
    if len(target) < cfg.min_top_points:
        raise HeightRefused('top has too few reliable multiview tracks (%d)' % len(target))
    report = measure_cloud(support, target, cfg)
    # Validate top coverage against the measured top model, not every foreground feature.
    n = np.array(report['top_plane']['normal'])
    offset = report['top_plane']['offset_m']
    top_pixels = np.float32([t['pixel'] for t in tracks if t['kind'] == 'target'
                            and abs(n @ t[field] + offset) <= cfg.plane_tol_m])
    return report, top_pixels


def measure_session(path, config=None):
    cfg, start = config or HeightConfig(), time.monotonic()
    session, images = load_session(path)
    tracks, counts = reconstruct_tracks(images, session['views'], session['reference_box'], cfg)
    try:
        return _finish_measurement(path, cfg, start, session, tracks, counts)
    except HeightRefused as e:
        e.diagnostics.update(counts, target_tracks=sum(t['kind'] == 'target' for t in tracks),
                             support_tracks=sum(t['kind'] == 'support' for t in tracks),
                             compute_seconds=time.monotonic() - start,
                             calibration_id=calibration_id(), config=asdict(cfg))
        raise


def _finish_measurement(path, cfg, start, session, tracks, counts):
    report, top_pixels = _measure_tracks(tracks, cfg)
    area = cv2.contourArea(cv2.convexHull(top_pixels))
    box = session['reference_box']
    box_area = (box[2] - box[0]) * geom.STREAM_W * (box[3] - box[1]) * geom.STREAM_H
    coverage = float(area / box_area)
    if coverage < cfg.min_image_coverage:
        raise HeightRefused('top feature image coverage insufficient (%.1f%%)' % (coverage * 100))
    a, _ = _measure_tracks(tracks, cfg, 'split_a')
    b, _ = _measure_tracks(tracks, cfg, 'split_b')
    height_diff = abs(a['object_height_m'] - b['object_height_m'])
    top_diff = abs(a['top_z_m'] - b['top_z_m'])
    support_diff = abs(a['support_z_m'] - b['support_z_m'])
    if max(height_diff, top_diff, support_diff) > cfg.max_split_height_m:
        raise HeightRefused('split-view height/plane disagreement exceeds %.1f mm'
                            % (cfg.max_split_height_m * 1000))
    report['quality'].update(counts, top_image_coverage=coverage,
                             split_height_difference_m=height_diff,
                             split_top_difference_m=top_diff,
                             split_support_difference_m=support_diff,
                             reconstructed_tracks=len(tracks))
    report.update(calibration_id=calibration_id(), calibration=calibration_record(),
                  config=asdict(cfg), session=str(Path(path).resolve()),
                  library_versions={'opencv': cv2.__version__, 'numpy': np.__version__},
                  reference_box=box, compute_seconds=time.monotonic() - start,
                  acquisition_seconds=session.get('acquisition_seconds'),
                  tracks=tracks)
    return report


def save_diagnostic(path, report, output):
    """Render the evidence in raw coordinates; labels avoid accuracy claims."""
    session, images = load_session(path)
    image = images[0].copy()
    for track in report.get('tracks', []):
        color = (0, 230, 0) if track['kind'] == 'support' else (0, 150, 255)
        cv2.circle(image, tuple(int(round(x)) for x in track['pixel']), 3, color, -1)
    box = session['reference_box']
    p = np.rint(np.array(box) * [geom.STREAM_W, geom.STREAM_H] * 2).astype(int)
    cv2.rectangle(image, tuple(p[:2]), tuple(p[2:]), (255, 180, 0), 2)
    cv2.putText(image, 'height %.1f mm | geometric checks only' % (report['object_height_m'] * 1000),
                (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 255), 2)
    if not cv2.imwrite(str(output), image):
        raise OSError('cannot save diagnostic image')
