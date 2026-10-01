"""Metric multiview geometry. No ROS, I/O, assumed table height or object size.

Thresholds reject inconsistent data; they are NOT an accuracy specification.
The first implementation requires a visible, textured, approximately flat top.
"""
from dataclasses import dataclass
import math

import numpy as np


class HeightRefused(ValueError):
    """Insufficient or inconsistent evidence; callers must not substitute a height."""

    def __init__(self, message, diagnostics=None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}


@dataclass(frozen=True)
class HeightConfig:
    min_baseline_m: float = .012
    min_parallax_deg: float = 2.0
    max_ray_rms_m: float = .003
    max_reprojection_px: float = 3.0
    min_range_m: float = .07
    max_range_m: float = .65
    plane_tol_m: float = .0025
    # Support features on wood grain localize poorly along the grain: on the
    # real desk single support points scattered +-5 mm while the plane they
    # define was ~0.7 mm certain. The support consensus band is therefore
    # wider, and the guard is the standard error of the support height at
    # the target instead (precision only; pose/calibration bias not included).
    support_tol_m: float = .006
    max_support_se_m: float = .0015
    # With a known support plane the support points only fix the depth
    # gauge during pose refinement; the plane itself is not fitted.
    min_known_support_points: int = 10
    min_support_points: int = 24
    min_top_points: int = 10
    min_support_fraction: float = .65
    min_top_fraction: float = .45
    min_support_span_m: float = .04
    min_top_span_m: float = .006
    min_object_height_m: float = .004
    max_object_height_m: float = .18
    max_support_tilt_deg: float = 30.0
    max_top_tilt_deg: float = 15.0
    max_split_height_m: float = .006
    max_split_point_m: float = .008
    min_image_coverage: float = .10
    ransac_iterations: int = 400
    seed: int = 42


def _points(value, minimum=1):
    a = np.asarray(value, dtype=float)
    if a.ndim != 2 or a.shape[1] != 3 or len(a) < minimum or not np.isfinite(a).all():
        raise HeightRefused('invalid/insufficient finite 3D points')
    return a


def triangulate(centres, directions, config=None):
    """Least-squares intersection of independently measured metric rays."""
    cfg = config or HeightConfig()
    c, d = _points(centres, 2), _points(directions, 2)
    if c.shape != d.shape:
        raise HeightRefused('ray/centre counts differ')
    norms = np.linalg.norm(d, axis=1)
    if np.any(norms < 1e-9):
        raise HeightRefused('zero ray direction')
    d = d / norms[:, None]
    baseline = float(np.max(np.linalg.norm(c[:, None] - c[None, :], axis=2)))
    if baseline < cfg.min_baseline_m:
        raise HeightRefused('baseline too small (%.1f mm)' % (baseline * 1000))
    angle = math.degrees(math.acos(float(np.clip(np.min(d @ d.T), -1, 1))))
    if angle < cfg.min_parallax_deg:
        raise HeightRefused('parallax too small (%.2f deg)' % angle)
    projectors = np.eye(3)[None] - d[:, :, None] * d[:, None, :]
    a = projectors.sum(axis=0)
    if np.linalg.cond(a) > 1e5:
        raise HeightRefused('ill-conditioned ray intersection')
    p = np.linalg.solve(a, np.einsum('nij,nj->i', projectors, c))
    ranges = np.einsum('ni,ni->n', p - c, d)
    if np.any(ranges <= 0):
        raise HeightRefused('intersection behind camera')
    if np.any(ranges < cfg.min_range_m) or np.any(ranges > cfg.max_range_m):
        raise HeightRefused('intersection outside calibrated working range')
    residuals = np.linalg.norm(np.einsum('nij,nj->ni', projectors, p - c), axis=1)
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    if rms > cfg.max_ray_rms_m or np.max(residuals) > 2 * cfg.max_ray_rms_m:
        raise HeightRefused('ray residual too large (%.1f mm)' % (rms * 1000))
    return {'point_m': p.tolist(), 'baseline_m': baseline,
            'parallax_deg': angle, 'ray_rms_m': rms}


def _svd_plane(points):
    centre = points.mean(axis=0)
    _, singular, vh = np.linalg.svd(points - centre, full_matrices=False)
    if len(singular) < 2 or singular[1] < 1e-5:
        raise HeightRefused('plane points are collinear')
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    return normal, -float(normal @ centre)


def fit_plane(points, config=None, normal_hint=None, support_plane=None, tol=None):
    """RANSAC plane, constrained by upward normal / a measured support plane."""
    cfg = config or HeightConfig()
    top = normal_hint is not None
    minimum = cfg.min_top_points if top else cfg.min_support_points
    p = _points(points, minimum)
    hint = np.array(normal_hint if top else [0., 0., 1.])
    cos_limit = math.cos(math.radians(cfg.max_top_tilt_deg if top
                                     else cfg.max_support_tilt_deg))
    tol = cfg.plane_tol_m if tol is None else tol
    rng, best = np.random.default_rng(cfg.seed), None
    for _ in range(cfg.ransac_iterations):
        q = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(q[1] - q[0], q[2] - q[0])
        size = np.linalg.norm(n)
        if size < 1e-10:
            continue
        n /= size
        if n[2] < 0:
            n = -n
        if n @ hint < cos_limit:
            continue
        offset = -float(n @ q[0])
        mask = np.abs(p @ n + offset) <= tol
        if support_plane is not None:
            sn, sd = support_plane
            height = float(sn @ np.median(p[mask], axis=0) + sd)
            if not (cfg.min_object_height_m <= height <= cfg.max_object_height_m):
                continue
        count = int(mask.sum())
        if best is None or count > best[0]:
            best = (count, mask)
    fraction = cfg.min_top_fraction if top else cfg.min_support_fraction
    label = 'top' if top else 'support'
    if best is None or best[0] < max(minimum, math.ceil(len(p) * fraction)):
        raise HeightRefused('%s plane lacks coherent evidence' % label)
    mask = best[1]
    for _ in range(2):
        n, offset = _svd_plane(p[mask])
        mask = np.abs(p @ n + offset) <= tol
    if n @ hint < cos_limit or mask.sum() < max(minimum, math.ceil(len(p) * fraction)):
        raise HeightRefused('%s plane unstable after refinement' % label)
    inliers = p[mask]
    span = np.ptp(inliers[:, :2], axis=0)
    min_span = cfg.min_top_span_m if top else cfg.min_support_span_m
    if np.min(span) < min_span:
        raise HeightRefused('%s plane spatial coverage too small' % label)
    # Require two-dimensional coverage, not a narrow diagonal line.
    singular = np.linalg.svd(inliers[:, :2] - inliers[:, :2].mean(0), compute_uv=False)
    if singular[-1] / math.sqrt(len(inliers)) < min_span / 6:
        raise HeightRefused('%s plane coverage is nearly collinear' % label)
    residual = inliers @ n + offset
    return {'normal': n, 'offset_m': offset, 'mask': mask,
            'rms_m': float(np.sqrt(np.mean(residual ** 2)))}


def _hull(points):
    """2D monotone-chain hull, avoiding an OpenCV dependency in geometry."""
    p = sorted(set(map(tuple, points)))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for q in p:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    for q in reversed(p):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return lower[:-1] + upper[:-1]


def _inside_hull(points, query):
    hull = _hull(points)
    if len(hull) < 3:
        return False
    return all((b[0]-a[0])*(query[1]-a[1]) - (b[1]-a[1])*(query[0]-a[0]) >= -1e-9
               for a, b in zip(hull, hull[1:] + hull[:1]))


def _plane_height_se(points, normal, offset, xy):
    """Standard error of a fitted plane's height at xy, from its own residuals."""
    residual = points @ normal + offset
    a = np.column_stack([points[:, :2], np.ones(len(points))])
    sigma2 = float(residual @ residual) / max(1, len(points) - 3)
    x0 = np.array([xy[0], xy[1], 1.0])
    return float(math.sqrt(sigma2 * float(x0 @ np.linalg.solve(a.T @ a, x0))) / abs(normal[2]))


def measure_cloud(support, target, config=None, known_support_z=None):
    """Measure a supported flat top from foreground and local background points.

    known_support_z: operator-measured horizontal support height (m, mount
    frame). The plane is then taken as given instead of fitted.
    """
    cfg = config or HeightConfig()
    known = known_support_z is not None
    s = _points(support, cfg.min_known_support_points if known else cfg.min_support_points)
    t = _points(target, cfg.min_top_points)
    if known:
        n, offset = np.array([0., 0., 1.]), -float(known_support_z)
        residual = s @ n + offset
        plane = {'normal': n, 'offset_m': offset, 'mask': np.ones(len(s), bool),
                 'rms_m': float(np.sqrt(np.mean(residual ** 2)))}
    else:
        plane = fit_plane(s, cfg, tol=cfg.support_tol_m)
        n, offset = plane['normal'], plane['offset_m']
    top = fit_plane(t, cfg, n, (n, offset))
    top_points = t[top['mask']]
    point = np.median(top_points, axis=0)
    point[2] = -(top['offset_m'] + top['normal'][:2] @ point[:2]) / top['normal'][2]
    # Reject a second coherent elevated layer: no justified choice of physical top.
    other = t[~top['mask']]
    if len(other) >= cfg.min_top_points:
        try:
            fit_plane(other, cfg, n, (n, offset))
        except HeightRefused:
            pass
        else:
            raise HeightRefused('top ambiguous: multiple elevated surfaces')
    if not known and not _inside_hull(s[plane['mask'], :2], point[:2]):
        raise HeightRefused('support coverage does not surround target')
    support_z = -float(offset + n[:2] @ point[:2]) / n[2]
    support_se = 0.0 if known else _plane_height_se(s[plane['mask']], n, offset, point[:2])
    if support_se > cfg.max_support_se_m:
        raise HeightRefused('support plane too uncertain at target (%.1f mm standard error)'
                            % (support_se * 1000))
    height = float(n @ point + offset)
    if not cfg.min_object_height_m <= height <= cfg.max_object_height_m:
        raise HeightRefused('top height outside measurable range')
    return {'schema': 'arm_grasp.height/v1', 'ok': True,
            'support_plane': {'normal': n.tolist(), 'offset_m': float(offset)},
            'top_plane': {'normal': top['normal'].tolist(), 'offset_m': top['offset_m']},
            'support_z_m': support_z, 'top_z_m': float(point[2]),
            'object_height_m': height, 'vertical_height_m': float(point[2] - support_z),
            'target_point_m': point.tolist(), 'target_definition': 'visible_top_feature_centroid',
            'quality': {'support_inliers': int(plane['mask'].sum()),
                        'top_inliers': int(top['mask'].sum()),
                        'support_rms_m': plane['rms_m'], 'top_rms_m': top['rms_m'],
                        'support_z_se_m': support_se,
                        'support_plane_source': 'known' if known else 'measured',
                        'accuracy_verified': False,
                        'meaning': 'geometric consistency; pose/calibration bias is unbounded'}}


def measurement_target(report):
    """Consume only successful, finite measurement reports; no height defaults."""
    if report.get('schema') != 'arm_grasp.height/v1' or report.get('ok') is not True:
        raise HeightRefused('height measurement failed')
    try:
        point = tuple(float(x) for x in report['target_point_m'])
        height = float(report['object_height_m'])
        support = float(report['support_z_m'])
        top = float(report['top_z_m'])
    except (KeyError, TypeError, ValueError) as e:
        raise HeightRefused('incomplete height report') from e
    if (len(point) != 3 or not all(math.isfinite(x) for x in point + (height, support, top))
            or height <= 0 or top <= support or abs(point[2] - top) > 1e-6):
        raise HeightRefused('invalid height report')
    return point
