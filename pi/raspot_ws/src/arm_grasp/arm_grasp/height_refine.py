"""Image-based refinement of measured scan poses (small bundle adjustment).

Why: the base is a motor-mode servo with a friction-stalling velocity loop;
on the real arm the images disagreed with the recorded base angles by
3-5 deg, and no feature triangulated (reprojection 24 px median). Fitting
four joint offsets per moved view brought the same data to 0.5 px.

Only joint offsets are estimated, with the reference view held fixed, so the
metric scale still comes from the kinematic chain (the camera centres move
with the corrected joints). Points are re-triangulated from the rays at every
evaluation, which keeps the problem at 4 x (views - 1) unknowns. Corrections
are bounded and reported; a refinement that needs more is refused, never
silently accepted. A good fit is geometric consistency, not verified accuracy.
"""
from dataclasses import dataclass

import numpy as np

from . import geom
from .arm_kin import fk
from .cam_model import tool_axes
from .height import HeightRefused

KEYS = ('base', 'shoulder', 'elbow', 'wrist_pitch')


@dataclass(frozen=True)
class RefineConfig:
    # Base + wrist pitch: on real data this fit as well as all four joints
    # (0.72 vs 0.70 px) without the near-degenerate shoulder/elbow pair
    # running to +-12 deg. The base is the unreliable joint; wrist pitch
    # absorbs residual tool-pitch error.
    free: tuple = ('base', 'wrist_pitch')
    prior_deg: tuple = (4.0, 1.5, 1.5, 1.5)     # weak priors, per KEYS
    max_offset_deg: tuple = (8.0, 3.0, 3.0, 3.0)
    loss_scale_px: float = 2.0
    loss_schedule_px: tuple = (32.0, 8.0, 2.0)
    min_tracks: int = 12

    def per_free(self, values):
        return np.array([values[KEYS.index(k)] for k in self.free], dtype=float)


def undistorted_normalized(pixels):
    """Stream pixels (N, 2) -> undistorted normalized camera coordinates."""
    k = geom.K_AI320x180_CHN2
    k1, k2, p1, p2, k3 = k.dist
    p = np.asarray(pixels, dtype=float)
    x0 = (p[:, 0] * k.w / geom.STREAM_W - k.cx) / k.fx
    y0 = (p[:, 1] * k.h / geom.STREAM_H - k.cy) / k.fy
    x, y = x0.copy(), y0.copy()
    for _ in range(30):
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        x = (x0 - (2 * p1 * x * y + p2 * (r2 + 2 * x * x))) / rad
        y = (y0 - (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)) / rad
    return np.column_stack([x, y])


def camera_frame(joints):
    tip, axis = fk(joints)
    xh, yh, zh = (np.array(a) for a in tool_axes(tip, axis))
    return np.array(geom.camera_center(joints)), xh, yh, zh


def project_stream(joints, points):
    """Base-frame points (N, 3) -> distorted stream pixels (N, 2), vectorized."""
    k = geom.K_AI320x180_CHN2
    k1, k2, p1, p2, k3 = k.dist
    c, xh, yh, zh = camera_frame(joints)
    d = np.asarray(points, dtype=float) - c
    z = d @ zh
    z = np.where(z > 1e-6, z, np.nan)
    x = geom.SIGMA * (d @ xh) / z
    y = geom.SIGMA * (d @ yh) / z
    r2 = x * x + y * y
    rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    u = k.cx + k.fx * (x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x))
    v = k.cy + k.fy * (y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)
    return np.column_stack([u * geom.STREAM_W / k.w, v * geom.STREAM_H / k.h])


def offset_joints(joints, offsets, keys=KEYS):
    return dict(joints, **{key: joints[key] + float(o) for key, o in zip(keys, offsets)})


class _Problem:
    def __init__(self, views, observations, keys=KEYS):
        self.keys = tuple(keys)
        # observations: [(track, view, (u, v)), ...] in stream pixels
        self.joints = [dict(v['joints']) for v in views]
        obs = np.array([(t, j) for t, j, _ in observations], dtype=int)
        self.track, self.view = obs[:, 0], obs[:, 1]
        self.pixels = np.array([p for _, _, p in observations], dtype=float)
        self.normalized = undistorted_normalized(self.pixels)
        self.n_tracks = int(self.track.max()) + 1

    def views_for(self, x):
        out = [self.joints[0]]
        n = len(self.keys)
        for j in range(1, len(self.joints)):
            out.append(offset_joints(self.joints[j], x[n * (j - 1):n * j], self.keys))
        return out

    def points(self, joints):
        centres = np.zeros((len(self.track), 3))
        dirs = np.zeros((len(self.track), 3))
        for j, pose in enumerate(joints):
            sel = self.view == j
            c, xh, yh, zh = camera_frame(pose)
            n = self.normalized[sel]
            d = (geom.SIGMA * n[:, :1] * xh + geom.SIGMA * n[:, 1:] * yh + zh)
            dirs[sel] = d / np.linalg.norm(d, axis=1, keepdims=True)
            centres[sel] = c
        proj = np.eye(3)[None] - dirs[:, :, None] * dirs[:, None, :]
        a = np.zeros((self.n_tracks, 3, 3))
        b = np.zeros((self.n_tracks, 3))
        np.add.at(a, self.track, proj)
        np.add.at(b, self.track, np.einsum('nij,nj->ni', proj, centres))
        return np.linalg.solve(a + 1e-12 * np.eye(3)[None], b[:, :, None])[:, :, 0]

    def reprojection(self, joints):
        pts = self.points(joints)
        err = np.zeros((len(self.track), 2))
        for j, pose in enumerate(joints):
            sel = self.view == j
            err[sel] = project_stream(pose, pts[self.track[sel]]) - self.pixels[sel]
        return np.nan_to_num(err, nan=1e3)


def refine_poses(views, observations, config=None):
    """Return (corrected views, diagnostics). Refuses implausible corrections."""
    from scipy.optimize import least_squares
    cfg = config or RefineConfig()
    if not observations or len({t for t, _, _ in observations}) < cfg.min_tracks:
        raise HeightRefused('too few multiview tracks for pose refinement')
    problem = _Problem(views, observations, cfg.free)
    n = len(cfg.free)
    prior = np.tile(cfg.per_free(cfg.prior_deg), len(views) - 1)

    def residuals(x, f):
        # Cauchy loss on reprojection only. Points are re-triangulated by
        # plain least squares inside, so one mismatched observation spoils
        # its whole track; a convex loss (soft-L1) let 12 such tracks drag a
        # perfect synthetic fit from 0.06 to 3.2 px. A redescending loss caps
        # their pull. Priors stay quadratic (with the loss on them too, a
        # shoulder/elbow pair ran to -27/+41 deg on real data).
        r = problem.reprojection(problem.views_for(x)).ravel()
        robust = np.sign(r) * f * np.sqrt(np.log1p((r / f) ** 2))
        return np.concatenate([robust, cfg.loss_scale_px * x / prior])

    x = np.zeros(n * (len(views) - 1))
    before = np.hypot(*problem.reprojection(problem.views_for(x)).T)
    # Coarse to fine: real poses start ~24 px off, inside the 32 px basin.
    for f in cfg.loss_schedule_px:
        x = least_squares(residuals, x, args=(f,), diff_step=1e-4, max_nfev=100).x
    sol_x = x
    after = np.hypot(*problem.reprojection(problem.views_for(sol_x)).T)
    offsets = sol_x.reshape(-1, n)
    diagnostics = {'offsets_deg': [dict(zip(cfg.free, map(float, o))) for o in offsets],
                   'reprojection_px_median_before': float(np.median(before)),
                   'reprojection_px_median_after': float(np.median(after)),
                   'reprojection_px_p90_after': float(np.percentile(after, 90)),
                   'tracks': problem.n_tracks, 'observations': len(problem.track)}
    limit = cfg.per_free(cfg.max_offset_deg)
    if np.any(np.abs(offsets) > limit):
        raise HeightRefused('pose refinement needs corrections beyond bounds', diagnostics)
    corrected = [dict(views[0])]
    for view, pose in zip(views[1:], problem.views_for(sol_x)[1:]):
        corrected.append(dict(view, joints=pose))
    return corrected, diagnostics
