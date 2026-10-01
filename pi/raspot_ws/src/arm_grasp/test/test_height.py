import numpy as np
import pytest

from arm_grasp.height import HeightConfig, HeightRefused, measure_cloud, triangulate


def test_independent_metric_rays_recover_point():
    point = np.array([0.21, -0.025, -0.09])
    centres = np.array([[0.12, -.04, .13], [.15, .01, .15], [.10, .03, .14]])
    directions = point - centres
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    result = triangulate(centres, directions)
    assert result['point_m'] == pytest.approx(point, abs=1e-10)
    assert result['ray_rms_m'] < 1e-10


def test_pure_rotation_and_tiny_baseline_are_refused():
    with pytest.raises(HeightRefused, match='baseline'):
        triangulate([[0, 0, 0]] * 3, [[0, 0, 1], [.1, 0, 1], [0, .1, 1]])
    with pytest.raises(HeightRefused, match='parallax'):
        triangulate([[0, 0, 0], [.02, 0, 0]], [[0, 0, 1]] * 2)


def test_inconsistent_rays_and_behind_camera_are_refused():
    with pytest.raises(HeightRefused, match='residual'):
        triangulate([[0, 0, 0], [.05, 0, 0], [0, .05, 0]],
                    [[0, 0, 1], [-.2, 0, 1], [.4, -.2, 1]])
    with pytest.raises(HeightRefused, match='behind'):
        triangulate([[0, 0, 0], [.05, 0, 0]], [[0, 0, -1], [.2, 0, -1]])


def clouds(tilt=0.0, height=.03, seed=42, support_noise=.0003, n_support=180):
    rng = np.random.default_rng(seed)
    xy = rng.uniform([.14, -.07], [.28, .07], (n_support, 2))
    xy = xy[(np.abs(xy[:, 0] - .21) > .025) | (np.abs(xy[:, 1]) > .025)]
    n = np.array([-tilt, 0, 1.0])
    n /= np.linalg.norm(n)
    z = -.12 + tilt * (xy[:, 0] - .21) + rng.normal(0, support_noise, len(xy))
    support = np.column_stack([xy, z])
    top_xy = rng.uniform([.195, -.015], [.225, .015], (45, 2))
    top_z = -.12 + tilt * (top_xy[:, 0] - .21) + height / n[2]
    top_z += rng.normal(0, .0003, len(top_xy))
    return support, np.column_stack([top_xy, top_z])


@pytest.mark.parametrize('tilt', [0.0, .15])
def test_plane_and_normal_object_height_with_outliers(tilt):
    support, top = clouds(tilt)
    outliers = np.array([[.15, .05, .07], [.27, -.03, -.04], [.26, .02, .02]])
    report = measure_cloud(np.vstack([support, outliers]), np.vstack([top, outliers]))
    assert report['object_height_m'] == pytest.approx(.03, abs=.001)
    assert report['support_z_m'] == pytest.approx(-.12, abs=.002)
    assert report['target_point_m'][2] > report['support_z_m']
    assert report['quality']['accuracy_verified'] is False


def test_height_is_not_inferred_without_top_evidence():
    support, _ = clouds()
    with pytest.raises(HeightRefused, match='top'):
        measure_cloud(support, support[:15])


def test_support_must_surround_target():
    support, top = clouds()
    with pytest.raises(HeightRefused, match='coverage'):
        measure_cloud(support[support[:, 0] < .18], top)


def test_scattered_target_points_do_not_become_a_top_plane():
    support, top = clouds()
    top[:, 2] += np.linspace(-.015, .035, len(top))
    with pytest.raises(HeightRefused, match='top'):
        measure_cloud(support, top, HeightConfig(min_top_fraction=.65))


def test_noisy_wood_grain_support_still_defines_the_plane():
    # Real desk: ~40 support points scattering +-5 mm, a precise top.
    support, top = clouds(support_noise=.003, n_support=55)
    report = measure_cloud(support, top)
    assert report['object_height_m'] == pytest.approx(.03, abs=.0015)
    assert report['quality']['support_z_se_m'] < .0015


def test_support_height_uncertainty_is_a_refusal():
    support, top = clouds(support_noise=.003, n_support=55)
    with pytest.raises(HeightRefused, match='uncertain'):
        measure_cloud(support, top, HeightConfig(max_support_se_m=.0003))
