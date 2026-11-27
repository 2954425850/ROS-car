from types import SimpleNamespace

from carhealth.probes.nav2_overlay import Nav2OverlayProbe


def _runner(stdout, returncode=0, stderr=""):
    def run(*args, **kwargs):
        return SimpleNamespace(stdout=stdout, stderr=stderr, returncode=returncode)
    return run


def test_overlay_prefix_passes():
    p = Nav2OverlayProbe(prefix="/home/cy/nav2_ws/install",
                         runner=_runner("/home/cy/nav2_ws/install/nav2_smac_planner\n"))
    ev = p.poll().evidence
    assert ev["is_overlay"] is True
    assert ev["prefix"].endswith("nav2_smac_planner")


def test_system_prefix_fails():
    p = Nav2OverlayProbe(prefix="/home/cy/nav2_ws/install",
                         runner=_runner("/opt/ros/jazzy\n"))
    assert p.poll().evidence["is_overlay"] is False


def test_prefix_that_merely_starts_with_same_letters_fails():
    p = Nav2OverlayProbe(prefix="/home/cy/nav2_ws/install",
                         runner=_runner("/home/cy/nav2_ws/install_backup/nav2_smac_planner\n"))
    assert p.poll().evidence["is_overlay"] is False


def test_probe_error_is_not_a_pass():
    def boom(*args, **kwargs):
        raise OSError("no bash")

    p = Nav2OverlayProbe(prefix="/home/cy/nav2_ws/install", runner=boom)
    ev = p.poll().evidence
    assert ev["is_overlay"] is False
    assert ev["error"]
