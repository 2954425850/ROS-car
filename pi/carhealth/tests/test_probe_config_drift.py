import os
import subprocess

import pytest

from carhealth.probes.config_drift import (
    ConfigDriftProbe, build_stale_packages, parse_ordering_cycles,
)


def test_parse_ordering_cycles():
    text = "systemd[1]: Job foo.service/start deleted to break ordering cycle\n"
    assert parse_ordering_cycles(text) == 1
    assert parse_ordering_cycles("nothing") == 0


def test_ordering_scan_is_bounded_and_cached(tmp_path):
    calls = []

    def runner(cmd, **kw):
        if cmd[0] == "journalctl":
            calls.append(cmd)
            return subprocess.CompletedProcess(cmd, 0, "ordering cycle here\n", "")
        if cmd[0] == "systemctl":
            return subprocess.CompletedProcess(
                cmd, 0, "ActiveState=active\nUnitFileState=enabled\n", "")
        if cmd[0] == "git":
            return subprocess.CompletedProcess(cmd, 0, "main\n", "")
        return subprocess.CompletedProcess(cmd, 0, "", "")

    p = ConfigDriftProbe(repo=str(tmp_path), rules_dir=str(tmp_path),
                         packages=[], units=[], runner=runner)
    p.poll()
    p.poll()
    assert len(calls) == 1, "同一 boot 内只该扫一次 journal"
    assert "-n" in calls[0] and "5000" in calls[0]


@pytest.mark.skipif(os.name == "nt", reason="Windows 路径分隔符与测试假路径不一致")
def test_build_stale_packages_uses_source_vs_install_mtime():
    mtimes = {
        "/repo/pi/raspot_ws/src/arm_grasp/setup.py": 200.0,
        "/repo/pi/raspot_ws/install/arm_grasp/share/arm_grasp/package.xml": 100.0,
        "/repo/pi/raspot_ws/src/tof_a010/package.xml": 100.0,
        "/repo/pi/raspot_ws/install/tof_a010/share/tof_a010/package.xml": 200.0,
    }
    stale = build_stale_packages(
        "/repo", ["arm_grasp", "tof_a010"], exists=lambda p: p in mtimes,
        mtime=lambda p: mtimes[p])
    assert stale == ["arm_grasp"]


def test_probe_collects_drift(tmp_path):
    repo = tmp_path / "repo"
    (repo / "pi/raspot_ws/src/arm_grasp").mkdir(parents=True)
    (repo / "pi/raspot_ws/src/arm_grasp/setup.py").write_text("x")
    (repo / "pi/raspot_ws/install/arm_grasp/share/arm_grasp").mkdir(parents=True)
    install_marker = repo / "pi/raspot_ws/install/arm_grasp/share/arm_grasp/package.xml"
    install_marker.write_text("x")
    os.utime(repo / "pi/raspot_ws/src/arm_grasp/setup.py", (200, 200))
    os.utime(install_marker, (100, 100))

    rules = tmp_path / "rules"
    rules.mkdir()          # 故意什么规则都不装

    def runner(cmd, **kw):
        if cmd[0] == "journalctl":
            return subprocess.CompletedProcess(cmd, 0, "ordering cycle here\n", "")
        if cmd[0] == "systemctl":
            return subprocess.CompletedProcess(
                cmd, 0, "ActiveState=inactive\nUnitFileState=enabled\n", "")
        if cmd[0] == "git":
            out = "main\n" if "rev-parse" in cmd and "--abbrev-ref" in cmd else "abc123\n"
            return subprocess.CompletedProcess(cmd, 0, out, "")
        return subprocess.CompletedProcess(cmd, 0, "", "")

    r = ConfigDriftProbe(repo=str(repo), rules_dir=str(rules),
                         packages=["arm_grasp"], units=["arm-grasp"],
                         runner=runner).poll()
    assert r.evidence["ordering_cycles"] == 1
    assert r.evidence["enabled_but_dead"] == ["arm-grasp"]
    assert "99-tof-a010.rules" in r.evidence["missing_udev_rules"]
    assert r.evidence["git_branch"] == "main"
    assert r.evidence["stale_packages"] == ["arm_grasp"]
