import subprocess

from carhealth.probes.dds import DdsProbe, parse_ss_local_addrs

SS = """\
UNCONN 0 0 10.0.0.5:7400 0.0.0.0:*
UNCONN 0 0 192.168.1.108:7401 0.0.0.0:*
UNCONN 0 0 127.0.0.1:7402 0.0.0.0:*
UNCONN 0 0 0.0.0.0:7403 0.0.0.0:*
"""


def test_parse_ss_skips_wildcard_and_loopback():
    assert parse_ss_local_addrs(SS) == {"10.0.0.5", "192.168.1.108"}


def test_probe_detects_stale_ip():
    def runner(cmd, **kw):
        out = "10.0.0.5 \n" if cmd[0] == "hostname" else SS
        return subprocess.CompletedProcess(cmd, 0, out, "")

    r = DdsProbe(runner=runner).poll()
    assert r.evidence["current_ips"] == ["10.0.0.5"]
    assert r.evidence["stale_binds"] == ["192.168.1.108"]


def test_probe_without_ss_is_unknown():
    def runner(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 1, "", "no ss")

    assert DdsProbe(runner=runner).poll().ok is None
