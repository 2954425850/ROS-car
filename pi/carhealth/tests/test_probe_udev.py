import subprocess

from carhealth.probes.udev import UdevProbe


class FakeRunner:
    def __init__(self, out="", rc=0):
        self.out, self.rc = out, rc

    def __call__(self, cmd, **kw):
        return subprocess.CompletedProcess(cmd, self.rc, self.out, "")


def test_missing_node_and_no_by_id():
    r = UdevProbe("udev:a010", ["/dev/a010", "/dev/serial/by-id/*SIPEED*"],
                  exists=lambda p: False, glob_fn=lambda p: []).poll()
    assert r.evidence["exists"] is False
    assert "interface" not in r.evidence


def test_short_name_and_interface_from_udevadm():
    r = UdevProbe(
        "udev:a010", ["/dev/a010"], expect_interface="00",
        exists=lambda p: True, realpath=lambda p: "/dev/ttyUSB0",
        runner=FakeRunner("DEVPATH=/x\nID_USB_INTERFACE_NUM=00\n"),
    ).poll()
    assert r.evidence["exists"] is True
    assert r.evidence["resolved_by"] == "short"
    assert r.evidence["real"] == "/dev/ttyUSB0"
    assert r.evidence["interface"] == "00"


def test_by_id_fallback_is_reported():
    by_id = "/dev/serial/by-id/usb-SIPEED_Meta_Sense_Lite_123-if00-port0"
    r = UdevProbe(
        "udev:a010", ["/dev/a010", "/dev/serial/by-id/*SIPEED*if00-port0"],
        expect_interface="00",
        exists=lambda p: False,
        glob_fn=lambda p: [by_id],
        realpath=lambda p: "/dev/ttyUSB0",
        runner=FakeRunner("ID_USB_INTERFACE_NUM=00\n"),
    ).poll()
    assert r.evidence["exists"] is True
    assert r.evidence["resolved_by"] == "by_id"
    assert r.evidence["path"] == by_id


def test_udevadm_failure_omits_interface_instead_of_guessing():
    r = UdevProbe(
        "udev:a010", ["/dev/a010"], expect_interface="00",
        exists=lambda p: True, realpath=lambda p: "/dev/ttyUSB0",
        runner=FakeRunner("", rc=1),
    ).poll()
    assert "interface" not in r.evidence
