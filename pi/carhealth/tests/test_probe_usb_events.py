import subprocess

from carhealth.probes.usb_events import UsbEventsProbe, parse_usb_disconnects

FIXTURE = """\
usb 1-1: new full-speed USB device number 3 using xhci_hcd
usb 1-1: New USB device found, idVendor=0403, idProduct=6010
usb 1-2: New USB device found, idVendor=1209, idProduct=abd1
usb 1-1: USB disconnect, device number 3
usb 1-1: New USB device found, idVendor=0403, idProduct=6010
usb 1-1: USB disconnect, device number 4
"""


def test_parse_counts_by_vid_pid():
    assert parse_usb_disconnects(FIXTURE) == {"0403:6010": 2}


def test_unknown_bus_counts_separately():
    text = "usb 9-9: USB disconnect, device number 7\n"
    assert parse_usb_disconnects(text) == {"unknown": 1}


def test_probe_permission_failure_is_unknown():
    def runner(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 1, "", "no permission")

    r = UsbEventsProbe(runner=runner).poll()
    assert r.ok is None
    assert "USB 事件" in r.error


def test_probe_counts_from_journal():
    def runner(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 0, FIXTURE, "")

    r = UsbEventsProbe(runner=runner).poll()
    assert r.evidence["disconnects"] == {"0403:6010": 2}
    assert r.evidence["total"] == 2
