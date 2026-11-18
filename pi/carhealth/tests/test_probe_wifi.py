import subprocess

from carhealth.probes.wifi import WifiProbe

IWCONFIG = """\
wlan0     IEEE 802.11  ESSID:"OPPO"
          Bit Rate=72.2 Mb/s   Tx-Power=31 dBm
          Power Management:on
"""


def test_parses_power_save_on():
    def runner(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 0, IWCONFIG, "")

    r = WifiProbe(runner=runner).poll()
    assert r.ok is True
    assert r.evidence["power_save"] is True


def test_parses_power_save_off():
    def runner(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 0, IWCONFIG.replace(":on", ":off"), "")

    assert WifiProbe(runner=runner).poll().evidence["power_save"] is False


def test_missing_iwconfig_is_unknown():
    def runner(cmd, **kw):
        raise OSError("not found")

    assert WifiProbe(runner=runner).poll().ok is None
