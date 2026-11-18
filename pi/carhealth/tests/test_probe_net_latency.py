import subprocess

from carhealth.probes.net_latency import GatewayLatencyProbe, parse_ping

PING = """\
PING 192.168.1.1 (192.168.1.1) 56(84) bytes of data.
64 bytes from 192.168.1.1: icmp_seq=1 ttl=64 time=3.9 ms
64 bytes from 192.168.1.1: icmp_seq=2 ttl=64 time=141.1 ms
64 bytes from 192.168.1.1: icmp_seq=3 ttl=64 time=321.8 ms

--- 192.168.1.1 ping statistics ---
3 packets transmitted, 3 received, 0% packet loss, time 2003ms
rtt min/avg/max/mdev = 3.896/141.130/321.789/133.362 ms
"""


def test_parse_ping():
    ev = parse_ping(PING)
    assert ev["loss_pct"] == 0.0
    assert ev["rtt_avg_ms"] == 141.130
    assert ev["rtt_max_ms"] == 321.789


def test_probe_reads_default_gateway():
    def runner(cmd, **kw):
        out = ("default via 192.168.1.1 dev wlan0 proto dhcp metric 600\n"
               if cmd[0] == "ip" else PING)
        return subprocess.CompletedProcess(cmd, 0, out, "")

    r = GatewayLatencyProbe(runner=runner).poll()
    assert r.ok is True
    assert r.evidence["gateway"] == "192.168.1.1"
    assert r.evidence["rtt_avg_ms"] == 141.1


def test_single_spike_is_smoothed():
    fast = PING.replace("3.896/141.130/321.789/133.362", "3.0/10.0/12.0/2.0")
    slow = PING.replace("3.896/141.130/321.789/133.362", "3.0/400.0/500.0/300.0")
    outputs = iter([fast, slow, fast, fast, fast])

    def runner(cmd, **kw):
        out = ("default via 192.168.1.1 dev wlan0\n"
               if cmd[0] == "ip" else next(outputs))
        return subprocess.CompletedProcess(cmd, 0, out, "")

    probe = GatewayLatencyProbe(runner=runner)
    for _ in range(5):
        r = probe.poll()
    assert r.evidence["rtt_avg_ms"] == 10.0
    assert r.evidence["samples"] == 5


def test_no_gateway_is_unknown():
    def runner(cmd, **kw):
        return subprocess.CompletedProcess(cmd, 0, "", "")

    r = GatewayLatencyProbe(runner=runner).poll()
    assert r.ok is None
    assert "网关" in r.error
