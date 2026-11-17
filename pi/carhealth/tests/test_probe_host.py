from carhealth.probes.host import HostProbe, parse_cpu_stat


def test_parse_cpu_stat():
    total, idle = parse_cpu_stat("cpu 100 0 100 800 0 0 0 0 0 0\ncpu0 ...\n")
    assert total == 1000
    assert idle == 800


def test_cpu_pct_needs_two_samples(tmp_path):
    stat = tmp_path / "stat"
    stat.write_text("cpu 100 0 100 800 0 0 0 0 0 0\n")
    now = [100.0]
    p = HostProbe(meminfo=str(tmp_path / "nope"), thermal=str(tmp_path / "nope"),
                  disk_path=str(tmp_path), wireless=str(tmp_path / "nope"),
                  stat_path=str(stat), getloadavg=lambda: (0.0, 0.0, 0.0),
                  clock=lambda: now[0])
    assert "cpu_pct" not in p.poll().evidence
    stat.write_text("cpu 200 0 200 1200 0 0 0 0 0 0\n")
    now[0] = 101.0
    assert p.poll().evidence["cpu_pct"] == 33.3


def test_host_evidence_from_fake_files(tmp_path):
    mem = tmp_path / "meminfo"
    mem.write_text(
        "MemTotal:       8000000 kB\n"
        "MemAvailable:   1048576 kB\n"
        "SwapTotal:      1000000 kB\n"
        "SwapFree:        500000 kB\n"
    )
    thermal = tmp_path / "thermal"
    thermal.write_text("55000\n")
    wireless = tmp_path / "wireless"
    wireless.write_text(
        "Inter-| sta-|   Quality        |   Discarded packets\n"
        " face | tus | link level noise |  nwid  crypt   frag  retry   misc\n"
        " wlan0: 0000   45.  -55.  -256        0      0      0      0      0\n"
    )
    r = HostProbe(meminfo=str(mem), thermal=str(thermal), disk_path=str(tmp_path),
                  wireless=str(wireless), getloadavg=lambda: (1.5, 1.2, 1.0)).poll()
    assert r.ok is True
    assert r.evidence["mem_available_mb"] == 1024.0
    assert r.evidence["soc_temp_c"] == 55.0
    assert r.evidence["load1"] == 1.5
    assert r.evidence["wifi_link"] == 45.0
    assert r.evidence["wifi_signal_dbm"] == -55.0
    assert 0 < r.evidence["disk_free_pct"] <= 100


def test_missing_files_leave_fields_absent(tmp_path):
    r = HostProbe(meminfo=str(tmp_path / "nope"), thermal=str(tmp_path / "nope2"),
                  disk_path=str(tmp_path), wireless=str(tmp_path / "nope3"),
                  getloadavg=lambda: (_ for _ in ()).throw(OSError("n/a"))).poll()
    assert "mem_available_mb" not in r.evidence
    assert "soc_temp_c" not in r.evidence
    assert "wifi_link" not in r.evidence
    assert "disk_free_pct" in r.evidence
