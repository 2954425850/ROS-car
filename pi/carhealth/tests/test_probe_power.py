from carhealth.probes.power import PowerProbe, parse_throttled


def test_parse_throttled():
    assert parse_throttled("throttled=0x50000\n") == 0x50000
    assert parse_throttled("garbage") is None


def test_undervoltage_now_blocks():
    r = PowerProbe(fake_output="throttled=0x1").poll()
    assert r.evidence["uv_now"] is True
    assert r.evidence["uv_recent"] is True


def test_fake_bare_hex_is_accepted():
    """CLI 的 --fake-throttled 0x1 不带 throttled= 前缀。"""
    r = PowerProbe(fake_output="0x1").poll()
    assert r.ok is True
    assert r.evidence["uv_now"] is True


def test_history_bit_alone_is_not_recent():
    """历史位只在开机后置位、时间未知；doctor 每次新建探针，
    绝不能因此永远判 FAIL。"""
    r = PowerProbe(fake_output="throttled=0x50000").poll()
    assert r.evidence["uv_now"] is False
    assert r.evidence["uv_recent"] is False
    assert r.evidence["uv_since_boot"] is True


def test_observed_undervoltage_stays_recent_for_window():
    now = [1000.0]
    outputs = iter(["throttled=0x1", "throttled=0x10000", "throttled=0x10000"])
    p = PowerProbe(fake_output=None, runner=lambda *a, **k: _cp(str(next(outputs))),
                   clock=lambda: now[0], recent_s=600)
    assert p.poll().evidence["uv_recent"] is True
    now[0] += 100
    assert p.poll().evidence["uv_recent"] is True
    now[0] += 601
    assert p.poll().evidence["uv_recent"] is False


def test_other_throttle_bits_are_exposed():
    r = PowerProbe(fake_output="throttled=0xe").poll()
    assert r.evidence["freq_capped"] is True
    assert r.evidence["throttled_now"] is True
    assert r.evidence["soft_temp"] is True


def test_no_vcgencmd_is_unknown_not_blocked():
    r = PowerProbe(fake_output=None, runner=lambda *a, **k: (_ for _ in ()).throw(OSError("no vcgencmd"))).poll()
    assert r.ok is None
    assert "vcgencmd" in r.error


def _cp(stdout, rc=0):
    import subprocess
    return subprocess.CompletedProcess([], rc, stdout, "")
