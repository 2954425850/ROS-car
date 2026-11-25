import math

from scan_limit import limit_max_range


def test_clamps_finite_max_range_to_inf():
    out = limit_max_range([0.5, 11.4, 11.5, 12.0,
                           float('inf'), float('nan')], 11.5)
    assert out[0] == 0.5
    assert out[1] == 11.4
    assert math.isinf(out[2]) and out[2] > 0
    assert math.isinf(out[3]) and out[3] > 0
    assert math.isinf(out[4]) and out[4] > 0
    assert math.isnan(out[5])


def test_keeps_ranges_below_threshold():
    out = limit_max_range([0.1, 5.0, 11.49], 11.5)
    assert out == [0.1, 5.0, 11.49]
