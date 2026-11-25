"""LaserScan range helpers (pure; no ROS).

The RPLIDAR A1M8 (and similar drivers) returns a finite value near range_max
for "no echo".  slam_toolbox treats that as a real hit, so repeated scans at
the same bearing grow a false wall at the range limit.  Everything at/above
the valid limit is turned into +inf here; inf/nan are left alone because the
downstream filter and slam_toolbox already ignore them.

The default 9.0 m comes from bench data: the A1M8 on this car returns finite
"no echo" values down to ~9.7 m, and the driver's intensity is a constant 47
(no quality signal to filter on).
"""
import math

DEFAULT_MAX_VALID_RANGE = 9.0


def limit_max_range(ranges, max_valid=DEFAULT_MAX_VALID_RANGE):
    return [math.inf if (math.isfinite(r) and r >= max_valid) else r
            for r in ranges]
