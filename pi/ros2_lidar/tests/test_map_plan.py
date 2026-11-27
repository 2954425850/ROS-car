import os

import pytest

from map_plan_core import UNKNOWN_OCC, convert_values

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_unknown_becomes_soft_occupancy_known_cells_untouched():
    # 87/100*254 ~= 221: above analytic_expansion_max_cost(200), below 253.
    assert UNKNOWN_OCC == 87
    assert convert_values([-1, 0, 100, 50]) == [87, 0, 100, 50]
    assert convert_values([]) == []


def test_both_global_costmaps_read_the_forwarded_map():
    yaml = pytest.importorskip("yaml")

    def load(name):
        with open(os.path.join(HERE, "config", name), encoding="utf-8") as f:
            return yaml.safe_load(f)

    for name in ("nav2_params.yaml", "nav2_params_fusion.yaml"):
        g = load(name)["global_costmap"]["global_costmap"]["ros__parameters"]
        static = g["static_layer"]
        assert static["map_topic"] == "/map_plan", name
        assert static["trinary_costmap"] is False, name
        # The forwarding replaces -1, so the boolean is moot; it stays false
        # from ab6fe95 and does not change known cells.
        assert g["track_unknown_space"] is False, name
