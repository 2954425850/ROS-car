"""Pure conversion for the /map -> /map_plan forwarding node (no ROS imports).

StaticLayer in non-trinary mode scales OccupancyGrid values by value/100*254.
87/100*254 ~= 221: above analytic_expansion_max_cost (200) so Reeds-Shepp
shortcuts avoid unknown, below INSCRIBED_INFLATED_OBSTACLE (253) so the A*
search and the start footprint check can still traverse it.
"""

UNKNOWN_OCC = 87


def convert_values(data):
    """Return a copy of OccupancyGrid.data with every negative cell -> 87."""
    return [UNKNOWN_OCC if v < 0 else v for v in data]
