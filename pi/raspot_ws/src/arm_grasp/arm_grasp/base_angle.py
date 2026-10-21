"""Continuous base-angle handling shared by arm tools.

The STM32 firmware already unwraps the 320.3 degree potentiometer period and
reports its cumulative estimate in the 0x56 ``base`` field.  This module keeps
that value usable by every Pi-side consumer and provides the raw-pot fallback
used when replaying older logs that do not contain the firmware estimate.
"""
from dataclasses import dataclass
import math


FIELD_DEG = 360.0 / 1000.0
POT_DEG_AT_RAW0 = -115.4
POT_DEG_PER_RAW = 0.2308
POT_PERIOD_DEG = POT_DEG_PER_RAW * 1388.0
BASE_KNOWN_BIT = 0x02
BASE_FAULT_BIT = 0x10


def _wrapped_delta(value, reference, period=360.0):
    delta = (float(value) - float(reference)) % period
    if delta > period / 2.0:
        delta -= period
    return delta


def field_to_deg(field):
    """Decode the firmware's signed cumulative base field."""
    return float(field) * FIELD_DEG


def pot_to_deg(raw):
    return POT_DEG_AT_RAW0 + POT_DEG_PER_RAW * float(raw)


def classify_orientation(angle_deg, boundary_deg=90.0):
    """Return ``front``/``rear`` from the anchored absolute angle.

    The two half-planes meet at +/-90 degrees.  Values exactly on a boundary
    are refused because choosing a table height there would be arbitrary.
    """
    a = float(angle_deg) % 360.0
    front_distance = abs(_wrapped_delta(a, 0.0))
    rear_distance = abs(_wrapped_delta(a, 180.0))
    if abs(front_distance - rear_distance) < 1e-9:
        raise ValueError('base angle lies on front/rear boundary')
    if min(front_distance, rear_distance) >= boundary_deg:
        raise ValueError('base angle is not anchored to front or rear')
    return 'front' if front_distance < rear_distance else 'rear'


def support_height(angle_deg, front_z_mm=-135.5, rear_z_mm=0.0):
    orientation = classify_orientation(angle_deg)
    return (float(front_z_mm) if orientation == 'front' else float(rear_z_mm),
            orientation)


@dataclass
class BaseAngleTracker:
    """Track firmware absolute angle, with a calibrated raw-pot fallback."""

    angle_deg: float | None = None
    field: float | None = None
    pot_raw: float | None = None
    status: int | None = None
    source: str = 'unknown'
    known: bool = False

    def update(self, field=None, pot_raw=None, status=None):
        if status is not None:
            self.status = int(status)
        if pot_raw is not None and math.isfinite(float(pot_raw)) and float(pot_raw) >= 0:
            self.pot_raw = float(pot_raw)
        if field is not None and math.isfinite(float(field)):
            self.field = float(field)
            # This is the firmware's already-unwrapped cumulative estimate.
            self.angle_deg = field_to_deg(field)
            self.known = (self.status is None or
                          bool(self.status & BASE_KNOWN_BIT and
                               not self.status & BASE_FAULT_BIT))
            self.source = 'firmware_0x56_base'
            return self.angle_deg
        if self.pot_raw is not None and self.angle_deg is not None:
            principal = pot_to_deg(self.pot_raw)
            self.angle_deg += _wrapped_delta(principal, self.angle_deg,
                                             POT_PERIOD_DEG)
            self.known = bool(self.status is None or
                              (self.status & BASE_KNOWN_BIT and
                               not self.status & BASE_FAULT_BIT))
            self.source = 'pot_unwrapped'
        return self.angle_deg

    def anchor(self, angle_deg, source='operator_anchor'):
        self.angle_deg = float(angle_deg)
        self.known = True
        self.source = str(source)

    def orientation(self):
        if not self.known or self.angle_deg is None:
            raise ValueError('absolute base angle is unknown')
        return classify_orientation(self.angle_deg)

    def default_support(self, front_z_mm=-135.5, rear_z_mm=0.0):
        if not self.known or self.angle_deg is None:
            raise ValueError('absolute base angle is unknown')
        z, orientation = support_height(self.angle_deg, front_z_mm, rear_z_mm)
        return {'support_z_mm': z, 'orientation': orientation,
                'source': self.source, 'angle_deg': self.angle_deg,
                'field': self.field, 'pot_raw': self.pot_raw,
                'status': self.status, 'known': self.known}

    def snapshot(self):
        return {'angle_deg': self.angle_deg, 'field': self.field,
                'pot_raw': self.pot_raw, 'status': self.status,
                'source': self.source, 'known': self.known}


def resolve_support(explicit_mm, tracker, front_z_mm=-135.5, rear_z_mm=0.0):
    """Resolve explicit operator height first, otherwise firmware angle rules."""
    if explicit_mm is not None:
        return {**tracker.snapshot(), 'support_z_mm': float(explicit_mm),
                'orientation': None, 'source': 'explicit_operator'}
    return tracker.default_support(front_z_mm, rear_z_mm)
