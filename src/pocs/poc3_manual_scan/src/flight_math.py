"""Stick shaping, frame conversion and geofence for manual flight. No ROS imports.

Frames: PX4 local NED (x North, y East, z Down), heading = yaw from North,
clockwise positive. Stick values follow ROS joy: +1 = up / left.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def shape(x: float, deadzone: float, expo: float) -> float:
    """Deadzone, then expo: fine control near centre, full rate at the end stop.

    The deadzone is removed (not just clipped) so output starts at 0 right at
    its edge instead of jumping; expo blends linear and cubic like RC radios.
    """
    if abs(x) <= deadzone:
        return 0.0
    s = math.copysign(min((abs(x) - deadzone) / (1.0 - deadzone), 1.0), x)
    return (1.0 - expo) * s + expo * s ** 3


def body_to_ned(forward: float, right: float, heading: float) -> tuple[float, float]:
    """Body-frame horizontal velocity (forward, right) -> NED (north, east)."""
    c, s = math.cos(heading), math.sin(heading)
    return forward * c - right * s, forward * s + right * c


def slew(current: float, target: float, max_step: float) -> float:
    """Move towards target by at most max_step (acceleration limit per tick)."""
    return current + max(-max_step, min(max_step, target - current))


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Geofence:
    """Box in PX4 local NED plus an altitude band (metres above home, positive up).

    Velocity towards a wall is limited to gain * distance-to-wall, so the drone
    slows down smoothly and stops at the wall instead of hitting a hard clamp;
    velocity away from a wall is never limited, so you can always fly back in.
    """
    north_min: float
    north_max: float
    east_min: float
    east_max: float
    alt_min: float
    alt_max: float
    gain: float = 0.8           # 1/s: at 5 m from the wall at most 4 m/s outwards

    @classmethod
    def from_enu_region(cls, region: dict, margin: float, alt_min: float, alt_max: float) -> Geofence:
        """Scene-manifest region (ENU: min/max = [east, north]) -> NED fence."""
        (e0, n0), (e1, n1) = region["min"], region["max"]
        return cls(n0 + margin, n1 - margin, e0 + margin, e1 - margin, alt_min, alt_max)

    def limit(self, n: float, e: float, alt: float, vn: float, ve: float, vup: float) -> tuple[float, float, float]:
        def axis(p, v, lo, hi):
            if v > 0:
                return min(v, max(0.0, self.gain * (hi - p)))
            if v < 0:
                return max(v, -max(0.0, self.gain * (p - lo)))
            return v
        return (axis(n, vn, self.north_min, self.north_max),
                axis(e, ve, self.east_min, self.east_max),
                axis(alt, vup, self.alt_min, self.alt_max))

    def contains(self, n: float, e: float, alt: float) -> bool:
        return (self.north_min <= n <= self.north_max and self.east_min <= e <= self.east_max
                and self.alt_min - 0.5 <= alt <= self.alt_max)
