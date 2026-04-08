from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DiffDriveParams:
    """Physical parameters for a differential drive robot."""

    wheel_radius: float
    wheelbase: float  # distance between left/right wheels


def twist_from_wheel_speeds(
    omega_l: float, omega_r: float, params: DiffDriveParams
) -> tuple[float, float]:
    """
    Convert wheel angular speeds (rad/s) to planar body twist (v, w).

    v: forward linear velocity (m/s)
    w: yaw rate (rad/s)
    """

    r = params.wheel_radius
    L = params.wheelbase
    if L <= 0:
        raise ValueError(f"wheelbase must be > 0, got: {L}")

    v = r * (omega_r + omega_l) / 2.0
    w = r * (omega_r - omega_l) / L
    return v, w


def integrate_unicycle(
    pose: tuple[float, float, float], v: float, w: float, dt: float
) -> tuple[float, float, float]:
    """
    Integrate a unicycle model exactly over dt.

    pose: (x, y, theta) in meters and radians
    v: forward linear velocity (m/s)
    w: yaw rate (rad/s)
    """

    if dt < 0:
        raise ValueError(f"dt must be >= 0, got: {dt}")

    x0, y0, th0 = pose
    if dt == 0 or (abs(v) == 0.0 and abs(w) == 0.0):
        return pose

    if abs(w) < 1e-12:
        # Straight line limit.
        dx = v * dt * math.cos(th0)
        dy = v * dt * math.sin(th0)
        return (x0 + dx, y0 + dy, th0)

    # Exact circular arc.
    th1 = th0 + w * dt
    R = v / w

    # Using standard unicycle exact integration.
    x1 = x0 + R * (math.sin(th1) - math.sin(th0))
    y1 = y0 - R * (math.cos(th1) - math.cos(th0))
    return (x1, y1, th1)

