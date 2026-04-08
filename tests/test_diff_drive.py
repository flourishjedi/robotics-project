import pytest

from robotics_project.kinematics.diff_drive import (
    DiffDriveParams,
    integrate_unicycle,
    twist_from_wheel_speeds,
)


def test_twist_straight_motion() -> None:
    params = DiffDriveParams(wheel_radius=0.1, wheelbase=0.5)

    v, w = twist_from_wheel_speeds(omega_l=10.0, omega_r=10.0, params=params)
    assert v == pytest.approx(1.0)
    assert w == pytest.approx(0.0)

    x, y, th = integrate_unicycle((0.0, 0.0, 0.0), v=v, w=w, dt=2.0)
    assert x == pytest.approx(2.0)
    assert y == pytest.approx(0.0)
    assert th == pytest.approx(0.0)


def test_twist_and_arc_integration() -> None:
    params = DiffDriveParams(wheel_radius=0.1, wheelbase=0.5)

    # omega_l=0, omega_r=10
    v, w = twist_from_wheel_speeds(omega_l=0.0, omega_r=10.0, params=params)
    assert v == pytest.approx(0.5)
    assert w == pytest.approx(2.0)

    dt = 1.0
    x, y, th = integrate_unicycle((0.0, 0.0, 0.0), v=v, w=w, dt=dt)

    # Exact integration expectations for theta0=0:
    # theta1 = w*dt = 2
    # R = v/w = 0.25
    # x1 = R * (sin(theta1) - sin(theta0))
    # y1 = -R * (cos(theta1) - cos(theta0)) = R * (1 - cos(theta1))
    th1 = 2.0
    R = 0.25
    # Compute expected values directly:
    import math

    assert x == pytest.approx(R * (math.sin(th1) - math.sin(0.0)))
    assert y == pytest.approx(R * (1.0 - math.cos(th1)))
    assert th == pytest.approx(th1)

