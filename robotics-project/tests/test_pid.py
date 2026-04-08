import pytest

from robotics_project.control.pid import PID


def test_pid_proportional_step() -> None:
    pid = PID(kp=2.0, ki=0.0, kd=0.0)

    # error = setpoint - measurement = 1 - 0 = 1
    u = pid.update(measurement=0.0, dt=1.0, setpoint=1.0)
    assert u == pytest.approx(2.0)


def test_pid_integral_accumulates() -> None:
    pid = PID(kp=0.0, ki=1.0, kd=0.0)

    # integral += error * dt; error stays at 1.
    u1 = pid.update(measurement=0.0, dt=1.0, setpoint=1.0)
    u2 = pid.update(measurement=0.0, dt=1.0, setpoint=1.0)

    assert u1 == pytest.approx(1.0)
    assert u2 == pytest.approx(2.0)

