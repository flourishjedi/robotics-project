from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PID:
    """
    Minimal PID controller.

    The controller is expressed in terms of the error:
      error = setpoint - measurement
    and computes:
      u = kp * error + ki * integral(error) + kd * derivative(error)
    """

    kp: float
    ki: float = 0.0
    kd: float = 0.0

    output_limits: tuple[float | None, float | None] = (None, None)

    def __post_init__(self) -> None:
        self._integral: float = 0.0
        self._prev_error: float | None = None

    def reset(self) -> None:
        """Reset internal state (integral + derivative history)."""

        self._integral = 0.0
        self._prev_error = None

    def update(self, measurement: float, dt: float, *, setpoint: float) -> float:
        """
        Update PID output for a single timestep.

        Args:
            measurement: Current measured value.
            dt: Timestep in seconds (must be > 0).
            setpoint: Target value.
        """

        if dt <= 0:
            raise ValueError(f"dt must be > 0, got: {dt}")

        error = setpoint - measurement

        # Integral term (simple accumulation).
        self._integral += error * dt

        # Derivative term from error change.
        if self._prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error

        u = self.kp * error + self.ki * self._integral + self.kd * derivative

        lo, hi = self.output_limits
        if lo is not None:
            u = max(lo, u)
        if hi is not None:
            u = min(hi, u)

        return u

