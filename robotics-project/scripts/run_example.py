from __future__ import annotations

from robotics_project.control.pid import PID
from robotics_project.kinematics.diff_drive import (
    DiffDriveParams,
    integrate_unicycle,
    twist_from_wheel_speeds,
)


def main() -> None:
    # --- PID usage (toy example) ---
    pid = PID(kp=2.0, ki=0.1, kd=0.0)
    measurement = 0.0
    setpoint = 1.0

    dt = 0.1
    for _ in range(10):
        u = pid.update(measurement=measurement, dt=dt, setpoint=setpoint)
        # Toy plant: measurement moves proportionally to control effort.
        measurement += dt * u

    print(f"PID toy run: measurement={measurement:.4f} (setpoint={setpoint})")

    # --- Differential drive usage ---
    params = DiffDriveParams(wheel_radius=0.1, wheelbase=0.5)
    v, w = twist_from_wheel_speeds(omega_l=10.0, omega_r=0.0, params=params)
    pose = integrate_unicycle((0.0, 0.0, 0.0), v=v, w=w, dt=1.0)
    x, y, th = pose
    print(f"Diff drive example pose: x={x:.4f}, y={y:.4f}, theta={th:.4f}")


if __name__ == "__main__":
    main()

