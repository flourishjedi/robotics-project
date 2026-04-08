from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Literal

from robotics_project.kinematics.diff_drive import integrate_unicycle


@dataclass(frozen=True)
class UnicycleCommand:
    v: float  # linear velocity (m/s)
    w: float  # yaw rate (rad/s)


def simulate_unicycle_trajectory(
    start_pose: tuple[float, float, float],
    cmd: UnicycleCommand,
    *,
    dt: float,
    steps: int,
    physics_hz: int = 60,
    view: Literal["gui", "direct"] = "gui",
) -> tuple[float, float, float]:
    """
    Run a tiny PyBullet scene while integrating a unicycle model.

    The simulation is "visual only": we don't spawn a full robot URDF yet.
    """

    try:
        import pybullet as p  # type: ignore
        import pybullet_data  # type: ignore
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(
            "PyBullet is not installed. Install it with: pip install -e '.[simulation]'"
        ) from e

    if dt <= 0:
        raise ValueError(f"dt must be > 0, got: {dt}")
    if steps <= 0:
        raise ValueError(f"steps must be > 0, got: {steps}")

    connection_mode = p.GUI if view == "gui" else p.DIRECT
    client = p.connect(connection_mode)
    if client < 0:
        raise RuntimeError("Failed to connect to PyBullet")

    # Configure world.
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / physics_hz)

    plane_id = p.loadURDF("plane.urdf")
    _ = plane_id

    pose = start_pose

    line_id = None
    prev = pose
    # Draw incremental segments at each integration step.
    for _ in range(steps):
        pose = integrate_unicycle(pose, v=cmd.v, w=cmd.w, dt=dt)
        x0, y0, th0 = prev
        x1, y1, th1 = pose
        prev = pose

        # Draw a segment on the ground plane.
        line_id = p.addUserDebugLine(
            lineFromXYZ=[x0, y0, 0.0],
            lineToXYZ=[x1, y1, 0.0],
            lineColorRGB=[0.2, 0.6, 1.0],
            lineWidth=2.0,
            lifeTime=dt * 1.5,
        )

        p.stepSimulation()

        if view == "gui":
            # Keep the GUI realtime-ish without assuming a specific monitor speed.
            # (PyBullet timing isn't guaranteed; this is a best-effort throttle.)
            import time

            time.sleep(max(0.0, (dt / max(1, math.ceil(dt * physics_hz))) * 0.3))

    # Keep the last frame visible briefly when in GUI mode.
    if view == "gui":
        import time

        time.sleep(0.5)

    p.disconnect()
    return pose


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--v", type=float, default=0.5, help="Linear velocity (m/s)")
    ap.add_argument("--w", type=float, default=0.5, help="Yaw rate (rad/s)")
    ap.add_argument("--dt", type=float, default=0.02, help="Integration timestep (s)")
    ap.add_argument("--steps", type=int, default=300, help="Number of integration steps")
    ap.add_argument("--view", choices=["gui", "direct"], default="gui", help="PyBullet connection mode")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    start_pose = (0.0, 0.0, 0.0)
    cmd = UnicycleCommand(v=args.v, w=args.w)

    pose = simulate_unicycle_trajectory(
        start_pose,
        cmd,
        dt=args.dt,
        steps=args.steps,
        view=args.view,  # gui or direct
    )
    x, y, th = pose
    print(f"Final pose: x={x:.3f} y={y:.3f} theta={th:.3f} (rad)")


if __name__ == "__main__":
    main()

