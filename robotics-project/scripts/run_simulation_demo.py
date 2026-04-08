from __future__ import annotations

import argparse

from robotics_project.simulation.pybullet_unicycle import main as unicycle_main


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--v", type=float, default=0.5)
    ap.add_argument("--w", type=float, default=0.5)
    ap.add_argument("--dt", type=float, default=0.02)
    ap.add_argument("--steps", type=int, default=300)
    ap.add_argument("--view", choices=["gui", "direct"], default="gui")
    return ap.parse_args()


def main() -> None:
    args = _parse_args()
    unicycle_main(
        [
            "--v",
            str(args.v),
            "--w",
            str(args.w),
            "--dt",
            str(args.dt),
            "--steps",
            str(args.steps),
            "--view",
            args.view,
        ]
    )


if __name__ == "__main__":
    main()

