# Architecture Notes

This project is intentionally ROS-agnostic at the start so you can reuse the core math and control code in:

- simulation,
- hardware bring-up,
- later ROS/ROS2 nodes.

## Layout

- `src/robotics_project/`
  - `control/`: controllers (currently `PID`)
  - `kinematics/`: robot motion models (currently differential drive + unicycle integration)
  - `config.py`: small YAML loader helper
- `tests/`: unit tests for the math/control modules
- `scripts/`: quick runnable examples

