# Robotics Project (Python)

This is a small, ROS-agnostic Python workspace scaffold for robotics work:

- Core utilities under `src/robotics_project/`
- Minimal starter modules:
  - PID controller (`control/pid.py`)
  - Differential drive kinematics (`kinematics/diff_drive.py`)
- Unit tests under `tests/`

## Quick start

```bash
cd robotics-project
python -m venv .venv
source .venv/bin/activate

pip install -e .
pytest
```

## Add dependencies

Edit `pyproject.toml` to add robotics- or hardware-specific libraries.

