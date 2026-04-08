from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class LoadConfigResult:
    """Result wrapper for loaded YAML data."""

    data: dict[str, Any]


def load_yaml_config(path: str | Path) -> LoadConfigResult:
    """
    Load a YAML file into a plain dictionary.

    This is intentionally lightweight and ROS-agnostic.
    """

    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        raise ValueError(f"Expected YAML object at top-level, got: {type(data).__name__}")

    return LoadConfigResult(data=data)

