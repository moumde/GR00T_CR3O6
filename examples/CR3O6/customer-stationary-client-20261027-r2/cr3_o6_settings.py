"""Validated local settings for the standalone CR3/O6 Customer client."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class CustomerSettings:
    api_url: str
    token_file: Path
    timeout_s: float
    robot_ip: str
    nrc_sdk_dir: Path
    linkerhand_sdk_dir: Path
    hand_config: Path
    o6_modbus: str | None
    top_camera: str
    wrist_camera: str
    side_camera: str


def _table(
    data: dict[str, Any],
    name: str,
    required: set[str],
    optional: set[str] | frozenset[str] = frozenset(),
) -> dict[str, Any]:
    value = data[name]
    if (
        not isinstance(value, dict)
        or not required <= value.keys()
        or value.keys() - required - optional
    ):
        raise ValueError(
            f"Invalid [{name}] settings: expected {sorted(required)}, "
            f"optional {sorted(optional)}"
        )
    return value


def _string(table: dict[str, Any], key: str) -> str:
    value = table[key]
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Configuration value {key} must be a nonempty string")
    return value


def _path(table: dict[str, Any], key: str, base: Path) -> Path:
    value = Path(_string(table, key)).expanduser()
    return (value if value.is_absolute() else base / value).resolve()


def load_settings(path: Path) -> CustomerSettings:
    """Load the one robot deployment profile; relative paths use its directory."""
    path = path.expanduser().resolve()
    try:
        with path.open("rb") as stream:
            data = tomllib.load(stream)
    except (OSError, tomllib.TOMLDecodeError) as exc:
        raise ValueError(f"Cannot read CR3/O6 configuration {path}: {exc}") from exc
    if not isinstance(data, dict) or set(data) != {"customer", "robot", "cameras"}:
        raise ValueError("Configuration must have exactly [customer], [robot], and [cameras]")
    customer = _table(data, "customer", {"api_url", "token_file", "timeout_s"})
    robot = _table(
        data,
        "robot",
        {"ip", "nrc_sdk_dir", "linkerhand_sdk_dir", "hand_config"},
        {"o6_modbus"},
    )
    cameras = _table(data, "cameras", {"top", "wrist", "side"})
    timeout = customer["timeout_s"]
    if (
        isinstance(timeout, bool)
        or not isinstance(timeout, (int, float))
        or not 0 < timeout <= 30
    ):
        raise ValueError("[customer].timeout_s must be between 0 and 30 seconds")
    modbus = robot.get("o6_modbus")
    if modbus is not None and (not isinstance(modbus, str) or not modbus.strip()):
        raise ValueError("[robot].o6_modbus must be a nonempty string when present")
    return CustomerSettings(
        api_url=_string(customer, "api_url"),
        token_file=_path(customer, "token_file", path.parent),
        timeout_s=float(timeout),
        robot_ip=_string(robot, "ip"),
        nrc_sdk_dir=_path(robot, "nrc_sdk_dir", path.parent),
        linkerhand_sdk_dir=_path(robot, "linkerhand_sdk_dir", path.parent),
        hand_config=_path(robot, "hand_config", path.parent),
        o6_modbus=modbus,
        top_camera=_string(cameras, "top"),
        wrist_camera=_string(cameras, "wrist"),
        side_camera=_string(cameras, "side"),
    )
