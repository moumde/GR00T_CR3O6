"""Standalone CR3 controller wrapper for the GR00T CR3/O6 client.

This module contains the Python-side control adapter for the CR3/O6 robot.
The vendor NRC binding is loaded from ``nrc_sdk_dir`` at runtime; the vendor
generated binding and shared object are not reimplemented here.
"""

from __future__ import annotations

import importlib
import logging
import math
from pathlib import Path
import sys
import threading
import time
from collections.abc import Iterable, Sequence
from typing import Any


LOGGER = logging.getLogger(__name__)

NUM_JOINTS = 6
SDK_NUM_AXES = 7
CONTROL_PORT = 6000
TEACH_PORT = 6001
TRACK_PORT = 7000
SERVO_STOPPED = 0
SERVO_READY = 1
SERVO_ERROR = 2
SERVO_RUNNING = 3


class NRCError(RuntimeError):
    """Base exception raised by :class:`CR3Interface`."""


class NRCConnectionError(NRCError):
    """Raised when an NRC socket cannot be connected or becomes unusable."""


class NRCCommandError(NRCError):
    """Raised when the vendor SDK rejects a command."""

    def __init__(self, command: str, code: Any):
        self.command = command
        self.code = code
        super().__init__(f"{command} failed with NRC return code {code!r}")


def _as_float_list(values: Iterable[float], *, name: str) -> list[float]:
    if hasattr(values, "tolist"):
        values = values.tolist()
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be a one-dimensional numeric sequence") from exc
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _return_code(result: Any) -> Any:
    if isinstance(result, (tuple, list)):
        return result[0] if result else 0
    return result


def _output_from_result(result: Any, fallback: Any) -> Any:
    if isinstance(result, (tuple, list)) and len(result) > 1:
        return result[1]
    return fallback


def _load_nrc_module(sdk_dir: str | Path):
    """Load the vendor SWIG binding from a local SDK directory.

    ``sdk_dir`` may point at the directory containing ``nrc_interface.py`` and
    ``_nrc_host.so`` or at its parent directory.
    """

    root = Path(sdk_dir).expanduser().resolve()
    candidates = [root, root / "nrc_linux_x86_64"]
    nrc_dir = next((path for path in candidates if (path / "nrc_interface.py").exists()), None)
    if nrc_dir is None:
        raise FileNotFoundError(
            "NRC SDK not found. Expected nrc_interface.py and _nrc_host.so under "
            f"{root} or {root / 'nrc_linux_x86_64'}"
        )
    if not (nrc_dir / "_nrc_host.so").exists():
        raise FileNotFoundError(f"NRC SDK shared library is missing: {nrc_dir / '_nrc_host.so'}")

    module = sys.modules.get("nrc_interface")
    if module is not None and Path(getattr(module, "__file__", "")).resolve() != (
        nrc_dir / "nrc_interface.py"
    ).resolve():
        sys.modules.pop("nrc_interface", None)
    host = sys.modules.get("_nrc_host")
    if host is not None:
        host_path = Path(getattr(host, "__file__", "")).resolve()
        if host_path.parent != nrc_dir:
            sys.modules.pop("_nrc_host", None)
    sys.path.insert(0, str(nrc_dir))
    try:
        return importlib.import_module("nrc_interface")
    except ImportError as exc:
        raise NRCConnectionError(
            "Failed to load NRC vendor binding. It requires the vendor SDK's "
            "supported Linux/CPython runtime and libpython."
        ) from exc


def _sdk_vector(nrc: Any, values: Sequence[float]):
    vector = nrc.VectorDouble()
    for value in values:
        vector.append(float(value))
    return vector


def _vector_values(value: Any) -> list[float]:
    if hasattr(value, "tolist"):
        value = value.tolist()
    return [float(item) for item in value]


class CR3Interface:
    """Direct controller for a six-axis Dobot CR3 using the NRC SDK."""

    def __init__(
        self,
        robot_ip: str = "192.168.2.14",
        *,
        nrc_sdk_dir: str | Path,
        connect_control: bool = False,
        connection_timeout: float = 10.0,
        connection_poll_period: float = 0.2,
        servo_vmax: Sequence[float] | None = None,
        servo_amax: Sequence[float] | None = None,
        servo_jmax: Sequence[float] | None = None,
    ) -> None:
        self.robot_ip = str(robot_ip)
        self._nrc = _load_nrc_module(nrc_sdk_dir)
        self.connect_control = connect_control
        self.connection_timeout = float(connection_timeout)
        self.connection_poll_period = float(connection_poll_period)
        if self.connection_timeout <= 0 or self.connection_poll_period <= 0:
            raise ValueError("connection_timeout and connection_poll_period must be positive")
        self.servo_vmax = self._constraint_vector(servo_vmax, "servo_vmax")
        self.servo_amax = self._constraint_vector(servo_amax, "servo_amax")
        self.servo_jmax = self._constraint_vector(servo_jmax, "servo_jmax")
        self._fds: dict[int, int] = {}
        self._servo_active = False
        self._lock = threading.RLock()

    @staticmethod
    def _constraint_vector(values: Sequence[float] | None, name: str) -> list[float]:
        if values is None:
            values = [500.0] * SDK_NUM_AXES
        result = _as_float_list(values, name=name)
        if len(result) != SDK_NUM_AXES:
            raise ValueError(f"{name} must contain {SDK_NUM_AXES} values")
        return result

    @property
    def connected(self) -> bool:
        return TEACH_PORT in self._fds and TRACK_PORT in self._fds

    @property
    def servo_active(self) -> bool:
        return self._servo_active

    def _fd(self, port: int) -> int:
        try:
            return self._fds[port]
        except KeyError as exc:
            raise NRCConnectionError(f"NRC port {port} is not connected") from exc

    def _call(self, command: str, function, *args) -> Any:
        result = function(*args)
        code = _return_code(result)
        if code != 0:
            raise NRCCommandError(command, code)
        return result

    def _connect_port(self, port: int) -> int:
        LOGGER.info("Connecting to CR3 at %s:%d", self.robot_ip, port)
        fd = int(self._nrc.connect_robot(self.robot_ip, str(port)))
        if fd <= 0:
            raise NRCConnectionError(f"connect_robot({self.robot_ip}, {port}) returned {fd}")
        deadline = time.monotonic() + self.connection_timeout
        try:
            status = int(self._nrc.get_connection_status(fd))
            while status != 0 and time.monotonic() < deadline:
                time.sleep(self.connection_poll_period)
                status = int(self._nrc.get_connection_status(fd))
            if status != 0:
                raise NRCConnectionError(
                    f"connection to {self.robot_ip}:{port} timed out with status {status}"
                )
        except BaseException:
            self._nrc.disconnect_robot(fd)
            raise
        return fd

    def connect(self) -> "CR3Interface":
        with self._lock:
            if self._fds:
                if self.connected and (not self.connect_control or CONTROL_PORT in self._fds):
                    return self
                raise NRCConnectionError("partial NRC connection exists; call disconnect() first")
            ports = [TEACH_PORT, TRACK_PORT]
            if self.connect_control:
                ports.insert(0, CONTROL_PORT)
            connected: list[int] = []
            try:
                for port in ports:
                    self._fds[port] = self._connect_port(port)
                    connected.append(port)
            except BaseException:
                for port in reversed(connected):
                    self._nrc.disconnect_robot(self._fds.pop(port))
                raise
        return self

    def disconnect(self) -> None:
        with self._lock:
            if self._servo_active:
                try:
                    self.close_servoj()
                except NRCError:
                    LOGGER.exception("Failed to close ServoJ during disconnect")
                self._servo_active = False
            for port, fd in list(self._fds.items()):
                try:
                    self._nrc.disconnect_robot(fd)
                finally:
                    self._fds.pop(port, None)

    close = disconnect

    def __enter__(self) -> "CR3Interface":
        return self.connect()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.disconnect()

    def get_servo_state(self) -> int:
        with self._lock:
            output = 0
            result = self._call(
                "get_servo_state", self._nrc.get_servo_state, self._fd(TEACH_PORT), output
            )
            output = _output_from_result(result, output)
            if isinstance(output, (tuple, list)):
                output = output[0]
            return int(output)

    def get_current_mode(self) -> int:
        with self._lock:
            output = 0
            result = self._call(
                "get_current_mode", self._nrc.get_current_mode, self._fd(TEACH_PORT), output
            )
            output = _output_from_result(result, output)
            if isinstance(output, (tuple, list)):
                output = output[0]
            return int(output)

    def clear_error(self) -> None:
        self._call("clear_error", self._nrc.clear_error, self._fd(TEACH_PORT))

    def power_on(self) -> None:
        with self._lock:
            fd = self._fd(TEACH_PORT)
            state = self.get_servo_state()
            if state == SERVO_RUNNING:
                return
            if state == SERVO_ERROR:
                self._call("clear_error", self._nrc.clear_error, fd)
                try:
                    self._call("set_servo_poweroff", self._nrc.set_servo_poweroff, fd)
                except NRCCommandError:
                    LOGGER.warning("Power-off after clear_error was rejected; continuing")
                time.sleep(0.2)
            if state in (SERVO_STOPPED, SERVO_ERROR):
                self._call("set_servo_state(ready)", self._nrc.set_servo_state, fd, SERVO_READY)
                time.sleep(0.2)
            self._call("set_servo_poweron", self._nrc.set_servo_poweron, fd)

    def power_off(self) -> None:
        with self._lock:
            fd = self._fd(TEACH_PORT)
            state = self.get_servo_state()
            if state != SERVO_RUNNING:
                raise NRCCommandError("set_servo_poweroff (servo is not running)", state)
            self._call("set_servo_poweroff", self._nrc.set_servo_poweroff, fd)

    def open_servoj(self) -> None:
        with self._lock:
            fd = self._fd(TRACK_PORT)
            state = self.get_servo_state()
            if state != SERVO_RUNNING:
                raise NRCCommandError("open_servoJ (servo is not running)", state)
            if self._servo_active:
                raise NRCCommandError("open_servoJ (already active)", "already active")
            self._call(
                "open_servoJ",
                self._nrc.open_servoJ,
                fd,
                _sdk_vector(self._nrc, self.servo_vmax),
                _sdk_vector(self._nrc, self.servo_amax),
                _sdk_vector(self._nrc, self.servo_jmax),
            )
            self._servo_active = True

    open_servoJ = open_servoj

    def close_servoj(self) -> None:
        with self._lock:
            if not self._servo_active:
                return
            close_fn = getattr(self._nrc, "close_servoJ", None)
            if close_fn is None:
                close_fn = getattr(self._nrc, "stop_servoJ", None)
            if close_fn is None:
                raise NRCError("NRC binding exposes neither close_servoJ nor stop_servoJ")
            self._call("close_servoJ", close_fn, self._fd(TRACK_PORT))
            self._servo_active = False

    close_servoJ = close_servoj

    def _joint_command_degrees(
        self, joint_positions_rad: Iterable[float], external_axis: float = 0.0
    ):
        joints = _as_float_list(joint_positions_rad, name="joint_positions_rad")
        if len(joints) != NUM_JOINTS:
            raise ValueError(f"joint_positions_rad must contain {NUM_JOINTS} values")
        if not math.isfinite(float(external_axis)):
            raise ValueError("external_axis must be finite")
        return _sdk_vector(
            self._nrc, [math.degrees(value) for value in joints] + [float(external_axis)]
        )

    def send_joint_positions(
        self,
        joint_positions_rad: Iterable[float],
        *,
        external_axis: float = 0.0,
        require_servoj: bool = True,
    ) -> None:
        with self._lock:
            if require_servoj and not self._servo_active:
                raise NRCCommandError("set_servoJ_pos (ServoJ is not active)", "inactive")
            self._call(
                "set_servoJ_pos",
                self._nrc.set_servoJ_pos,
                self._fd(TRACK_PORT),
                self._joint_command_degrees(joint_positions_rad, external_axis),
            )

    send_servoj_position = send_joint_positions

    def get_joint_positions(self) -> list[float]:
        with self._lock:
            output = _sdk_vector(self._nrc, [])
            result = self._call(
                "get_current_position(joint)",
                self._nrc.get_current_position,
                self._fd(TEACH_PORT),
                0,
                output,
            )
            values = _vector_values(_output_from_result(result, output))
            if len(values) < NUM_JOINTS:
                raise NRCError(f"Expected at least 6 joint values, received {values!r}")
            return [math.radians(value) for value in values[:NUM_JOINTS]]

    get_joint_positions_rad = get_joint_positions

    def get_cr3_o6_state(self, hand_state: Iterable[float]) -> list[float]:
        hand = _as_float_list(hand_state, name="hand_state")
        if len(hand) != HAND_DIM:
            raise ValueError(f"hand_state must contain {HAND_DIM} values")
        return self.get_joint_positions() + hand



HAND_DIM = 6
NRCInterface = CR3Interface

__all__ = [
    "CR3Interface",
    "NRCCommandError",
    "NRCConnectionError",
    "NRCError",
    "NRCInterface",
    "SERVO_ERROR",
    "SERVO_READY",
    "SERVO_RUNNING",
    "SERVO_STOPPED",
]
