"""Run continuous CR3/O6 execution against the Customer HTTPS inference API.

``--execute-live --confirm-motion`` homes both devices, continuously exchanges
observations and actions with the remote service, and attempts to return home
on exit. Remote ``mode=shadow`` does not gate explicitly confirmed local motion.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import importlib.util
import logging
import math
from pathlib import Path
import queue
import sys
import threading
import time
from typing import Any, Callable
from zoneinfo import ZoneInfo

import numpy as np

from cr3_o6_settings import load_settings


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = SCRIPT_DIR / "config.toml"
CUSTOMER_CLIENT_PATH = (
    SCRIPT_DIR / "customer-integration-kit-20261019" / "scripts" / "customer_client.py"
)
CUSTOMER_CAMERAS = ("top", "wrist", "side")
LOCAL_CAMERAS = ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb")
STATE_DIM = 12
ACTION_SHAPE = (16, 12)
ACTION_DT_S = 0.05
LIVE_MAX_OBSERVATION_AGE_S = 3.0
LIVE_DRIFT_LOG_ARM_RAD = 0.02
LIVE_DRIFT_LOG_HAND_RAW = 5.0
HAND_CONTROL_DT_S = 0.1
CONTROL_SCHEDULE_TOLERANCE_S = 0.02
OPENPI_INITIAL_HAND_POSITION = (157, 95, 175, 0, 0, 0)
INITIAL_ARM_POSITION = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
HAND_HOME_STEP_RAW = 6
HAND_HOME_DT_S = 0.1
HAND_HOME_TRACKING_TOLERANCE_RAW = 5
ARM_HOME_DT_S = 0.05
ARM_HOME_MIN_DURATION_S = 3.0
ARM_HOME_MAX_STEP_RAD = 0.04
ARM_HOME_TRACKING_TOLERANCE_RAD = 0.03
ARM_HOME_SETTLE_TIMEOUT_S = 5.0
ARM_HOME_STABLE_SAMPLES = 5
LIVE_ARM_MAX_STEP_RAD = 0.04
LIVE_ARM_MAX_TRACKING_ERROR_RAD = 0.2
LIVE_ARM_ABSOLUTE_LIMIT_RAD = 2 * math.pi
LOGGER = logging.getLogger("cr3_o6_customer")


def _load_customer_client():
    if not CUSTOMER_CLIENT_PATH.is_file():
        raise RuntimeError(f"Customer SDK not found: {CUSTOMER_CLIENT_PATH}")
    spec = importlib.util.spec_from_file_location("customer_client", CUSTOMER_CLIENT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load Customer SDK: {CUSTOMER_CLIENT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.CustomerPolicyClient


def _load_hand_config(path: Path) -> str:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("读取 LinkerHand setting.yaml 需要 PyYAML") from exc

    try:
        config = yaml.safe_load(path.expanduser().read_text()) or {}
        right = config["LINKER_HAND"]["RIGHT_HAND"]
        joint = str(right.get("JOINT", "")).upper()
        modbus = right.get("MODBUS")
    except (OSError, KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"无法读取 LinkerHand 配置 {path}: {exc}") from exc
    if joint != "O6":
        raise RuntimeError(f"右手配置必须是 O6，当前是 {joint!r}: {path}")
    if not modbus or str(modbus).lower() == "none":
        raise RuntimeError(f"当前右手不是 RS485 配置，请检查 {path} 的 RIGHT_HAND.MODBUS")
    return str(modbus)


def _load_hand_api():
    try:
        from LinkerHand.linker_hand_api import LinkerHandApi
    except ImportError as exc:
        raise RuntimeError(
            "LinkerHand SDK import failed. Set PYTHONPATH to the linkerhand-python-sdk tree."
        ) from exc
    return LinkerHandApi


def _close_hand(hand_api: Any) -> None:
    backend = getattr(hand_api, "hand", None)
    close = getattr(backend, "close", None)
    if callable(close):
        try:
            close()
        except Exception:
            LOGGER.exception("Failed to close LinkerHand backend")


def _finite_vector(values: Any, length: int, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float32)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise RuntimeError(f"{name} must be finite and have shape ({length},), got {result.shape}")
    return result


def _parse_camera_mapping(args: argparse.Namespace) -> dict[str, str]:
    mapping = {
        "top": args.top_camera,
        "wrist": args.wrist_camera,
        "side": args.side_camera,
    }
    if set(mapping.values()) != set(LOCAL_CAMERAS):
        raise SystemExit(
            "[cameras] in config.toml must be a one-to-one mapping "
            f"of {LOCAL_CAMERAS}"
        )
    return mapping


def _read_observation(
    robot: Any,
    hand: Any,
    cameras: Any,
    camera_mapping: dict[str, str],
    *,
    report_mapping: bool = False,
):
    hand_state = _finite_vector(hand.get_state(), 6, "O6 state")
    if np.any((hand_state < 0) | (hand_state > 255)):
        raise RuntimeError(f"O6 state outside [0, 255]: {hand_state.tolist()}")
    state = _finite_vector(robot.get_cr3_o6_state(hand_state.tolist()), STATE_DIM, "CR3/O6 state")
    raw_unused_state = state[9:].copy()
    state[9:] = 0.0
    if report_mapping:
        LOGGER.info(
            "O6 state mapping: device_channels_9_11=%s uploaded_channels_9_11=%s "
            "(fixed-zero training/API channels)",
            raw_unused_state.tolist(),
            state[9:].tolist(),
        )

    local_images = cameras.read()
    images = {}
    for customer_name, local_name in camera_mapping.items():
        image = np.asarray(local_images[local_name])
        if image.shape != (224, 224, 3) or image.dtype != np.uint8:
            raise RuntimeError(
                f"Camera {local_name} returned {image.shape}/{image.dtype}; "
                "expected RGB uint8 (224, 224, 3)"
            )
        images[customer_name] = np.ascontiguousarray(image)
    return images, state


def _check_metadata(configuration: dict[str, Any], api_url: str) -> None:
    required = {
        "ready",
        "expires_utc",
        "cameras",
        "image_shape",
        "state_names",
        "action_names",
        "action_shape",
        "fixed_zero_channels",
        "dt_seconds",
        "mode",
        "robot_execution_enabled",
    }
    missing = sorted(required - set(configuration))
    if missing:
        raise RuntimeError(f"Customer metadata is missing fields: {missing}")
    if configuration["ready"] is not True:
        raise RuntimeError("Customer service is not ready")
    if configuration["cameras"] != list(CUSTOMER_CAMERAS):
        raise RuntimeError(f"Unexpected Customer cameras: {configuration['cameras']!r}")
    if configuration["image_shape"] != [224, 224, 3]:
        raise RuntimeError(f"Unexpected Customer image shape: {configuration['image_shape']!r}")
    if configuration["action_shape"] != list(ACTION_SHAPE):
        raise RuntimeError(f"Unexpected Customer action shape: {configuration['action_shape']!r}")
    if configuration["fixed_zero_channels"] != [9, 10, 11]:
        raise RuntimeError(
            f"Unexpected fixed-zero channels: {configuration['fixed_zero_channels']!r}"
        )
    if configuration["dt_seconds"] != 0.05:
        raise RuntimeError(f"Unexpected action dt_seconds: {configuration['dt_seconds']!r}")

    try:
        expires = datetime.fromisoformat(configuration["expires_utc"].replace("Z", "+00:00"))
    except (AttributeError, TypeError, ValueError) as exc:
        raise RuntimeError("Customer metadata has an invalid expires_utc") from exc
    if expires.timestamp() <= time.time():
        raise RuntimeError(f"Customer API session expired at {expires.isoformat()}")
    LOGGER.info(
        "Remote configuration: url=%s contract_version=%s ready=%s mode=%s "
        "robot_execution_enabled=%s expires_utc=%s expires_beijing=%s "
        "cameras=%s action_shape=%s dt_seconds=%s",
        api_url,
        configuration["contract_version"],
        configuration["ready"],
        configuration["mode"],
        configuration["robot_execution_enabled"],
        expires.isoformat(),
        expires.astimezone(ZoneInfo("Asia/Shanghai")).isoformat(),
        configuration["cameras"],
        configuration["action_shape"],
        configuration["dt_seconds"],
    )


def _home_hand_before_motion(hand: Any) -> None:
    """Move all six physical O6 joints to the OpenPI start pose before observing."""
    initial = _finite_vector(hand.get_state(), 6, "O6 position before homing")
    if np.any((initial < 0) | (initial > 255)) or not np.allclose(
        initial, np.rint(initial), atol=1e-4, rtol=0
    ):
        raise RuntimeError("O6 position before homing is not a valid raw setpoint")
    target = np.asarray(OPENPI_INITIAL_HAND_POSITION, dtype=np.float32)
    delta = target - initial
    steps = int(np.ceil(np.max(np.abs(delta)) / HAND_HOME_STEP_RAW))
    if steps == 0:
        LOGGER.info("O6 already at OpenPI initial position: %s", initial.tolist())
        return
    LOGGER.warning(
        "Homing all six physical O6 channels: measured=%s target=%s "
        "steps=%d at 10Hz; this sends real motor commands (including inactive fingers)",
        initial.tolist(),
        list(OPENPI_INITIAL_HAND_POSITION),
        steps,
    )
    started = time.monotonic()
    previous = initial.copy()
    for index in range(steps):
        deadline = started + index * HAND_HOME_DT_S
        delay = deadline - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        lateness = time.monotonic() - deadline
        if lateness > CONTROL_SCHEDULE_TOLERANCE_S:
            raise RuntimeError(f"O6 homing schedule missed by {lateness * 1000:.1f}ms")
        command = np.rint(initial + delta * ((index + 1) / steps)).astype(np.int32)
        if np.max(np.abs(command - previous)) > HAND_HOME_STEP_RAW:
            raise RuntimeError("Generated O6 homing step exceeds the raw position limit")
        previous = command.astype(np.float32)
        sent_at = time.monotonic()
        hand.finger_move(command.tolist())
        elapsed = time.monotonic() - sent_at
        LOGGER.info(
            "O6 homing step %d/%d: target=%s write=%.1fms",
            index + 1,
            steps,
            command.tolist(),
            elapsed * 1000,
        )
        if elapsed > HAND_HOME_DT_S:
            raise RuntimeError(f"O6 homing write took {elapsed * 1000:.1f}ms, over 100ms")
    time.sleep(HAND_HOME_DT_S)
    final = _finite_vector(hand.get_state(), 6, "O6 position after homing")
    if np.max(np.abs(final - target)) > HAND_HOME_TRACKING_TOLERANCE_RAW:
        raise RuntimeError(
            f"O6 failed to reach initial position: measured={final.tolist()} "
            f"target={target.tolist()}"
        )
    LOGGER.warning("O6 homing complete: measured=%s", final.tolist())


def _home_arm(robot: Any, servo: _LiveServoKeeper | None = None) -> None:
    """Return CR3 to zero, reusing the live ServoJ session when one is active."""
    from cr3_o6_interface import SERVO_RUNNING

    if robot.get_servo_state() != SERVO_RUNNING:
        raise RuntimeError("CR3 servo is not running; cannot return to the initial pose")
    initial = _finite_vector(robot.get_joint_positions(), 6, "CR3 position before homing")
    target = np.asarray(INITIAL_ARM_POSITION, dtype=np.float32)
    start_target = initial
    opened = False
    chunk_started = False
    try:
        if servo is not None:
            start_target = servo.begin_chunk()
            chunk_started = True
            if np.max(np.abs(start_target - initial)) > LIVE_ARM_MAX_TRACKING_ERROR_RAD:
                raise RuntimeError("CR3 hold target differs too far from measured position")
        distance = float(np.max(np.abs(target - start_target)))
        if distance <= ARM_HOME_TRACKING_TOLERANCE_RAD and np.max(
            np.abs(target - initial)
        ) <= ARM_HOME_TRACKING_TOLERANCE_RAD:
            LOGGER.info("CR3 already at zero joint pose: %s", initial.tolist())
            return
        steps = max(
            math.ceil(ARM_HOME_MIN_DURATION_S / ARM_HOME_DT_S),
            math.ceil(distance / ARM_HOME_MAX_STEP_RAD),
        )
        LOGGER.warning(
            "Returning CR3 to zero joint pose: measured=%s last_target=%s steps=%d",
            initial.tolist(),
            start_target.tolist(),
            steps,
        )
        if servo is None:
            robot.open_servoj()
            opened = True
        else:
            # Keep the current target fresh while switching from inference to homing.
            servo.send_chunk_target(start_target)
        started = time.monotonic() + ARM_HOME_DT_S
        for index in range(steps):
            deadline = started + index * ARM_HOME_DT_S
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            lateness = time.monotonic() - deadline
            if lateness > CONTROL_SCHEDULE_TOLERANCE_S:
                raise RuntimeError(f"CR3 homing schedule missed by {lateness * 1000:.1f}ms")
            if robot.get_servo_state() != SERVO_RUNNING:
                raise RuntimeError("CR3 servo stopped during return to zero")
            command = start_target + (target - start_target) * ((index + 1) / steps)
            current = _finite_vector(robot.get_joint_positions(), 6, "CR3 homing feedback")
            if np.max(np.abs(command - current)) > LIVE_ARM_MAX_TRACKING_ERROR_RAD:
                raise RuntimeError("CR3 is not tracking the return-to-zero trajectory")
            if servo is None:
                robot.send_joint_positions(command.tolist())
            else:
                servo.send_chunk_target(command)
        # A ServoJ target being accepted is not proof that the joints have arrived.
        # Continue streaming zero while waiting for measured feedback to settle.
        settled = 0
        settle_started = time.monotonic()
        settle_deadline = settle_started + ARM_HOME_SETTLE_TIMEOUT_S
        while True:
            if robot.get_servo_state() != SERVO_RUNNING:
                raise RuntimeError("CR3 servo stopped while settling at zero")
            current = _finite_vector(robot.get_joint_positions(), 6, "CR3 homing settle feedback")
            error = float(np.max(np.abs(current - target)))
            settled = settled + 1 if error <= ARM_HOME_TRACKING_TOLERANCE_RAD else 0
            if settled >= ARM_HOME_STABLE_SAMPLES:
                break
            if time.monotonic() >= settle_deadline:
                raise RuntimeError(
                    f"CR3 did not settle at zero within {ARM_HOME_SETTLE_TIMEOUT_S:.1f}s: "
                    f"measured={current.tolist()} error={error:.4f}rad"
                )
            if servo is None:
                robot.send_joint_positions(target.tolist())
            else:
                servo.send_chunk_target(target)
            time.sleep(ARM_HOME_DT_S)
    finally:
        try:
            if opened:
                robot.close_servoj()
        finally:
            if chunk_started:
                servo.end_chunk()
    if robot.get_servo_state() != SERVO_RUNNING:
        raise RuntimeError(
            "CR3 servo stopped after homing"
            if servo is not None
            else "CR3 servo stopped when ServoJ closed after homing"
        )
    final = _finite_vector(robot.get_joint_positions(), 6, "CR3 position after homing")
    if np.max(np.abs(final - target)) > ARM_HOME_TRACKING_TOLERANCE_RAD:
        raise RuntimeError(f"CR3 did not reach zero joint pose: measured={final.tolist()}")
    LOGGER.warning(
        "CR3 return to zero complete: measured=%s settle_seconds=%.3f",
        final.tolist(),
        time.monotonic() - settle_started,
    )


def _log_cr3_servo_state(robot: Any, phase: str) -> int | None:
    """Record the servo state without hiding an earlier shutdown failure."""
    from cr3_o6_interface import SERVO_ERROR

    try:
        state = robot.get_servo_state()
    except (OSError, RuntimeError, ValueError) as exc:
        LOGGER.error("Could not read CR3 servo state %s: %s", phase, exc)
        return None
    if state == SERVO_ERROR:
        LOGGER.error("CR3 servo is in ERROR state (2) %s; inspect the controller alarm", phase)
    else:
        LOGGER.info("CR3 servo state %s: %s", phase, state)
    return state


def _live_hand_commands(actions: np.ndarray, initial_hand: np.ndarray) -> list[list[int]]:
    """Track the three modeled fingers at 10 Hz; the other three remain at zero."""
    previous = np.rint(_finite_vector(initial_hand, 6, "O6 chunk start")).astype(np.float32)
    commands = []
    for action in actions[::2]:
        target = np.concatenate((action[6:9], np.zeros(3, dtype=np.float32)))
        previous += np.clip(target - previous, -HAND_HOME_STEP_RAW, HAND_HOME_STEP_RAW)
        commands.append(np.rint(previous).astype(np.int32).tolist())
    return commands


class _LiveServoKeeper:
    """Own ServoJ for a whole live session and hold the last target between chunks."""

    def __init__(self, robot: Any):
        self.robot = robot
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._target: np.ndarray | None = None
        self._chunk_active = False
        self._failure: BaseException | None = None
        self._closed_by_worker = False
        self._holding_since = 0.0
        self._chunks_started = 0

    def start(self) -> None:
        from cr3_o6_interface import SERVO_RUNNING

        if self._thread is not None or self._stop.is_set():
            raise RuntimeError("Live ServoJ keeper cannot be restarted")
        if self.robot.get_servo_state() != SERVO_RUNNING:
            raise RuntimeError("CR3 servo is not running")
        self._target = _finite_vector(self.robot.get_joint_positions(), 6, "CR3 hold start")
        opened = False
        try:
            self.robot.open_servoj()
            opened = True
            thread = threading.Thread(
                target=self._hold_loop, name="cr3-live-servoj-hold", daemon=True
            )
            thread.start()
            self._thread = thread
            self._holding_since = time.monotonic()
        except BaseException:
            if opened:
                self.robot.close_servoj()
            raise
        LOGGER.info("Live ServoJ opened; holding the last CR3 target between remote chunks")

    def _hold_loop(self) -> None:
        from cr3_o6_interface import SERVO_RUNNING

        next_tick = time.monotonic()
        try:
            while not self._stop.wait(max(0.0, next_tick - time.monotonic())):
                with self._lock:
                    if self._stop.is_set():
                        return
                    if not self._chunk_active:
                        if self.robot.get_servo_state() != SERVO_RUNNING:
                            raise RuntimeError("CR3 servo stopped while awaiting remote inference")
                        assert self._target is not None
                        self.robot.send_joint_positions(self._target.tolist())
                next_tick = max(next_tick + ACTION_DT_S, time.monotonic() + ACTION_DT_S)
        except BaseException as exc:
            with self._lock:
                self._failure = exc
                self._stop.set()
                try:
                    self.robot.close_servoj()
                    self._closed_by_worker = True
                except Exception:
                    LOGGER.exception("Failed to close ServoJ after hold-loop failure")
            LOGGER.error("Live ServoJ hold failed: %s", exc)

    def raise_if_failed(self) -> None:
        with self._lock:
            if self._failure is not None:
                raise RuntimeError(f"Live ServoJ hold failed: {self._failure}") from self._failure

    def begin_chunk(self) -> np.ndarray:
        with self._lock:
            self.raise_if_failed()
            if self._thread is None or self._stop.is_set() or self._chunk_active:
                raise RuntimeError("Live ServoJ keeper is not ready for a chunk")
            if self._chunks_started:
                LOGGER.info(
                    "CR3 held its last target for %.3fs while awaiting the next action",
                    time.monotonic() - self._holding_since,
                )
            self._chunks_started += 1
            self._chunk_active = True
            assert self._target is not None
            return self._target.copy()

    def send_chunk_target(self, target: np.ndarray) -> None:
        with self._lock:
            self.raise_if_failed()
            if not self._chunk_active:
                raise RuntimeError("CR3 chunk command sent outside an active chunk")
            self.robot.send_joint_positions(target.tolist())
            self._target = target.copy()

    def end_chunk(self) -> None:
        with self._lock:
            self._chunk_active = False
            self._holding_since = time.monotonic()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
            if not self._closed_by_worker:
                self.robot.close_servoj()
            LOGGER.info("Live ServoJ closed at session end")


class _SynchronizedHand:
    """Serialize observation reads and motor writes on the single O6 Modbus bus."""

    def __init__(self, hand: Any):
        self._hand = hand
        self._lock = threading.Lock()

    def get_state(self) -> Any:
        with self._lock:
            return self._hand.get_state()

    def finger_move(self, target: list[int]) -> None:
        with self._lock:
            self._hand.finger_move(target)

    def finger_move_if_on_time(self, target: list[int], deadline: float) -> bool:
        with self._lock:
            if time.monotonic() - deadline > CONTROL_SCHEDULE_TOLERANCE_S:
                return False
            self._hand.finger_move(target)
            return True


def _execute_live_chunk(
    robot: Any,
    hand: Any,
    actions: np.ndarray,
    observed_state: np.ndarray,
    observed_at: float,
    servo: _LiveServoKeeper,
    *,
    on_step: Callable[[int], None] | None = None,
    cancel: threading.Event | None = None,
) -> None:
    """Execute one fresh remote chunk, with bounded command speed but no task envelope."""
    from cr3_o6_interface import SERVO_RUNNING

    if actions.shape != ACTION_SHAPE or not np.all(np.isfinite(actions)):
        raise RuntimeError("Remote actions must be a finite (16, 12) array")
    if time.monotonic() - observed_at > LIVE_MAX_OBSERVATION_AGE_S:
        raise RuntimeError("Remote action chunk is stale; no motion sent")
    if robot.get_servo_state() != SERVO_RUNNING:
        raise RuntimeError("CR3 servo is not running")
    initial_arm = _finite_vector(robot.get_joint_positions(), 6, "CR3 chunk start")
    initial_hand = _finite_vector(hand.get_state(), 6, "O6 chunk start")
    observation_arm_drift = float(np.max(np.abs(initial_arm - observed_state[:6])))
    observation_hand_drift = float(np.max(np.abs(initial_hand[:3] - observed_state[6:9])))
    if observation_arm_drift > LIVE_DRIFT_LOG_ARM_RAD or (
        observation_hand_drift > LIVE_DRIFT_LOG_HAND_RAW
    ):
        LOGGER.info(
            "Live observation-to-execution drift: CR3=%.4frad O6=%.1f raw; "
            "bridging from current measured position",
            observation_arm_drift,
            observation_hand_drift,
        )
    if np.any((initial_hand < 0) | (initial_hand > 255)):
        raise RuntimeError("O6 state is outside the physical raw range")
    if np.max(np.abs(actions[:, :6])) > LIVE_ARM_ABSOLUTE_LIMIT_RAD:
        raise RuntimeError("Predicted CR3 joint position exceeds the absolute input limit")
    if np.any((actions[:, 6:9] < 0) | (actions[:, 6:9] > 255)):
        raise RuntimeError("Predicted O6 raw position is outside [0, 255]")
    if np.max(np.abs(actions[:, 9:])) > 1e-4:
        raise RuntimeError("Predicted O6 padding channels are not zero")
    hand_commands = _live_hand_commands(actions, initial_hand)
    hand_targets = np.asarray(hand_commands, dtype=np.float32)
    hand_steps = np.diff(np.vstack((initial_hand, hand_targets)), axis=0)
    if hand_targets.shape != (len(actions) // 2, 6) or np.max(np.abs(hand_steps)) > (
        HAND_HOME_STEP_RAW + 0.5
    ):
        raise RuntimeError("Generated O6 live commands exceed the step limit")
    if np.any((hand_targets < 0) | (hand_targets > 255)):
        raise RuntimeError("Generated O6 live commands are outside [0, 255]")
    if time.monotonic() - observed_at > LIVE_MAX_OBSERVATION_AGE_S:
        raise RuntimeError("Remote action chunk became stale during preflight")

    stop_event = threading.Event()
    ready_event = threading.Event()
    start_event = threading.Event()
    hand_errors: list[BaseException] = []
    hand_commands_sent = 0
    arm_commands_sent = 0
    start_at = 0.0

    def raise_hand_error() -> None:
        if hand_errors:
            error = hand_errors[0]
            raise RuntimeError(f"O6 live control failed: {type(error).__name__}: {error}") from error

    def hand_worker() -> None:
        nonlocal hand_commands_sent
        ready_event.set()
        start_event.wait()
        if stop_event.is_set():
            return
        last_hand_command = np.rint(initial_hand).astype(np.int32)
        try:
            for index, target in enumerate(hand_commands):
                if cancel is not None and cancel.is_set():
                    return
                deadline = start_at + index * HAND_CONTROL_DT_S
                if stop_event.wait(max(0.0, deadline - time.monotonic())):
                    return
                if cancel is not None and cancel.is_set():
                    return
                lateness = time.monotonic() - deadline
                if lateness > CONTROL_SCHEDULE_TOLERANCE_S:
                    if isinstance(hand, _SynchronizedHand):
                        LOGGER.debug(
                            "Skipping delayed O6 target %d (%.1fms late)",
                            index,
                            lateness * 1000,
                        )
                        continue
                    raise RuntimeError(f"O6 10Hz deadline missed by {lateness * 1000:.1f}ms")
                sent_at = time.monotonic()
                if isinstance(hand, _SynchronizedHand):
                    bounded = last_hand_command + np.clip(
                        np.asarray(target) - last_hand_command,
                        -HAND_HOME_STEP_RAW,
                        HAND_HOME_STEP_RAW,
                    )
                    if not hand.finger_move_if_on_time(bounded.tolist(), deadline):
                        LOGGER.debug("Skipping O6 target %d; bus busy with observation", index)
                        continue
                    last_hand_command = bounded
                else:
                    hand.finger_move(target)
                hand_commands_sent += 1
                if time.monotonic() - sent_at > HAND_CONTROL_DT_S:
                    raise RuntimeError("O6 write exceeded its 100ms control period")
        except BaseException as exc:
            hand_errors.append(exc)
            stop_event.set()

    worker = threading.Thread(target=hand_worker, name="cr3-o6-live-hand", daemon=True)
    worker_started = False
    chunk_started = False
    try:
        last_arm_command = servo.begin_chunk()
        chunk_started = True
        worker.start()
        worker_started = True
        if not ready_event.wait(1.0):
            raise RuntimeError("O6 live control worker did not start")
        if time.monotonic() - observed_at > LIVE_MAX_OBSERVATION_AGE_S:
            raise RuntimeError("Remote action chunk became stale before first command")
        start_at = time.monotonic() + ACTION_DT_S
        start_event.set()
        for index, action in enumerate(actions):
            if cancel is not None and cancel.is_set():
                raise RuntimeError("Live control cancelled")
            deadline = start_at + index * ACTION_DT_S
            if stop_event.wait(max(0.0, deadline - time.monotonic())):
                raise_hand_error()
                raise RuntimeError("Live chunk stopped before CR3 command")
            if cancel is not None and cancel.is_set():
                raise RuntimeError("Live control cancelled")
            lateness = time.monotonic() - deadline
            if lateness > CONTROL_SCHEDULE_TOLERANCE_S:
                raise RuntimeError(f"CR3 20Hz deadline missed by {lateness * 1000:.1f}ms")
            raise_hand_error()
            if robot.get_servo_state() != SERVO_RUNNING:
                raise RuntimeError("CR3 servo stopped during live control")
            current = _finite_vector(robot.get_joint_positions(), 6, "CR3 live feedback")
            feedback_target = current + np.clip(
                action[:6] - current, -LIVE_ARM_MAX_STEP_RAD, LIVE_ARM_MAX_STEP_RAD
            )
            command = last_arm_command + np.clip(
                feedback_target - last_arm_command,
                -LIVE_ARM_MAX_STEP_RAD,
                LIVE_ARM_MAX_STEP_RAD,
            )
            if np.max(np.abs(command - current)) > LIVE_ARM_MAX_TRACKING_ERROR_RAD:
                raise RuntimeError("CR3 live command exceeds the tracking-error limit")
            servo.send_chunk_target(command)
            last_arm_command = command
            arm_commands_sent += 1
            if on_step is not None:
                on_step(index)
        worker.join()
        raise_hand_error()
        servo.raise_if_failed()
    except BaseException:
        LOGGER.error(
            "Live chunk interrupted after arm_commands=%d hand_commands=%d; ending session",
            arm_commands_sent,
            hand_commands_sent,
        )
        raise
    finally:
        stop_event.set()
        start_event.set()
        try:
            if chunk_started:
                servo.end_chunk()
        finally:
            if worker_started:
                worker.join()
    final_arm = _finite_vector(robot.get_joint_positions(), 6, "CR3 after live chunk")
    final_hand = _finite_vector(hand.get_state(), 6, "O6 after live chunk")
    LOGGER.info(
        "Live chunk completed: arm_commands=%d hand_commands=%d final_arm=%s final_hand=%s",
        arm_commands_sent,
        hand_commands_sent,
        np.round(final_arm, 4).tolist(),
        final_hand.tolist(),
    )


def _run_inference_loop(
    args: argparse.Namespace,
    customer: Any,
    robot: Any,
    hand: Any,
    cameras: Any,
    camera_mapping: dict[str, str],
    retain_servo: Callable[[_LiveServoKeeper], None] | None = None,
) -> None:
    servo = _LiveServoKeeper(robot)
    retained = False
    try:
        servo.start()
        if retain_servo is not None:
            retain_servo(servo)
            retained = True
        _run_live_inference_pipeline(args, customer, robot, hand, cameras, camera_mapping, servo)
    finally:
        if not retained:
            try:
                servo.stop()
            finally:
                _log_cr3_servo_state(robot, "after closing live ServoJ")


def _run_live_inference_pipeline(
    args: argparse.Namespace,
    customer: Any,
    robot: Any,
    hand: Any,
    cameras: Any,
    camera_mapping: dict[str, str],
    servo: _LiveServoKeeper,
) -> None:
    """Request fresh predictions while a separate controller streams the prior trajectory."""
    started = time.monotonic()
    synchronized_hand = _SynchronizedHand(hand)
    pending: queue.Queue[tuple[np.ndarray, np.ndarray, float, dict[str, Any], threading.Event]] = (
        queue.Queue(maxsize=1)
    )
    stop = threading.Event()
    controller_errors: list[BaseException] = []
    waiting_for_motion: threading.Event | None = None
    request_count = 0

    def check_controller() -> None:
        servo.raise_if_failed()
        if controller_errors:
            error = controller_errors[0]
            raise RuntimeError(f"Continuous controller failed: {error}") from error

    def control_worker() -> None:
        motion_started: threading.Event | None = None
        try:
            while not stop.is_set():
                try:
                    actions, state, observed_at, info, motion_started = pending.get(timeout=0.05)
                except queue.Empty:
                    continue
                if stop.is_set():
                    motion_started.set()
                    return
                _execute_live_chunk(
                    robot,
                    synchronized_hand,
                    actions,
                    state,
                    observed_at,
                    servo,
                    on_step=lambda index: motion_started.set() if index == 2 else None,
                    cancel=stop,
                )
                LOGGER.info(
                    "Continuous trajectory applied: request_id=%s; awaiting fresh targets",
                    info.get("request_id"),
                )
                motion_started.set()
        except BaseException as exc:
            if (
                stop.is_set()
                and isinstance(exc, RuntimeError)
                and str(exc) == "Live control cancelled"
            ):
                return
            controller_errors.append(exc)
            stop.set()
            if motion_started is not None:
                # Wake the inference loop even if execution failed before the third step.
                motion_started.set()

    worker = threading.Thread(target=control_worker, name="cr3-live-controller", daemon=True)
    worker.start()
    try:
        while not stop.is_set():
            check_controller()
            if args.duration_s and time.monotonic() - started >= args.duration_s:
                break
            if waiting_for_motion is not None:
                while not waiting_for_motion.wait(0.05):
                    check_controller()
                check_controller()
            capture_started = time.monotonic()
            images, state = _read_observation(
                robot,
                synchronized_hand,
                cameras,
                camera_mapping,
                report_mapping=request_count == 0,
            )
            observed_at = time.monotonic()
            check_controller()
            actions, info = customer.infer(images, state)
            check_controller()
            if actions.shape != ACTION_SHAPE:
                raise RuntimeError(f"Customer action shape changed: {actions.shape}")
            inference_age = time.monotonic() - observed_at
            if inference_age > LIVE_MAX_OBSERVATION_AGE_S:
                raise RuntimeError(
                    f"Remote trajectory too old ({inference_age:.2f}s > "
                    f"{LIVE_MAX_OBSERVATION_AGE_S:.2f}s); no motion sent"
                )
            request_count += 1
            LOGGER.info(
                "Remote inference #%d: request_id=%s mode=%s action_shape=%s "
                "first_action=%s server_elapsed_seconds=%s capture_and_infer_seconds=%.3f",
                request_count,
                info.get("request_id"),
                info.get("mode"),
                actions.shape,
                np.round(actions[0], 4).tolist(),
                info.get("server_elapsed_seconds"),
                time.monotonic() - capture_started,
            )
            waiting_for_motion = threading.Event()
            item = (actions.copy(), state.copy(), observed_at, info, waiting_for_motion)
            try:
                pending.put_nowait(item)
            except queue.Full:
                # The newest measured observation supersedes an unstarted prediction.
                try:
                    old = pending.get_nowait()
                except queue.Empty:
                    pass  # Controller took it between the full check and this read.
                else:
                    old[4].set()
                    LOGGER.warning("Discarded an unstarted trajectory for a newer observation")
                pending.put_nowait(item)
        check_controller()
    finally:
        stop.set()
        worker.join()
    check_controller()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument(
        "--execute-live",
        action="store_true",
        help="Home CR3/O6, execute remote action chunks continuously, and return home on exit",
    )
    parser.add_argument(
        "--confirm-motion",
        action="store_true",
        help="Required with --execute-live before sending any motor commands",
    )
    return parser


def main() -> int:
    parser = _parser()
    args = parser.parse_args()
    if not args.execute_live or not args.confirm_motion:
        parser.error("Continuous robot execution requires --execute-live --confirm-motion")
    if args.duration_s < 0:
        parser.error("--duration-s must be non-negative")
    try:
        settings = load_settings(args.config)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    vars(args).update(vars(settings))
    camera_mapping = _parse_camera_mapping(args)
    if str(args.linkerhand_sdk_dir) not in sys.path:
        sys.path.insert(0, str(args.linkerhand_sdk_dir))

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        force=True,
    )
    LOGGER.info("Customer camera mapping: %s", camera_mapping)

    CustomerPolicyClient = _load_customer_client()

    from cr3_o6_camera import CR3O6CameraInterface
    from cr3_o6_interface import CR3Interface, SERVO_ERROR, SERVO_RUNNING
    import linkerhand_compat

    linkerhand_compat.patch_pymodbus_slave_argument()
    LinkerHandApi = _load_hand_api()
    modbus = args.o6_modbus or _load_hand_config(args.hand_config)

    customer = CustomerPolicyClient(args.api_url, args.token_file, timeout=args.timeout_s)
    robot = CR3Interface(
        args.robot_ip,
        nrc_sdk_dir=args.nrc_sdk_dir,
        servo_vmax=[50.1] * 7,
        servo_amax=[100.0] * 7,
        servo_jmax=[200.0] * 7,
    )
    cameras = CR3O6CameraInterface()
    hand_api = None
    robot_connected = False
    startup_arm_home_started = False
    startup_arm_home_completed = False
    live_servo: _LiveServoKeeper | None = None
    exit_code = 0

    def retain_live_servo(servo: _LiveServoKeeper) -> None:
        nonlocal live_servo
        live_servo = servo

    try:
        configuration = customer.get_modality_config()
        _check_metadata(configuration, args.api_url)
        customer.reset()

        LOGGER.info("Connecting CR3 at %s", args.robot_ip)
        robot.connect()
        robot_connected = True
        initial_servo_state = robot.get_servo_state()
        if initial_servo_state == SERVO_ERROR:
            raise RuntimeError(
                "CR3 servo is already in ERROR state (2); inspect the controller alarm "
                "and recover it manually before starting live execution"
            )
        if initial_servo_state != SERVO_RUNNING:
            LOGGER.warning("CR3 servo is not running; powering on for live execution")
            robot.power_on()
            power_deadline = time.monotonic() + 10.0
            while robot.get_servo_state() != SERVO_RUNNING:
                if time.monotonic() >= power_deadline:
                    raise RuntimeError("CR3 did not reach the running servo state after power-on")
                time.sleep(0.2)
        LOGGER.info("Connecting right O6 over RS485 at %s", modbus)
        hand_api = LinkerHandApi(hand_type="right", hand_joint="O6", modbus=modbus, can="None")
        _home_hand_before_motion(hand_api)
        startup_arm_home_started = True
        _home_arm(robot)
        startup_arm_home_completed = True
        cameras.connect()

        _run_inference_loop(
            args, customer, robot, hand_api, cameras, camera_mapping, retain_live_servo
        )
    except KeyboardInterrupt:
        LOGGER.warning("Robot execution interrupted; returning to initial pose if available")
        exit_code = 130
    except (OSError, RuntimeError, ValueError) as exc:
        LOGGER.error("Customer session failed: %s", exc)
        exit_code = 1
    finally:
        if robot_connected:
            state_before_home = _log_cr3_servo_state(robot, "before exit homing")
            LOGGER.warning(
                "Live session ending; attempting CR3 return to zero and O6 return to %s if safe",
                OPENPI_INITIAL_HAND_POSITION,
            )
            arm_returned = False
            if startup_arm_home_started and not startup_arm_home_completed:
                LOGGER.error(
                    "Startup CR3 homing did not complete; not opening ServoJ again "
                    "during cleanup. Inspect the measured pose and controller alarm."
                )
                exit_code = 1
            elif state_before_home != SERVO_RUNNING:
                LOGGER.error(
                    "CR3 return to zero skipped: servo is not running (state=%s); "
                    "manual recovery is required",
                    state_before_home,
                )
                exit_code = 1
            else:
                try:
                    _home_arm(robot, servo=live_servo)
                    arm_returned = True
                except KeyboardInterrupt:
                    LOGGER.error("CR3 return interrupted; O6 return skipped to avoid releasing a load")
                    exit_code = 130
                except (OSError, RuntimeError, ValueError) as exc:
                    LOGGER.error("CR3 return to zero failed: %s", exc)
                    exit_code = 1
            state_after_home = _log_cr3_servo_state(robot, "after exit homing")
            if state_after_home != SERVO_RUNNING:
                arm_returned = False
                exit_code = 1
            if arm_returned and hand_api is not None:
                try:
                    _home_hand_before_motion(hand_api)
                except KeyboardInterrupt:
                    LOGGER.error("O6 return interrupted; some movement may have occurred")
                    exit_code = 130
                except (OSError, RuntimeError, ValueError) as exc:
                    LOGGER.error("O6 return to initial position failed: %s", exc)
                    exit_code = 1
            elif not arm_returned:
                LOGGER.error("O6 return skipped because safe CR3 homing could not be verified")
        if live_servo is not None:
            try:
                live_servo.stop()
            except (OSError, RuntimeError, ValueError) as exc:
                LOGGER.error("Could not close live ServoJ after exit homing: %s", exc)
                exit_code = 1
            if _log_cr3_servo_state(robot, "after closing live ServoJ") != SERVO_RUNNING:
                exit_code = 1
        try:
            cameras.disconnect()
        except Exception:
            LOGGER.exception("Could not disconnect cameras")
            exit_code = 1
        if hand_api is not None:
            _close_hand(hand_api)
        try:
            robot.disconnect()
        except Exception:
            LOGGER.exception("Could not disconnect CR3")
            exit_code = 1
        try:
            customer.close()
        except Exception:
            LOGGER.exception("Could not close Customer API client")
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
