"""Read observations and request predictions from Customer API v2.

Motion is opt-in and requires both ``--execute-live`` and ``--confirm-motion``.
"""

from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import importlib.util
import logging
import math
from pathlib import Path
import queue
import sys
import threading
import time
from typing import Any
from zoneinfo import ZoneInfo

import numpy as np

from cr3_o6_settings import load_settings


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR / "config-v2.toml"
SDK_PATH = SCRIPT_DIR / "customer-integration-kit-20261026" / "scripts" / "customer_client.py"
CAMERA_NAMES = ("top", "wrist", "side")
LOCAL_CAMERA_NAMES = ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb")
IMAGE_SHAPE = (480, 640, 3)
STATE_SHAPE = (12,)
ACTION_SHAPE = (16, 12)
ACTION_DT_S = 0.05
ACTION_HORIZON_S = ACTION_SHAPE[0] * ACTION_DT_S
ARM_MAX_STEP_RAD = 0.04
ARM_MAX_TRACKING_ERROR_RAD = 0.2
HAND_MAX_STEP_RAW = 6
HAND_DT_S = 0.1
CONTROL_TOLERANCE_S = 0.02
INITIAL_ARM_POSITION = np.zeros(6, dtype=np.float32)
INITIAL_HAND_POSITION = np.asarray((157, 40, 175, 175, 0, 0), dtype=np.float32)
LOGGER = logging.getLogger("cr3_o6_customer_v2")


def _load_sdk():
    if not SDK_PATH.is_file():
        raise RuntimeError(f"Customer v2 SDK not found: {SDK_PATH}")
    spec = importlib.util.spec_from_file_location("customer_client_v2", SDK_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load Customer v2 SDK: {SDK_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_hand_config(path: Path) -> str:
    try:
        import yaml

        config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        right = config["LINKER_HAND"]["RIGHT_HAND"]
        joint = str(right.get("JOINT", "")).upper()
        modbus = right.get("MODBUS")
    except (OSError, KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"Unable to read LinkerHand O6 config {path}: {exc}") from exc
    if joint != "O6":
        raise RuntimeError(f"Expected right O6 hand in {path}, got {joint!r}")
    if not modbus or str(modbus).lower() == "none":
        raise RuntimeError(f"Right O6 must use RS485 Modbus in {path}")
    return str(modbus)


def _load_expiry(metadata: dict[str, Any]) -> datetime:
    try:
        expires = datetime.fromisoformat(metadata["expires_utc"].replace("Z", "+00:00"))
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("Customer metadata has an invalid expires_utc") from exc
    if expires.tzinfo is None:
        raise RuntimeError("Customer metadata expires_utc must include a timezone")
    expires = expires.astimezone(timezone.utc)
    if expires <= datetime.now(timezone.utc):
        raise RuntimeError(f"Customer v2 session expired at {expires.isoformat()}")
    return expires


def _camera_mapping(settings) -> dict[str, str]:
    mapping = {
        "top": settings.top_camera,
        "wrist": settings.wrist_camera,
        "side": settings.side_camera,
    }
    if set(mapping) != set(CAMERA_NAMES) or set(mapping.values()) != set(LOCAL_CAMERA_NAMES):
        raise RuntimeError(
            f"Camera config must map top/wrist/side one-to-one to {LOCAL_CAMERA_NAMES}; "
            f"got {mapping}"
        )
    return mapping


def _read_observation(robot, hand, cameras, mapping):
    observation_started = time.monotonic()
    state_started = observation_started
    hand_state = np.asarray(hand.get_state(), dtype=np.float32)
    if hand_state.shape != (6,) or not np.isfinite(hand_state).all():
        raise RuntimeError(f"O6 state must contain six finite values; got {hand_state}")
    if np.any((hand_state < 0) | (hand_state > 255)):
        raise RuntimeError(f"O6 state outside raw range [0, 255]: {hand_state.tolist()}")

    state = np.asarray(robot.get_cr3_o6_state(hand_state.tolist()), dtype=np.float32)
    if state.shape != STATE_SHAPE or not np.isfinite(state).all():
        raise RuntimeError(f"CR3/O6 state must be 12 finite values; got shape {state.shape}")
    if np.any(state[10:] != 0):
        raise RuntimeError(
            "Customer v2 requires measured O6 channels 10-11 to be zero; "
            f"received {state[10:].tolist()}. No state channels were rewritten."
        )
    state_read_s = time.monotonic() - state_started

    camera_started = time.monotonic()
    local_images = cameras.read()
    camera_read_s = time.monotonic() - camera_started
    validation_started = time.monotonic()
    images = {}
    for api_name, local_name in mapping.items():
        image = np.asarray(local_images[local_name])
        if image.shape != IMAGE_SHAPE or image.dtype != np.uint8:
            raise RuntimeError(
                f"Camera {local_name} returned {image.shape}/{image.dtype}; "
                f"expected raw RGB uint8 {IMAGE_SHAPE} (no resize/crop/pad)"
            )
        images[api_name] = np.ascontiguousarray(image)

    timing = {
        "state_read_s": state_read_s,
        "camera_read_s": camera_read_s,
        "image_validation_s": time.monotonic() - validation_started,
        "observation_total_s": time.monotonic() - observation_started,
    }
    return images, state, timing


def _instrument_image_encoding(sdk):
    """Measure SDK PNG encoding without changing or storing image contents."""
    records = []
    original = sdk.encode_image

    def measured_encode(image):
        started = time.monotonic()
        payload = original(image)
        elapsed = time.monotonic() - started
        padding = len(payload) - len(payload.rstrip("="))
        png_bytes = len(payload) // 4 * 3 - padding
        records.append((elapsed, png_bytes))
        return payload

    sdk.encode_image = measured_encode
    return records


def _log_timing(observation_timing, encoding_records, infer_s, total_age_s, server_s):
    encoded_s = sum(elapsed for elapsed, _ in encoding_records)
    sizes = [size for _, size in encoding_records]
    nonserver_s = infer_s - encoded_s - (server_s or 0.0)
    LOGGER.info(
        "Latency breakdown: state_read=%.3fs camera_read=%.3fs image_validation=%.3fs "
        "observation_total=%.3fs png_encode_total=%.3fs png_encode_per_camera_s=%s "
        "png_bytes_per_camera=%s infer_wall=%.3fs infer_minus_encode_and_server=%.3fs "
        "server_elapsed=%s observation_to_response=%.3fs",
        observation_timing["state_read_s"],
        observation_timing["camera_read_s"],
        observation_timing["image_validation_s"],
        observation_timing["observation_total_s"],
        encoded_s,
        [round(elapsed, 4) for elapsed, _ in encoding_records],
        sizes,
        infer_s,
        nonserver_s,
        server_s,
        total_age_s,
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture real CR3/O6 observations and request Customer API v2 predictions. "
            "Hardware motion requires both --execute-live and --confirm-motion."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="v2 deployment TOML")
    parser.add_argument(
        "--task",
        choices=("two_finger", "three_finger"),
        default="three_finger",
        help="SDK task preset (default: three_finger)",
    )
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="keep requesting fresh observation/prediction cycles until Ctrl-C",
    )
    parser.add_argument(
        "--execute-live",
        action="store_true",
        help="enable real CR3/O6 motion; homes devices at startup and attempts return on exit",
    )
    parser.add_argument(
        "--confirm-motion",
        action="store_true",
        help="required with --execute-live before any motor command is sent",
    )
    return parser.parse_args()


class _SynchronizedHand:
    """Serialize O6 Modbus reads and writes on the shared serial connection."""

    def __init__(self, hand):
        self.hand = hand
        self.lock = threading.Lock()

    def get_state(self):
        with self.lock:
            return self.hand.get_state()

    def finger_move(self, target):
        with self.lock:
            self.hand.finger_move(target)


class _ServoKeeper:
    """Keep ServoJ open for the session and hold its latest target between chunks."""

    def __init__(self, robot):
        self.robot = robot
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.target = None
        self.chunk_active = False
        self.failure = None

    def start(self):
        from cr3_o6_interface import SERVO_RUNNING

        if self.robot.get_servo_state() != SERVO_RUNNING:
            raise RuntimeError("CR3 servo is not running; refusing to open ServoJ")
        self.target = np.asarray(self.robot.get_joint_positions(), dtype=np.float32)
        self.robot.open_servoj()
        self.thread = threading.Thread(target=self._hold_loop, name="cr3-v2-servo-hold", daemon=True)
        self.thread.start()
        LOGGER.info("CR3 ServoJ opened for the live session")

    def _hold_loop(self):
        from cr3_o6_interface import SERVO_RUNNING

        deadline = time.monotonic()
        try:
            while not self.stop_event.wait(max(0.0, deadline - time.monotonic())):
                with self.lock:
                    if not self.chunk_active:
                        if self.robot.get_servo_state() != SERVO_RUNNING:
                            raise RuntimeError("CR3 servo stopped while holding its last target")
                        self.robot.send_joint_positions(self.target.tolist())
                deadline += ACTION_DT_S
                if deadline < time.monotonic():
                    deadline = time.monotonic() + ACTION_DT_S
        except BaseException as exc:
            self.failure = exc
            self.stop_event.set()
            try:
                self.robot.close_servoj()
            except Exception:
                LOGGER.exception("Failed to close ServoJ after hold-loop failure")
            LOGGER.error("CR3 ServoJ hold failed: %s", exc)

    def raise_if_failed(self):
        if self.failure is not None:
            raise RuntimeError(f"CR3 ServoJ hold failed: {self.failure}") from self.failure

    def begin_chunk(self):
        with self.lock:
            self.raise_if_failed()
            if self.thread is None or self.stop_event.is_set() or self.chunk_active:
                raise RuntimeError("CR3 ServoJ keeper is not ready for a trajectory")
            self.chunk_active = True
            return self.target.copy()

    def send_target(self, target):
        with self.lock:
            self.raise_if_failed()
            self.robot.send_joint_positions(target.tolist())
            self.target = target.copy()

    def end_chunk(self):
        with self.lock:
            self.chunk_active = False

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join()
            self.thread = None
            self.robot.close_servoj()


def _move_arm_home(robot, servo: _ServoKeeper | None = None):
    """Move CR3 to the user's configured all-zero joint pose at a bounded rate."""
    from cr3_o6_interface import SERVO_RUNNING

    if robot.get_servo_state() != SERVO_RUNNING:
        raise RuntimeError("CR3 servo is not running; cannot move to the initial pose")
    measured = np.asarray(robot.get_joint_positions(), dtype=np.float32)
    start = servo.begin_chunk() if servo is not None else measured.copy()
    opened = False
    try:
        if servo is not None and np.max(np.abs(start - measured)) > ARM_MAX_TRACKING_ERROR_RAD:
            raise RuntimeError("CR3 hold target differs too far from measured position")
        distance = float(np.max(np.abs(INITIAL_ARM_POSITION - start)))
        if distance < 0.03:
            LOGGER.info("CR3 is already near the all-zero initial pose")
            return
        steps = max(60, math.ceil(distance / ARM_MAX_STEP_RAD))
        if servo is None:
            robot.open_servoj()
            opened = True
        LOGGER.warning("Returning CR3 to all-zero joint pose over %d steps", steps)
        started = time.monotonic() + ACTION_DT_S
        for index in range(steps):
            deadline = started + index * ACTION_DT_S
            time.sleep(max(0.0, deadline - time.monotonic()))
            if time.monotonic() - deadline > CONTROL_TOLERANCE_S:
                raise RuntimeError("CR3 return-to-zero control period was missed")
            if robot.get_servo_state() != SERVO_RUNNING:
                raise RuntimeError("CR3 servo stopped during return to zero")
            target = start + (INITIAL_ARM_POSITION - start) * ((index + 1) / steps)
            current = np.asarray(robot.get_joint_positions(), dtype=np.float32)
            if np.max(np.abs(target - current)) > ARM_MAX_TRACKING_ERROR_RAD:
                raise RuntimeError("CR3 is not tracking the return-to-zero trajectory")
            if servo is None:
                robot.send_joint_positions(target.tolist())
            else:
                servo.send_target(target)
        settled = 0
        deadline = time.monotonic() + 5.0
        while settled < 5:
            if time.monotonic() >= deadline or robot.get_servo_state() != SERVO_RUNNING:
                raise RuntimeError("CR3 did not settle safely at zero")
            current = np.asarray(robot.get_joint_positions(), dtype=np.float32)
            settled = settled + 1 if np.max(np.abs(current)) <= 0.03 else 0
            if servo is None:
                robot.send_joint_positions(INITIAL_ARM_POSITION.tolist())
            else:
                servo.send_target(INITIAL_ARM_POSITION)
            time.sleep(ACTION_DT_S)
        LOGGER.info("CR3 reached all-zero initial pose")
    finally:
        if servo is not None:
            servo.end_chunk()
        elif opened:
            robot.close_servoj()


def _move_hand_home(hand):
    """Move all six O6 channels to the previously specified initial pose."""
    current = np.asarray(hand.get_state(), dtype=np.float32)
    if current.shape != (6,) or not np.isfinite(current).all() or np.any((current < 0) | (current > 255)):
        raise RuntimeError("Cannot home O6: invalid measured six-channel position")
    steps = int(math.ceil(np.max(np.abs(INITIAL_HAND_POSITION - current)) / HAND_MAX_STEP_RAW))
    if not steps:
        LOGGER.info("O6 is already at the configured initial pose")
        return
    LOGGER.warning("Returning all six O6 channels to %s over %d steps", INITIAL_HAND_POSITION.tolist(), steps)
    started = time.monotonic()
    for index in range(steps):
        deadline = started + index * HAND_DT_S
        time.sleep(max(0.0, deadline - time.monotonic()))
        if time.monotonic() - deadline > CONTROL_TOLERANCE_S:
            raise RuntimeError("O6 return-to-initial control period was missed")
        target = np.rint(current + (INITIAL_HAND_POSITION - current) * ((index + 1) / steps))
        hand.finger_move(target.astype(np.int32).tolist())
    final = np.asarray(hand.get_state(), dtype=np.float32)
    if final.shape != (6,) or np.max(np.abs(final - INITIAL_HAND_POSITION)) > 5:
        raise RuntimeError(f"O6 did not reach initial pose: measured={final.tolist()}")


def _execute_action_chunk(robot, hand, servo, actions, observed_at, motion_started, cancel):
    """Execute only trajectory samples whose original observation-time deadlines remain."""
    from cr3_o6_interface import SERVO_RUNNING

    if actions.shape != ACTION_SHAPE or not np.isfinite(actions).all():
        raise RuntimeError("Remote actions must be a finite (16,12) array")
    if np.max(np.abs(actions[:, :6])) > 2 * math.pi:
        raise RuntimeError("Predicted CR3 joint target exceeds ±2π rad")
    if np.any((actions[:, 6:10] < 0) | (actions[:, 6:10] > 255)):
        raise RuntimeError("Predicted O6 command is outside raw range [0,255]")
    if np.any(actions[:, 10:] != 0):
        raise RuntimeError("Predicted fixed O6 channels 10-11 are not zero")

    now = time.monotonic()
    first_index = max(0, math.ceil((now - observed_at) / ACTION_DT_S) - 1)
    if first_index >= len(actions):
        LOGGER.warning(
            "Discarding expired prediction: observation_age=%.3fs exceeds %.3fs action horizon",
            now - observed_at,
            ACTION_HORIZON_S,
        )
        motion_started.set()
        return

    hand_lock = threading.Lock()
    initial_hand = np.asarray(hand.get_state(), dtype=np.float32)
    if initial_hand.shape != (6,) or not np.isfinite(initial_hand).all():
        raise RuntimeError("Invalid O6 feedback before action execution")
    last_hand = np.rint(initial_hand).astype(np.int32)
    hand_errors = []
    hand_stop = threading.Event()
    start_target = servo.begin_chunk()
    last_arm = start_target.copy()
    hand_thread = None

    def send_hand_targets():
        nonlocal last_hand
        try:
            for index in range(first_index, len(actions)):
                if index % 2:
                    continue
                if cancel.is_set() or hand_stop.is_set():
                    return
                deadline = observed_at + (index + 1) * ACTION_DT_S
                hand_stop.wait(max(0.0, deadline - time.monotonic()))
                if cancel.is_set() or hand_stop.is_set():
                    return
                lateness = time.monotonic() - deadline
                if lateness > CONTROL_TOLERANCE_S:
                    continue
                target = np.rint(actions[index, 6:12]).astype(np.int32)
                with hand_lock:
                    bounded = last_hand + np.clip(target - last_hand, -HAND_MAX_STEP_RAW, HAND_MAX_STEP_RAW)
                    sent_at = time.monotonic()
                    hand.finger_move(bounded.tolist())
                    last_hand = bounded
                    if time.monotonic() - sent_at > HAND_DT_S:
                        raise RuntimeError("O6 write exceeded its 100ms control interval")
        except BaseException as exc:
            hand_errors.append(exc)
            hand_stop.set()

    try:
        hand_thread = threading.Thread(target=send_hand_targets, name="o6-v2-live-actions", daemon=True)
        hand_thread.start()
        for index in range(first_index, len(actions)):
            if cancel.is_set():
                raise RuntimeError("Live action execution cancelled")
            deadline = observed_at + (index + 1) * ACTION_DT_S
            time.sleep(max(0.0, deadline - time.monotonic()))
            if time.monotonic() - deadline > CONTROL_TOLERANCE_S:
                continue
            if robot.get_servo_state() != SERVO_RUNNING:
                raise RuntimeError("CR3 servo stopped during action execution")
            current = np.asarray(robot.get_joint_positions(), dtype=np.float32)
            target = current + np.clip(actions[index, :6] - current, -ARM_MAX_STEP_RAD, ARM_MAX_STEP_RAD)
            command = last_arm + np.clip(target - last_arm, -ARM_MAX_STEP_RAD, ARM_MAX_STEP_RAD)
            if np.max(np.abs(command - current)) > ARM_MAX_TRACKING_ERROR_RAD:
                raise RuntimeError("CR3 target exceeds the tracking-error limit")
            servo.send_target(command)
            last_arm = command
            motion_started.set()
        hand_thread.join()
        if hand_errors:
            raise RuntimeError(f"O6 live execution failed: {hand_errors[0]}") from hand_errors[0]
        servo.raise_if_failed()
    finally:
        hand_stop.set()
        if hand_thread is not None:
            hand_thread.join()
        servo.end_chunk()
        motion_started.set()


def _run_live_inference(client, robot, hand, cameras, camera_mapping, task, servo, encoding_records):
    synchronized_hand = _SynchronizedHand(hand)
    pending = queue.Queue(maxsize=1)
    stop = threading.Event()
    failures = []

    def controller():
        while not stop.is_set():
            try:
                actions, observed_at, info, started = pending.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                _execute_action_chunk(robot, synchronized_hand, servo, actions, observed_at, started, stop)
                LOGGER.info("Trajectory handled: request_id=%s", info.get("request_id"))
            except BaseException as exc:
                failures.append(exc)
                stop.set()
                started.set()
                return

    thread = threading.Thread(target=controller, name="cr3-o6-v2-controller", daemon=True)
    thread.start()
    count = 0
    started_event = None
    try:
        while not stop.is_set():
            if failures:
                raise RuntimeError(f"Live controller failed: {failures[0]}") from failures[0]
            if started_event is not None:
                started_event.wait()
                if failures:
                    raise RuntimeError(f"Live controller failed: {failures[0]}") from failures[0]
            observed_at = time.monotonic()
            images, state, observation_timing = _read_observation(
                robot, synchronized_hand, cameras, camera_mapping
            )
            encoding_records.clear()
            infer_started = time.monotonic()
            actions, info = client.infer(images, state, task=task)
            infer_s = time.monotonic() - infer_started
            count += 1
            if failures:
                raise RuntimeError(f"Live controller failed: {failures[0]}") from failures[0]
            age = time.monotonic() - observed_at
            _log_timing(
                observation_timing,
                encoding_records,
                infer_s,
                age,
                info.get("server_elapsed_seconds"),
            )
            LOGGER.info(
                "Live inference #%d: request_id=%s age_s=%.3f server_elapsed_s=%s "
                "action_shape=%s first_action=%s",
                count,
                info.get("request_id"),
                age,
                info.get("server_elapsed_seconds"),
                tuple(actions.shape),
                np.round(actions[0], 3).tolist(),
            )
            started_event = threading.Event()
            pending.put((actions, observed_at, info, started_event))
            started_event.wait()
    finally:
        stop.set()
        thread.join()
    if failures:
        raise RuntimeError(f"Live controller failed: {failures[0]}") from failures[0]


def main() -> int:
    args = _parse_args()
    if args.execute_live != args.confirm_motion:
        raise SystemExit("Real robot motion requires both --execute-live and --confirm-motion")
    settings = load_settings(args.config)
    camera_mapping = _camera_mapping(settings)
    if str(settings.linkerhand_sdk_dir) not in sys.path:
        sys.path.insert(0, str(settings.linkerhand_sdk_dir))

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    LOGGER.info("Customer v2 camera mapping: %s", camera_mapping)

    sdk = _load_sdk()
    encoding_records = _instrument_image_encoding(sdk)
    from cr3_o6_camera import CR3O6CameraInterface
    from cr3_o6_interface import CR3Interface
    import linkerhand_compat

    linkerhand_compat.patch_pymodbus_slave_argument()
    try:
        from LinkerHand.linker_hand_api import LinkerHandApi
    except ImportError as exc:
        raise RuntimeError("LinkerHand SDK import failed; check linkerhand_sdk_dir") from exc

    client = sdk.CustomerPolicyClient(settings.api_url, settings.token_file, timeout=settings.timeout_s)
    robot = CR3Interface(
        settings.robot_ip,
        nrc_sdk_dir=settings.nrc_sdk_dir,
        servo_vmax=[50.1] * 7,
        servo_amax=[100.0] * 7,
        servo_jmax=[200.0] * 7,
    )
    cameras = CR3O6CameraInterface(
        output_width=640,
        output_height=480,
        strict_output_size=True,
    )
    hand_api = None
    robot_connected = False
    live_started = False
    exit_code = 0
    servo = None
    try:
        metadata = client.get_modality_config()
        expires = _load_expiry(metadata)
        task = sdk.TASKS[args.task]
        if task not in metadata["supported_tasks"].values():
            raise RuntimeError(f"Selected task {args.task!r} is not supported by remote metadata")
        LOGGER.info(
            "Customer v2 ready: url=%s version=%s role=%s scope=%s expires_beijing=%s "
            "image_shape=%s action_shape=%s dt_seconds=%s task=%s",
            settings.api_url,
            metadata["contract_version"],
            metadata["service_role"],
            metadata["integration_scope"],
            expires.astimezone(ZoneInfo("Asia/Shanghai")).isoformat(),
            metadata["image_shape"],
            metadata["action_shape"],
            metadata["dt_seconds"],
            args.task,
        )
        LOGGER.info(
            "Action execution: %s",
            "enabled by --execute-live --confirm-motion" if args.execute_live else "disabled; predictions only",
        )

        client.reset()
        LOGGER.info("Connecting CR3 state interface at %s", settings.robot_ip)
        robot.connect()
        robot_connected = True
        if args.execute_live:
            from cr3_o6_interface import SERVO_ERROR, SERVO_RUNNING

            servo_state = robot.get_servo_state()
            if servo_state == SERVO_ERROR:
                raise RuntimeError("CR3 servo is in ERROR state; recover it manually before motion")
            if servo_state != SERVO_RUNNING:
                LOGGER.warning("CR3 servo is not running; powering on for confirmed live motion")
                robot.power_on()
                power_deadline = time.monotonic() + 10.0
                while robot.get_servo_state() != SERVO_RUNNING:
                    if time.monotonic() >= power_deadline:
                        raise RuntimeError("CR3 did not reach RUNNING servo state")
                    time.sleep(0.2)
        modbus = settings.o6_modbus or _load_hand_config(settings.hand_config)
        LOGGER.info("Connecting right O6 interface at %s", modbus)
        hand_api = LinkerHandApi(hand_type="right", hand_joint="O6", modbus=modbus, can="None")
        if args.execute_live:
            _move_hand_home(hand_api)
            _move_arm_home(robot)
        cameras.connect()

        if args.execute_live:
            servo = _ServoKeeper(robot)
            servo.start()
            live_started = True
            _run_live_inference(
                client, robot, hand_api, cameras, camera_mapping, task, servo, encoding_records
            )
        else:
            count = 0
            while True:
                images, state, observation_timing = _read_observation(
                    robot, hand_api, cameras, camera_mapping
                )
                encoding_records.clear()
                infer_started = time.monotonic()
                actions, info = client.infer(images, state, task=task)
                infer_s = time.monotonic() - infer_started
                count += 1
                _log_timing(
                    observation_timing,
                    encoding_records,
                    infer_s,
                    observation_timing["observation_total_s"] + infer_s,
                    info.get("server_elapsed_seconds"),
                )
                LOGGER.info(
                    "Prediction #%d: request_id=%s task=%s server_elapsed_s=%s "
                    "action_shape=%s actions=%s",
                    count,
                    info.get("request_id"),
                    args.task,
                    info.get("server_elapsed_seconds"),
                    tuple(actions.shape),
                    np.round(actions, 3).tolist(),
                )
                if not args.continuous:
                    break
    except KeyboardInterrupt:
        LOGGER.warning("Interrupted; stopping Customer v2 session")
        exit_code = 130
    except (OSError, RuntimeError, ValueError) as exc:
        LOGGER.error("Customer v2 session failed: %s", exc)
        exit_code = 1
    finally:
        if live_started and robot_connected:
            from cr3_o6_interface import SERVO_RUNNING

            try:
                servo_state = robot.get_servo_state()
            except (OSError, RuntimeError, ValueError):
                LOGGER.exception("Could not read CR3 servo state during shutdown")
                servo_state = None
                exit_code = 1
            if servo_state == SERVO_RUNNING:
                arm_home_ok = False
                try:
                    _move_arm_home(robot, servo=servo)
                    arm_home_ok = True
                except (OSError, RuntimeError, ValueError):
                    LOGGER.exception("CR3 return to zero failed")
                    exit_code = 1
                if arm_home_ok and hand_api is not None:
                    try:
                        _move_hand_home(hand_api)
                    except (OSError, RuntimeError, ValueError):
                        LOGGER.exception("O6 return to initial pose failed")
                        exit_code = 1
            elif servo_state is not None:
                LOGGER.error("CR3 servo is not RUNNING; exit homing skipped")
                exit_code = 1
        if servo is not None:
            try:
                servo.stop()
            except (OSError, RuntimeError, ValueError):
                LOGGER.exception("Could not close live ServoJ")
                exit_code = 1
        cameras.disconnect()
        if hand_api is not None:
            backend = getattr(hand_api, "hand", None)
            close = getattr(backend, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    LOGGER.exception("Failed to close LinkerHand read interface")
        if robot_connected:
            robot.disconnect()
        client.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
