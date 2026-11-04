"""Sequential CR3/O6 action chunks with a fresh observation between segments."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import logging
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np


LOGGER = logging.getLogger("stationary_customer")
CLIENT_REVISION = "segmented-redeploy-auto-v1"
REDEPLOY_PACKAGE_DIR = Path(__file__).resolve().parent / "customer-stationary-client-20261027-redeploy"
PROXY_SDK_PATH = Path(__file__).resolve().with_name("customer_client_proxy.py")
PROXY_SDK_SHA256 = "c1f364dec131a029f6ca8e539c5c8c3a78c8509404c64435f4b0dbd8fea0e731"
TRIAL_SECONDS = 120
DT = 0.05
HAND_DT = 0.1
SCHEDULE_TOLERANCE = 0.02
ARM_STEP = 0.04
ARM_TRACKING_ERROR = 0.2
HAND_STEP = 6
WAIT_ARM_TOLERANCE = 0.02
WAIT_HAND_TOLERANCE = 5
ARM_SETTLE_SECONDS = 2.0
ARM_SETTLE_SAMPLES = 5
CONTROLLER_FAULT_STATE = 2
IMAGE_SHAPE = (480, 640, 3)
CAMERAS = ("top", "wrist", "side")
INITIAL_ARM = np.zeros(6, dtype=np.float32)
# Keep the configured local v2 three-finger home pose, including active middle flex.
INITIAL_HAND = np.asarray((157, 40, 175, 175, 0, 0), dtype=np.float32)


def ensure_time(deadline):
    if time.monotonic() >= deadline:
        raise TimeoutError("trial_budget_exhausted")


def vector(value, size, label):
    result = np.asarray(value)
    if result.shape != (size,) or result.dtype.kind not in "fiu" or not np.isfinite(result).all():
        raise ValueError(label + "_invalid")
    if isinstance(value, (list, tuple)) and any(isinstance(item, (bool, np.bool_)) for item in np.asarray(value, dtype=object).flat):
        raise ValueError(label + "_invalid")
    if np.any(np.abs(result) > np.finfo(np.float32).max):
        raise ValueError(label + "_invalid")
    return result.astype(np.float32)


def read_arm_feedback(robot):
    arm = vector(robot.get_joint_positions(), 6, "arm_feedback")
    if np.max(np.abs(arm)) > 2 * math.pi:
        raise ValueError("arm_feedback_units_or_range_invalid")
    return arm


def read_feedback(robot, hand):
    arm = read_arm_feedback(robot)
    fingers = vector(hand.get_state(), 6, "hand_feedback")
    if np.any((fingers < 0) | (fingers > 255)) or np.any(fingers[4:] != 0):
        raise ValueError("hand_feedback_range_or_fixed_channels_invalid")
    return np.concatenate((arm, fingers))


def read_state(robot, hand, running_state):
    if robot.get_servo_state() != running_state:
        raise RuntimeError("controller_not_in_confirmed_running_state")
    return read_feedback(robot, hand)


def assert_stationary(reference, current):
    reference = vector(reference, 12, "reference_state")
    current = vector(current, 12, "current_state")
    if np.max(np.abs(current[:6] - reference[:6])) > WAIT_ARM_TOLERANCE:
        raise RuntimeError("arm_moved_while_waiting")
    if np.max(np.abs(current[6:] - reference[6:])) > WAIT_HAND_TOLERANCE:
        raise RuntimeError("hand_moved_while_waiting")


def verify_stationary(robot, hand, running_state, deadline, reference=None):
    ensure_time(deadline)
    reference = read_state(robot, hand, running_state) if reference is None else reference
    for _ in range(5):
        ensure_time(deadline)
        current = read_state(robot, hand, running_state)
        assert_stationary(reference, current)
        if robot.servo_active:
            robot.send_joint_positions(reference[:6].tolist())
        time.sleep(DT)
    return current


def snapshot(robot, hand, cameras, mapping, running_state, deadline):
    total_started = time.monotonic()
    state_started = time.monotonic()
    reference = verify_stationary(robot, hand, running_state, deadline)
    stationarity_seconds = time.monotonic() - state_started
    started = time.monotonic()
    camera_started = time.monotonic()
    local = cameras.read()
    camera_read_seconds = time.monotonic() - camera_started
    validation_started = time.monotonic()
    images = {}
    for name in CAMERAS:
        image = np.asarray(local[mapping[name]])
        if image.shape != IMAGE_SHAPE or image.dtype != np.uint8:
            raise ValueError("raw_rgb_image_contract_invalid")
        images[name] = image.copy()
    image_validation_seconds = time.monotonic() - validation_started
    state_started = time.monotonic()
    current = read_state(robot, hand, running_state)
    final_state_read_seconds = time.monotonic() - state_started
    assert_stationary(reference, current)
    ensure_time(deadline)
    timings = {
        "stationarity_check_seconds": stationarity_seconds,
        "camera_read_seconds": camera_read_seconds,
        "image_validation_copy_seconds": image_validation_seconds,
        "final_state_read_seconds": final_state_read_seconds,
        "snapshot_total_seconds": time.monotonic() - total_started,
    }
    return images, current, started, timings


def wait_for_prediction(future, robot, hand, reference, running_state, deadline, cancelled=None):
    try:
        while not future.done():
            ensure_time(deadline)
            assert_stationary(reference, read_state(robot, hand, running_state))
            if robot.servo_active:
                robot.send_joint_positions(reference[:6].tolist())
            time.sleep(DT)
        ensure_time(deadline)
        result = future.result()
        assert_stationary(reference, read_state(robot, hand, running_state))
        return result
    except BaseException:
        if cancelled is not None:
            cancelled.set()
        raise


def validate_actions(actions):
    values = np.asarray(actions)
    if values.shape != (16, 12) or values.dtype.kind not in "fiu" or not np.isfinite(values).all():
        raise ValueError("prediction_shape_or_values_invalid")
    if isinstance(actions, (list, tuple)) and any(isinstance(item, (bool, np.bool_)) for item in np.asarray(actions, dtype=object).flat):
        raise ValueError("prediction_shape_or_values_invalid")
    if np.any((values[:, :6] < -2 * math.pi) | (values[:, :6] > 2 * math.pi)):
        raise ValueError("prediction_arm_values_invalid")
    if np.any((values[:, 6:10] < 0) | (values[:, 6:10] > 255)) or np.any(values[:, 10:] != 0):
        raise ValueError("prediction_hand_values_invalid")
    return values.astype(np.float32)


def ensure_controller_running(robot, running_state, fault_state, report):
    state = robot.get_servo_state()
    if state == fault_state:
        raise RuntimeError("controller_in_error_state_inspect_alarm_manually")
    if state == running_state:
        report["automatic_power_on"] = False
        return
    report["automatic_power_on"] = True
    LOGGER.warning("CR3 servo state=%s; powering on and waiting for RUNNING", state)
    robot.power_on()
    deadline = time.monotonic() + 10.0
    while robot.get_servo_state() != running_state:
        if time.monotonic() >= deadline:
            raise TimeoutError("controller_did_not_reach_running_after_power_on")
        time.sleep(0.2)


def home_hand_to_initial(hand):
    initial = vector(hand.get_state(), 6, "hand_before_homing")
    if np.any((initial < 0) | (initial > 255)):
        raise ValueError("hand_before_homing_out_of_range")
    steps = int(math.ceil(np.max(np.abs(INITIAL_HAND - initial)) / HAND_STEP))
    previous = np.rint(initial).astype(np.int32)
    started = time.monotonic()
    for index in range(steps):
        deadline = started + index * HAND_DT
        time.sleep(max(0, deadline - time.monotonic()))
        if time.monotonic() - deadline > SCHEDULE_TOLERANCE:
            raise TimeoutError("hand_homing_schedule_missed")
        target = np.rint(initial + (INITIAL_HAND - initial) * ((index + 1) / steps)).astype(np.int32)
        if np.max(np.abs(target - previous)) > HAND_STEP:
            raise ValueError("hand_homing_step_limit_exceeded")
        hand.finger_move(target.tolist())
        previous = target
    final = vector(hand.get_state(), 6, "hand_after_homing")
    if np.max(np.abs(final - INITIAL_HAND)) > WAIT_HAND_TOLERANCE:
        raise RuntimeError("hand_did_not_reach_configured_initial_pose")
    LOGGER.info("O6 reached configured initial pose %s", final.tolist())


def home_arm_to_initial(robot, running_state, *, keep_servo_open):
    if robot.get_servo_state() != running_state:
        raise RuntimeError("controller_not_running_for_arm_homing")
    measured = read_arm_feedback(robot)
    start = measured.copy()
    opened_here = False
    if not robot.servo_active:
        robot.open_servoj()
        opened_here = True
    distance = float(np.max(np.abs(INITIAL_ARM - start)))
    try:
        if distance > WAIT_ARM_TOLERANCE:
            steps = max(60, math.ceil(distance / ARM_STEP))
            LOGGER.warning("Returning CR3 to all-zero joint pose over %d steps", steps)
            started = time.monotonic() + DT
            for index in range(steps):
                deadline = started + index * DT
                time.sleep(max(0, deadline - time.monotonic()))
                if time.monotonic() - deadline > SCHEDULE_TOLERANCE:
                    raise TimeoutError("arm_homing_schedule_missed")
                if robot.get_servo_state() != running_state:
                    raise RuntimeError("controller_state_changed_during_arm_homing")
                target = start + (INITIAL_ARM - start) * ((index + 1) / steps)
                current = read_arm_feedback(robot)
                if np.max(np.abs(target - current)) > ARM_TRACKING_ERROR:
                    raise RuntimeError("arm_tracking_error_during_homing")
                robot.send_joint_positions(target.tolist())
        deadline = time.monotonic() + 5.0
        settled = 0
        while settled < ARM_SETTLE_SAMPLES:
            if time.monotonic() >= deadline or robot.get_servo_state() != running_state:
                raise TimeoutError("arm_did_not_settle_at_zero")
            current = read_arm_feedback(robot)
            settled = settled + 1 if np.max(np.abs(current - INITIAL_ARM)) <= WAIT_ARM_TOLERANCE else 0
            robot.send_joint_positions(INITIAL_ARM.tolist())
            time.sleep(DT)
        LOGGER.info("CR3 reached all-zero joint pose")
    finally:
        if opened_here and not keep_servo_open:
            robot.close_servoj()


def execute_segment(robot, hand, actions, reference, running_state, deadline, report, chunk_index):
    actions = validate_actions(actions)
    current = verify_stationary(robot, hand, running_state, deadline, reference)
    if deadline - time.monotonic() < len(actions) * DT + ARM_SETTLE_SECONDS + DT:
        raise TimeoutError("insufficient_remaining_segment_budget")
    stop = threading.Event()
    hand_errors = []
    hand_thread = None
    opened = False
    hand_io_lock = threading.Lock()
    hand_io = {"started": None}
    command_trace = {
        "arm_first": None, "arm_last": None,
        "hand_first_model_target": None, "hand_last_model_target": None,
        "hand_first_command": None, "hand_last_command": None,
    }
    counts = {
        "arm_target_calls_attempted": 0, "arm_targets_submitted": 0,
        "hand_target_calls_attempted": 0, "hand_targets_submitted": 0,
        "initial_hold_targets": 0, "arm_hold_calls_attempted": 0, "arm_hold_targets_submitted": 0,
    }
    report.update(counts)
    last_arm = current[:6].copy()
    initial_hand = np.rint(current[6:]).astype(np.int32)

    def fail_if_hand_failed():
        if hand_errors:
            raise RuntimeError("hand_submission_failed") from hand_errors[0]
        with hand_io_lock:
            write_started = hand_io["started"]
        if write_started is not None and time.monotonic() - write_started > HAND_DT:
            raise TimeoutError("hand_write_exceeded_control_period")

    def check_deadline(target_time, component="arm"):
        ensure_time(deadline)
        lateness = time.monotonic() - target_time
        if lateness > SCHEDULE_TOLERANCE:
            report.setdefault("first_deadline_miss", {"component": component, "lateness_seconds": lateness})
            raise TimeoutError("actuator_schedule_missed")

    def hand_worker(started):
        previous = initial_hand.copy()
        try:
            for index in range(0, 16, 2):
                target_time = started + (index + 1) * DT
                if stop.wait(max(0, target_time - time.monotonic())):
                    return
                check_deadline(target_time, "hand")
                target = np.rint(actions[index, 6:12]).astype(np.int32)
                command = previous + np.clip(target - previous, -HAND_STEP, HAND_STEP)
                if stop.is_set():
                    return
                before = time.monotonic()
                counts["hand_target_calls_attempted"] += 1
                with hand_io_lock:
                    hand_io["started"] = before
                try:
                    hand.finger_move(command.tolist())
                finally:
                    with hand_io_lock:
                        hand_io["started"] = None
                previous = command
                counts["hand_targets_submitted"] += 1
                write_seconds = time.monotonic() - before
                report["maximum_hand_target_call_seconds"] = max(
                    report.get("maximum_hand_target_call_seconds", 0.0), write_seconds
                )
                if command_trace["hand_first_model_target"] is None:
                    command_trace["hand_first_model_target"] = target.tolist()
                    command_trace["hand_first_command"] = command.tolist()
                command_trace["hand_last_model_target"] = target.tolist()
                command_trace["hand_last_command"] = command.tolist()
                if write_seconds > HAND_DT:
                    raise TimeoutError("hand_write_exceeded_control_period")
        except BaseException as error:
            hand_errors.append(error)
            stop.set()

    try:
        if not robot.servo_active:
            robot.open_servoj()
            opened = True
        robot.send_joint_positions(last_arm.tolist())
        counts["initial_hold_targets"] = 1
        started = time.monotonic()
        report["execution_epoch_monotonic"] = started
        hand_thread = threading.Thread(target=hand_worker, args=(started,), name="one-segment-hand", daemon=True)
        hand_thread.start()
        for index in range(16):
            target_time = started + (index + 1) * DT
            fail_if_hand_failed()
            ensure_time(deadline)
            feedback_started = time.monotonic()
            if robot.get_servo_state() != running_state:
                raise RuntimeError("controller_state_changed")
            measured = read_arm_feedback(robot)
            report["maximum_arm_feedback_read_seconds"] = max(
                report.get("maximum_arm_feedback_read_seconds", 0.0),
                time.monotonic() - feedback_started,
            )
            target = measured + np.clip(actions[index, :6] - measured, -ARM_STEP, ARM_STEP)
            command = last_arm + np.clip(target - last_arm, -ARM_STEP, ARM_STEP)
            if np.max(np.abs(command - measured)) > ARM_TRACKING_ERROR:
                raise RuntimeError("arm_tracking_error_exceeded")
            if np.max(np.abs(command)) > 2 * math.pi:
                raise ValueError("bounded_arm_target_out_of_range")
            # Read and prepare within the preceding cycle, not after the send deadline.
            if stop.wait(max(0, target_time - time.monotonic())):
                fail_if_hand_failed()
                raise RuntimeError("segment_cancelled")
            check_deadline(target_time)
            fail_if_hand_failed()
            if stop.is_set():
                raise RuntimeError("segment_cancelled")
            counts["arm_target_calls_attempted"] += 1
            call_started = time.monotonic()
            try:
                robot.send_joint_positions(command.tolist())
            finally:
                report["maximum_arm_target_call_seconds"] = max(
                    report.get("maximum_arm_target_call_seconds", 0.0),
                    time.monotonic() - call_started,
                )
            last_arm = command
            counts["arm_targets_submitted"] += 1
            if command_trace["arm_first"] is None:
                command_trace["arm_first"] = command.tolist()
            command_trace["arm_last"] = command.tolist()
        settle_deadline = min(deadline, time.monotonic() + ARM_SETTLE_SECONDS)
        hold_time = started + 17 * DT
        stable_samples = 0
        report["arm_final_target_feedback_confirmed"] = False
        while True:
            fail_if_hand_failed()
            ensure_time(deadline)
            if time.monotonic() >= settle_deadline:
                raise TimeoutError("final_arm_target_not_settled")
            if robot.get_servo_state() != running_state:
                raise RuntimeError("controller_state_changed_while_finishing")
            measured = read_arm_feedback(robot)
            error = float(np.max(np.abs(last_arm - measured)))
            report["latest_final_arm_feedback_error_rad"] = error
            if error > ARM_TRACKING_ERROR:
                raise RuntimeError("arm_tracking_error_exceeded")
            stable_samples = stable_samples + 1 if error <= WAIT_ARM_TOLERANCE else 0
            if stable_samples >= ARM_SETTLE_SAMPLES and not hand_thread.is_alive():
                fail_if_hand_failed()
                if counts["hand_targets_submitted"] != 8:
                    raise RuntimeError("hand_segment_incomplete")
                report.update(
                    arm_final_target_feedback_confirmed=True, arm_settle_feedback_samples=stable_samples,
                    completion_reference="last_submitted_arm_target_after_step_limits",
                )
                break
            if stop.wait(max(0, hold_time - time.monotonic())):
                fail_if_hand_failed()
                raise RuntimeError("segment_cancelled")
            check_deadline(hold_time, "arm_final_hold")
            if time.monotonic() >= settle_deadline:
                raise TimeoutError("final_arm_target_not_settled")
            fail_if_hand_failed()
            counts["arm_hold_calls_attempted"] += 1
            call_started = time.monotonic()
            try:
                robot.send_joint_positions(last_arm.tolist())
            finally:
                report["maximum_arm_target_call_seconds"] = max(
                    report.get("maximum_arm_target_call_seconds", 0.0),
                    time.monotonic() - call_started,
                )
            counts["arm_hold_targets_submitted"] += 1
            hold_time += DT
        report["segment_command_calls_completed"] = True
        report["servo_left_open_for_return_home"] = robot.servo_active
    except BaseException as error:
        report["segment_failure_type"] = type(error).__name__
        raise
    finally:
        stop.set()
        try:
            if opened:
                robot.close_servoj()
        except Exception as error:
            report["segment_stop_error_type"] = type(error).__name__
            raise
        finally:
            if hand_thread is not None:
                hand_thread.join(timeout=1)
                report["hand_worker_stopped"] = not hand_thread.is_alive()
                report["hand_command_completion_known"] = not hand_thread.is_alive()
            report.update(counts)
    report["controller_state_after_segment"] = robot.get_servo_state()
    if report["controller_state_after_segment"] == CONTROLLER_FAULT_STATE:
        raise RuntimeError("controller_reported_fault_after_stop")
    feedback = read_feedback(robot, hand)
    report.update(
        action_chunk_first=actions[0].tolist(),
        action_chunk_last=actions[-1].tolist(),
        action_chunk_arm_min=actions[:, :6].min(axis=0).tolist(),
        action_chunk_arm_max=actions[:, :6].max(axis=0).tolist(),
        action_chunk_hand_min=actions[::2, 6:].min(axis=0).tolist(),
        action_chunk_hand_max=actions[::2, 6:].max(axis=0).tolist(),
        observed_state_at_segment_start=current.tolist(),
        arm_first_command=command_trace["arm_first"],
        arm_last_command=command_trace["arm_last"],
        hand_first_model_target=command_trace["hand_first_model_target"],
        hand_last_model_target=command_trace["hand_last_model_target"],
        hand_first_command=command_trace["hand_first_command"],
        hand_last_command=command_trace["hand_last_command"],
        feedback_after_segment=feedback.tolist(),
        arm_feedback_change_rad=float(np.max(np.abs(feedback[:6] - reference[:6]))),
        hand_feedback_change_raw=float(np.max(np.abs(feedback[6:] - reference[6:]))),
        final_arm_target_error_rad=float(np.max(np.abs(feedback[:6] - last_arm))),
        physical_task_success_verified=False,
    )
    LOGGER.info(
        "Actuator diagnostics chunk=%d observed_start=%s arm_command_first=%s arm_command_last=%s hand_model_first=%s hand_model_last=%s hand_command_first=%s hand_command_last=%s feedback_after_segment=%s",
        chunk_index,
        np.array2string(current, precision=3), command_trace["arm_first"], command_trace["arm_last"],
        command_trace["hand_first_model_target"], command_trace["hand_last_model_target"],
        command_trace["hand_first_command"], command_trace["hand_last_command"],
        np.array2string(feedback, precision=3),
    )


def load_sdk(directory, manifest):
    original = directory / "customer_client.py"
    if hashlib.sha256(original.read_bytes()).hexdigest() != manifest["sdk_sha256"]:
        raise ValueError("sdk_checksum_changed")
    if hashlib.sha256(PROXY_SDK_PATH.read_bytes()).hexdigest() != PROXY_SDK_SHA256:
        raise ValueError("local_proxy_sdk_checksum_changed")
    path = PROXY_SDK_PATH
    spec = importlib.util.spec_from_file_location("stationary_https_sdk", path)
    if spec is None or spec.loader is None:
        raise ValueError("sdk_import_failed")
    sdk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sdk)
    return sdk


def prepare_remote_client(client, manifest, report):
    operation_started = time.monotonic()
    report["prediction_stage"] = "metadata"
    try:
        metadata = client.get_modality_config()
        expires = datetime.fromisoformat(metadata["expires_utc"])
        if metadata["expires_utc"] != manifest["expires_utc"] or expires <= datetime.now(timezone.utc):
            raise ValueError("unexpected_or_expired_service")
        report["metadata_seconds"] = time.monotonic() - operation_started
        LOGGER.info("Remote metadata stage took %.3fs", report["metadata_seconds"])
        report["prediction_stage"] = "reset"
        operation_started = time.monotonic()
        client.reset()
        report["reset_seconds"] = time.monotonic() - operation_started
        LOGGER.info("Remote reset stage took %.3fs; reset is done once per session", report["reset_seconds"])
    except Exception as error:
        LOGGER.exception(
            "Remote session preparation failed code=%s cause_type=%s cause_errno=%s",
            getattr(error, "code", type(error).__name__),
            getattr(error, "cause_type", None), getattr(error, "cause_errno", None),
        )
        raise


def request_prediction(client, images, state, task, deadline, report, chunk_index):
    ensure_time(deadline)
    report["prediction_stage"] = "infer"
    report["prediction_requests_attempted"] += 1
    client.timeout = min(report["request_timeout_limit_seconds"], deadline - time.monotonic())
    started = time.monotonic()
    try:
        actions, info = client.infer(images, state, task=task)
    except Exception as error:
        report["request_id"] = getattr(client, "last_request_id", None)
        report["inference_wall_seconds"] = time.monotonic() - started
        report.update(getattr(client, "last_infer_timing", {}))
        LOGGER.exception(
            "Inference chunk %d failed request_id=%s timings=%s cause_type=%s cause_errno=%s",
            chunk_index, report.get("request_id"), getattr(client, "last_infer_timing", {}),
            getattr(error, "cause_type", None), getattr(error, "cause_errno", None),
        )
        raise
    if info["task"] != task or info["dt_seconds"] != DT:
        raise ValueError("prediction_task_or_spacing_changed")
    actions = validate_actions(actions)
    report.update(
        inference_wall_seconds=time.monotonic() - started,
        request_id=info["request_id"],
        server_elapsed_seconds=info["server_elapsed_seconds"],
        prediction_requests_completed=report["prediction_requests_completed"] + 1,
        prediction_stage="complete",
    )
    report.update(getattr(client, "last_infer_timing", {}))
    LOGGER.info(
        "Inference chunk %d request_id=%s client_total=%.3fs server=%.3fs details=%s",
        chunk_index, info["request_id"], report["inference_wall_seconds"],
        info["server_elapsed_seconds"], getattr(client, "last_infer_timing", {}),
    )
    ensure_time(deadline)
    LOGGER.info(
        "Received action chunk %d request_id=%s shape=%s dt=%.3fs task=%s; rows are steps, columns 0-5=arm(rad), 6-11=hand(raw)\n%s",
        chunk_index, info["request_id"], actions.shape, info["dt_seconds"], info["task"],
        np.array2string(actions, precision=4, suppress_small=False),
    )
    return actions


def hand_device(settings):
    import yaml

    modbus = settings.o6_modbus
    if modbus is None:
        config = yaml.safe_load(settings.hand_config.read_text(encoding="utf-8"))
        root = config.get("LINKER_HAND") if isinstance(config, dict) else None
        right = root.get("RIGHT_HAND") if isinstance(root, dict) else None
        if not isinstance(right, dict) or str(right.get("JOINT", "")).upper() != "O6":
            raise ValueError("existing_right_hand_configuration_invalid")
        modbus = right.get("MODBUS")
    if not isinstance(modbus, str) or not modbus.strip() or modbus.strip().lower() == "none":
        raise ValueError("existing_right_hand_modbus_invalid")
    if str(settings.linkerhand_sdk_dir) not in sys.path:
        sys.path.insert(0, str(settings.linkerhand_sdk_dir))
    import linkerhand_compat
    from LinkerHand.linker_hand_api import LinkerHandApi

    linkerhand_compat.patch_pymodbus_slave_argument()
    return LinkerHandApi(hand_type="right", hand_joint="O6", modbus=modbus, can="None")


@contextmanager
def local_claim(config_path):
    import fcntl

    path = Path(config_path).expanduser().resolve().parent / ".customer-stationary.lock"
    descriptor = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        if os.fstat(descriptor).st_uid != os.getuid():
            raise ValueError("local_lock_not_owned_by_current_user")
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("another_stationary_client_is_active") from None
        yield
    finally:
        os.close(descriptor)


def cleanup_devices(robot, hand, cameras, connected, report):
    for name, close in (
        ("camera", cameras.disconnect if cameras is not None else None),
        ("arm", robot.disconnect if robot is not None and connected else None),
        ("hand", getattr(getattr(hand, "hand", None), "close", None) if report.get("hand_worker_stopped", True) else None),
    ):
        if callable(close):
            try:
                close()
            except Exception as error:
                report.setdefault("cleanup_errors", []).append({"component": name, "error_type": type(error).__name__})
                print(f"Cleanup failed for {name}; use the onsite approved stop procedure.")
    if report.get("hand_worker_stopped") is False:
        report.setdefault("cleanup_errors", []).append({"component": "hand", "error_type": "inflight_call_not_finished"})
    if report.get("cleanup_errors"):
        report["outcome_before_cleanup_failure"] = report["status"]
        report["status"] = "cleanup_failed"
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--task", choices=("two_finger", "three_finger"), default="three_finger")
    args = parser.parse_args()
    if os.name != "posix":
        parser.error("Use the customer's existing supported Linux device environment.")
    report_dir = Path(__file__).resolve().parent / "stationary-reports"
    report_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-%f")
    args.report = report_dir / f"stationary-result-{timestamp}.json"
    args.log_path = args.report.with_suffix(".log")
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=[logging.StreamHandler(), logging.FileHandler(args.log_path, encoding="utf-8")],
        force=True,
    )
    LOGGER.info("Diagnostic log: %s", args.log_path)
    LOGGER.info("Run report: %s", args.report)
    with local_claim(args.config):
        return run_once(args, parser)


def run_once(args, parser):
    directory = REDEPLOY_PACKAGE_DIR
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
    manifest = json.loads((directory / "session.json").read_text())
    for name, expected in manifest["component_sha256"].items():
        if Path(name).name != name or hashlib.sha256((directory / name).read_bytes()).hexdigest() != expected:
            parser.error("A supplied component changed; do not mix this package with older adapters.")
    service_expires = datetime.fromisoformat(manifest["expires_utc"])
    if service_expires.tzinfo is None or service_expires <= datetime.now(timezone.utc):
        parser.error("This prediction session has expired.")
    sdk = load_sdk(directory, manifest)
    from cr3_o6_settings import load_settings
    from cr3_o6_interface import CR3Interface, SERVO_ERROR, SERVO_RUNNING
    from cr3_o6_camera import CR3O6CameraInterface

    settings = load_settings(args.config)
    if settings.api_url.rstrip("/") != manifest["api_url"]:
        parser.error("The configured API URL does not match this session.")
    mapping = {"top": settings.top_camera, "wrist": settings.wrist_camera, "side": settings.side_camera}
    if set(mapping.values()) != {"base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb"}:
        parser.error("The camera mapping must preserve all three documented roles.")
    print("Repeated segmented inference until Ctrl-C or the 120s trial deadline; holds pose while awaiting each next chunk.")
    print("Automatic startup/exit homing enabled; no automatic fault clearing or retry.")
    print(f"Homing pose: CR3 all-zero joints; O6 {INITIAL_HAND.astype(int).tolist()}.")
    print(f"Client revision: {CLIENT_REVISION}")
    print(f"Software caps: arm {ARM_STEP} rad/50ms; tracking {ARM_TRACKING_ERROR} rad; hand {HAND_STEP} raw/100ms.")
    print(f"Waiting gates: arm {WAIT_ARM_TOLERANCE} rad, hand {WAIT_HAND_TOLERANCE} raw. These are not hardware safety certification.")
    started = time.monotonic()
    deadline = min(started + TRIAL_SECONDS, started + (service_expires - datetime.now(timezone.utc)).total_seconds())
    report = {
        "status": "starting", "control_mode": "sequential_segmented_inference", "client_revision": CLIENT_REVISION,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "api_origin": manifest["api_url"], "service_expires_utc": manifest["expires_utc"],
        "diagnostics_log": str(args.log_path),
        "maximum_trial_seconds": TRIAL_SECONDS,
        "package_manifest_prediction_request_limit": manifest.get("maximum_prediction_requests"),
        "package_manifest_prediction_segment_limit": manifest.get("maximum_prediction_segments"),
        "local_prediction_limit": "until_trial_deadline_or_ctrl_c",
        "prediction_requests_attempted": 0, "prediction_requests_completed": 0,
        "segments_completed": 0, "segments": [], "automatic_retries": 0,
        "automatic_power_on": False, "automatic_homing": False,
        "operator_confirmed_onsite_conditions": False,
        "interactive_confirmations_required": False,
        "scene_stability_confirmed_by_operator_not_software": False,
        "physical_task_success_verified": False,
    }
    with args.report.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
    robot = None
    hand = None
    cameras = None
    client = None
    network = None
    connected = False
    startup_homing_complete = False
    code = 1
    try:
        client = sdk.CustomerPolicyClient(settings.api_url, settings.token_file, timeout=min(settings.timeout_s, 10))
        report["request_timeout_limit_seconds"] = min(settings.timeout_s, 10)
        network = ThreadPoolExecutor(max_workers=1, thread_name_prefix="customer-inference")
        network.submit(prepare_remote_client, client, manifest, report).result()
        robot = CR3Interface(settings.robot_ip, nrc_sdk_dir=settings.nrc_sdk_dir, servo_vmax=[50.1] * 7, servo_amax=[100.0] * 7, servo_jmax=[200.0] * 7)
        robot.connect()
        connected = True
        ensure_controller_running(robot, SERVO_RUNNING, SERVO_ERROR, report)
        report["controller_reported_running_at_start"] = True
        hand = hand_device(settings)
        hand_home_started = time.monotonic()
        home_hand_to_initial(hand)
        LOGGER.info("Startup hand home took %.3fs", time.monotonic() - hand_home_started)
        robot.open_servoj()
        arm_home_started = time.monotonic()
        home_arm_to_initial(robot, SERVO_RUNNING, keep_servo_open=True)
        LOGGER.info("Startup arm home took %.3fs", time.monotonic() - arm_home_started)
        report["automatic_homing"] = True
        startup_homing_complete = True
        camera_connect_started = time.monotonic()
        cameras = CR3O6CameraInterface(output_width=640, output_height=480, strict_output_size=True, async_read=False)
        cameras.connect()
        LOGGER.info("Camera connect took %.3fs", time.monotonic() - camera_connect_started)
        minimum_cycle_budget = min(settings.timeout_s, 10) + 16 * DT + ARM_SETTLE_SECONDS + DT
        previous_segment_finished = None
        while deadline - time.monotonic() >= minimum_cycle_budget:
            chunk_index = report["prediction_requests_attempted"] + 1
            images, observed, observed_at, snapshot_timings = snapshot(
                robot, hand, cameras, mapping, SERVO_RUNNING, deadline
            )
            report.update(snapshot_timings)
            report["observed_state_before_inference"] = observed.tolist()
            LOGGER.info(
                "Chunk %d observation timing=%s state=%s",
                chunk_index, snapshot_timings, np.array2string(observed, precision=3),
            )
            report["status"] = "waiting_for_prediction_while_holding_pose"
            future = network.submit(
                request_prediction, client, images, observed.copy(), sdk.TASKS[args.task],
                deadline, report, chunk_index,
            )
            actions = wait_for_prediction(future, robot, hand, observed, SERVO_RUNNING, deadline)
            report["observation_age_at_response_seconds"] = time.monotonic() - observed_at
            if previous_segment_finished is not None:
                report["between_segments_wait_seconds"] = time.monotonic() - previous_segment_finished
                LOGGER.info(
                    "Segment %d waited %.3fs after the previous segment while holding pose",
                    chunk_index, report["between_segments_wait_seconds"],
                )
            assert_stationary(observed, read_state(robot, hand, SERVO_RUNNING))
            report["scene_stability_confirmed_by_feedback_checks"] = True
            report["status"] = "executing_segment"
            segment_started = time.monotonic()
            execute_segment(robot, hand, actions, observed, SERVO_RUNNING, deadline, report, chunk_index)
            segment_seconds = time.monotonic() - segment_started
            report["segment_execution_seconds"] = segment_seconds
            segment_fields = (
                "action_chunk_first", "action_chunk_last", "action_chunk_arm_min", "action_chunk_arm_max",
                "action_chunk_hand_min", "action_chunk_hand_max", "observed_state_at_segment_start",
                "arm_first_command", "arm_last_command", "hand_first_model_target", "hand_last_model_target",
                "hand_first_command", "hand_last_command", "feedback_after_segment",
                "arm_feedback_change_rad", "hand_feedback_change_raw", "final_arm_target_error_rad",
                "arm_targets_submitted", "hand_targets_submitted", "arm_hold_targets_submitted",
                "inference_wall_seconds", "server_elapsed_seconds", "png_encode_seconds",
                "request_roundtrip_seconds", "inference_client_total_seconds", "snapshot_total_seconds",
                "between_segments_wait_seconds",
            )
            segment_record = {
                "segment_index": chunk_index,
                "request_id": report.get("request_id"),
                "observation_age_at_response_seconds": report["observation_age_at_response_seconds"],
                "execution_seconds": segment_seconds,
                **{name: report[name] for name in segment_fields if name in report},
            }
            report.setdefault("segments", []).append(segment_record)
            report["segments_completed"] = len(report["segments"])
            for name in ("arm_targets_submitted", "hand_targets_submitted", "arm_hold_targets_submitted"):
                total_name = f"{name}_total"
                report[total_name] = report.get(total_name, 0) + report.get(name, 0)
            LOGGER.info(
                "Segment %d complete request_id=%s execution=%.3fs observation_age=%.3fs; holding pose for next observation",
                chunk_index, report.get("request_id"), segment_seconds,
                report["observation_age_at_response_seconds"],
            )
            previous_segment_finished = time.monotonic()
            report["status"] = "holding_between_segments"
        report["status"] = "trial_budget_exhausted"
        LOGGER.info("Trial budget reached after %d segments", report["segments_completed"])
        code = 0
    except KeyboardInterrupt:
        report.update(status="interrupted", error_code="operator_interrupted")
        code = 130
    except Exception as error:
        report.update(status="stopped", error_type=type(error).__name__, error_code=getattr(error, "code", str(error) if type(error) in (RuntimeError, ValueError, TimeoutError) else type(error).__name__))
        report["prediction_stage_at_failure"] = report.get("prediction_stage")
        if getattr(error, "status", None) is not None:
            report["http_status"] = error.status
        if getattr(error, "cause_type", None) is not None:
            report["network_error_type"] = error.cause_type
        if getattr(error, "cause_errno", None) is not None:
            report["network_error_errno"] = error.cause_errno
        LOGGER.error(
            "Session stopped stage=%s code=%s error_type=%s cause_type=%s cause_errno=%s",
            report.get("prediction_stage"), report["error_code"], type(error).__name__,
            getattr(error, "cause_type", None), getattr(error, "cause_errno", None),
        )
        print(f"Stopped: {report['error_code']}. No retry.")
    finally:
        if network is not None and client is not None:
            try:
                network.submit(client.close).result()
            except Exception as error:
                report["customer_client_close_error"] = type(error).__name__
                report["outcome_before_cleanup_failure"] = report.get("status")
                report["status"] = "cleanup_failed"
                code = 1
                LOGGER.exception("Could not close the remote inference client")
            finally:
                network.shutdown(wait=True, cancel_futures=True)
        if startup_homing_complete and connected and hand is not None:
            try:
                if robot.get_servo_state() == SERVO_RUNNING:
                    exit_home_started = time.monotonic()
                    home_arm_to_initial(robot, SERVO_RUNNING, keep_servo_open=True)
                    if robot.servo_active:
                        robot.close_servoj()
                    home_hand_to_initial(hand)
                    report["exit_homing_completed"] = True
                    report["exit_homing_seconds"] = time.monotonic() - exit_home_started
                    LOGGER.info("Exit arm/hand home took %.3fs", report["exit_homing_seconds"])
                else:
                    report["exit_homing_skipped"] = "controller_not_running"
                    print("Exit homing skipped because CR3 is not RUNNING; inspect the controller onsite.")
            except Exception as error:
                report["exit_homing_error_type"] = type(error).__name__
                print("Exit homing failed; use the onsite approved recovery procedure.")
        if not cleanup_devices(robot, hand, cameras, connected, report):
            code = 1
        report.update(finished_utc=datetime.now(timezone.utc).isoformat(), elapsed_seconds=time.monotonic() - started)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        LOGGER.info("Final status=%s elapsed=%.3fs report=%s", report["status"], report["elapsed_seconds"], args.report)
    print(f"Result: {report['status']}; report: {args.report}. This is not grasp-success certification.")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
