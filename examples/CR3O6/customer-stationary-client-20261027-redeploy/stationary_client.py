"""One locally approved stationary-observe-wait-validate-execute segment."""

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
import select
import sys
import threading
import time

import numpy as np


LOGGER = logging.getLogger("stationary_customer")
CLIENT_REVISION = "stationary-single-segment-r2"
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
        time.sleep(DT)
    return current


def snapshot(robot, hand, cameras, mapping, running_state, deadline):
    reference = verify_stationary(robot, hand, running_state, deadline)
    started = time.monotonic()
    local = cameras.read()
    images = {}
    for name in CAMERAS:
        image = np.asarray(local[mapping[name]])
        if image.shape != IMAGE_SHAPE or image.dtype != np.uint8:
            raise ValueError("raw_rgb_image_contract_invalid")
        images[name] = image.copy()
    current = read_state(robot, hand, running_state)
    assert_stationary(reference, current)
    ensure_time(deadline)
    return images, current, started


def wait_for_prediction(future, robot, hand, reference, running_state, deadline, cancelled=None):
    try:
        while not future.done():
            ensure_time(deadline)
            assert_stationary(reference, read_state(robot, hand, running_state))
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


def confirmation(word, message, deadline=None, poll=None):
    if not sys.stdin.isatty():
        raise RuntimeError("interactive_local_confirmation_required")
    print(message, flush=True)
    print(f"Type exactly {word!r}; any other input cancels.", flush=True)
    while True:
        if deadline is not None:
            ensure_time(deadline)
        if poll is not None:
            poll()
        timeout = 0.1 if deadline is None else min(0.1, max(0, deadline - time.monotonic()))
        if select.select([sys.stdin], [], [], timeout)[0]:
            if sys.stdin.readline().strip() != word:
                raise RuntimeError("operator_cancelled")
            return


def execute_segment(robot, hand, actions, reference, running_state, deadline, report):
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
                if time.monotonic() - before > HAND_DT:
                    raise TimeoutError("hand_write_exceeded_control_period")
        except BaseException as error:
            hand_errors.append(error)
            stop.set()

    try:
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
    report["controller_state_after_stop"] = robot.get_servo_state()
    if report["controller_state_after_stop"] == CONTROLLER_FAULT_STATE:
        raise RuntimeError("controller_reported_fault_after_stop")
    feedback = read_feedback(robot, hand)
    report.update(
        arm_feedback_change_rad=float(np.max(np.abs(feedback[:6] - reference[:6]))),
        hand_feedback_change_raw=float(np.max(np.abs(feedback[6:] - reference[6:]))),
        final_arm_target_error_rad=float(np.max(np.abs(feedback[:6] - last_arm))),
        physical_task_success_verified=False,
    )


def load_sdk(directory, manifest):
    path = directory / "customer_client.py"
    if hashlib.sha256(path.read_bytes()).hexdigest() != manifest["sdk_sha256"]:
        raise ValueError("sdk_checksum_changed")
    spec = importlib.util.spec_from_file_location("stationary_https_sdk", path)
    if spec is None or spec.loader is None:
        raise ValueError("sdk_import_failed")
    sdk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sdk)
    return sdk


def request_prediction(sdk, settings, manifest, images, state, task, deadline, report, cancelled=None):
    def check_request_allowed():
        ensure_time(deadline)
        if cancelled is not None and cancelled.is_set():
            raise RuntimeError("prediction_cancelled_before_motion")

    check_request_allowed()
    with sdk.CustomerPolicyClient(settings.api_url, settings.token_file, timeout=min(settings.timeout_s, 10)) as client:
        for operation in ("metadata", "reset", "infer"):
            check_request_allowed()
            client.timeout = min(settings.timeout_s, 10, deadline - time.monotonic())
            if operation == "metadata":
                metadata = client.get_modality_config()
                expires = datetime.fromisoformat(metadata["expires_utc"])
                if metadata["expires_utc"] != manifest["expires_utc"] or expires <= datetime.now(timezone.utc):
                    raise ValueError("unexpected_or_expired_service")
            elif operation == "reset":
                client.reset()
            else:
                report["prediction_requests_attempted"] = 1
                started = time.monotonic()
                actions, info = client.infer(images, state, task=task)
                if info["task"] != task or info["dt_seconds"] != DT:
                    raise ValueError("prediction_task_or_spacing_changed")
                report.update(
                    inference_wall_seconds=time.monotonic() - started,
                    request_id=info["request_id"], server_elapsed_seconds=info["server_elapsed_seconds"],
                    prediction_requests_completed=1,
                )
                check_request_allowed()
                return validate_actions(actions)
    raise RuntimeError("prediction_missing")


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
    parser.add_argument("--report", type=Path, default=Path("stationary-result.json"))
    parser.add_argument("--task", choices=("two_finger", "three_finger"), default="three_finger")
    args = parser.parse_args()
    if os.name != "posix":
        parser.error("Use the customer's existing supported Linux device environment.")
    if args.report.exists():
        parser.error("The report exists; preserve it and use a new filename for any separately approved trial.")
    with local_claim(args.config):
        return run_once(args, parser)


def run_once(args, parser):
    directory = Path(__file__).resolve().parent
    manifest = json.loads((directory / "session.json").read_text())
    for name, expected in manifest["component_sha256"].items():
        if Path(name).name != name or hashlib.sha256((directory / name).read_bytes()).hexdigest() != expected:
            parser.error("A supplied component changed; do not mix this package with older adapters.")
    service_expires = datetime.fromisoformat(manifest["expires_utc"])
    if service_expires.tzinfo is None or service_expires <= datetime.now(timezone.utc):
        parser.error("This prediction session has expired.")
    sdk = load_sdk(directory, manifest)
    from cr3_o6_settings import load_settings
    from cr3_o6_interface import CR3Interface, SERVO_RUNNING
    from cr3_o6_camera import CR3O6CameraInterface

    settings = load_settings(args.config)
    if settings.api_url.rstrip("/") != manifest["api_url"]:
        parser.error("The configured API URL does not match this session.")
    mapping = {"top": settings.top_camera, "wrist": settings.wrist_camera, "side": settings.side_camera}
    if set(mapping.values()) != {"base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb"}:
        parser.error("The camera mapping must preserve all three documented roles.")
    logging.basicConfig(level=logging.WARNING)
    print("One stationary segment only; no automatic power-on, fault clearing, homing or retry.")
    print(f"Client revision: {CLIENT_REVISION}")
    print(f"Software caps: arm {ARM_STEP} rad/50ms; tracking {ARM_TRACKING_ERROR} rad; hand {HAND_STEP} raw/100ms.")
    print(f"Waiting gates: arm {WAIT_ARM_TOLERANCE} rad, hand {WAIT_HAND_TOLERANCE} raw. These are not hardware safety certification.")
    confirmation("SITE CHECKED", "The onsite engineer must confirm mapping, units, hardware limits, emergency stop, clear workspace and existing hold/stop behavior. Other execution programs must be stopped.")
    started = time.monotonic()
    deadline = min(started + TRIAL_SECONDS, started + (service_expires - datetime.now(timezone.utc)).total_seconds())
    report = {
        "status": "starting", "control_mode": "stationary_single_segment", "client_revision": CLIENT_REVISION,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "api_origin": manifest["api_url"], "service_expires_utc": manifest["expires_utc"],
        "maximum_trial_seconds": TRIAL_SECONDS, "maximum_prediction_requests": 1,
        "maximum_prediction_segments": 1, "prediction_requests_attempted": 0,
        "prediction_requests_completed": 0, "automatic_retries": 0,
        "automatic_power_on": False, "automatic_homing": False,
        "operator_confirmed_onsite_conditions": True,
        "scene_stability_confirmed_by_operator_not_software": False,
        "physical_task_success_verified": False,
    }
    with args.report.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
    robot = None
    hand = None
    cameras = None
    connected = False
    code = 1
    try:
        robot = CR3Interface(settings.robot_ip, nrc_sdk_dir=settings.nrc_sdk_dir, servo_vmax=[50.1] * 7, servo_amax=[100.0] * 7, servo_jmax=[200.0] * 7)
        robot.connect()
        connected = True
        if robot.get_servo_state() != SERVO_RUNNING:
            raise RuntimeError("controller_not_ready_contact_onsite_engineer")
        report["controller_reported_running_at_start"] = True
        hand = hand_device(settings)
        cameras = CR3O6CameraInterface(output_width=640, output_height=480, strict_output_size=True, async_read=False)
        cameras.connect()
        images, observed, observed_at = snapshot(robot, hand, cameras, mapping, SERVO_RUNNING, deadline)
        report["status"] = "waiting_for_prediction_while_stationary"
        cancelled = threading.Event()
        with ThreadPoolExecutor(max_workers=1) as network:
            future = network.submit(request_prediction, sdk, settings, manifest, images, observed.copy(), sdk.TASKS[args.task], deadline, report, cancelled)
            try:
                actions = wait_for_prediction(future, robot, hand, observed, SERVO_RUNNING, deadline, cancelled)
            finally:
                cancelled.set()
        report["observation_age_at_response_seconds"] = time.monotonic() - observed_at
        print("Prediction received. No movement target has been submitted.")
        print("Confirm that the workpiece, fixture and workspace have not changed while waiting.")
        confirmation("RUN ONE SEGMENT", f"This submits one segment, then holds only its last arm target for feedback checking (at most {ARM_SETTLE_SECONDS}s). No homing or next segment.", deadline, lambda: assert_stationary(observed, read_state(robot, hand, SERVO_RUNNING)))
        report["operator_approved_single_segment"] = True
        report["scene_stability_confirmed_by_operator_not_software"] = True
        report["status"] = "executing_single_segment"
        execute_segment(robot, hand, actions, observed, SERVO_RUNNING, deadline, report)
        report["status"] = "single_segment_submitted"
        code = 0
    except KeyboardInterrupt:
        report.update(status="interrupted", error_code="operator_interrupted")
        code = 130
    except Exception as error:
        report.update(status="stopped", error_type=type(error).__name__, error_code=getattr(error, "code", str(error) if type(error) in (RuntimeError, ValueError, TimeoutError) else type(error).__name__))
        print(f"Stopped: {report['error_code']}. No retry or automatic homing.")
    finally:
        if not cleanup_devices(robot, hand, cameras, connected, report):
            code = 1
        report.update(finished_utc=datetime.now(timezone.utc).isoformat(), elapsed_seconds=time.monotonic() - started)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Result: {report['status']}; report: {args.report}. This is not grasp-success certification.")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
