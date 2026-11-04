"""Hardware-free checks for continuous CR3/O6 Customer API execution."""

from __future__ import annotations

from contextlib import ExitStack, redirect_stderr
import importlib.util
from io import StringIO
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "examples" / "CR3O6"
sys.path.insert(0, str(SCRIPT_DIR))
spec = importlib.util.spec_from_file_location(
    "deploy_cr3_o6_customer", SCRIPT_DIR / "deploy_cr3_o6_customer.py"
)
assert spec is not None and spec.loader is not None
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


class FakeClock:
    def __init__(self, now: float = 100.0):
        self.now = now

    def monotonic(self) -> float:
        return self.now

    def sleep(self, delay: float) -> None:
        self.now += delay


class FakeRobot:
    def __init__(self):
        self.position = np.zeros(6, dtype=np.float32)
        self.commands: list[np.ndarray] = []
        self.command_threads: list[str] = []
        self.servoj_open = False
        self.open_count = 0
        self.close_count = 0
        self.fail_hold = False

    def get_servo_state(self) -> int:
        return 3

    def get_joint_positions(self) -> list[float]:
        return self.position.tolist()

    def open_servoj(self) -> None:
        self.servoj_open = True
        self.open_count += 1

    def close_servoj(self) -> None:
        self.servoj_open = False
        self.close_count += 1

    def send_joint_positions(self, positions: list[float]) -> None:
        assert self.servoj_open
        if self.fail_hold and threading.current_thread().name == "cr3-live-servoj-hold":
            raise RuntimeError("simulated hold write failure")
        self.position = np.asarray(positions, dtype=np.float32)
        self.commands.append(self.position.copy())
        self.command_threads.append(threading.current_thread().name)


class FakeHand:
    def __init__(self, fail: bool = False):
        self.position = np.array([157, 95, 175, 174, 0, 0], dtype=np.float32)
        self.commands: list[list[int]] = []
        self.fail = fail
        self.reads = 0

    def get_state(self) -> list[float]:
        self.reads += 1
        return self.position.tolist()

    def finger_move(self, positions: list[int]) -> None:
        if self.fail:
            raise RuntimeError("simulated O6 command failure")
        self.commands.append(positions)
        self.position = np.asarray(positions, dtype=np.float32)


class CustomerLiveExecutionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.original_time = client.time
        client.time = FakeClock()
        self.robot = FakeRobot()
        self.hand = FakeHand()
        self.actions = np.zeros((16, 12), dtype=np.float32)
        self.actions[:, :6] = 0.03
        self.actions[:, 6:9] = [150, 90, 170]
        self.observed_state = np.array(
            [0] * 6 + [157, 95, 175, 0, 0, 0], dtype=np.float32
        )

    def tearDown(self) -> None:
        client.time = self.original_time

    def test_cli_loads_an_alternate_configuration(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary) / "robot.toml"
            original = (SCRIPT_DIR / "config.toml").read_text()
            profile.write_text(original.replace('ip = "192.168.2.14"', 'ip = "192.168.2.15"'))
            args = client._parser().parse_args(
                ["--config", str(profile), "--execute-live", "--confirm-motion"]
            )
            self.assertEqual(client.load_settings(args.config).robot_ip, "192.168.2.15")

    def test_initial_hand_home_moves_all_six_channels_in_small_steps(self) -> None:
        self.hand.position = np.array([111, 40, 129, 174, 0, 0], dtype=np.float32)
        client._home_hand_before_motion(self.hand)
        self.assertEqual(self.hand.commands[-1], [157, 95, 175, 0, 0, 0])
        self.assertEqual(self.hand.reads, 2)
        self.assertEqual(len(self.hand.commands), 29)
        previous = [111, 40, 129, 174, 0, 0]
        for command in self.hand.commands:
            self.assertTrue(
                all(abs(value - old) <= client.HAND_HOME_STEP_RAW
                    for value, old in zip(command, previous))
            )
            previous = command
        self.assertEqual(self.robot.commands, [])

    def test_initial_hand_home_does_not_write_when_already_at_target(self) -> None:
        self.hand.position = np.array(client.OPENPI_INITIAL_HAND_POSITION, dtype=np.float32)
        client._home_hand_before_motion(self.hand)
        self.assertEqual(self.hand.commands, [])
        self.assertEqual(self.hand.reads, 1)

    def test_initial_hand_home_aborts_on_slow_write(self) -> None:
        original_move = self.hand.finger_move

        def slow_move(positions: list[int]) -> None:
            original_move(positions)
            client.time.sleep(0.12)

        self.hand.finger_move = slow_move
        with self.assertRaisesRegex(RuntimeError, "over 100ms"):
            client._home_hand_before_motion(self.hand)
        self.assertEqual(len(self.hand.commands), 1)
        self.assertEqual(self.hand.reads, 1)

    def test_arm_returns_to_zero_before_and_after_a_session(self) -> None:
        self.robot.position = np.full(6, 0.12, dtype=np.float32)
        client._home_arm(self.robot)
        self.assertFalse(self.robot.servoj_open)
        self.assertGreaterEqual(len(self.robot.commands), 60)
        np.testing.assert_allclose(self.robot.position, np.zeros(6), atol=1e-6)
        self.robot.position = np.full(6, -0.24, dtype=np.float32)
        client._home_arm(self.robot)
        self.assertFalse(self.robot.servoj_open)
        np.testing.assert_allclose(self.robot.position, np.zeros(6), atol=1e-6)

    def test_arm_home_holds_zero_until_measured_pose_settles(self) -> None:
        class LaggingRobot(FakeRobot):
            def send_joint_positions(self, positions: list[float]) -> None:
                assert self.servoj_open
                target = np.asarray(positions, dtype=np.float32)
                self.position += (target - self.position) * 0.12
                self.commands.append(target.copy())

        robot = LaggingRobot()
        robot.position = np.full(6, 0.78, dtype=np.float32)
        client._home_arm(robot)
        self.assertGreater(len(robot.commands), 60)
        self.assertTrue(np.all(robot.commands[-1] == 0))
        self.assertLessEqual(
            float(np.max(np.abs(robot.position))), client.ARM_HOME_TRACKING_TOLERANCE_RAD
        )
        self.assertFalse(robot.servoj_open)

    def test_arm_home_timeout_closes_servoj_without_claiming_success(self) -> None:
        class StalledRobot(FakeRobot):
            def send_joint_positions(self, positions: list[float]) -> None:
                assert self.servoj_open
                self.commands.append(np.asarray(positions, dtype=np.float32))

        robot = StalledRobot()
        robot.position = np.full(6, 0.12, dtype=np.float32)
        with self.assertRaisesRegex(RuntimeError, "did not settle at zero"):
            client._home_arm(robot)
        self.assertEqual(robot.close_count, 1)
        self.assertFalse(robot.servoj_open)

    def test_arm_home_detects_servo_fault_after_servoj_closes(self) -> None:
        class FaultOnCloseRobot(FakeRobot):
            def __init__(self):
                super().__init__()
                self.state = 3

            def get_servo_state(self) -> int:
                return self.state

            def close_servoj(self) -> None:
                super().close_servoj()
                self.state = 2

        robot = FaultOnCloseRobot()
        robot.position = np.full(6, 0.12, dtype=np.float32)
        with self.assertRaisesRegex(RuntimeError, "stopped when ServoJ closed"):
            client._home_arm(robot)
        self.assertEqual(robot.close_count, 1)

    def test_exit_homing_reuses_live_servoj_until_zero_is_reached(self) -> None:
        client.time = self.original_time
        self.robot.position = np.full(6, 0.2, dtype=np.float32)
        retained = []
        with patch.object(client, "_run_live_inference_pipeline", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                client._run_inference_loop(
                    SimpleNamespace(duration_s=0.0),
                    Mock(),
                    self.robot,
                    self.hand,
                    None,
                    {},
                    retained.append,
                )
        self.assertEqual(len(retained), 1)
        self.assertTrue(self.robot.servoj_open)
        self.assertEqual((self.robot.open_count, self.robot.close_count), (1, 0))
        try:
            client._home_arm(self.robot, servo=retained[0])
            np.testing.assert_allclose(self.robot.position, np.zeros(6), atol=1e-6)
            self.assertEqual((self.robot.open_count, self.robot.close_count), (1, 0))
        finally:
            retained[0].stop()
        self.assertEqual((self.robot.open_count, self.robot.close_count), (1, 1))

    def test_live_chunk_keeps_servoj_open_until_session_end(self) -> None:
        client.time = self.original_time
        self.hand.position = np.asarray(client.OPENPI_INITIAL_HAND_POSITION, dtype=np.float32)
        observed = np.concatenate((np.zeros(6, dtype=np.float32), self.hand.position))
        actions = np.zeros((16, 12), dtype=np.float32)
        actions[:, :6] = 1.0
        actions[:, 6:9] = [100, 80, 130]
        servo = client._LiveServoKeeper(self.robot)
        servo.start()
        try:
            client._execute_live_chunk(
                self.robot, self.hand, actions, observed, time.monotonic(), servo
            )
            self.assertTrue(self.robot.servoj_open)
            self.assertEqual(self.robot.open_count, 1)
            self.assertEqual(self.robot.close_count, 0)
        finally:
            servo.stop()
        chunk_commands = [
            target
            for target, source in zip(self.robot.commands, self.robot.command_threads)
            if source == threading.current_thread().name
        ]
        self.assertEqual(len(chunk_commands), 16)
        self.assertEqual(len(self.hand.commands), 8)
        self.assertFalse(self.robot.servoj_open)
        self.assertEqual(self.robot.close_count, 1)
        self.assertGreater(chunk_commands[-1][0], 0.15)
        arm_history = np.vstack((np.zeros((1, 6)), np.stack(chunk_commands)))
        self.assertLessEqual(
            np.max(np.abs(np.diff(arm_history, axis=0))),
            client.LIVE_ARM_MAX_STEP_RAD + 1e-5,
        )
        self.assertTrue(all(command[3:] == [0, 0, 0] for command in self.hand.commands))

    def test_live_keeper_holds_last_target_only_between_chunks(self) -> None:
        client.time = self.original_time
        servo = client._LiveServoKeeper(self.robot)
        servo.start()
        try:
            time.sleep(0.12)
            self.assertGreaterEqual(len(self.robot.commands), 2)
            np.testing.assert_allclose(servo.begin_chunk(), np.zeros(6))
            target = np.full(6, 0.02, dtype=np.float32)
            servo.send_chunk_target(target)
            count_during_chunk = len(self.robot.commands)
            time.sleep(0.12)
            self.assertEqual(len(self.robot.commands), count_during_chunk)
            servo.end_chunk()
            time.sleep(0.12)
            self.assertGreater(len(self.robot.commands), count_during_chunk)
            np.testing.assert_allclose(self.robot.commands[-1], target)
        finally:
            servo.stop()
        self.assertEqual(self.robot.open_count, 1)
        self.assertEqual(self.robot.close_count, 1)

    def test_two_live_chunks_reuse_one_servoj_session(self) -> None:
        client.time = self.original_time
        self.hand.position = np.asarray(client.OPENPI_INITIAL_HAND_POSITION, dtype=np.float32)
        actions = np.zeros((16, 12), dtype=np.float32)
        actions[:, :6] = 1.0
        actions[:, 6:9] = [100, 80, 130]
        servo = client._LiveServoKeeper(self.robot)
        servo.start()
        try:
            for _ in range(2):
                observed = np.concatenate((self.robot.position, self.hand.position))
                client._execute_live_chunk(
                    self.robot, self.hand, actions, observed, time.monotonic(), servo
                )
                self.assertTrue(self.robot.servoj_open)
                self.assertEqual(self.robot.close_count, 0)
        finally:
            servo.stop()
        self.assertEqual(self.robot.open_count, 1)
        self.assertEqual(self.robot.close_count, 1)
        self.assertEqual(
            self.robot.command_threads.count(threading.current_thread().name), 32
        )

    def test_live_hold_failure_aborts_and_closes_servoj(self) -> None:
        client.time = self.original_time
        self.robot.fail_hold = True
        servo = client._LiveServoKeeper(self.robot)
        servo.start()
        try:
            time.sleep(0.08)
            with self.assertRaisesRegex(RuntimeError, "simulated hold write failure"):
                servo.raise_if_failed()
            self.assertFalse(self.robot.servoj_open)
        finally:
            servo.stop()
        self.assertEqual(self.robot.close_count, 1)

    def test_live_loop_reobserves_and_keeps_requesting_until_interrupted(self) -> None:
        client.time = self.original_time
        args = SimpleNamespace(
            duration_s=0.0,
        )
        customer = Mock()
        customer.infer.side_effect = [
            (self.actions, {"request_id": "a", "mode": "shadow"}),
            (self.actions, {"request_id": "b", "mode": "shadow"}),
            KeyboardInterrupt(),
        ]
        with patch.object(
            client, "_read_observation", return_value=(None, self.observed_state)
        ) as read:
            with patch.object(client, "_execute_live_chunk") as execute:
                with self.assertRaises(KeyboardInterrupt):
                    client._run_inference_loop(args, customer, self.robot, self.hand, None, {})
        self.assertEqual(read.call_count, 3)
        self.assertEqual(execute.call_count, 2)
        self.assertEqual(self.robot.open_count, 1)
        self.assertEqual(self.robot.close_count, 1)

    def test_live_pipeline_requests_next_action_while_motion_is_running(self) -> None:
        client.time = self.original_time
        args = SimpleNamespace(
            duration_s=0.0,
        )
        motion_running = threading.Event()
        next_request_during_motion = threading.Event()

        def infer(_images, _state):
            if not motion_running.is_set():
                return self.actions, {"request_id": "first", "mode": "shadow"}
            next_request_during_motion.set()
            raise KeyboardInterrupt()

        def execute(_robot, _hand, _actions, _state, _at, _servo, *, on_step, cancel):
            motion_running.set()
            on_step(2)
            while not cancel.wait(0.01):
                pass
            motion_running.clear()

        with patch.object(client, "_read_observation", return_value=(None, self.observed_state)):
            with patch.object(client, "_execute_live_chunk", side_effect=execute):
                with self.assertRaises(KeyboardInterrupt):
                    client._run_inference_loop(args, Mock(infer=infer), self.robot, self.hand, None, {})
        self.assertTrue(next_request_during_motion.is_set())
        self.assertEqual(self.robot.open_count, 1)
        self.assertEqual(self.robot.close_count, 1)

    def test_live_pipeline_accepts_result_just_over_two_seconds_old(self) -> None:
        client.time = self.original_time
        args = SimpleNamespace(
            duration_s=0.0,
        )
        customer = Mock()

        def infer(_images, _state):
            if customer.infer.call_count == 1:
                time.sleep(2.05)
                return self.actions, {"request_id": "delayed", "mode": "shadow"}
            raise KeyboardInterrupt()

        customer.infer.side_effect = infer

        def execute(_robot, _hand, _actions, _state, _at, _servo, *, on_step, cancel):
            on_step(2)

        with patch.object(client, "_read_observation", return_value=(None, self.observed_state)):
            with patch.object(client, "_execute_live_chunk", side_effect=execute) as applied:
                with self.assertRaises(KeyboardInterrupt):
                    client._run_inference_loop(
                        args, customer, self.robot, self.hand, None, {}
                    )
        applied.assert_called_once()

    def test_live_chunk_rejects_result_over_three_seconds_old(self) -> None:
        client.time = self.original_time
        with self.assertRaisesRegex(RuntimeError, "stale"):
            client._execute_live_chunk(
                self.robot,
                self.hand,
                self.actions,
                self.observed_state,
                time.monotonic() - client.LIVE_MAX_OBSERVATION_AGE_S - 0.1,
                Mock(),
            )

    def test_live_chunk_bridges_motion_drift_after_slow_inference(self) -> None:
        client.time = self.original_time
        self.hand.position = np.asarray(client.OPENPI_INITIAL_HAND_POSITION, dtype=np.float32)
        observed = np.concatenate((np.zeros(6, dtype=np.float32), self.hand.position))
        self.robot.position = np.full(6, 0.08, dtype=np.float32)
        actions = np.zeros((16, 12), dtype=np.float32)
        actions[:, :6] = 0.15
        actions[:, 6:9] = [157, 95, 175]
        servo = client._LiveServoKeeper(self.robot)
        servo.start()
        try:
            client._execute_live_chunk(
                self.robot, self.hand, actions, observed, time.monotonic() - 2.1, servo
            )
        finally:
            servo.stop()
        self.assertEqual(self.robot.open_count, 1)
        self.assertEqual(self.robot.close_count, 1)
        self.assertEqual(
            self.robot.command_threads.count(threading.current_thread().name), 16
        )

    def test_live_hand_observation_and_write_share_one_bus_lock(self) -> None:
        client.time = self.original_time
        overlap = []
        busy = False
        start = threading.Event()

        class CheckedHand:
            def get_state(self):
                nonlocal busy
                if busy:
                    overlap.append("read")
                busy = True
                time.sleep(0.02)
                busy = False
                return [157, 95, 175, 0, 0, 0]

            def finger_move(self, _target):
                nonlocal busy
                if busy:
                    overlap.append("write")
                busy = True
                time.sleep(0.02)
                busy = False

        hand = client._SynchronizedHand(CheckedHand())

        def write():
            start.wait()
            hand.finger_move([157, 95, 175, 0, 0, 0])

        thread = threading.Thread(target=write)
        thread.start()
        start.set()
        hand.get_state()
        thread.join()
        self.assertEqual(overlap, [])

    def test_live_cli_requires_explicit_motion_confirmation(self) -> None:
        for arguments in ([], ["--execute-live"], ["--confirm-motion"]):
            with self.subTest(arguments=arguments):
                output = StringIO()
                with patch.object(sys, "argv", ["deploy.py", *arguments]):
                    with redirect_stderr(output), patch.object(client, "_load_customer_client") as sdk:
                        with self.assertRaises(SystemExit) as raised:
                            client.main()
                self.assertEqual(raised.exception.code, 2)
                self.assertIn("--execute-live --confirm-motion", output.getvalue())
                sdk.assert_not_called()

    def test_removed_diagnostic_options_are_rejected(self) -> None:
        for option in (
            "--once",
            "--period-s",
            "--execute-shadow-one-chunk",
            "--execute-shadow-multi-chunk",
            "--max-chunks",
            "--home-hand-only",
            "--metadata-only",
            "--measure-control-io",
            "--measure-o6-write",
            "--verbose",
        ):
            with self.subTest(option=option), redirect_stderr(StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    client._parser().parse_args([option])
                self.assertEqual(raised.exception.code, 2)

    def test_negative_duration_is_rejected_before_loading_hardware(self) -> None:
        output = StringIO()
        with patch.object(
            sys,
            "argv",
            ["deploy.py", "--execute-live", "--confirm-motion", "--duration-s", "-1"],
        ):
            with redirect_stderr(output), patch.object(client, "_load_customer_client") as sdk:
                with self.assertRaises(SystemExit) as raised:
                    client.main()
        self.assertEqual(raised.exception.code, 2)
        self.assertIn("--duration-s must be non-negative", output.getvalue())
        sdk.assert_not_called()

    def test_live_main_homes_before_inference_and_after_interrupt(self) -> None:
        events = []

        class SessionRobot:
            def __init__(self, *args, **kwargs):
                events.append("robot created")
                self.state = 1

            def get_servo_state(self):
                return self.state

            def power_on(self):
                events.append("power on")
                self.state = 3

            def power_off(self):
                events.append("power off")

            def connect(self):
                events.append("robot connected")

            def disconnect(self):
                events.append("robot disconnected")

        class SessionHand:
            def __init__(self, *args, **kwargs):
                events.append("hand connected")

        class SessionCameras:
            def connect(self):
                events.append("cameras connected")

            def disconnect(self):
                events.append("cameras disconnected")

        class SessionCustomer:
            def __init__(self, *args, **kwargs):
                pass

            def get_modality_config(self):
                events.append("metadata")
                return {}

            def reset(self):
                events.append("reset")

            def close(self):
                events.append("customer closed")

        fake_camera_module = ModuleType("cr3_o6_camera")
        fake_camera_module.CR3O6CameraInterface = SessionCameras

        def home_hand(_hand):
            events.append("hand home")

        def home_arm(_robot, servo=None):
            events.append("arm home")

        class SessionServo:
            def stop(self):
                events.append("ServoJ closed")

        def stop_during_loop(*args):
            events.append("inference loop")
            args[-1](SessionServo())
            raise KeyboardInterrupt()

        import cr3_o6_interface
        import linkerhand_compat

        with patch.object(sys, "argv", ["deploy.py", "--execute-live", "--confirm-motion"]):
            with patch.dict(sys.modules, {"cr3_o6_camera": fake_camera_module}):
                with patch.object(cr3_o6_interface, "CR3Interface", SessionRobot):
                    with patch.object(linkerhand_compat, "patch_pymodbus_slave_argument"):
                        with patch.object(client, "_load_customer_client", return_value=SessionCustomer):
                            with patch.object(client, "_load_hand_api", return_value=SessionHand):
                                with patch.object(client, "_load_hand_config", return_value="tty"):
                                    with patch.object(client, "_check_metadata"):
                                        with patch.object(client, "_home_hand_before_motion", home_hand):
                                            with patch.object(client, "_home_arm", home_arm):
                                                with patch.object(client, "_close_hand"):
                                                    with patch.object(
                                                        client,
                                                        "_run_inference_loop",
                                                        stop_during_loop,
                                                    ):
                                                        result = client.main()
        self.assertEqual(result, 130)
        self.assertEqual(events.count("arm home"), 2)
        self.assertEqual(events.count("hand home"), 2)
        self.assertEqual(events.count("ServoJ closed"), 1)
        self.assertLess(events.index("arm home", events.index("inference loop")), events.index("ServoJ closed"))
        self.assertLess(events.index("ServoJ closed"), events.index("robot disconnected"))
        self.assertEqual(events.count("power on"), 1)
        self.assertNotIn("power off", events)
        self.assertLess(events.index("hand home"), events.index("cameras connected"))
        self.assertLess(events.index("arm home"), events.index("cameras connected"))
        self.assertLess(events.index("inference loop"), len(events) - 4)
        self.assertEqual(events[-1], "customer closed")

    def test_live_interrupt_with_servo_fault_skips_exit_motion_and_power_off(self) -> None:
        events = []

        class SessionRobot:
            def __init__(self, *args, **kwargs):
                self.state = 1

            def connect(self):
                events.append("robot connected")

            def get_servo_state(self):
                return self.state

            def power_on(self):
                self.state = 3
                events.append("power on")

            def power_off(self):
                events.append("power off")

            def disconnect(self):
                events.append("robot disconnected")

        class SessionHand:
            def __init__(self, *args, **kwargs):
                events.append("hand connected")

        class SessionCameras:
            def connect(self):
                events.append("cameras connected")

            def disconnect(self):
                events.append("cameras disconnected")

        class SessionCustomer:
            def __init__(self, *args, **kwargs):
                pass

            def get_modality_config(self):
                return {}

            def reset(self):
                pass

            def close(self):
                events.append("customer closed")

        def stop_with_fault(_args, _customer, robot, *_rest):
            robot.state = 2
            raise KeyboardInterrupt()

        fake_camera_module = ModuleType("cr3_o6_camera")
        fake_camera_module.CR3O6CameraInterface = SessionCameras
        import cr3_o6_interface
        import linkerhand_compat

        with ExitStack() as stack:
            stack.enter_context(
                patch.object(sys, "argv", ["deploy.py", "--execute-live", "--confirm-motion"])
            )
            stack.enter_context(patch.dict(sys.modules, {"cr3_o6_camera": fake_camera_module}))
            stack.enter_context(patch.object(cr3_o6_interface, "CR3Interface", SessionRobot))
            stack.enter_context(patch.object(linkerhand_compat, "patch_pymodbus_slave_argument"))
            stack.enter_context(
                patch.object(client, "_load_customer_client", return_value=SessionCustomer)
            )
            stack.enter_context(patch.object(client, "_load_hand_api", return_value=SessionHand))
            stack.enter_context(patch.object(client, "_load_hand_config", return_value="tty"))
            stack.enter_context(patch.object(client, "_check_metadata"))
            stack.enter_context(
                patch.object(
                    client,
                    "_home_hand_before_motion",
                    side_effect=lambda _hand: events.append("hand home"),
                )
            )
            stack.enter_context(
                patch.object(
                    client, "_home_arm", side_effect=lambda _robot: events.append("arm home")
                )
            )
            stack.enter_context(patch.object(client, "_close_hand"))
            stack.enter_context(
                patch.object(client, "_run_inference_loop", side_effect=stop_with_fault)
            )
            result = client.main()

        self.assertEqual(result, 1)
        self.assertEqual(events.count("arm home"), 1)
        self.assertEqual(events.count("hand home"), 1)
        self.assertNotIn("power off", events)
        self.assertEqual(events[-1], "customer closed")

    def test_live_startup_does_not_clear_existing_servo_fault(self) -> None:
        import cr3_o6_interface
        import linkerhand_compat

        robot = Mock()
        robot.get_servo_state.return_value = cr3_o6_interface.SERVO_ERROR
        camera = Mock()
        customer = Mock()
        fake_camera_module = ModuleType("cr3_o6_camera")
        fake_camera_module.CR3O6CameraInterface = lambda: camera

        with ExitStack() as stack:
            stack.enter_context(
                patch.object(sys, "argv", ["deploy.py", "--execute-live", "--confirm-motion"])
            )
            stack.enter_context(patch.dict(sys.modules, {"cr3_o6_camera": fake_camera_module}))
            stack.enter_context(patch.object(cr3_o6_interface, "CR3Interface", return_value=robot))
            stack.enter_context(patch.object(linkerhand_compat, "patch_pymodbus_slave_argument"))
            stack.enter_context(
                patch.object(client, "_load_customer_client", return_value=lambda *a, **k: customer)
            )
            stack.enter_context(patch.object(client, "_load_hand_api"))
            stack.enter_context(patch.object(client, "_load_hand_config", return_value="tty"))
            stack.enter_context(patch.object(client, "_check_metadata"))
            stack.enter_context(patch.object(client, "_home_arm"))
            self.assertEqual(client.main(), 1)

        robot.power_on.assert_not_called()
        robot.power_off.assert_not_called()
        robot.disconnect.assert_called_once_with()
        customer.close.assert_called_once_with()

    def test_failed_startup_homing_does_not_reopen_servoj_on_exit(self) -> None:
        import cr3_o6_interface
        import linkerhand_compat

        robot = Mock()
        robot.get_servo_state.return_value = cr3_o6_interface.SERVO_RUNNING
        camera = Mock()
        customer = Mock()
        fake_camera_module = ModuleType("cr3_o6_camera")
        fake_camera_module.CR3O6CameraInterface = lambda: camera

        with ExitStack() as stack:
            stack.enter_context(
                patch.object(sys, "argv", ["deploy.py", "--execute-live", "--confirm-motion"])
            )
            stack.enter_context(patch.dict(sys.modules, {"cr3_o6_camera": fake_camera_module}))
            stack.enter_context(patch.object(cr3_o6_interface, "CR3Interface", return_value=robot))
            stack.enter_context(patch.object(linkerhand_compat, "patch_pymodbus_slave_argument"))
            stack.enter_context(
                patch.object(client, "_load_customer_client", return_value=lambda *a, **k: customer)
            )
            stack.enter_context(patch.object(client, "_load_hand_api"))
            stack.enter_context(patch.object(client, "_load_hand_config", return_value="tty"))
            stack.enter_context(patch.object(client, "_check_metadata"))
            home_arm = stack.enter_context(
                patch.object(client, "_home_arm", side_effect=RuntimeError("home did not settle"))
            )
            home_hand = stack.enter_context(patch.object(client, "_home_hand_before_motion"))
            self.assertEqual(client.main(), 1)

        home_arm.assert_called_once_with(robot)
        home_hand.assert_called_once()
        camera.connect.assert_not_called()
        robot.power_off.assert_not_called()
        robot.disconnect.assert_called_once_with()

    def test_live_interrupt_during_startup_still_homes_both_devices(self) -> None:
        events = []

        class SessionRobot:
            def __init__(self, *args, **kwargs):
                pass

            def connect(self):
                events.append("robot connected")

            def get_servo_state(self):
                return 3

            def disconnect(self):
                events.append("robot disconnected")

        class SessionHand:
            def __init__(self, *args, **kwargs):
                events.append("hand connected")

        class SessionCameras:
            def disconnect(self):
                events.append("cameras disconnected")

        class SessionCustomer:
            def __init__(self, *args, **kwargs):
                pass

            def get_modality_config(self):
                return {}

            def reset(self):
                pass

            def close(self):
                events.append("customer closed")

        def home_hand(_hand):
            events.append("hand home")
            if events.count("hand home") == 1:
                raise KeyboardInterrupt()

        def home_arm(_robot, servo=None):
            events.append("arm home")

        fake_camera_module = ModuleType("cr3_o6_camera")
        fake_camera_module.CR3O6CameraInterface = SessionCameras
        import cr3_o6_interface
        import linkerhand_compat

        with ExitStack() as stack:
            stack.enter_context(
                patch.object(sys, "argv", ["deploy.py", "--execute-live", "--confirm-motion"])
            )
            stack.enter_context(patch.dict(sys.modules, {"cr3_o6_camera": fake_camera_module}))
            stack.enter_context(patch.object(cr3_o6_interface, "CR3Interface", SessionRobot))
            stack.enter_context(patch.object(linkerhand_compat, "patch_pymodbus_slave_argument"))
            stack.enter_context(patch.object(client, "_load_customer_client", return_value=SessionCustomer))
            stack.enter_context(patch.object(client, "_load_hand_api", return_value=SessionHand))
            stack.enter_context(patch.object(client, "_load_hand_config", return_value="tty"))
            stack.enter_context(patch.object(client, "_check_metadata"))
            stack.enter_context(patch.object(client, "_home_hand_before_motion", home_hand))
            stack.enter_context(patch.object(client, "_home_arm", home_arm))
            stack.enter_context(patch.object(client, "_close_hand"))
            result = client.main()

        self.assertEqual(result, 130)
        self.assertEqual(events.count("arm home"), 1)
        self.assertEqual(events.count("hand home"), 2)
        hand_home_indices = [index for index, event in enumerate(events) if event == "hand home"]
        self.assertLess(events.index("arm home"), hand_home_indices[1])
        self.assertLess(events.index("arm home"), events.index("robot disconnected"))
        self.assertEqual(events[-1], "customer closed")


if __name__ == "__main__":
    unittest.main()
