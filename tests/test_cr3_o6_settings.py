"""Hardware-free checks for the CR3/O6 deployment profile."""

import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "examples" / "CR3O6"
sys.path.insert(0, str(SCRIPT_DIR))

from cr3_o6_settings import load_settings  # noqa: E402


class CustomerSettingsTest(unittest.TestCase):
    def test_default_profile_matches_current_robot(self) -> None:
        settings = load_settings(SCRIPT_DIR / "config.toml")
        self.assertEqual(settings.robot_ip, "192.168.2.14")
        self.assertEqual(settings.nrc_sdk_dir, SCRIPT_DIR / "nrc_linux_x86_64")
        self.assertEqual(settings.top_camera, "base_0_rgb")
        self.assertEqual(settings.wrist_camera, "left_wrist_0_rgb")
        self.assertEqual(settings.side_camera, "right_wrist_0_rgb")
        self.assertEqual(settings.timeout_s, 10.0)
        self.assertIsNone(settings.o6_modbus)

    def test_relative_paths_use_profile_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            config = Path(temporary) / "robot.toml"
            config.write_text(
                textwrap.dedent(
                    """
                    [customer]
                    api_url = "https://example.com"
                    token_file = "secrets/token"
                    timeout_s = 4.5

                    [robot]
                    ip = "192.168.2.14"
                    nrc_sdk_dir = "vendor"
                    linkerhand_sdk_dir = "hand-sdk"
                    hand_config = "hand-sdk/setting.yaml"
                    o6_modbus = "/dev/ttyUSB1"

                    [cameras]
                    top = "base_0_rgb"
                    wrist = "left_wrist_0_rgb"
                    side = "right_wrist_0_rgb"
                    """
                )
            )
            settings = load_settings(config)
            self.assertEqual(settings.token_file, Path(temporary) / "secrets/token")
            self.assertEqual(settings.nrc_sdk_dir, Path(temporary) / "vendor")
            self.assertEqual(settings.linkerhand_sdk_dir, Path(temporary) / "hand-sdk")
            self.assertEqual(settings.o6_modbus, "/dev/ttyUSB1")

    def test_profile_cannot_enable_motion(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            config = Path(temporary) / "robot.toml"
            original = (SCRIPT_DIR / "config.toml").read_text()
            config.write_text(original + "\n[execution]\nexecute_live = true\n")
            with self.assertRaisesRegex(ValueError, "exactly"):
                load_settings(config)


if __name__ == "__main__":
    unittest.main()
