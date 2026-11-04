"""RealSense camera adapter with an explicit raw-image validation mode."""

from __future__ import annotations

import logging
from collections.abc import Mapping
from typing import Any

import cv2
import numpy as np


LOGGER = logging.getLogger(__name__)
CAMERA_NAMES = ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb")
DEFAULT_CAMERA_SERIALS = {
    "base_0_rgb": "254622075848",
    "left_wrist_0_rgb": "317222074617",
    "right_wrist_0_rgb": "262422074985",
}


class CameraInterfaceError(RuntimeError):
    """Base error raised by the camera adapter."""


def _load_lerobot_backend() -> tuple[str, type, type]:
    try:
        from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

        return "new", RealSenseCamera, RealSenseCameraConfig
    except ImportError as new_error:
        try:
            from lerobot.common.robot_devices.cameras.configs import IntelRealSenseCameraConfig
            from lerobot.common.robot_devices.cameras.intelrealsense import IntelRealSenseCamera

            return "old", IntelRealSenseCamera, IntelRealSenseCameraConfig
        except ImportError as old_error:
            raise CameraInterfaceError(
                "No compatible LeRobot RealSense backend found. Install LeRobot and pyrealsense2. "
                f"New backend: {new_error}; old backend: {old_error}"
            ) from old_error


class CR3O6CameraInterface:
    """Capture three RGB streams with optional strict raw-size validation."""

    def __init__(
        self,
        *,
        camera_serials: Mapping[str, str | int] | None = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        output_width: int = 224,
        output_height: int = 224,
        async_read: bool = True,
        read_timeout_ms: int = 500,
        strict_output_size: bool = False,
    ) -> None:
        serials = dict(DEFAULT_CAMERA_SERIALS if camera_serials is None else camera_serials)
        if set(serials) != set(CAMERA_NAMES):
            raise ValueError(f"camera_serials must contain exactly {CAMERA_NAMES}")
        if min(width, height, fps, output_width, output_height, read_timeout_ms) <= 0:
            raise ValueError("camera dimensions, fps, and timeout must be positive")
        self.camera_serials = {name: str(serials[name]) for name in CAMERA_NAMES}
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.output_width = int(output_width)
        self.output_height = int(output_height)
        self.async_read = bool(async_read)
        self.read_timeout_ms = int(read_timeout_ms)
        self.strict_output_size = strict_output_size
        self.backend_name, self._camera_class, self._config_class = _load_lerobot_backend()
        self._cameras = {
            name: self._make_camera(self.camera_serials[name]) for name in CAMERA_NAMES
        }

    def _make_camera(self, serial_number: str) -> Any:
        if self.backend_name == "new":
            config = self._config_class(
                serial_number_or_name=serial_number,
                fps=self.fps,
                width=self.width,
                height=self.height,
                use_rgb=True,
                use_depth=False,
            )
        else:
            config = self._config_class(
                serial_number=int(serial_number),
                fps=self.fps,
                width=self.width,
                height=self.height,
                color_mode="rgb",
                use_depth=False,
            )
        return self._camera_class(config)

    @property
    def connected(self) -> bool:
        return all(
            bool(getattr(camera, "is_connected", False)) for camera in self._cameras.values()
        )

    def connect(self) -> "CR3O6CameraInterface":
        connected: list[str] = []
        try:
            for name, camera in self._cameras.items():
                LOGGER.info("Connecting camera %s (serial=%s)", name, self.camera_serials[name])
                camera.connect()
                connected.append(name)
        except BaseException as exc:
            for name in reversed(connected):
                self._safe_disconnect(self._cameras[name])
            raise CameraInterfaceError(f"Failed to connect camera {name}: {exc}") from exc
        return self

    @staticmethod
    def _safe_disconnect(camera: Any) -> None:
        try:
            if bool(getattr(camera, "is_connected", False)):
                camera.disconnect()
        except Exception:
            LOGGER.exception("Failed to disconnect camera")

    def _read_one(self, camera: Any) -> np.ndarray:
        if self.async_read:
            frame = (
                camera.async_read(timeout_ms=self.read_timeout_ms)
                if self.backend_name == "new"
                else camera.async_read()
            )
        else:
            frame = camera.read()
        if isinstance(frame, tuple):
            frame = frame[0]
        image = np.asarray(frame)
        if image.ndim != 3 or image.shape[-1] != 3:
            raise CameraInterfaceError(f"Camera returned invalid image shape {image.shape}")
        if self.strict_output_size:
            if image.shape != (self.output_height, self.output_width, 3) or image.dtype != np.uint8:
                raise CameraInterfaceError("Raw camera size or dtype differs from the required RGB contract.")
            return np.ascontiguousarray(image)
        image = np.clip(image, 0, 255).astype(np.uint8, copy=False)
        if image.shape[:2] == (self.output_height, self.output_width):
            return np.ascontiguousarray(image)
        image_height, image_width = image.shape[:2]
        ratio = max(image_width / self.output_width, image_height / self.output_height)
        resized_width = max(1, int(image_width / ratio))
        resized_height = max(1, int(image_height / ratio))
        interpolation = cv2.INTER_AREA if ratio >= 1.0 else cv2.INTER_LINEAR
        resized = cv2.resize(image, (resized_width, resized_height), interpolation=interpolation)
        output = np.zeros((self.output_height, self.output_width, 3), dtype=np.uint8)
        top = (self.output_height - resized_height) // 2
        left = (self.output_width - resized_width) // 2
        output[top : top + resized_height, left : left + resized_width] = resized
        return output

    def read(self) -> dict[str, np.ndarray]:
        if not self.connected:
            raise CameraInterfaceError("Cameras are not connected; call connect() first")
        return {name: self._read_one(camera) for name, camera in self._cameras.items()}

    get_observation = read

    def disconnect(self) -> None:
        for camera in reversed(tuple(self._cameras.values())):
            self._safe_disconnect(camera)

    close = disconnect

    def __enter__(self) -> "CR3O6CameraInterface":
        return self.connect()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.disconnect()
