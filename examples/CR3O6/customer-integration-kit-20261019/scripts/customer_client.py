"""Customer HTTPS client with lossless observation encoding and response validation."""

import asyncio
import base64
import binascii
import io
import json
import os
from pathlib import Path
import re
import threading
import time
from urllib.parse import urlsplit
import uuid
import warnings

import numpy as np
from PIL import Image


CONTRACT_VERSION = "customer-api-v1"
TASK = "Pick up the motor and place it on the right side with the protruding side facing left."
CAMERAS = ("top", "wrist", "side")
IMAGE_SHAPE = (224, 224, 3)
MAX_BODY_BYTES = 2 * 1024 * 1024
MAX_IMAGE_BYTES = 512 * 1024
STATE_NAMES = tuple(f"cr3.q{index}.rad" for index in range(1, 7)) + tuple(
    f"o6.{finger}.position" for finger in ("thumb_flex", "thumb_yaw", "index_flex", "middle_flex", "ring_flex", "little_flex")
)
ACTION_NAMES = tuple(f"cr3.target_q{index}.rad" for index in range(1, 7)) + tuple(
    f"o6.{finger}.command" for finger in ("thumb_flex", "thumb_yaw", "index_flex", "middle_flex", "ring_flex", "little_flex")
)


def request_identifier(value):
    if not isinstance(value, str) or re.fullmatch(r"[A-Za-z0-9_-]{1,64}", value) is None:
        raise ValueError("request_id must contain 1-64 ASCII letters, digits, underscores or hyphens.")
    return value


def numeric_array(value, shape, name):
    try:
        array = np.asarray(value)
    except (ValueError, TypeError):
        raise ValueError(f"{name} must be a numeric array of shape {shape}.") from None
    if array.shape != shape or array.dtype.kind not in "fiu":
        raise ValueError(f"{name} must be a numeric array of shape {shape}.")
    if isinstance(value, (list, tuple)) and any(isinstance(item, (bool, np.bool_)) for item in np.asarray(value, dtype=object).flat):
        raise ValueError(f"{name} must not contain booleans.")
    if not np.isfinite(array).all() or np.any(np.abs(array) > np.finfo(np.float32).max):
        raise ValueError(f"{name} must contain finite float32 values.")
    return array.astype(np.float32)


def checked_state(value):
    state = numeric_array(value, (12,), "state")
    if np.any(state[9:] != 0):
        raise ValueError("State channels 9-11 are intentionally fixed at zero.")
    return state


def encode_image(value):
    if not isinstance(value, np.ndarray) or value.shape != IMAGE_SHAPE or value.dtype != np.uint8:
        raise ValueError("Each image must be a uint8 RGB array of shape (224, 224, 3).")
    buffer = io.BytesIO()
    Image.fromarray(value).save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def decode_image(value):
    if not isinstance(value, str) or len(value) > 4 * ((MAX_IMAGE_BYTES + 2) // 3):
        raise ValueError("Each image must be a bounded base64 PNG string.")
    try:
        encoded = base64.b64decode(value, validate=True)
        if len(encoded) > MAX_IMAGE_BYTES:
            raise ValueError("Image is too large.")
        with warnings.catch_warnings():
            warnings.simplefilter("error", Image.DecompressionBombWarning)
            with Image.open(io.BytesIO(encoded)) as image:
                if image.format != "PNG" or image.size != (224, 224) or image.mode != "RGB" or getattr(image, "n_frames", 1) != 1:
                    raise ValueError("Wrong image format.")
                image.load()
                return np.array(image, dtype=np.uint8)
    except (ValueError, OSError, binascii.Error, Image.DecompressionBombError, Image.DecompressionBombWarning):
        raise ValueError("Each image must be a single-frame 224x224 RGB PNG.") from None


def encode_request(images, state, task=TASK, request_id=None):
    if not isinstance(images, dict) or set(images) != set(CAMERAS):
        raise ValueError("Exactly the top, wrist and side cameras are required.")
    if task != TASK:
        raise ValueError("This service only supports the agreed task.")
    return {
        "contract_version": CONTRACT_VERSION,
        "request_id": request_identifier(request_id if request_id is not None else uuid.uuid4().hex),
        "images": {name: encode_image(images[name]) for name in CAMERAS},
        "state": checked_state(state).tolist(),
        "task": task,
    }


def decode_request(payload):
    required = {"contract_version", "request_id", "images", "state"}
    if not isinstance(payload, dict) or not required <= set(payload) or set(payload) - (required | {"task"}):
        raise ValueError("Invalid inference request fields.")
    request_identifier(payload["request_id"])
    if payload["contract_version"] != CONTRACT_VERSION or payload.get("task", TASK) != TASK:
        raise ValueError("Unsupported contract version or task.")
    images = payload["images"]
    if not isinstance(images, dict) or set(images) != set(CAMERAS):
        raise ValueError("Exactly the top, wrist and side cameras are required.")
    state = checked_state(payload["state"])
    return {
        "video": {name: decode_image(images[name])[None, None] for name in CAMERAS},
        "state": {"arm_joint": state[:6][None, None], "hand_position": state[6:][None, None]},
        "language": {"annotation.human.task_description": [[TASK]]},
    }


def flatten_action(action):
    if not isinstance(action, dict) or set(action) != {"arm_joint", "hand_position"}:
        raise ValueError("The service returned invalid action groups.")
    arm = numeric_array(action["arm_joint"], (1, 16, 6), "arm_joint")
    hand = numeric_array(action["hand_position"], (1, 16, 6), "hand_position")
    if np.any(hand[:, :, 3:] != 0):
        raise ValueError("The service returned nonzero fixed hand channels.")
    return np.concatenate((arm, hand), axis=-1)[0]


def contract_metadata():
    return {
        "contract_version": CONTRACT_VERSION,
        "mode": "shadow",
        "robot_execution_enabled": False,
        "cameras": list(CAMERAS),
        "image_shape": list(IMAGE_SHAPE),
        "image_encoding": "base64_png_rgb",
        "state_names": list(STATE_NAMES),
        "action_names": list(ACTION_NAMES),
        "action_shape": [16, 12],
        "fixed_zero_channels": [9, 10, 11],
        "arm_units": "radians_absolute_joint_target",
        "hand_units": "original_raw_command",
        "dt_seconds": 0.05,
        "task": TASK,
    }


def read_token_file(path):
    path = Path(path)
    if os.name == "posix" and path.stat().st_mode & 0o077:
        raise ValueError("Token files must be private to their owner (chmod 600).")
    token = path.read_text(encoding="ascii").strip()
    if not 32 <= len(token) <= 128 or not token.isascii() or not all(character.isalnum() or character in "-_" for character in token):
        raise ValueError("Token files must contain one URL-safe token of 32-128 characters.")
    return token


class CustomerAPIError(RuntimeError):
    def __init__(self, code, status=None):
        self.code = code
        self.status = status
        super().__init__(f"Customer API request failed: {code}" + (f" (HTTP {status})" if status else ""))


class CustomerPolicyClient:
    def __init__(self, base_url, token_file, timeout=10):
        parsed = urlsplit(base_url)
        if not parsed.hostname or parsed.username is not None or parsed.password is not None or parsed.query or parsed.fragment or parsed.path not in ("", "/"):
            raise ValueError("Use an HTTPS origin URL without credentials, paths or query parameters.")
        if parsed.scheme != "https" and not (parsed.scheme == "http" and parsed.hostname in ("127.0.0.1", "::1", "localhost")):
            raise ValueError("HTTPS is required outside loopback.")
        if not 0 < timeout <= 30:
            raise ValueError("Request timeout must be between zero and 30 seconds.")
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self._token = read_token_file(token_file)
        self._runner = None
        self._session = None
        self._owner = None
        self._closed = False

    async def _request_async(self, method, path, payload, deadline):
        import aiohttp

        if self._session is None:
            self._session = aiohttp.ClientSession(trust_env=False, auto_decompress=False, cookie_jar=aiohttp.DummyCookieJar(), headers={"Authorization": f"Bearer {self._token}", "Accept": "application/json", "Accept-Encoding": "identity"})
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise CustomerAPIError("request_timeout")
        try:
            data = None if payload is None else json.dumps(payload, allow_nan=False).encode("utf-8")
            async with self._session.request(method, self.base_url + path, data=data, headers={"Content-Type": "application/json"} if data is not None else None, timeout=aiohttp.ClientTimeout(total=remaining), allow_redirects=False) as response:
                if 300 <= response.status < 400:
                    raise CustomerAPIError("redirect_refused", response.status)
                if response.headers.get("Content-Encoding", "identity").lower() != "identity":
                    raise CustomerAPIError("invalid_response", response.status)
                body = bytearray()
                async for chunk in response.content.iter_chunked(16384):
                    body.extend(chunk)
                    if len(body) > 128 * 1024:
                        raise CustomerAPIError("invalid_response", response.status)
                try:
                    result = json.loads(body)
                except (ValueError, RecursionError):
                    raise CustomerAPIError("invalid_response", response.status) from None
                if not isinstance(result, dict):
                    raise CustomerAPIError("invalid_response", response.status)
                if response.status != 200:
                    codes = {"unauthorized", "invalid_request", "payload_too_large", "unsupported_media_type", "unsupported_encoding", "busy", "backend_timeout", "request_timeout", "backend_unavailable", "invalid_backend_response", "expired", "http_error"}
                    error = result.get("error")
                    code = error.get("code") if isinstance(error, dict) else None
                    raise CustomerAPIError(code if isinstance(code, str) and code in codes else "request_failed", response.status)
                if time.monotonic() >= deadline:
                    raise CustomerAPIError("stale_response")
                return result
        except TimeoutError:
            raise CustomerAPIError("request_timeout") from None
        except aiohttp.ClientError:
            raise CustomerAPIError("network_error") from None

    def _request(self, method, path, payload=None, deadline=None):
        if self._closed:
            raise CustomerAPIError("client_closed")
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            pass
        else:
            raise CustomerAPIError("use_client_in_a_synchronous_worker")
        owner = threading.get_ident()
        if self._owner is not None and self._owner != owner:
            raise CustomerAPIError("client_must_stay_on_its_own_thread")
        if self._runner is None:
            self._runner = asyncio.Runner()
            self._owner = owner
        return self._runner.run(self._request_async(method, path, payload, deadline if deadline is not None else time.monotonic() + self.timeout))

    def get_modality_config(self):
        metadata = self._request("GET", "/v1/metadata")
        expected = contract_metadata()
        if set(metadata) != set(expected) | {"ready", "expires_utc"} or metadata.get("ready") is not True or any(metadata.get(key) != value for key, value in expected.items()):
            raise CustomerAPIError("incompatible_service")
        return metadata

    def _check_reply(self, reply, identifier, payload_fields):
        allowed = {"request_id", "contract_version", "mode", "robot_execution_enabled", "server_elapsed_seconds"} | payload_fields
        if set(reply) != allowed or reply.get("request_id") != identifier or reply.get("contract_version") != CONTRACT_VERSION or reply.get("mode") != "shadow" or reply.get("robot_execution_enabled") is not False:
            raise CustomerAPIError("invalid_response")

    def infer(self, images, state, task=TASK):
        deadline = time.monotonic() + self.timeout
        payload = encode_request(images, state, task)
        reply = self._request("POST", "/v1/infer", payload, deadline)
        self._check_reply(reply, payload["request_id"], {"actions", "dt_seconds"})
        try:
            actions = numeric_array(reply.get("actions"), (16, 12), "actions")
            if np.any(actions[:, 9:] != 0) or reply.get("dt_seconds") != 0.05:
                raise ValueError("Invalid fixed channels or time step.")
        except ValueError:
            raise CustomerAPIError("invalid_response") from None
        return actions, {key: value for key, value in reply.items() if key != "actions"}

    def get_action(self, observation, options=None):
        if options not in (None, {}):
            raise ValueError("Custom inference options are not supported.")
        if not isinstance(observation, dict) or set(observation) != {"video", "state", "language"}:
            raise ValueError("Use the nested video/state/language observation.")
        video, state, language = (observation[key] for key in ("video", "state", "language"))
        if not isinstance(video, dict) or set(video) != set(CAMERAS) or not isinstance(state, dict) or set(state) != {"arm_joint", "hand_position"} or language != {"annotation.human.task_description": [[TASK]]}:
            raise ValueError("Observation modalities or task do not match the agreed contract.")
        images = {}
        for name in CAMERAS:
            image = video[name]
            if not isinstance(image, np.ndarray) or image.shape != (1, 1, *IMAGE_SHAPE) or image.dtype != np.uint8:
                raise ValueError("Video inputs must be uint8 RGB (1,1,224,224,3).")
            images[name] = image[0, 0]
        values = np.concatenate([numeric_array(state[name], (1, 1, 6), name)[0, 0] for name in ("arm_joint", "hand_position")])
        actions, info = self.infer(images, values)
        return {"arm_joint": actions[None, :, :6], "hand_position": actions[None, :, 6:]}, info

    def reset(self, options=None):
        if options not in (None, {}):
            raise ValueError("Custom reset options are not supported.")
        payload = {"request_id": uuid.uuid4().hex, "contract_version": CONTRACT_VERSION}
        reply = self._request("POST", "/v1/reset", payload)
        self._check_reply(reply, payload["request_id"], {"reset"})
        if reply.get("reset") is not True:
            raise CustomerAPIError("invalid_response")
        return reply

    def close(self):
        if self._closed:
            return
        if self._owner is not None and self._owner != threading.get_ident():
            raise CustomerAPIError("client_must_stay_on_its_own_thread")
        if self._runner is not None:
            try:
                if self._session is not None:
                    self._runner.run(self._session.close())
            finally:
                self._runner.close()
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        self.close()