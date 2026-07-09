#!/usr/bin/env python

import asyncio
import json
import ssl
import threading
import time
from typing import Any

import numpy as np

from .shared_controller import (
    Quest3WebXRControllerBase,
    Quest3RemotePacket,
)
from lerobot.utils.errors import DeviceAlreadyConnectedError
from lerobot.utils.robot_utils import matrix_to_pose7d, normalize_quaternion, xyz_rpy_to_matrix

from .config_quest3_webxr import Quest3WebXRConfig


class Quest3WebXRTeleop(Quest3WebXRControllerBase):
    """Quest 3 WebXR teleoperator using the vr-teleop-kit relay.

    The WebSocket receiver converts WebXR ``xr_frame`` messages into the shared
    controller packet form. The inherited mapping path handles enable warmup,
    controller delta mapping, filtering, rate limiting, continuous gripper
    control, and TCP action output.
    """

    config_class = Quest3WebXRConfig
    name = "quest3_webxr"

    def __init__(self, config: Quest3WebXRConfig):
        super().__init__(config)
        self.config = config
        self._ws_thread: threading.Thread | None = None
        self._ws_loop: asyncio.AbstractEventLoop | None = None
        self._ws_stop: threading.Event | None = None
        self._ws_connected = threading.Event()
        self._ws = None
        self._xr_lock = threading.Lock()
        self._latest_xr_frame: dict[str, Any] | None = None
        self._last_xr_frame_time: float | None = None
        self._raw_pause_state = False
        self._pause_low_start_time: float | None = None

    def connect(self, calibrate: bool = True, current_tcp_pose_euler: np.ndarray | None = None) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self._seed_from_current_tcp_pose_euler(current_tcp_pose_euler)
        self._is_connected = True

        self._ws_stop = threading.Event()
        self._ws_connected.clear()
        self._ws_thread = threading.Thread(
            target=self._ws_thread_main,
            name="quest3-webxr-ws",
            daemon=True,
        )
        self._ws_thread.start()
        if not self._ws_connected.wait(timeout=float(self.config.connect_timeout_s)):
            self.disconnect()
            raise RuntimeError(
                f"Quest3WebXR: timed out connecting to {self.config.ws_url}. "
                "Start the WebXR relay first."
            )
        self.logger.info(f"Quest3 WebXR connected to {self.config.ws_url}")

    def _seed_from_current_tcp_pose_euler(self, current_tcp_pose_euler: np.ndarray | None) -> None:
        if current_tcp_pose_euler is None:
            raise ValueError(
                "current_tcp_pose_euler is required for Quest3 WebXR teleop. "
                "Pass robot.get_current_tcp_pose_euler()."
            )

        pose = np.asarray(current_tcp_pose_euler, dtype=np.float64)
        if pose.shape[0] < 7:
            raise ValueError(
                f"current_tcp_pose_euler must have at least 7 values [x,y,z,roll,pitch,yaw,gripper], got {pose.shape}"
            )

        self._target_pos = pose[:3].copy()
        self._robot_init_H = xyz_rpy_to_matrix(pose[:6].copy())
        self._controller_delta_reference_H = self._robot_init_H.copy()
        pose7 = matrix_to_pose7d(self._robot_init_H, output_format="wxyz")
        self._target_quat = normalize_quaternion(pose7[3:7], input_format="wxyz")
        self._target_gripper_pos = float(pose[6])
        self._prev_target_pos = self._target_pos.copy()
        self._prev_target_quat = self._target_quat.copy()
        self._last_action_time = None
        self._latest_packet = None
        self._last_packet_time = None
        self._controller_init_H = None
        self._was_active = False
        self._pos_filter.clear()
        self._quat_filter.clear()
        self._raw_pause_state = False
        self._latest_pause_state = False
        self._pause_low_start_time = None
        if hasattr(self, "_reset_enable_warmup"):
            self._reset_enable_warmup()

    def disconnect(self) -> None:
        if self._ws_stop is not None:
            self._ws_stop.set()
        if self._ws_loop is not None and self._ws is not None:
            try:
                future = asyncio.run_coroutine_threadsafe(self._ws.close(), self._ws_loop)
                future.result(timeout=1.0)
            except Exception:
                pass
        if self._ws_thread is not None:
            self._ws_thread.join(timeout=2.0)
        self._ws_connected.clear()
        self._ws = None
        self._ws_loop = None
        self._ws_thread = None
        self._ws_stop = None
        self._is_connected = False
        self.logger.info("Quest3 WebXR disconnected")

    def _poll_pause(self) -> None:
        # The right grip button is the deadman enable signal.
        self._poll_remote()
        packet = self._latest_packet
        self._raw_pause_state = packet is not None and self._last_packet_grip_pressed
        self._latest_pause_state = self._raw_pause_state
        if self._raw_pause_state:
            self._pause_low_start_time = None
        elif self._pause_low_start_time is None:
            self._pause_low_start_time = time.time()

    def _poll_remote(self) -> None:
        with self._xr_lock:
            frame = self._latest_xr_frame
            frame_time = self._last_xr_frame_time

        if frame is None or frame_time is None:
            return
        packet = self._xr_frame_to_packet(frame)
        if packet is None:
            return
        self._latest_packet = packet
        self._last_packet_time = frame_time
        trigger_value = getattr(packet, "trigger_value", None)
        if trigger_value is not None:
            self._update_gripper_from_trigger_value(trigger_value)

    @property
    def _last_packet_grip_pressed(self) -> bool:
        packet = self._latest_packet
        return bool(getattr(packet, "grip_pressed", False)) if packet is not None else False

    def _xr_frame_to_packet(self, frame: dict[str, Any]) -> Quest3RemotePacket | None:
        controllers = frame.get("controllers")
        if not isinstance(controllers, dict):
            return None
        hand = str(self.config.hand).strip().lower()
        ctrl = controllers.get(hand)
        if not isinstance(ctrl, dict):
            return None

        try:
            position = np.asarray(ctrl["position"], dtype=np.float64).reshape(3)
            qx, qy, qz, qw = np.asarray(ctrl["orientation"], dtype=np.float64).reshape(4)
        except Exception:
            return None

        quat_wxyz = normalize_quaternion(np.asarray([qw, qx, qy, qz], dtype=np.float64), input_format="wxyz")
        trigger_value = self._button_value(ctrl, int(self.config.trigger_button_index))
        trigger_pressed = trigger_value > 0.5
        grip_pressed = self._button_pressed(ctrl, int(self.config.grip_button_index))
        a_pressed = self._button_pressed(ctrl, int(self.config.reset_button_index))

        packet = Quest3RemotePacket(
            marker="webxr",
            position=position,
            quaternion_wxyz=quat_wxyz,
            trigger_pressed=trigger_pressed,
            a_pressed=a_pressed,
            offset_forward=position.copy(),
            offset_right=position.copy(),
            offset_up=position.copy(),
        )
        setattr(packet, "trigger_value", trigger_value)
        setattr(packet, "grip_pressed", grip_pressed)
        return packet

    @staticmethod
    def _button_value(ctrl: dict[str, Any], index: int) -> float:
        buttons = ctrl.get("buttons") or []
        if index < 0 or len(buttons) <= index:
            return 0.0
        button = buttons[index]
        if not isinstance(button, dict):
            return 0.0
        if "v" in button:
            value = float(button.get("v", 0.0) or 0.0)
        else:
            value = 1.0 if bool(button.get("p", False)) else 0.0
        return float(np.clip(value, 0.0, 1.0))

    @staticmethod
    def _button_pressed(ctrl: dict[str, Any], index: int) -> bool:
        buttons = ctrl.get("buttons") or []
        if index < 0 or len(buttons) <= index:
            return False
        button = buttons[index]
        if not isinstance(button, dict):
            return False
        return bool(button.get("p", False)) or float(button.get("v", 0.0) or 0.0) > 0.5

    def _update_gripper_from_trigger(self, pressed: bool) -> None:
        packet = self._latest_packet
        trigger_value = getattr(packet, "trigger_value", None) if packet is not None else None
        if trigger_value is None:
            super()._update_gripper_from_trigger(pressed)
            return
        self._update_gripper_from_trigger_value(trigger_value)

    def _update_gripper_from_trigger_value(self, trigger_value: float) -> None:
        trigger = float(np.clip(trigger_value, 0.0, 1.0))
        open_pos = float(self.config.gripper_open)
        closed_pos = float(self.config.gripper_closed)
        self._target_gripper_pos = open_pos + trigger * (closed_pos - open_pos)
        midpoint = (open_pos + closed_pos) * 0.5
        self._gripper_open = self._target_gripper_pos >= midpoint if open_pos >= closed_pos else self._target_gripper_pos <= midpoint
        self._trigger_prev = trigger > 0.5

    def _ws_thread_main(self) -> None:
        asyncio.run(self._ws_run())

    async def _ws_run(self) -> None:
        self._ws_loop = asyncio.get_running_loop()
        while self._ws_stop is not None and not self._ws_stop.is_set():
            try:
                async with self._connect_ws() as ws:
                    self._ws = ws
                    self._ws_connected.set()
                    await self._send_json({"type": "request_settings", "source": self.name})
                    while self._ws_stop is not None and not self._ws_stop.is_set():
                        try:
                            raw = await asyncio.wait_for(ws.recv(), timeout=0.1)
                        except asyncio.TimeoutError:
                            continue
                        try:
                            msg = json.loads(raw)
                        except Exception:
                            continue
                        if msg.get("type") == "xr_frame":
                            with self._xr_lock:
                                self._latest_xr_frame = msg
                                self._last_xr_frame_time = time.time()
            except Exception as exc:
                self._ws = None
                if not self._ws_connected.is_set():
                    self.logger.warn(f"Waiting for WebXR relay {self.config.ws_url}: {exc}")
                await asyncio.sleep(0.5)

    def _connect_ws(self):
        try:
            import websockets
        except ImportError as e:
            raise ImportError(
                "websockets is required for quest3_webxr. Install uvicorn[standard] "
                "or pip install websockets."
            ) from e

        ssl_context = None
        if str(self.config.ws_url).startswith("wss://"):
            ssl_context = ssl.create_default_context()
            ssl_context.check_hostname = False
            ssl_context.verify_mode = ssl.CERT_NONE
        return websockets.connect(str(self.config.ws_url), ssl=ssl_context)

    async def _send_json(self, payload: dict[str, Any]) -> None:
        if self._ws is None:
            return
        try:
            await self._ws.send(json.dumps(payload))
        except Exception:
            pass
