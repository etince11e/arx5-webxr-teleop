#!/usr/bin/env python

import asyncio
import json
import ssl
import threading
import time
from dataclasses import replace
from typing import Any

import numpy as np

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.quest3_webxr.config_quest3_webxr import Quest3WebXRConfig
from lerobot.teleoperators.quest3_webxr.teleop_quest3_webxr import Quest3WebXRTeleop
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import get_logger

from .config_bi_quest3_webxr import BiQuest3WebXRConfig


class BiQuest3WebXR(Teleoperator):
    """Bimanual Quest 3 WebXR teleoperator.

    Uses one WebSocket relay subscription and two single-arm
    ``Quest3WebXRTeleop`` cores. Each core owns its per-arm target pose,
    warmup, filtering, rate limiting, and gripper state; this wrapper prefixes
    their actions for bimanual robots.
    """

    config_class = BiQuest3WebXRConfig
    name = "bi_quest3_webxr"

    def __init__(self, config: BiQuest3WebXRConfig):
        super().__init__(config)
        self.config = config
        self.logger = get_logger(f"BiQuest3WebXR/{config.id}")
        self._is_connected = False

        self._left = Quest3WebXRTeleop(self._side_config("left"))
        self._right = Quest3WebXRTeleop(self._side_config("right"))

        self._ws_thread: threading.Thread | None = None
        self._ws_loop: asyncio.AbstractEventLoop | None = None
        self._ws_stop: threading.Event | None = None
        self._ws_connected = threading.Event()
        self._ws = None
        self._xr_lock = threading.Lock()
        self._latest_xr_frame: dict[str, Any] | None = None
        self._last_xr_frame_time: float | None = None
        self._reset_prev = False
        self._reset_requested = False
        self._enabled = False

    def _side_config(self, side: str) -> Quest3WebXRConfig:
        grip = (
            self.config.left_grip_button_index
            if side == "left"
            else self.config.right_grip_button_index
        )
        trigger = (
            self.config.left_trigger_button_index
            if side == "left"
            else self.config.right_trigger_button_index
        )
        tcp_offset_xyz = (
            self.config.left_controller_tcp_offset_xyz
            if side == "left"
            else self.config.right_controller_tcp_offset_xyz
        )
        tcp_offset_rpy = (
            self.config.left_controller_tcp_offset_rpy
            if side == "left"
            else self.config.right_controller_tcp_offset_rpy
        )
        gripper_open = (
            self.config.left_gripper_open
            if side == "left"
            else self.config.right_gripper_open
        )
        gripper_closed = (
            self.config.left_gripper_closed
            if side == "left"
            else self.config.right_gripper_closed
        )
        return replace(
            self.config,
            id=f"{self.config.id}_{side}",
            hand=side,
            grip_button_index=self.config.grip_button_index if grip is None else int(grip),
            trigger_button_index=(
                self.config.trigger_button_index if trigger is None else int(trigger)
            ),
            controller_tcp_offset_xyz=(
                self.config.controller_tcp_offset_xyz if tcp_offset_xyz is None else tcp_offset_xyz
            ),
            controller_tcp_offset_rpy=(
                self.config.controller_tcp_offset_rpy if tcp_offset_rpy is None else tcp_offset_rpy
            ),
            gripper_open=(
                self.config.gripper_open if gripper_open is None else float(gripper_open)
            ),
            gripper_closed=(
                self.config.gripper_closed if gripper_closed is None else float(gripper_closed)
            ),
        )

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_connected

    @property
    def action_features(self) -> dict[str, Any]:
        return {
            "dtype": "float32",
            "shape": (20,),
            "names": {
                "left_tcp.x": 0,
                "left_tcp.y": 1,
                "left_tcp.z": 2,
                "left_tcp.r1": 3,
                "left_tcp.r2": 4,
                "left_tcp.r3": 5,
                "left_tcp.r4": 6,
                "left_tcp.r5": 7,
                "left_tcp.r6": 8,
                "left_gripper.pos": 9,
                "right_tcp.x": 10,
                "right_tcp.y": 11,
                "right_tcp.z": 12,
                "right_tcp.r1": 13,
                "right_tcp.r2": 14,
                "right_tcp.r3": 15,
                "right_tcp.r4": 16,
                "right_tcp.r5": 17,
                "right_tcp.r6": 18,
                "right_gripper.pos": 19,
            },
        }

    @property
    def feedback_features(self) -> dict[str, Any]:
        return {}

    def connect(
        self,
        calibrate: bool = True,
        left_tcp_pose_euler: np.ndarray | None = None,
        right_tcp_pose_euler: np.ndarray | None = None,
    ) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected.")
        if left_tcp_pose_euler is None or right_tcp_pose_euler is None:
            raise ValueError(
                "left_tcp_pose_euler and right_tcp_pose_euler are required for "
                "BiQuest3WebXR. Pass robot.get_current_tcp_poses_euler()."
            )

        self._seed_side(self._left, left_tcp_pose_euler)
        self._seed_side(self._right, right_tcp_pose_euler)

        self._ws_stop = threading.Event()
        self._ws_connected.clear()
        self._ws_thread = threading.Thread(
            target=self._ws_thread_main,
            name="bi-quest3-webxr-ws",
            daemon=True,
        )
        self._ws_thread.start()
        if not self._ws_connected.wait(timeout=float(self.config.connect_timeout_s)):
            self.disconnect()
            raise RuntimeError(
                f"BiQuest3WebXR: timed out connecting to {self.config.ws_url}. "
                "Start the WebXR relay first."
            )

        self._is_connected = True
        self.logger.info(f"BiQuest3 WebXR connected to {self.config.ws_url}")

    def _seed_side(self, teleop: Quest3WebXRTeleop, pose_euler: np.ndarray) -> None:
        teleop._seed_from_current_tcp_pose_euler(np.asarray(pose_euler, dtype=np.float64))
        teleop._is_connected = True

    def reset_to_pose(
        self,
        left_pose_6d: np.ndarray,
        right_pose_6d: np.ndarray,
        left_gripper_pos: float = 0.0,
        right_gripper_pos: float = 0.0,
    ) -> None:
        self._left.reset_to_pose(np.asarray(left_pose_6d, dtype=np.float64), left_gripper_pos)
        self._right.reset_to_pose(np.asarray(right_pose_6d, dtype=np.float64), right_gripper_pos)

    def get_action(self) -> dict[str, Any]:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        with self._xr_lock:
            frame = self._latest_xr_frame
            frame_time = self._last_xr_frame_time

        self._set_side_frame(self._left, frame, frame_time)
        self._set_side_frame(self._right, frame, frame_time)
        left_action = self._left.get_action()
        right_action = self._right.get_action()
        self._enabled = bool(getattr(self._left, "_enabled", False) or getattr(self._right, "_enabled", False))

        self._update_reset_button(frame)
        return {f"left_{k}": v for k, v in left_action.items()} | {
            f"right_{k}": v for k, v in right_action.items()
        }

    def _is_stale(self) -> bool:
        return bool(self._left._is_stale() or self._right._is_stale())

    @staticmethod
    def _set_side_frame(
        teleop: Quest3WebXRTeleop,
        frame: dict[str, Any] | None,
        frame_time: float | None,
    ) -> None:
        with teleop._xr_lock:
            teleop._latest_xr_frame = frame
            teleop._last_xr_frame_time = frame_time

    def get_reset_button(self) -> bool:
        requested = self._reset_requested
        self._reset_requested = False
        return requested

    def _update_reset_button(self, frame: dict[str, Any] | None) -> None:
        current = False
        controllers = frame.get("controllers") if isinstance(frame, dict) else None
        if isinstance(controllers, dict):
            hand = str(self.config.reset_hand).strip().lower()
            ctrl = controllers.get(hand)
            if isinstance(ctrl, dict):
                current = Quest3WebXRTeleop._button_pressed(
                    ctrl, int(self.config.reset_button_index)
                )
        if current and not self._reset_prev:
            self._reset_requested = True
        self._reset_prev = current

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

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
        self._left._is_connected = False
        self._right._is_connected = False
        self._is_connected = False
        self.logger.info("BiQuest3 WebXR disconnected")

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
                "websockets is required for bi_quest3_webxr. Install uvicorn[standard] "
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
