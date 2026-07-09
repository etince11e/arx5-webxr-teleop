#!/usr/bin/env python

# Copyright 2025 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

from lerobot.teleoperators.teleoperator import RobotAction, Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import (
    get_logger,
    matrix_to_pose7d,
    normalize_quaternion,
    quaternion_to_matrix,
    quaternion_to_rotation_6d,
    slerp_quaternion,
    xyz_rpy_to_matrix,
)

from .config_shared_controller import Quest3WebXRMappingConfig


@dataclass
class Quest3RemotePacket:
    marker: str
    position: np.ndarray
    quaternion_wxyz: np.ndarray
    trigger_pressed: bool
    a_pressed: bool
    offset_forward: np.ndarray
    offset_right: np.ndarray
    offset_up: np.ndarray


class Quest3WebXRControllerBase(Teleoperator):
    """Shared Quest WebXR controller mapping for Cartesian TCP control."""

    config_class = Quest3WebXRMappingConfig
    name = "quest3_webxr_mapping"

    def __init__(self, config: Quest3WebXRMappingConfig):
        super().__init__(config)
        self.config = config
        self.logger = get_logger(f"Quest3WebXR/{config.id}")

        self._is_connected = False

        self._latest_packet: Quest3RemotePacket | None = None
        self._raw_pause_state = False
        self._latest_pause_state = False
        self._pause_low_start_time: float | None = None
        self._was_active = False
        self._last_raw_active = False
        self._last_packet_time: float | None = None

        self._robot_init_H: np.ndarray | None = None
        self._controller_init_H: np.ndarray | None = None
        self._controller_delta_reference_H: np.ndarray | None = None

        self._target_pos = np.zeros(3, dtype=np.float64)
        self._target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self._target_gripper_pos = float(config.gripper_open if config.start_gripper_open else config.gripper_closed)

        self._prev_target_pos: np.ndarray | None = None
        self._prev_target_quat: np.ndarray | None = None
        self._last_action_time: float | None = None
        self._pos_filter = deque(maxlen=max(1, int(config.filter_window_size)))
        self._quat_filter = deque(maxlen=max(1, int(config.filter_window_size)))
        self._enable_samples = deque(maxlen=max(1, int(config.enable_stable_samples)))
        self._enable_warmup_start_time: float | None = None
        self._soft_start_start_time: float | None = None
        self._last_controller_H: np.ndarray | None = None
        self._last_warmup_log_time = 0.0

        self._trigger_prev = False
        self._reset_prev = False
        self._reset_requested = False
        self._gripper_open = bool(config.start_gripper_open)
        self._enabled = False
        self._last_controller_tcp_debug_time = 0.0
        self._last_controller_delta_debug_time = 0.0
        self._controller_tcp_offset_H = self._make_controller_tcp_offset_matrix()
        self._controller_world_to_robot_H = self._make_controller_world_to_robot_matrix()
        self._controller_world_to_robot_inv_H = np.linalg.inv(self._controller_world_to_robot_H)

    @property
    def action_features(self) -> dict[str, Any]:
        return {
            "dtype": "float32",
            "shape": (10,),
            "names": {
                "tcp.x": 0,
                "tcp.y": 1,
                "tcp.z": 2,
                "tcp.r1": 3,
                "tcp.r2": 4,
                "tcp.r3": 5,
                "tcp.r4": 6,
                "tcp.r5": 7,
                "tcp.r6": 8,
                "gripper.pos": 9,
            },
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True, current_tcp_pose_euler: np.ndarray | None = None) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        self._seed_from_current_tcp_pose_euler(current_tcp_pose_euler)
        self._is_connected = True

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
        self._reset_enable_warmup()

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def reset_to_pose(self, pose_6d: np.ndarray, gripper_pos: float = 0.0) -> None:
        pose = np.asarray(pose_6d, dtype=np.float64)
        if pose.shape[0] != 6:
            raise ValueError(f"pose_6d must be [x,y,z,roll,pitch,yaw], got shape {pose.shape}")

        self._robot_init_H = xyz_rpy_to_matrix(pose)
        self._controller_delta_reference_H = self._robot_init_H.copy()
        pose7 = matrix_to_pose7d(self._robot_init_H, output_format="wxyz")
        self._target_pos = pose[:3].copy()
        self._target_quat = normalize_quaternion(pose7[3:7], input_format="wxyz")
        self._target_gripper_pos = float(gripper_pos)
        self._prev_target_pos = self._target_pos.copy()
        self._prev_target_quat = self._target_quat.copy()
        self._last_action_time = None
        self._controller_init_H = None
        self._was_active = False
        self._pos_filter.clear()
        self._quat_filter.clear()
        self._reset_enable_warmup()

    def get_action(self) -> RobotAction:
        # 设备连接
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")


        self._poll_pause()
        self._poll_remote()

        raw_active = (
            self._latest_pause_state    # 是否暂停
            and self._latest_packet is not None # 收到quest3数据包
            and not self._is_stale()    # 数据没有过期
        )
        self._log_raw_active_transition(raw_active)
        self._enabled = raw_active

        # 根据a键更新reset状态（还未实现）
        self._update_reset_from_a_button(self._latest_packet.a_pressed if self._latest_packet is not None else False)

        control_ready = False
        if raw_active:
            # 进入遥操模式，把quest3手柄数据转换成齐次变换矩阵
            current_H = self._packet_to_matrix(self._latest_packet)
            needs_reference = (
                not self._was_active
                or self._controller_init_H is None
                or self._robot_init_H is None
            )

            if needs_reference and self._has_controller_position_jump(current_H):
                jump_m = float(np.linalg.norm(current_H[:3, 3] - self._last_controller_H[:3, 3]))
                self._restart_enable_warmup(
                    current_H,
                    f"Quest3 单帧位置跳变 {jump_m * 1000.0:.1f} mm，丢弃本次启用参考并重新 warmup",
                )
            elif needs_reference:
                control_ready = self._collect_enable_warmup(current_H)
                if control_ready:
                    # warmup 通过后才记录参考位姿，避免 Quest 刚启用时的缓存/跳变 pose 直接进机械臂。
                    self._controller_init_H = current_H.copy()
                    self._controller_delta_reference_H = self._current_target_matrix()
                    self._soft_start_start_time = time.time()
                    self._pos_filter.clear()
                    self._quat_filter.clear()
                    self.logger.info("Quest3 WebXR teleop reference reset")
                else:
                    self._last_controller_H = current_H.copy()
            else:
                control_ready = True
                self._last_controller_H = current_H.copy()
            
            if control_ready:
                controller_delta_raw = np.linalg.pinv(self._controller_init_H) @ current_H
                controller_delta = self._map_controller_delta_to_robot_delta(controller_delta_raw)
                translation_source = str(self.config.controller_translation_source).strip().lower()
                if translation_source not in {"local", "world"}:
                    raise ValueError(
                        "controller_translation_source must be 'local' or 'world', "
                        f"got {self.config.controller_translation_source!r}"
                    )
                rotation_source = str(self.config.controller_rotation_source).strip().lower()
                if rotation_source not in {"local", "world"}:
                    raise ValueError(
                        "controller_rotation_source must be 'local' or 'world', "
                        f"got {self.config.controller_rotation_source!r}"
                    )
                controller_world_delta_raw = current_H[:3, 3] - self._controller_init_H[:3, 3]
                controller_world_delta_mapped = (
                    self._controller_world_to_robot_H[:3, :3] @ controller_world_delta_raw
                )
                controller_world_rot_delta_raw = current_H[:3, :3] @ self._controller_init_H[:3, :3].T
                controller_world_rot_delta_mapped = (
                    self._controller_world_to_robot_H[:3, :3]
                    @ controller_world_rot_delta_raw
                    @ self._controller_world_to_robot_inv_H[:3, :3]
                )

                if self.config.debug_controller_tcp_offset:
                    now = time.time()
                    if now - self._last_controller_delta_debug_time > 1.0:
                        world_rot_delta_H = np.eye(4, dtype=np.float64)
                        world_rot_delta_H[:3, :3] = controller_world_rot_delta_mapped
                        self.logger.info(
                            "Quest delta mapping | "
                            f"raw_xyz={controller_delta_raw[:3, 3]} | "
                            f"mapped_xyz={controller_delta[:3, 3]} | "
                            f"world_raw_xyz={controller_world_delta_raw} | "
                            f"world_mapped_xyz={controller_world_delta_mapped} | "
                            f"translation_source={translation_source} | "
                            f"rotation_source={rotation_source} | "
                            f"raw_rot_deg={self._rotation_angle_deg(controller_delta_raw):.2f} | "
                            f"mapped_rot_deg={self._rotation_angle_deg(controller_delta):.2f} | "
                            f"world_mapped_rot_deg={self._rotation_angle_deg(world_rot_delta_H):.2f} | "
                            f"control_orientation={self.config.control_orientation} | "
                            f"world_to_robot_axes={self._controller_world_to_robot_H[:3, :3]}"
                        )
                        self._last_controller_delta_debug_time = now
                base_H = (
                    self._controller_delta_reference_H
                    if self._controller_delta_reference_H is not None
                    else self._robot_init_H
                )
                target_H_unscaled = base_H @ controller_delta
                if translation_source == "world":
                    target_H_unscaled[:3, 3] = base_H[:3, 3] + controller_world_delta_mapped
                if rotation_source == "world":
                    target_H_unscaled[:3, :3] = controller_world_rot_delta_mapped @ base_H[:3, :3]
                soft_scale = self._soft_start_scale()

                target_pos = base_H[:3, 3] + (
                    target_H_unscaled[:3, 3] - base_H[:3, 3]
                ) * float(self.config.pos_sensitivity) * soft_scale
                base_pose7 = matrix_to_pose7d(base_H, output_format="wxyz")
                base_quat = normalize_quaternion(base_pose7[3:7], input_format="wxyz")
                if self.config.control_orientation:
                    target_pose7 = matrix_to_pose7d(target_H_unscaled, output_format="wxyz")
                    raw_target_quat = normalize_quaternion(target_pose7[3:7], input_format="wxyz")
                    target_quat = slerp_quaternion(
                        base_quat,
                        raw_target_quat,
                        soft_scale,
                        input_format="wxyz",
                    )
                else:
                    target_quat = base_quat

                target_pos, target_quat = self._filter_pose(target_pos, target_quat)
                target_pos, target_quat = self._limit_pose_rate(target_pos, target_quat)

                self._target_pos = target_pos
                self._target_quat = target_quat

                self._update_gripper_from_trigger(self._latest_packet.trigger_pressed)
        elif not self._latest_pause_state or not raw_active:
            self._controller_init_H = None
            self._reset_enable_warmup()

        self._was_active = control_ready
        self._prev_target_pos = self._target_pos.copy()
        self._prev_target_quat = self._target_quat.copy()
        self._last_action_time = time.time()

        r6d = quaternion_to_rotation_6d(
            float(self._target_quat[0]),
            float(self._target_quat[1]),
            float(self._target_quat[2]),
            float(self._target_quat[3]),
        )
        return {
            "tcp.x": float(self._target_pos[0]),
            "tcp.y": float(self._target_pos[1]),
            "tcp.z": float(self._target_pos[2]),
            "tcp.r1": float(r6d[0]),
            "tcp.r2": float(r6d[1]),
            "tcp.r3": float(r6d[2]),
            "tcp.r4": float(r6d[3]),
            "tcp.r5": float(r6d[4]),
            "tcp.r6": float(r6d[5]),
            "gripper.pos": float(self._target_gripper_pos),
        }

    def get_reset_button(self) -> bool:
        reset_requested = self._reset_requested
        self._reset_requested = False
        return reset_requested

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if not self._is_connected:
            return

        self._is_connected = False
        self.logger.info("Quest3 WebXR controller mapping disconnected")

#----------------------------------------------------------------------------------------------------
    
    def _poll_remote(self) -> None:
        pass

    def _poll_pause(self) -> None:
        pass

    def _update_debounced_pause_state(self) -> None:
        if self._raw_pause_state:
            self._latest_pause_state = True
            self._pause_low_start_time = None
            return

        now = time.time()
        if self._pause_low_start_time is None:
            self._pause_low_start_time = now

        debounce_s = max(0.0, float(self.config.pause_low_debounce_s))
        if now - self._pause_low_start_time >= debounce_s:
            self._latest_pause_state = False

    def _is_stale(self) -> bool:
        if self._last_packet_time is None:
            return True
        return time.time() - self._last_packet_time > float(self.config.stale_timeout_s)

    def _log_raw_active_transition(self, raw_active: bool) -> None:
        if raw_active == self._last_raw_active:
            return

        now = time.time()
        has_packet = self._latest_packet is not None
        packet_age_s = None if self._last_packet_time is None else now - self._last_packet_time
        packet_age_ms = "none" if packet_age_s is None else f"{packet_age_s * 1000.0:.1f}"
        pause_low_age_s = None if self._pause_low_start_time is None else now - self._pause_low_start_time
        pause_low_age_ms = "none" if pause_low_age_s is None else f"{pause_low_age_s * 1000.0:.1f}"
        stale = packet_age_s is None or packet_age_s > float(self.config.stale_timeout_s)
        reasons = []
        if not self._latest_pause_state:
            reasons.append("deadman_released")
        elif not self._raw_pause_state:
            reasons.append("deadman_debouncing")
        if not has_packet:
            reasons.append("no_webxr_packet")
        if stale:
            if packet_age_s is None:
                reasons.append("remote_packet_age=none")
            else:
                reasons.append(
                    f"webxr_packet_stale age={packet_age_s * 1000.0:.1f}ms>"
                    f"{float(self.config.stale_timeout_s) * 1000.0:.1f}ms"
                )
        if not reasons:
            reasons.append("all_inputs_live")

        level = self.logger.info if raw_active else self.logger.warn
        level(
            "Quest3 raw_active transition | "
            f"{'ON' if raw_active else 'OFF'} | "
            f"deadman_state={'High' if self._latest_pause_state else 'Low'} | "
            f"raw_deadman_state={'High' if self._raw_pause_state else 'Low'} | "
            f"deadman_low_age_ms={pause_low_age_ms} | "
            f"has_packet={has_packet} | "
            f"packet_age_ms={packet_age_ms} | "
            f"stale_timeout_ms={float(self.config.stale_timeout_s) * 1000.0:.1f} | "
            f"reasons={','.join(reasons)}"
        )
        self._last_raw_active = raw_active

    @staticmethod
    def _normalize(vec: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(vec)
        if norm < 1e-8:
            raise ValueError(f"Cannot normalize near-zero vector: {vec}")
        return vec / norm

    def _packet_to_matrix(self, packet: Quest3RemotePacket) -> np.ndarray:
        base = packet.position
        orientation_source = self.config.controller_orientation_source.strip().lower()
        if orientation_source == "offsets":
            x_axis = self._normalize(packet.offset_forward - base)
            y_axis = self._normalize(packet.offset_right - base)
            z_axis = self._normalize(base - packet.offset_up)

            matrix = np.eye(4, dtype=np.float64)
            matrix[:3, :3] = np.transpose(np.vstack([x_axis, y_axis, z_axis]))
            matrix[:3, 3] = base
        elif orientation_source == "quat":
            matrix = quaternion_to_matrix(
                np.concatenate([base, packet.quaternion_wxyz]),
                input_format="wxyz",
            )
        else:
            raise ValueError(
                "controller_orientation_source must be 'offsets' or 'quat', "
                f"got {self.config.controller_orientation_source!r}"
            )
        virtual_tcp_matrix = matrix @ self._controller_tcp_offset_H

        if self.config.debug_controller_tcp_offset:
            now = time.time()
            if now - self._last_controller_tcp_debug_time > 1.0:
                delta = virtual_tcp_matrix[:3, 3] - matrix[:3, 3]
                self.logger.info(
                    "Quest controller virtual TCP | "
                    f"controller_pos={matrix[:3, 3]} | "
                    f"virtual_tcp_pos={virtual_tcp_matrix[:3, 3]} | "
                    f"delta_world={delta} | "
                    f"offset_local={np.asarray(self.config.controller_tcp_offset_xyz, dtype=np.float64)}"
                )
                self._last_controller_tcp_debug_time = now

        return virtual_tcp_matrix

    def _make_controller_tcp_offset_matrix(self) -> np.ndarray:
        xyz = np.asarray(self.config.controller_tcp_offset_xyz, dtype=np.float64)
        rpy = np.asarray(self.config.controller_tcp_offset_rpy, dtype=np.float64)
        if xyz.shape != (3,):
            raise ValueError(f"controller_tcp_offset_xyz must have 3 values, got {xyz.shape}")
        if rpy.shape != (3,):
            raise ValueError(f"controller_tcp_offset_rpy must have 3 values, got {rpy.shape}")
        return xyz_rpy_to_matrix(np.concatenate([xyz, rpy]))

    def _make_controller_world_to_robot_matrix(self) -> np.ndarray:
        if self.config.controller_world_to_robot_rpy is not None:
            rpy = np.asarray(self.config.controller_world_to_robot_rpy, dtype=np.float64)
            if rpy.shape != (3,):
                raise ValueError(f"controller_world_to_robot_rpy must have 3 values, got {rpy.shape}")
            transform = xyz_rpy_to_matrix(np.concatenate([np.zeros(3, dtype=np.float64), rpy]))
            return transform

        axes = np.asarray(self.config.controller_world_to_robot_axes, dtype=np.float64)
        if axes.shape != (3, 3):
            raise ValueError(f"controller_world_to_robot_axes must be 3x3, got {axes.shape}")

        orthogonal_error = np.linalg.norm(axes.T @ axes - np.eye(3, dtype=np.float64))
        determinant = np.linalg.det(axes)
        if orthogonal_error > 1e-6:
            raise ValueError(
                "controller_world_to_robot_axes must be orthogonal. "
                f"orthogonal_error={orthogonal_error:.3e}, axes={axes}"
            )
        if abs(abs(determinant) - 1.0) > 1e-6:
            raise ValueError(
                "controller_world_to_robot_axes must have determinant +1 or -1. "
                f"determinant={determinant:.6f}, axes={axes}"
            )

        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = axes
        return transform

    def _map_controller_delta_to_robot_delta(self, controller_delta: np.ndarray) -> np.ndarray:
        mapped_delta = self._controller_world_to_robot_H @ controller_delta @ self._controller_world_to_robot_inv_H
        return mapped_delta

    @staticmethod
    def _rotation_angle_deg(transform: np.ndarray) -> float:
        rotation = transform[:3, :3]
        cos_angle = (np.trace(rotation) - 1.0) * 0.5
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        return float(np.degrees(angle))

    def _reset_enable_warmup(self) -> None:
        self._enable_samples.clear()
        self._enable_warmup_start_time = None
        self._soft_start_start_time = None
        self._last_controller_H = None

    def _restart_enable_warmup(self, current_H: np.ndarray, reason: str) -> None:
        self.logger.warn(reason)
        self._controller_init_H = None
        self._was_active = False
        self._enable_samples.clear()
        self._enable_warmup_start_time = time.time()
        self._soft_start_start_time = None
        self._last_controller_H = current_H.copy()
        self._enable_samples.append(current_H.copy())

    def _has_controller_position_jump(self, current_H: np.ndarray) -> bool:
        if self._last_controller_H is None:
            return False
        threshold = float(self.config.position_jump_threshold_m)
        if threshold <= 0:
            return False
        jump_m = float(np.linalg.norm(current_H[:3, 3] - self._last_controller_H[:3, 3]))
        return jump_m > threshold

    def _collect_enable_warmup(self, current_H: np.ndarray) -> bool:
        now = time.time()
        if self._enable_warmup_start_time is None:
            self._enable_warmup_start_time = now
            self._enable_samples.clear()
            self.logger.info("Quest3 启用 warmup：等待手柄位姿稳定")

        self._enable_samples.append(current_H.copy())
        warmup_elapsed = now - self._enable_warmup_start_time
        warmup_done = warmup_elapsed >= float(self.config.enable_warmup_s)
        stable_done, pos_span_m, rot_span_deg = self._enable_samples_are_stable()
        ready = warmup_done and stable_done

        if not ready and now - self._last_warmup_log_time > 0.5:
            self.logger.info(
                "Quest3 warmup 中 | "
                f"time={warmup_elapsed:.2f}/{float(self.config.enable_warmup_s):.2f}s | "
                f"samples={len(self._enable_samples)}/{max(1, int(self.config.enable_stable_samples))} | "
                f"pos_span={pos_span_m * 1000.0:.1f}mm/"
                f"{float(self.config.enable_stable_pos_threshold_m) * 1000.0:.1f}mm | "
                f"rot_span={rot_span_deg:.1f}deg/"
                f"{float(self.config.enable_stable_rot_threshold_deg):.1f}deg"
            )
            self._last_warmup_log_time = now

        if ready:
            self.logger.info(
                "Quest3 warmup 通过 | "
                f"samples={len(self._enable_samples)} | "
                f"pos_span={pos_span_m * 1000.0:.1f}mm | "
                f"rot_span={rot_span_deg:.1f}deg"
            )
        return ready

    def _enable_samples_are_stable(self) -> tuple[bool, float, float]:
        configured_required = int(self.config.enable_stable_samples)
        if configured_required <= 1:
            return len(self._enable_samples) >= 1, 0.0, 0.0

        required = max(1, configured_required)
        if len(self._enable_samples) < required:
            return False, 0.0, 0.0

        samples = list(self._enable_samples)
        positions = np.asarray([sample[:3, 3] for sample in samples], dtype=np.float64)
        pos_span_m = float(np.max(np.linalg.norm(positions - positions[0], axis=1)))

        ref_rotation = samples[0][:3, :3]
        max_rot_deg = 0.0
        for sample in samples[1:]:
            relative = np.eye(4, dtype=np.float64)
            relative[:3, :3] = ref_rotation.T @ sample[:3, :3]
            max_rot_deg = max(max_rot_deg, self._rotation_angle_deg(relative))

        stable = (
            pos_span_m <= float(self.config.enable_stable_pos_threshold_m)
            and max_rot_deg <= float(self.config.enable_stable_rot_threshold_deg)
        )
        return stable, pos_span_m, max_rot_deg

    def _soft_start_scale(self) -> float:
        duration = float(self.config.enable_soft_start_s)
        if duration <= 0 or self._soft_start_start_time is None:
            return 1.0
        elapsed = time.time() - self._soft_start_start_time
        return float(np.clip(elapsed / duration, 0.0, 1.0))

    def _current_target_matrix(self) -> np.ndarray:
        pose7 = np.concatenate([self._target_pos, self._target_quat])
        from lerobot.utils.robot_utils import quaternion_to_matrix

        return quaternion_to_matrix(pose7, input_format="wxyz")

    def _filter_pose(self, pos: np.ndarray, quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self.config.filter_window_size <= 1:
            return pos, quat

        self._pos_filter.append(pos.copy())
        filtered_pos = np.mean(np.asarray(self._pos_filter), axis=0)

        self._quat_filter.append(quat.copy())
        quats = list(self._quat_filter)
        filtered_quat = quats[0]
        for idx in range(1, len(quats)):
            filtered_quat = slerp_quaternion(
                filtered_quat,
                quats[idx],
                1.0 / (idx + 1),
                input_format="wxyz",
            )
        filtered_quat = normalize_quaternion(filtered_quat, input_format="wxyz")
        return filtered_pos, filtered_quat

    def _limit_pose_rate(self, pos: np.ndarray, quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self._prev_target_pos is None or self._prev_target_quat is None or self._last_action_time is None:
            return pos, quat

        now = time.time()
        dt = now - self._last_action_time
        if dt <= 0:
            return pos, quat

        if self.config.max_pos_velocity > 0:
            delta = pos - self._prev_target_pos
            delta_norm = np.linalg.norm(delta)
            max_delta = float(self.config.max_pos_velocity) * dt
            if delta_norm > max_delta > 0:
                pos = self._prev_target_pos + delta * (max_delta / delta_norm)

        if self.config.max_rot_velocity > 0:
            if np.dot(quat, self._prev_target_quat) < 0.0:
                quat = -quat
            dot = np.clip(abs(np.dot(quat, self._prev_target_quat)), 0.0, 1.0)
            angle = 2.0 * np.arccos(dot)
            max_angle = float(self.config.max_rot_velocity) * dt
            if angle > max_angle > 0:
                quat = slerp_quaternion(self._prev_target_quat, quat, max_angle / angle, input_format="wxyz")
        return pos, normalize_quaternion(quat, input_format="wxyz")

    def _update_gripper_from_trigger(self, pressed: bool) -> None:
        if pressed and not self._trigger_prev:
            self._gripper_open = not self._gripper_open
            self._target_gripper_pos = (
                float(self.config.gripper_open)
                if self._gripper_open
                else float(self.config.gripper_closed)
            )
        self._trigger_prev = pressed

    def _update_reset_from_a_button(self, pressed: bool) -> None:
        if pressed and not self._reset_prev:
            self._reset_requested = True
        self._reset_prev = pressed
