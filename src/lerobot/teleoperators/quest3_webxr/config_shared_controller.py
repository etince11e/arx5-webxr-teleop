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

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@dataclass
class Quest3WebXRMappingConfig(TeleoperatorConfig):
    """Shared Quest controller-to-ARX5 Cartesian mapping config."""

    id: str = "quest3_webxr"

    stale_timeout_s: float = 1.0

    pos_sensitivity: float = 0.5
    max_pos_velocity: float = 1
    max_rot_velocity: float = 1
    filter_window_size: int = 2

    # Virtual control point relative to the Quest controller tracking frame.
    # Translation is expressed in the selected controller orientation frame.
    controller_tcp_offset_xyz: list[float] | None = None
    controller_tcp_offset_rpy: list[float] | None = None
    controller_orientation_source: str = "quat"  # "offsets" or "quat"
    # Translation delta mode:
    #   local: compose full SE(3) deltas in the controller reference frame.
    #   world: use controller world-position delta directly, then map axes to robot world.
    controller_translation_source: str = "world"
    # Rotation delta mode:
    #   local: legacy body-frame delta, R_init.T @ R_now, then base @ delta.
    #   world: world-frame delta, R_now @ R_init.T, mapped into robot world.
    controller_rotation_source: str = "local"
    # Maps Quest/WebXR world deltas into robot TCP deltas.
    #
    # Quest/WebXR world: X right, Y up, Z toward the operator.
    # Robot TCP world used by ARX/Pico/Flexiv: X forward, Y left, Z up.
    #
    # Default mapping:
    #   robot_x =  quest_z
    #   robot_y = -quest_x
    #   robot_z =  quest_y
    #
    # This matrix can have determinant -1 because Unity's world convention is
    # left-handed. It is still the correct coordinate-basis mapping for deltas.
    controller_world_to_robot_axes: list[list[float]] | None = None
    # Optional legacy/debug override. Leave unset for the explicit axis mapping above.
    controller_world_to_robot_rpy: list[float] | None = None
    # Keep this disabled while validating Quest/ARX translation axes and TCP offset.
    # When disabled, controller motion only changes tcp.x/y/z; TCP orientation stays
    # at the pose captured when teleop is enabled/reset. This prevents the calibrated
    # TCP offset from making the SDK flange command rotate around the TCP during
    # pure translation tests.
    control_orientation: bool = True
    debug_controller_tcp_offset: bool = False
    debug_raw_packets: bool = False
    debug_raw_packet_interval_s: float = 0.2
    enable_warmup_s: float = 0.1
    enable_stable_samples: int = 3
    enable_stable_pos_threshold_m: float = 0.03
    enable_stable_rot_threshold_deg: float = 12.0
    enable_soft_start_s: float = 0.12
    position_jump_threshold_m: float = 0.10

    # ARX5 SDK gripper commands are expressed in gripper_width units, not a
    # normalized 0..1 range. X5 uses 1.57 for fully open and 0.0 for closed.
    gripper_open: float = 1.57
    gripper_closed: float = 0.0
    start_gripper_open: bool = True

    def __post_init__(self):
        if self.controller_tcp_offset_xyz is None:
            self.controller_tcp_offset_xyz = [0.007434, 0.005841, 0.033875]
        if self.controller_tcp_offset_rpy is None:
            self.controller_tcp_offset_rpy = [0.0, 0.0, 0.0]
        if self.controller_world_to_robot_axes is None:
            self.controller_world_to_robot_axes = [
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ]
