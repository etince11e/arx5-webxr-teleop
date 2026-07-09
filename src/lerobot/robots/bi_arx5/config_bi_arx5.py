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

from dataclasses import dataclass, field
from enum import Enum

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


class BiARX5ControlMode(Enum):
    """Control modes for BiARX5 robot arms.

    Attributes:
        JOINT_CONTROL: Joint space position control mode.
            Robot tracks target joint positions directly.
        CARTESIAN_CONTROL: Cartesian/EEF space control mode.
            Robot tracks end-effector pose in 6D space (x, y, z, roll, pitch, yaw).
        TEACH_MODE: Teaching mode with gravity compensation.
            Robot maintains zero torque while compensating for gravity,
            allowing free movement by hand for demonstration recording.
    """

    JOINT_CONTROL = "joint_control"
    CARTESIAN_CONTROL = "cartesian_control"
    TEACH_MODE = "teach_mode"  # Teaching mode with gravity compensation


@RobotConfig.register_subclass("bi_arx5")
@dataclass
class BiARX5Config(RobotConfig):
    """Configuration for BiARX5 dual-arm robot."""

    # Arm configuration
    left_arm_model: str = "X5"
    left_arm_port: str = "can1"
    right_arm_model: str = "X5"
    right_arm_port: str = "can3"

    # Logging and threading
    log_level: str = "DEBUG"
    use_multithreading: bool = True  # For SDK background_send_recv

    # Control parameters
    controller_dt: float = 0.005  # 200Hz low-level control frequency
    interpolation_controller_dt: float = (
        0.02  # 50Hz high-level interpolation control frequency
    )

    # Control mode (default: joint control for teleoperation)
    control_mode: BiARX5ControlMode = BiARX5ControlMode.TEACH_MODE

    # Inference mode
    inference_mode: bool = False

    # Preview time in seconds for control interpolation
    # Higher values (0.03-0.05) provide smoother motion but more delay
    # Lower values (0.01-0.02) are more responsive but may cause jittering
    # For Cartesian mode: use default preview time 0.1s in low-level SDK
    preview_time: float = 0.03  # Default 30ms for Joint control

    # Gripper calibration (calibrated values from calibrate.py for left and right arms)
    gripper_open_readout: list[float] = field(default_factory=lambda: [-3.4, -3.4])
    enable_tactile_sensors: bool = False
    gripper_control_mode: str = "mit"
    gripper_mit_kp: float = 0.8
    gripper_mit_kd: float = 0.05
    gripper_mit_torque: float = 0.0
    gripper_vel_max: float = 20.0
    gripper_torque_max: float = 3.0
    gripper_over_current_cnt_max: int = 120

    # Real TCP offsets from each SDK/URDF eef_link, expressed in that arm's eef_link frame.
    # Calibrated by examples/arx5_tcp_four_point_calibration.py.
    left_tcp_offset_xyz: list[float] = field(
        default_factory=lambda: [
            0.03289615157942764,
            0.002644292774758826,
            -0.006774168923815918,
        ]
    )
    right_tcp_offset_xyz: list[float] = field(
        default_factory=lambda: [
            0.03289615157942764,
            0.002644292774758826,
            -0.006774168923815918,
        ]
    )

    # Position settings (Joint space: 6 joints + gripper)
    home_position: list[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )

    start_position: list[float] = field(
        default_factory=lambda: [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, 1.57]
    )
    # Camera configuration
    cameras: dict[str, CameraConfig] = field(default_factory=lambda: {})

    def __post_init__(self):
        if self.enable_tactile_sensors:
            raise ValueError(
                "arx5-webxr-teleop is the reduced WebXR package and does not include Xense tactile cameras. "
                "Use --robot.enable_tactile_sensors=false or the full lerobot-xense package."
            )
