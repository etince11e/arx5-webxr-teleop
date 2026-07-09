#!/usr/bin/env python

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

from .config_shared_controller import Quest3WebXRMappingConfig


@TeleoperatorConfig.register_subclass("quest3_webxr")
@dataclass
class Quest3WebXRConfig(Quest3WebXRMappingConfig):
    """Quest 3 WebXR teleoperation config.

    This keeps the controller-to-ARX5 Cartesian mapping knobs and receives
    Quest data through the vr-teleop-kit WebXR relay protocol.
    """

    id: str = "quest3_webxr"

    ws_url: str = "ws://127.0.0.1:8443/ws"
    connect_timeout_s: float = 5.0
    hand: str = "right"
    grip_button_index: int = 1
    trigger_button_index: int = 0
    reset_button_index: int = 4
    controller_translation_source: str = "world"
    controller_rotation_source: str = "world"

    def __post_init__(self):
        user_axes = self.controller_world_to_robot_axes
        user_controller_tcp_offset_xyz = self.controller_tcp_offset_xyz
        super().__post_init__()
        self.controller_orientation_source = "quat"
        if user_controller_tcp_offset_xyz is None:
            self.controller_tcp_offset_xyz = [
                -0.014441072515729273,
                -0.02176408094211002,
                -0.09802969712496709,
            ]
        # WebXR / vr-teleop-kit world axes:
        #   X right, Y up, Z toward the operator.
        # Robot world:
        #   X forward away from the base, Y left, Z up.
        if user_axes is None:
            self.controller_world_to_robot_axes = [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
                [0.0, 1.0, 0.0],
            ]
