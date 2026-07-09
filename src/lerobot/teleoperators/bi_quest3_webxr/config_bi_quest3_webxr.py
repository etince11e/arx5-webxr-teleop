#!/usr/bin/env python

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.quest3_webxr.config_quest3_webxr import Quest3WebXRConfig


@TeleoperatorConfig.register_subclass("bi_quest3_webxr")
@dataclass
class BiQuest3WebXRConfig(Quest3WebXRConfig):
    """Bimanual Quest 3 WebXR teleoperation config.

    The output action schema matches ``bi_pico4`` and bimanual Cartesian robots:
    ``left_tcp.*`` / ``right_tcp.*`` plus per-side gripper keys.
    """

    id: str = "bi_quest3_webxr"

    left_grip_button_index: int | None = None
    right_grip_button_index: int | None = None
    left_trigger_button_index: int | None = None
    right_trigger_button_index: int | None = None
    left_controller_tcp_offset_xyz: list[float] | None = None
    right_controller_tcp_offset_xyz: list[float] | None = None
    left_controller_tcp_offset_rpy: list[float] | None = None
    right_controller_tcp_offset_rpy: list[float] | None = None
    left_gripper_open: float | None = 1.57
    right_gripper_open: float | None = 1.57
    left_gripper_closed: float | None = 0.0
    right_gripper_closed: float | None = 0.0
    reset_hand: str = "right"
