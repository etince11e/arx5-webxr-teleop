# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

from enum import Enum
from typing import TYPE_CHECKING

from .config import TeleoperatorConfig

if TYPE_CHECKING:
    from .teleoperator import Teleoperator


class TeleopEvents(Enum):
    """Shared constants for teleoperator events across teleoperators."""

    SUCCESS = "success"
    FAILURE = "failure"
    RERECORD_EPISODE = "rerecord_episode"
    IS_INTERVENTION = "is_intervention"
    TERMINATE_EPISODE = "terminate_episode"
    BACK_HOME = "back_home"
    

def make_teleoperator_from_config(config: TeleoperatorConfig) -> "Teleoperator":
    if config.type == "quest3_webxr":
        from .quest3_webxr.teleop_quest3_webxr import Quest3WebXRTeleop

        return Quest3WebXRTeleop(config)
    if config.type == "bi_quest3_webxr":
        from .bi_quest3_webxr.bi_quest3_webxr import BiQuest3WebXR

        return BiQuest3WebXR(config)
    raise ValueError(
        f"This reduced build only supports quest3_webxr and bi_quest3_webxr, got {config.type!r}"
    )
