#!/usr/bin/env python

"""ARX5 + Quest3 teleoperation entry point.

This reduced build intentionally keeps only the ARX5 follower and the
``quest3_webxr`` teleoperator used by this project.
"""

import time
import traceback
from dataclasses import asdict, dataclass
from pprint import pformat

import numpy as np

from lerobot.configs import parser
from lerobot.robots import Robot, RobotConfig, arx5_follower, bi_arx5, make_robot_from_config  # noqa: F401
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_quest3_webxr,
    make_teleoperator_from_config,
    quest3_webxr,
)
from lerobot.utils.robot_utils import get_logger, precise_sleep
from lerobot.utils.utils import move_cursor_up

logger = get_logger("Teleoperate")


@dataclass
class TeleoperateConfig:
    teleop: TeleoperatorConfig
    robot: RobotConfig
    fps: int = 60
    teleop_time_s: float | None = None
    display_data: bool = False
    debug_timing: bool = False
    dryrun: bool = False


def _safe_disconnect(obj, name: str) -> None:
    if obj is None:
        return
    try:
        if obj.is_connected:
            obj.disconnect()
            logger.info(f"{name} disconnected")
    except Exception as e:
        logger.error(f"Error disconnecting {name}: {e}\n{traceback.format_exc()}")


def _print_display_panel(
    robot: Robot,
    obs: dict,
    action: dict[str, float],
    loop_s: float,
    display_len: int,
) -> None:
    panel_lines = []
    panel_lines.append("-" * (display_len + 34))
    panel_lines.append(f"{'NAME':<{display_len}} | {'CMD':>10} | {'OBS':>10}")
    for key in robot.action_features:
        if key not in action:
            continue
        cmd = float(action[key])
        obs_val = obs.get(key, None)
        obs_str = "-"
        if obs_val is not None and not isinstance(obs_val, np.ndarray):
            obs_str = f"{float(obs_val):>10.4f}"
        panel_lines.append(f"{key:<{display_len}} | {cmd:>10.4f} | {obs_str}")
    panel_lines.append(f"{'timing':<{display_len}} | {'loop':>10} | {loop_s:>8.3f}s")
    print("\n".join(panel_lines), flush=True)
    move_cursor_up(len(panel_lines))


def quest_teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    display_data: bool = False,
    duration: float | None = None,
    dryrun: bool = False,
    debug_timing: bool = False,
) -> None:
    """Dedicated loop for ARX5 follower driven by Quest3 input."""
    display_len = max(len(key) for key in robot.action_features)
    start = time.perf_counter()
    robot_action_keys = set(robot.action_features.keys())
    warned_unmapped_keys = False

    while True:
        loop_start = time.perf_counter()

        obs_start = time.perf_counter()
        obs = robot.get_observation()
        obs_dt_ms = (time.perf_counter() - obs_start) * 1e3

        raw_action = teleop.get_action()
        robot_action_to_send = {k: v for k, v in raw_action.items() if k in robot_action_keys}
        if len(robot_action_to_send) != len(raw_action) and not warned_unmapped_keys:
            dropped = sorted(set(raw_action) - robot_action_keys)
            logger.warning(f"Quest3 action keys not present in robot schema, dropping: {dropped}")
            warned_unmapped_keys = True

        if not robot_action_to_send:
            raise ValueError(
                "No overlapping action keys between Quest3 output and robot action schema. "
                "Use --robot.control_mode=cartesian_control so tcp.* keys are available."
            )

        if hasattr(teleop, "get_reset_button") and teleop.get_reset_button():
            try:
                if dryrun:
                    if teleop.name == "bi_quest3_webxr" and hasattr(robot, "get_current_tcp_poses_euler"):
                        left_pose, right_pose = robot.get_current_tcp_poses_euler()
                        teleop.reset_to_pose(left_pose[:6], right_pose[:6], left_pose[6], right_pose[6])
                    elif hasattr(robot, "get_start_eef_pose"):
                        eef = robot.get_start_eef_pose()
                        teleop.reset_to_pose(eef[:6], float(eef[6]))
                    logger.info("Quest3 reset requested (dryrun: robot motion skipped)")
                elif teleop.name == "bi_quest3_webxr" and hasattr(robot, "smooth_go_start"):
                    robot.smooth_go_start(duration=2.0)
                    left_pose, right_pose = robot.get_current_tcp_poses_euler()
                    teleop.reset_to_pose(left_pose[:6], right_pose[:6], left_pose[6], right_pose[6])
                    logger.info("Quest3 reset: BiARX5 moved to start pose")
                elif hasattr(robot, "smooth_go_start") and hasattr(robot, "get_start_eef_pose"):
                    robot.smooth_go_start(duration=2.0)
                    eef = robot.get_start_eef_pose()
                    teleop.reset_to_pose(eef[:6], float(eef[6]))
                    logger.info("Quest3 reset: ARX5 moved to start pose")
                else:
                    logger.warning("Quest3 reset requested, but robot has no ARX5 reset API")
            except Exception as e:
                logger.error(f"Failed to reset ARX5 from Quest3 button: {e}\n{traceback.format_exc()}")

            dt_s = time.perf_counter() - loop_start
            precise_sleep(max(1 / fps - dt_s, 0))
            continue

        if not dryrun:
            robot.send_action(robot_action_to_send)

        dt_s = time.perf_counter() - loop_start
        precise_sleep(max(1 / fps - dt_s, 0))
        loop_s = time.perf_counter() - loop_start

        if display_data:
            _print_display_panel(robot, obs, robot_action_to_send, loop_s, display_len)
        elif debug_timing:
            dryrun_tag = " | DRYRUN" if dryrun else ""
            print(
                f"\r\033[KARX5+Quest3 obs: {obs_dt_ms:5.1f}ms | loop: {loop_s * 1e3:5.1f}ms "
                f"({1 / loop_s:4.0f}Hz){dryrun_tag}",
                end="",
                flush=True,
            )
        else:
            enabled = "ON " if getattr(teleop, "_enabled", False) else "OFF"
            stale = "STALE" if getattr(teleop, "_is_stale", lambda: True)() else "LIVE "
            if teleop.name == "bi_quest3_webxr":
                grip = robot_action_to_send.get("right_gripper.pos", 0.0)
                pos_str = (
                    f"L=[{robot_action_to_send.get('left_tcp.x', 0.0):+.3f},"
                    f"{robot_action_to_send.get('left_tcp.y', 0.0):+.3f},"
                    f"{robot_action_to_send.get('left_tcp.z', 0.0):+.3f}] "
                    f"R=[{robot_action_to_send.get('right_tcp.x', 0.0):+.3f},"
                    f"{robot_action_to_send.get('right_tcp.y', 0.0):+.3f},"
                    f"{robot_action_to_send.get('right_tcp.z', 0.0):+.3f}]"
                )
            else:
                grip = robot_action_to_send.get("gripper.pos", 0.0)
                pos_str = (
                    f"pos=[{robot_action_to_send.get('tcp.x', 0.0):+.3f}, "
                    f"{robot_action_to_send.get('tcp.y', 0.0):+.3f}, "
                    f"{robot_action_to_send.get('tcp.z', 0.0):+.3f}]"
                )
            dryrun_tag = "[DRYRUN] " if dryrun else ""
            print(
                f"\r\033[K{dryrun_tag}{loop_s * 1e3:5.1f}ms ({1 / loop_s:4.0f}Hz) | "
                f"quest={enabled} {stale} | {pos_str} | gripper={float(grip):.2f}",
                end="",
                flush=True,
            )

        if duration is not None and time.perf_counter() - start >= duration:
            return


@parser.wrap()
def teleoperate(cfg: TeleoperateConfig) -> None:
    logger.info(pformat(asdict(cfg)))
    if cfg.dryrun:
        logger.warn("DRYRUN MODE ENABLED - Actions will be printed but NOT sent to robot")

    supported_pairs = {
        ("arx5_follower", "quest3_webxr"),
        ("bi_arx5", "bi_quest3_webxr"),
    }
    if (cfg.robot.type, cfg.teleop.type) not in supported_pairs:
        raise ValueError(
            "This reduced build supports only "
            "arx5_follower+quest3_webxr and bi_arx5+bi_quest3_webxr, "
            f"got {cfg.robot.type!r}+{cfg.teleop.type!r}"
        )

    robot = None
    teleop = None
    try:
        logger.info(f"Detected {cfg.robot.type} + {cfg.teleop.type}")
        robot = make_robot_from_config(cfg.robot)
        robot.connect()

        teleop = make_teleoperator_from_config(cfg.teleop)
        if cfg.robot.type == "bi_arx5":
            left_pose, right_pose = robot.get_current_tcp_poses_euler()
            logger.info(f"Current left TCP pose (euler+gripper): {left_pose}")
            logger.info(f"Current right TCP pose (euler+gripper): {right_pose}")
            teleop.connect(left_tcp_pose_euler=left_pose, right_tcp_pose_euler=right_pose)
        else:
            current_pose = robot.get_current_tcp_pose_euler()
            logger.info(f"Current TCP pose (euler+gripper): {current_pose}")
            teleop.connect(current_tcp_pose_euler=current_pose)

        quest_teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            display_data=cfg.display_data,
            duration=cfg.teleop_time_s,
            dryrun=cfg.dryrun,
            debug_timing=cfg.debug_timing,
        )
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Error in teleoperation: {e}\n{traceback.format_exc()}")
        raise
    finally:
        _safe_disconnect(teleop, "teleop")
        _safe_disconnect(robot, "robot")


def main() -> None:
    teleoperate()


if __name__ == "__main__":
    main()
