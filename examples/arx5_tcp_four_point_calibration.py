#!/usr/bin/env python3
"""Four-point TCP pivot calibration for ARX5.

Keep the real tool tip fixed at the same physical point, move the wrist to four
different orientations, and press Enter after each pose. The script solves:

    p_i + R_i * t = c

where t is the offset from the SDK/URDF eef_link to the real TCP, expressed in
the eef_link frame, and c is the fixed pivot point in the robot base frame.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from lerobot.utils.robot_utils import xyz_rpy_to_matrix


def _import_pyarx():
    try:
        import pyarx as arx5

        return arx5
    except ImportError:
        repo_root = Path(__file__).resolve().parents[1]
        sdk_python = repo_root / "third_party" / "ARX5_SDK" / "python"
        if str(sdk_python) not in sys.path:
            sys.path.append(str(sdk_python))
        import pyarx as arx5

        return arx5


def _fmt_vec(values: np.ndarray, unit: str = "m") -> str:
    if unit == "mm":
        return "[" + ", ".join(f"{float(v) * 1000.0:+.2f}" for v in values) + "] mm"
    return "[" + ", ".join(f"{float(v):+.6f}" for v in values) + "] m"


def _make_controller(arx5, model: str, interface: str, background: bool):
    robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
    controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", robot_config.joint_dof
    )
    controller_config.background_send_recv = bool(background)
    controller = arx5.Arx5JointController(robot_config, controller_config, interface)
    controller.set_log_level(arx5.LogLevel.INFO)
    return controller, robot_config, controller_config


def _set_soft_teach_mode(arx5, controller, robot_config, keep_gripper: bool) -> None:
    if hasattr(controller, "set_to_gravity_compensation"):
        try:
            controller.set_to_gravity_compensation()
            return
        except Exception as exc:
            print(f"[WARN] set_to_gravity_compensation failed, falling back to zero gain: {exc}")

    gain = arx5.Gain(robot_config.joint_dof)
    gain.kp()[:] = 0.0
    gain.kd()[:] = 0.0
    if keep_gripper:
        old_gain = controller.get_gain()
        gain.gripper_kp = float(old_gain.gripper_kp)
        gain.gripper_kd = float(old_gain.gripper_kd)
    else:
        gain.gripper_kp = 0.0
        gain.gripper_kd = 0.0
    controller.set_gain(gain)


def _read_pose(controller, settle_s: float) -> np.ndarray:
    if settle_s > 0:
        time.sleep(settle_s)
    eef_state = controller.get_eef_state()
    pose = np.asarray(eef_state.pose_6d(), dtype=np.float64).reshape(6).copy()
    if not np.all(np.isfinite(pose)):
        raise RuntimeError(f"Non-finite EEF pose read from SDK: {pose}")
    return pose


def _solve_pivot(poses: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    # p_i + R_i * t = c  ->  [R_i, -I] [t, c]^T = -p_i
    rows = []
    rhs = []
    for pose in poses:
        transform = xyz_rpy_to_matrix(pose)
        rotation = transform[:3, :3]
        position = transform[:3, 3]
        rows.append(np.hstack([rotation, -np.eye(3)]))
        rhs.append(-position)

    a = np.vstack(rows)
    b = np.concatenate(rhs)
    solution, *_ = np.linalg.lstsq(a, b, rcond=None)
    tcp_offset_eef = solution[:3]
    pivot_base = solution[3:6]

    residuals = []
    for pose in poses:
        transform = xyz_rpy_to_matrix(pose)
        predicted_pivot = transform[:3, 3] + transform[:3, :3] @ tcp_offset_eef
        residuals.append(np.linalg.norm(predicted_pivot - pivot_base))
    return tcp_offset_eef, pivot_base, np.asarray(residuals)


def _save_result(path: Path, model: str, interface: str, poses: list[np.ndarray], tcp_offset: np.ndarray, pivot: np.ndarray, residuals: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "robot": "arx5",
        "model": model,
        "interface": interface,
        "tcp_offset_xyz_m": [float(v) for v in tcp_offset],
        "pivot_point_base_xyz_m": [float(v) for v in pivot],
        "residuals_m": [float(v) for v in residuals],
        "rms_residual_m": float(math.sqrt(np.mean(np.square(residuals)))),
        "samples_eef_xyzrpy": [[float(v) for v in pose] for pose in poses],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Four-point pivot calibration for ARX5 real TCP offset."
    )
    parser.add_argument("--model", default="X5", help="ARX arm model, usually X5.")
    parser.add_argument("--interface", "--arm-port", default="can0", help="CAN interface, for example can0/can3.")
    parser.add_argument("--samples", type=int, default=4, help="Number of pivot samples. Four is the minimum.")
    parser.add_argument("--settle-s", type=float, default=0.3, help="Delay before reading each sample.")
    parser.add_argument("--no-background", action="store_true", help="Disable SDK background send/recv.")
    parser.add_argument("--release-gripper", action="store_true", help="Also zero gripper gains during calibration.")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("tcp_calibration_arx5.json"),
        help="Where to save the calibration result JSON.",
    )
    args = parser.parse_args()

    if args.samples < 4:
        raise ValueError("--samples must be at least 4")

    arx5 = _import_pyarx()
    controller = None
    poses: list[np.ndarray] = []

    print("\nARX5 four-point TCP calibration")
    print("1. Fix the real TCP tip on one physical point.")
    print("2. Move the wrist to different orientations while keeping that tip fixed.")
    print("3. Press Enter to record each pose. Use Ctrl+C to abort.\n")
    input("Press Enter to connect and enter soft teach mode...")

    try:
        controller, robot_config, controller_config = _make_controller(
            arx5, args.model, args.interface, background=not args.no_background
        )
        _set_soft_teach_mode(
            arx5,
            controller,
            robot_config,
            keep_gripper=not args.release_gripper,
        )

        print("\nConnected. Move the arm by hand; do not let the TCP tip slip.")
        print(f"Sampling {args.samples} poses from SDK eef_link pose_6d().")

        for idx in range(args.samples):
            input(f"\nPose {idx + 1}/{args.samples}: hold TCP fixed, change wrist orientation, then press Enter...")
            if not controller_config.background_send_recv:
                controller.send_recv_once()
            pose = _read_pose(controller, args.settle_s)
            poses.append(pose)
            print(f"  eef xyz    : {_fmt_vec(pose[:3])}")
            print(f"  eef rpy rad: [{pose[3]:+.4f}, {pose[4]:+.4f}, {pose[5]:+.4f}]")

        tcp_offset, pivot, residuals = _solve_pivot(poses)
        rms = math.sqrt(float(np.mean(np.square(residuals))))
        max_residual = float(np.max(residuals))

        print("\n================ TCP CALIBRATION RESULT ================")
        print(f"tcp_offset_xyz in eef_link frame : {_fmt_vec(tcp_offset)}")
        print(f"tcp_offset_xyz in eef_link frame : {_fmt_vec(tcp_offset, unit='mm')}")
        print(f"fixed pivot point in base frame  : {_fmt_vec(pivot)}")
        print(f"residuals                        : {_fmt_vec(residuals, unit='mm')}")
        print(f"rms / max residual               : {rms * 1000.0:.2f} mm / {max_residual * 1000.0:.2f} mm")
        print("========================================================")

        if max_residual > 0.01:
            print("[WARN] Max residual is above 10 mm. Repeat with less tip slipping and wider wrist orientation changes.")
        elif max_residual > 0.005:
            print("[WARN] Max residual is above 5 mm. Usable for rough teleop, but repeat if you need precision.")

        _save_result(args.output, args.model, args.interface, poses, tcp_offset, pivot, residuals)
        print(f"\nSaved result to: {args.output.resolve()}")
        print("\nUse tcp_offset_xyz_m as the initial value for ARX5 tcp_offset_xyz.")
        return 0

    except KeyboardInterrupt:
        print("\nCalibration aborted.")
        return 130
    finally:
        if controller is not None:
            try:
                controller.set_to_damping()
                print("Controller switched to damping.")
            except Exception as exc:
                print(f"[WARN] Failed to switch controller to damping: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
