#!/usr/bin/env python3
"""Pivot calibration for the Quest WebXR controller virtual TCP point.

Keep the desired virtual control point fixed in space, rotate the Quest
controller through several orientations, and press Enter for each sample. The
script solves:

    p_i + R_i * t = c

where t is the local offset from the WebXR controller tracking frame to the
virtual TCP point. The result is used as:

    --teleop.controller_tcp_offset_xyz="[x,y,z]"
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import ssl
import time
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import numpy as np

from lerobot.utils.robot_utils import normalize_quaternion, quaternion_to_matrix


def _fmt_vec(values: np.ndarray, unit: str = "m") -> str:
    if unit == "mm":
        return "[" + ", ".join(f"{float(v) * 1000.0:+.2f}" for v in values) + "] mm"
    return "[" + ", ".join(f"{float(v):+.6f}" for v in values) + "] m"


def _controller_transform_from_xr_frame(frame: dict[str, Any], hand: str) -> np.ndarray | None:
    controllers = frame.get("controllers")
    if not isinstance(controllers, dict):
        return None

    ctrl = controllers.get(hand)
    if not isinstance(ctrl, dict):
        return None

    try:
        position = np.asarray(ctrl["position"], dtype=np.float64).reshape(3)
        qx, qy, qz, qw = np.asarray(ctrl["orientation"], dtype=np.float64).reshape(4)
    except Exception:
        return None

    quat_wxyz = normalize_quaternion(
        np.asarray([qw, qx, qy, qz], dtype=np.float64),
        input_format="wxyz",
    )
    return quaternion_to_matrix(np.concatenate([position, quat_wxyz]), input_format="wxyz")


async def _recv_latest_transform(
    ws: Any,
    *,
    hand: str,
    timeout_s: float,
    sample_delay_s: float,
) -> np.ndarray:
    deadline = time.monotonic() + timeout_s
    settle_deadline = time.monotonic() + max(0.0, sample_delay_s)
    latest: np.ndarray | None = None

    while time.monotonic() < deadline:
        wait_s = max(0.01, min(0.1, deadline - time.monotonic()))
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=wait_s)
        except asyncio.TimeoutError:
            continue

        try:
            msg = json.loads(raw)
        except Exception:
            continue
        if msg.get("type") != "xr_frame":
            continue

        transform = _controller_transform_from_xr_frame(msg, hand)
        if transform is None:
            continue

        latest = transform
        if time.monotonic() >= settle_deadline:
            return latest

    if latest is None:
        raise TimeoutError(
            f"No WebXR controller frame for hand={hand!r} before timeout. "
            "Check that the Quest browser is in WebXR session and connected to the relay."
        )
    return latest


def _rotation_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    relative = a[:3, :3].T @ b[:3, :3]
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def _validate_samples(transforms: list[np.ndarray]) -> None:
    positions = np.asarray([transform[:3, 3] for transform in transforms])
    position_range = np.ptp(positions, axis=0)
    max_rotation_deg = 0.0
    for i in range(len(transforms)):
        for j in range(i + 1, len(transforms)):
            max_rotation_deg = max(max_rotation_deg, _rotation_angle_deg(transforms[i], transforms[j]))

    rows = []
    for transform in transforms:
        rows.append(np.hstack([transform[:3, :3], -np.eye(3)]))
    design = np.vstack(rows)
    rank = np.linalg.matrix_rank(design)
    singular = np.linalg.svd(design, compute_uv=False)
    cond = np.inf if singular[-1] < 1e-12 else float(singular[0] / singular[-1])

    print("\nSample quality:")
    print(f"  controller position range: {_fmt_vec(position_range, unit='mm')}")
    print(f"  max rotation separation  : {max_rotation_deg:.2f} deg")
    print(f"  solve matrix rank / cond : {rank} / {cond:.2e}")

    if rank < 6:
        raise RuntimeError(
            "Degenerate samples: solve matrix rank is below 6. "
            "The WebXR frames were repeated or the controller orientation did not change enough."
        )
    if max_rotation_deg < 20.0:
        raise RuntimeError(
            "Degenerate samples: max controller orientation change is below 20 deg. "
            "Rotate the controller through larger, different wrist angles."
        )
    if cond > 1e4:
        print("[WARN] Sample geometry is poorly conditioned. More diverse orientations will improve the result.")


def _solve_pivot(transforms: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows = []
    rhs = []
    for transform in transforms:
        rotation = transform[:3, :3]
        position = transform[:3, 3]
        rows.append(np.hstack([rotation, -np.eye(3)]))
        rhs.append(-position)

    a = np.vstack(rows)
    b = np.concatenate(rhs)
    solution, *_ = np.linalg.lstsq(a, b, rcond=None)
    offset_controller = solution[:3]
    pivot_world = solution[3:6]

    residuals = []
    for transform in transforms:
        predicted_pivot = transform[:3, 3] + transform[:3, :3] @ offset_controller
        residuals.append(np.linalg.norm(predicted_pivot - pivot_world))
    return offset_controller, pivot_world, np.asarray(residuals)


def _save_result(
    path: Path,
    ws_url: str,
    hand: str,
    transforms: list[np.ndarray],
    offset: np.ndarray,
    pivot: np.ndarray,
    residuals: np.ndarray,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "teleop": "quest3_webxr",
        "ws_url": ws_url,
        "hand": hand,
        "orientation_source": "quat",
        "controller_tcp_offset_xyz_m": [float(v) for v in offset],
        "controller_tcp_offset_rpy_rad": [0.0, 0.0, 0.0],
        "pivot_point_world_xyz_m": [float(v) for v in pivot],
        "residuals_m": [float(v) for v in residuals],
        "rms_residual_m": float(math.sqrt(np.mean(np.square(residuals)))),
        "samples_controller_transform": [transform.tolist() for transform in transforms],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _ssl_context_for(ws_url: str) -> ssl.SSLContext | None:
    parsed = urlparse(ws_url)
    if parsed.scheme != "wss":
        return None
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    return context


async def _main_async(args: argparse.Namespace) -> int:
    try:
        import websockets
    except ImportError as exc:
        raise RuntimeError("websockets is required for Quest3 WebXR calibration.") from exc

    ssl_context = _ssl_context_for(args.ws_url)
    transforms: list[np.ndarray] = []

    print("\nQuest WebXR controller virtual TCP pivot calibration")
    print("1. Start vr-teleop-relay and open the WebXR page in Quest Browser.")
    print("2. Click Start Teleop and keep the target controller visible/tracked.")
    print("3. Pick the point on/near the controller that should behave like the robot TCP.")
    print("4. Keep that point fixed in space, rotate the controller, and press Enter for each sample.\n")
    print(f"Connecting to relay: {args.ws_url}")
    print(f"Hand: {args.hand}")

    async with websockets.connect(args.ws_url, ssl=ssl_context) as ws:
        for idx in range(args.samples):
            input(f"Sample {idx + 1}/{args.samples}: hold virtual TCP fixed, rotate controller, press Enter...")
            transform = await _recv_latest_transform(
                ws,
                hand=args.hand,
                timeout_s=float(args.timeout_s),
                sample_delay_s=float(args.sample_delay_s),
            )
            transforms.append(transform)
            print(f"  controller_pos: {_fmt_vec(transform[:3, 3])}")
            if len(transforms) > 1:
                dp = np.linalg.norm(transforms[-1][:3, 3] - transforms[0][:3, 3])
                da = _rotation_angle_deg(transforms[0], transforms[-1])
                print(f"  vs sample 1   : position {dp * 1000.0:.1f} mm, rotation {da:.1f} deg")

    _validate_samples(transforms)
    offset, pivot, residuals = _solve_pivot(transforms)
    rms = math.sqrt(float(np.mean(np.square(residuals))))
    max_residual = float(np.max(residuals))

    print("\n=========== QUEST WEBXR TCP CALIBRATION ===========")
    print(f"controller_tcp_offset_xyz : {_fmt_vec(offset)}")
    print(f"controller_tcp_offset_xyz : {_fmt_vec(offset, unit='mm')}")
    print(f"fixed pivot point world   : {_fmt_vec(pivot)}")
    print(f"residuals                 : {_fmt_vec(residuals, unit='mm')}")
    print(f"rms / max residual        : {rms * 1000.0:.2f} mm / {max_residual * 1000.0:.2f} mm")
    print("===================================================")
    if max_residual > 0.03:
        print("[WARN] Max residual is above 30 mm. Repeat while keeping the same point more fixed.")
    elif max_residual > 0.015:
        print("[WARN] Max residual is above 15 mm. Usable for rough testing, but repeat for better control.")

    _save_result(args.output, args.ws_url, args.hand, transforms, offset, pivot, residuals)
    print(f"\nSaved result to: {args.output.resolve()}")
    print("\nUse it with:")
    print("  --teleop.controller_orientation_source=quat")
    print(f'  --teleop.controller_tcp_offset_xyz="[{offset[0]:.6f},{offset[1]:.6f},{offset[2]:.6f}]"')
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Quest WebXR controller virtual TCP pivot calibration.")
    parser.add_argument("--ws-url", default="ws://127.0.0.1:8443/ws")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--samples", type=int, default=6, help="Number of samples. Four is the minimum.")
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument(
        "--sample-delay-s",
        type=float,
        default=0.2,
        help="After Enter, wait this long for a fresh WebXR frame.",
    )
    parser.add_argument("--output", type=Path, default=Path("quest3_webxr_tcp_calibration.json"))
    args = parser.parse_args()

    if args.samples < 4:
        raise ValueError("--samples must be at least 4")
    return asyncio.run(_main_async(args))


if __name__ == "__main__":
    raise SystemExit(main())
