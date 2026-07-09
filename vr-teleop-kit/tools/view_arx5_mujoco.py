"""Open the ARX5 X5 model in the Mujoco viewer.

Run:
    python tools/view_arx5_mujoco.py
    python tools/view_arx5_mujoco.py --animate
    python tools/view_arx5_mujoco.py --qpos 0,0.8,1.2,-0.7,0.2,0.0
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402
import numpy as np  # noqa: E402

from vr_teleop_kit.ik.arx5_model import (  # noqa: E402
    DEFAULT_TCP_OFFSET_XYZ,
    build_arx5_model_with_sites,
)


POSES = {
    "zero": np.zeros(6),
    "start": np.array([0.0, 0.967, 1.290, -0.970, 0.0, 0.0]),
}


def parse_vec(raw: str, *, expected: int, name: str) -> np.ndarray:
    parts = [float(x.strip()) for x in raw.split(",") if x.strip()]
    if len(parts) != expected:
        raise argparse.ArgumentTypeError(
            f"{name} must contain {expected} comma-separated floats"
        )
    return np.asarray(parts, dtype=float)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=None, help="path to ARX5 X5.urdf")
    ap.add_argument("--model", default="X5", help="ARX model name for auto URDF resolution")
    ap.add_argument(
        "--pose",
        choices=sorted(POSES),
        default="start",
        help="initial qpos preset",
    )
    ap.add_argument(
        "--qpos",
        default=None,
        help="override initial six-joint qpos, e.g. 0,0.8,1.2,-0.7,0.2,0",
    )
    ap.add_argument(
        "--tcp-offset",
        default=None,
        help="override TCP offset xyz in eef_link frame, e.g. 0.03,0,-0.006",
    )
    ap.add_argument(
        "--animate",
        action="store_true",
        help="slowly move the joints around the initial pose",
    )
    args = ap.parse_args()

    tcp_offset = (
        DEFAULT_TCP_OFFSET_XYZ
        if args.tcp_offset is None
        else parse_vec(args.tcp_offset, expected=3, name="--tcp-offset")
    )
    qpos0 = (
        POSES[args.pose].copy()
        if args.qpos is None
        else parse_vec(args.qpos, expected=6, name="--qpos")
    )

    model, data = build_arx5_model_with_sites(
        args.urdf,
        model=args.model,
        tcp_offset_xyz=tcp_offset,
    )
    data.qpos[:6] = qpos0
    mujoco.mj_forward(model, data)

    print("ARX5 Mujoco viewer")
    print(f"  model nq/nv: {model.nq}/{model.nv}")
    print(f"  initial qpos: {qpos0.tolist()}")
    print("  close the viewer window to exit")

    t0 = time.perf_counter()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if args.animate:
                t = time.perf_counter() - t0
                data.qpos[:6] = qpos0 + np.array([
                    0.25 * np.sin(0.35 * t),
                    0.20 * np.sin(0.45 * t),
                    0.20 * np.sin(0.40 * t + 0.6),
                    0.20 * np.sin(0.55 * t),
                    0.25 * np.sin(0.65 * t),
                    0.25 * np.sin(0.75 * t),
                ])
                data.qpos[:6] = np.clip(data.qpos[:6], model.jnt_range[:6, 0], model.jnt_range[:6, 1])
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()
