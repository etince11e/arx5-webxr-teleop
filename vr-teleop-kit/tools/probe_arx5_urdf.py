"""Probe the ARX5 X5 URDF as loaded by Mujoco.

Run from the vr-teleop-kit checkout:

    python tools/probe_arx5_urdf.py
    python tools/probe_arx5_urdf.py --urdf /path/to/X5.urdf

The script adds the ARX5 `tool0` and `j4_anchor` sites through
vr_teleop_kit.ik.arx5_model and prints joints, bodies, sites, and FK at a
couple of useful poses.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from vr_teleop_kit.ik.arx5_model import (  # noqa: E402
    DEFAULT_TCP_OFFSET_XYZ,
    arx5_tool0_pose_from_urdf,
    build_arx5_model_with_sites,
    resolve_arx5_urdf_path,
)


JOINT_TYPE_NAMES = {
    mujoco.mjtJoint.mjJNT_FREE: "free",
    mujoco.mjtJoint.mjJNT_BALL: "ball",
    mujoco.mjtJoint.mjJNT_SLIDE: "slide",
    mujoco.mjtJoint.mjJNT_HINGE: "hinge",
}


def name_of(model: mujoco.MjModel, obj_type, idx: int) -> str:
    name = mujoco.mj_id2name(model, obj_type, idx)
    return name if name is not None else f"<unnamed#{idx}>"


def print_model_summary(model: mujoco.MjModel) -> None:
    print("=== Model summary ===")
    print(f"  nq    (generalized coords) = {model.nq}")
    print(f"  nv    (DoFs)               = {model.nv}")
    print(f"  njnt  (joints)             = {model.njnt}")
    print(f"  nbody (bodies, incl world) = {model.nbody}")
    print(f"  nsite (sites)              = {model.nsite}")


def print_joints(model: mujoco.MjModel) -> None:
    print("\n=== Joints ===")
    for j in range(model.njnt):
        jname = name_of(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        jtype = JOINT_TYPE_NAMES.get(int(model.jnt_type[j]), f"type#{int(model.jnt_type[j])}")
        qadr = int(model.jnt_qposadr[j])
        rng = model.jnt_range[j]
        limited = bool(model.jnt_limited[j])
        rng_str = f"[{rng[0]:+.3f}, {rng[1]:+.3f}]" if limited else "(unlimited)"
        print(f"  [{j}] {jname:12s} type={jtype:6s} qpos_idx={qadr:>2}  range={rng_str}")


def print_bodies(model: mujoco.MjModel) -> None:
    print("\n=== Bodies ===")
    for b in range(model.nbody):
        bname = name_of(model, mujoco.mjtObj.mjOBJ_BODY, b)
        parent = int(model.body_parentid[b])
        pname = name_of(model, mujoco.mjtObj.mjOBJ_BODY, parent) if parent != b else "(self/world)"
        print(f"  [{b}] {bname:12s} parent={pname}")


def print_sites(model: mujoco.MjModel) -> None:
    print("\n=== Sites ===")
    for s in range(model.nsite):
        sname = name_of(model, mujoco.mjtObj.mjOBJ_SITE, s)
        body = int(model.site_bodyid[s])
        bname = name_of(model, mujoco.mjtObj.mjOBJ_BODY, body)
        pos = model.site_pos[s]
        quat = model.site_quat[s]
        print(
            f"  [{s}] {sname:12s} attached_to={bname:8s} "
            f"local_pos=({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f}) "
            f"local_quat_wxyz=({quat[0]:+.4f},{quat[1]:+.4f},{quat[2]:+.4f},{quat[3]:+.4f})"
        )


def fk_dump(model: mujoco.MjModel, data: mujoco.MjData, label: str) -> None:
    print(f"\n=== FK at {label} ===")
    mujoco.mj_forward(model, data)

    for bname in ("base_link", "link1", "link2", "link3", "link4", "link5", "link6"):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname)
        if bid == -1:
            continue
        pos = data.xpos[bid]
        quat = data.xquat[bid]
        print(
            f"  body  {bname:10s}: "
            f"pos=({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f}) "
            f"quat_wxyz=({quat[0]:+.4f},{quat[1]:+.4f},{quat[2]:+.4f},{quat[3]:+.4f})"
        )

    for sname in ("j4_anchor", "tool0"):
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, sname)
        if sid == -1:
            continue
        pos = data.site_xpos[sid]
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, data.site_xmat[sid])
        print(
            f"  site  {sname:10s}: "
            f"pos=({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f}) "
            f"quat_wxyz=({quat[0]:+.4f},{quat[1]:+.4f},{quat[2]:+.4f},{quat[3]:+.4f})"
        )


def parse_vec3(raw: str | None, fallback: np.ndarray) -> np.ndarray:
    if raw is None:
        return fallback
    parts = [float(x.strip()) for x in raw.split(",") if x.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--tcp-offset must contain three comma-separated floats")
    return np.asarray(parts, dtype=float)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=None, help="path to ARX5 X5.urdf (default: auto/ARX5_URDF)")
    ap.add_argument("--model", default="X5", help="ARX model name used for auto resolution")
    ap.add_argument(
        "--tcp-offset",
        default=None,
        help="override calibrated TCP offset xyz in eef_link frame, e.g. 0.03,0,-0.006",
    )
    args = ap.parse_args()

    tcp_offset = parse_vec3(args.tcp_offset, DEFAULT_TCP_OFFSET_XYZ)
    urdf = resolve_arx5_urdf_path(args.urdf, model=args.model)
    tool_pos, tool_quat = arx5_tool0_pose_from_urdf(
        urdf,
        model=args.model,
        tcp_offset_xyz=tcp_offset,
    )

    print(f"loading: {urdf}")
    print(f"tcp_offset_xyz: {tcp_offset.tolist()}")
    print(f"tool0 on link6: pos={tool_pos.tolist()} quat_wxyz={tool_quat.tolist()}\n")

    model, data = build_arx5_model_with_sites(
        urdf,
        model=args.model,
        tcp_offset_xyz=tcp_offset,
    )
    print_model_summary(model)
    print_joints(model)
    print_bodies(model)
    print_sites(model)

    data.qpos[:] = 0.0
    fk_dump(model, data, "qpos = 0")

    start = np.zeros(model.nq)
    start[: min(6, model.nq)] = [0.0, 0.967, 1.290, -0.970, 0.0, 0.0][: min(6, model.nq)]
    data.qpos[:] = start
    fk_dump(model, data, "ARX5 cartesian start qpos")


if __name__ == "__main__":
    main()
