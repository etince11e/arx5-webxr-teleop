"""Mujoco model construction for the ARX5 X5 arm.

The ARX5 SDK URDF lives outside this repository, usually at:

    third_party/ARX5_SDK/models/X5.urdf

This module loads that URDF and adds the two sites the VR teleop IK/viewer
pipeline expects:

  tool0      - real TCP site on link6. The base eef_link fixed-joint offset is
               read from the URDF, then the calibrated TCP offset is applied.
  j4_anchor  - wrist-invariant position-task anchor on link3, near joint4.

The existing DecoupledIKSolver is still DK1-tuned; this loader is the first
piece needed to test ARX5 in Mujoco and to build an ARX5-specific solver.
"""

from __future__ import annotations

import os
import shutil
import tempfile
from contextlib import contextmanager
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np


ARX5_URDF_ENV = "ARX5_URDF"
ARX5_SDK_ROOT_ENV = "ARX5_SDK_ROOT"
LEROBOT_XENSE_ROOT_ENV = "LEROBOT_XENSE_ROOT"

DEFAULT_ARX5_MODEL = "X5"

# ARX5FollowerConfig.tcp_offset_xyz default, calibrated by
# lerobot-xense/examples/arx5_tcp_four_point_calibration.py.
DEFAULT_TCP_OFFSET_XYZ = np.array([
    0.03289615157942764,
    0.002644292774758826,
    -0.006774168923815918,
])

# joint4 origin in link3 is (0.245, 0, -0.056). Place the anchor 10 cm past
# that pivot along the same link3->link4 direction so the position task does
# not sit exactly on the wrist joint.
J4_ANCHOR_LINK = "link3"
J4_ANCHOR_XYZ = np.array([0.3425, 0.0, -0.0783])

TOOL_PARENT_LINK = "link6"
EEF_FIXED_JOINT = "gripper_fixed_joint"


def rpy_to_wxyz(rpy: np.ndarray) -> np.ndarray:
    """URDF rpy convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    r, p, y = np.asarray(rpy, dtype=float).reshape(3)
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    r, p, y = np.asarray(rpy, dtype=float).reshape(3)
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _candidate_roots() -> list[Path]:
    roots: list[Path] = []
    for env_var in (LEROBOT_XENSE_ROOT_ENV, ARX5_SDK_ROOT_ENV):
        raw = os.environ.get(env_var)
        if raw:
            roots.append(Path(raw).expanduser())

    anchors = [Path.cwd(), Path(__file__).resolve()]
    for anchor in anchors:
        for base in [anchor, *anchor.parents]:
            roots.extend([
                base,
                base / "lerobot-xense",
                base / "quest_arx" / "lerobot-xense",
            ])

    unique: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        try:
            resolved = root.resolve()
        except OSError:
            resolved = root
        if resolved not in seen:
            unique.append(root)
            seen.add(resolved)
    return unique


def _urdf_under_root(root: Path, model: str) -> Path:
    if root.name == "ARX5_SDK":
        return root / "models" / f"{model}.urdf"
    return root / "third_party" / "ARX5_SDK" / "models" / f"{model}.urdf"


def resolve_arx5_urdf_path(
    explicit: str | Path | None = None,
    *,
    model: str = DEFAULT_ARX5_MODEL,
) -> Path:
    """Resolve an ARX5 URDF path.

    Priority:
      1. explicit argument
      2. ARX5_URDF
      3. ARX5_SDK_ROOT/models/{model}.urdf
      4. nearby lerobot-xense / quest_arx/lerobot-xense checkouts
    """
    raw = explicit or os.environ.get(ARX5_URDF_ENV)
    if raw:
        path = Path(raw).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"ARX5 URDF not found at {path}")
        return path

    for root in _candidate_roots():
        candidate = _urdf_under_root(root, model)
        if candidate.exists():
            return candidate

    raise FileNotFoundError(
        "No ARX5 URDF configured. Pass urdf_path=..., set ARX5_URDF, "
        "or set LEROBOT_XENSE_ROOT / ARX5_SDK_ROOT."
    )


def _parse_origin_xyz_rpy(urdf_path: Path, joint_name: str) -> tuple[np.ndarray, np.ndarray]:
    tree = ET.parse(urdf_path)
    for joint in tree.getroot().findall("joint"):
        if joint.attrib.get("name") != joint_name:
            continue
        origin = joint.find("origin")
        if origin is None:
            return np.zeros(3), np.zeros(3)
        xyz = np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.attrib.get("rpy", "0 0 0"), sep=" ")
        if xyz.size != 3 or rpy.size != 3:
            raise ValueError(f"Malformed origin on joint {joint_name!r} in {urdf_path}")
        return xyz.astype(float), rpy.astype(float)
    raise RuntimeError(f"Joint {joint_name!r} not found in {urdf_path}")


def arx5_tool0_pose_from_urdf(
    urdf_path: str | Path | None = None,
    *,
    model: str = DEFAULT_ARX5_MODEL,
    tcp_offset_xyz: np.ndarray | list[float] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (tool0 local pos, tool0 local quat wxyz) attached to link6."""
    resolved = resolve_arx5_urdf_path(urdf_path, model=model)
    eef_xyz, eef_rpy = _parse_origin_xyz_rpy(resolved, EEF_FIXED_JOINT)
    tcp_offset = (
        DEFAULT_TCP_OFFSET_XYZ
        if tcp_offset_xyz is None
        else np.asarray(tcp_offset_xyz, dtype=float).reshape(3)
    )
    tool_pos = eef_xyz + rpy_to_matrix(eef_rpy) @ tcp_offset
    tool_quat = rpy_to_wxyz(eef_rpy)
    return tool_pos, tool_quat


def _patched_urdf_text_if_needed(urdf_path: Path) -> str | None:
    """Patch known ARX5 SDK X5 mesh path typo without editing SDK files."""
    text = urdf_path.read_text(encoding="utf-8")
    parent = urdf_path.parent
    bad = "./meshes/base_link.STL"
    fixed = "./meshes/X5/base_link.STL"
    if bad in text and not (parent / "meshes" / "base_link.STL").exists():
        if (parent / "meshes" / "X5" / "base_link.STL").exists():
            return text.replace(bad, fixed)
    return None


@contextmanager
def _mujoco_urdf_path(urdf_path: Path):
    patched = _patched_urdf_text_if_needed(urdf_path)
    if patched is None:
        yield urdf_path
        return

    with tempfile.TemporaryDirectory(prefix="vr_teleop_arx5_") as tmp:
        tmp_path = Path(tmp)
        staged_urdf = tmp_path / urdf_path.name
        staged_urdf.write_text(patched, encoding="utf-8")
        src_meshes = urdf_path.parent / "meshes"
        dst_meshes = tmp_path / "meshes"
        try:
            os.symlink(src_meshes, dst_meshes, target_is_directory=True)
        except OSError:
            shutil.copytree(src_meshes, dst_meshes)
        yield staged_urdf


def build_arx5_model_with_sites(
    urdf_path: str | Path | None = None,
    *,
    model: str = DEFAULT_ARX5_MODEL,
    tcp_offset_xyz: np.ndarray | list[float] | None = None,
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load ARX5 X5 into Mujoco and add tool0 + j4_anchor sites."""
    resolved = resolve_arx5_urdf_path(urdf_path, model=model)
    tool_pos, tool_quat = arx5_tool0_pose_from_urdf(
        resolved,
        model=model,
        tcp_offset_xyz=tcp_offset_xyz,
    )

    with _mujoco_urdf_path(resolved) as load_path:
        spec = mujoco.MjSpec.from_file(str(load_path))

        tool_parent = spec.body(TOOL_PARENT_LINK)
        if tool_parent is None:
            raise RuntimeError(f"{TOOL_PARENT_LINK} body not found in ARX5 URDF spec")
        tool_parent.add_site(
            name="tool0",
            pos=tool_pos.tolist(),
            quat=tool_quat.tolist(),
            size=[0.012, 0.0, 0.0],
            rgba=[0.0, 0.8, 0.2, 1.0],
        )

        anchor_parent = spec.body(J4_ANCHOR_LINK)
        if anchor_parent is None:
            raise RuntimeError(f"{J4_ANCHOR_LINK} body not found in ARX5 URDF spec")
        anchor_parent.add_site(
            name="j4_anchor",
            pos=J4_ANCHOR_XYZ.tolist(),
            size=[0.015, 0.0, 0.0],
            rgba=[1.0, 0.5, 0.0, 1.0],
        )

        compiled = spec.compile()
    return compiled, mujoco.MjData(compiled)
