"""Generic damped least-squares IK for ARX5 X5 Mujoco simulation."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from .arx5_model import build_arx5_model_with_sites


DEFAULT_X5_Q_REST = np.array([0.0, 0.967, 1.290, -0.970, 0.0, 0.0])

# From ARX5_SDK/include/app/config.h for model X5. These are closer to the
# real controller than the broad limits in the SolidWorks URDF.
DEFAULT_X5_JOINT_LIMITS = np.array([
    [-3.14, 2.618],
    [-0.05, 3.50],
    [-0.20, 3.20],
    [-1.60, 1.55],
    [-1.57, 1.57],
    [-2.00, 2.00],
])

# Quest3/WebXR and Xense/Pico4 controller poses use the same lab alignment
# before they reach the ARX5 IK target: controller +Z points toward the
# operator, controller +X points right, and +Y is up. ARX5/world uses
# X forward away from the base, Y left, Z up.
ARX5_R_VR_TO_BASE = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])
QUEST3_R_TO_ARX5_BASE = ARX5_R_VR_TO_BASE.copy()
PICO4_R_TO_ARX5_BASE = ARX5_R_VR_TO_BASE.copy()
VR_TO_ARX5_QUAT_WXYZ = np.array([0.5, 0.5, -0.5, -0.5])


def _quat_wxyz_to_R(q: np.ndarray) -> np.ndarray:
    out = np.zeros(9)
    mujoco.mju_quat2Mat(out, np.asarray(q, dtype=float))
    return out.reshape(3, 3)


def _R_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_mat2Quat(out, np.ascontiguousarray(R, dtype=float).ravel())
    return out


def _quat_mul(qa: np.ndarray, qb: np.ndarray) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(qa, dtype=float), np.asarray(qb, dtype=float))
    return out


def _quat_conj(q: np.ndarray) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_negQuat(out, np.asarray(q, dtype=float))
    return out


def vr_pose_to_arx5_base(
    pos_vr: np.ndarray | list[float],
    quat_vr_wxyz: np.ndarray | list[float],
    *,
    R_vr_to_base: np.ndarray | list[list[float]] = ARX5_R_VR_TO_BASE,
) -> tuple[np.ndarray, np.ndarray]:
    """Map a Quest3/Pico4 controller pose into the ARX5 base/world frame.

    This mirrors the conversion from ``lerobot-xense``'s VR teleop path:
    ``vr_x`` right -> ``base_y`` left (negated), ``vr_y`` up -> ``base_z``
    up, and ``vr_z`` toward the operator -> ``base_x`` forward (negated).

    Args:
        pos_vr: Position in VR controller/world axes, shape ``(3,)``.
        quat_vr_wxyz: Orientation in the same VR axes, MuJoCo/wxyz order.
        R_vr_to_base: 3x3 rotation taking VR vectors into ARX5 base vectors.

    Returns:
        ``(pos_base, quat_base_wxyz)`` in ARX5 base/world axes.
    """
    R = np.asarray(R_vr_to_base, dtype=float).reshape(3, 3)
    pos_base = R @ np.asarray(pos_vr, dtype=float).reshape(3)
    q_frame = _R_to_quat_wxyz(R)
    quat_base = _quat_mul(
        _quat_mul(q_frame, np.asarray(quat_vr_wxyz, dtype=float).reshape(4)),
        _quat_conj(q_frame),
    )
    quat_base /= np.linalg.norm(quat_base)
    return pos_base, quat_base


def pico_pose_xyzw_to_arx5_base(
    pose_pico_xyzw: np.ndarray | list[float],
    *,
    R_vr_to_base: np.ndarray | list[list[float]] = ARX5_R_VR_TO_BASE,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert an Xense/Pico4 SDK pose to ARX5 base coordinates.

    The SDK pose layout is ``[x, y, z, qx, qy, qz, qw]``. The returned
    quaternion is ``[qw, qx, qy, qz]`` for direct use with ``solve()``.
    Quest3/WebXR callers that already have ``position`` and ``orientation``
    arrays should use :func:`quest3_pose_xyzw_to_arx5_base` instead.
    """
    pose = np.asarray(pose_pico_xyzw, dtype=float).reshape(7)
    quat_wxyz = np.array([pose[6], pose[3], pose[4], pose[5]], dtype=float)
    quat_wxyz /= np.linalg.norm(quat_wxyz)
    return vr_pose_to_arx5_base(pose[:3], quat_wxyz, R_vr_to_base=R_vr_to_base)


def quest3_pose_xyzw_to_arx5_base(
    pos_quest: np.ndarray | list[float],
    quat_quest_xyzw: np.ndarray | list[float],
    *,
    R_vr_to_base: np.ndarray | list[list[float]] = ARX5_R_VR_TO_BASE,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert a Quest3/WebXR pose to ARX5 base coordinates.

    WebXR orientations arrive as ``[qx, qy, qz, qw]``; this returns wxyz
    quaternions, matching MuJoCo and :meth:`Arx5DLSIKSolver.solve`.
    """
    qx, qy, qz, qw = np.asarray(quat_quest_xyzw, dtype=float).reshape(4)
    quat_wxyz = np.array([qw, qx, qy, qz], dtype=float)
    quat_wxyz /= np.linalg.norm(quat_wxyz)
    return vr_pose_to_arx5_base(pos_quest, quat_wxyz, R_vr_to_base=R_vr_to_base)


class Arx5DLSIKSolver:
    """Full 6D DLS IK for ARX5 X5.

    Unlike the DK1 DecoupledIKSolver, this uses all six joints for both
    position and orientation. It is intended for simulation/debug first; once
    the mapping feels correct, commands can be translated to the real ARX5
    Cartesian or joint control path.
    """

    def __init__(
        self,
        urdf_path: str | Path | None = None,
        *,
        model_name: str = "X5",
        tcp_offset_xyz: np.ndarray | list[float] | None = None,
        q_rest: np.ndarray | list[float] | None = None,
        joint_limits: np.ndarray | None = None,
        lam: float = 0.06,
        mu: float = 0.015,
        rot_weight: float = 0.5,
        n_iters: int = 4,
        max_dq_per_joint: np.ndarray | list[float] | float | None = 0.08,
    ) -> None:
        self.model, self.data = build_arx5_model_with_sites(
            urdf_path,
            model=model_name,
            tcp_offset_xyz=tcp_offset_xyz,
        )
        self.q_rest = (
            DEFAULT_X5_Q_REST.copy()
            if q_rest is None else np.asarray(q_rest, dtype=float).reshape(6).copy()
        )
        self.joint_limits = (
            DEFAULT_X5_JOINT_LIMITS.copy()
            if joint_limits is None else np.asarray(joint_limits, dtype=float).reshape(6, 2).copy()
        )
        self.lam = float(lam)
        self.mu = float(mu)
        self.rot_weight = float(rot_weight)
        self.n_iters = int(max(1, n_iters))
        if max_dq_per_joint is None:
            self.max_dq_per_joint = None
        else:
            arr = np.asarray(max_dq_per_joint, dtype=float)
            self.max_dq_per_joint = (
                np.full(6, float(arr)) if arr.shape == () else arr.reshape(6).copy()
            )

        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "tool0")
        self.j4_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "j4_anchor")
        if -1 in (self.site_id, self.j4_site_id):
            raise RuntimeError("Arx5DLSIKSolver: required site missing from model")

        self.last_limit_pressure = 0.0
        self.last_pos_err_norm = 0.0
        self.last_rot_err_norm = 0.0

    def _fk(self, qpos: np.ndarray) -> None:
        self.data.qpos[: self.model.nq] = 0.0
        self.data.qpos[:6] = np.asarray(qpos, dtype=float).reshape(6)
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)

    def fk(self, qpos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self._fk(qpos)
        pos = self.data.site_xpos[self.site_id].copy()
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, self.data.site_xmat[self.site_id])
        return pos, quat

    def j4_anchor_xpos(self) -> np.ndarray:
        return self.data.site_xpos[self.j4_site_id].copy()

    def solve(
        self,
        target_pos: np.ndarray,
        target_quat_wxyz: np.ndarray,
        qpos_seed: np.ndarray,
    ) -> np.ndarray:
        target_pos = np.asarray(target_pos, dtype=float).reshape(3)
        R_target = _quat_wxyz_to_R(np.asarray(target_quat_wxyz, dtype=float).reshape(4))
        q_seed = np.asarray(qpos_seed, dtype=float).reshape(-1)[:6]
        q = q_seed.copy()

        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))

        for _ in range(self.n_iters):
            self._fk(q)
            cur_pos = self.data.site_xpos[self.site_id].copy()
            R_cur = self.data.site_xmat[self.site_id].reshape(3, 3).copy()

            pos_err = target_pos - cur_pos
            R_err = R_target @ R_cur.T
            q_err = np.zeros(4)
            mujoco.mju_mat2Quat(q_err, np.ascontiguousarray(R_err).ravel())
            if q_err[0] < 0.0:
                q_err = -q_err
            rot_err = np.zeros(3)
            mujoco.mju_quat2Vel(rot_err, q_err, 1.0)

            self.last_pos_err_norm = float(np.linalg.norm(pos_err))
            self.last_rot_err_norm = float(np.linalg.norm(rot_err))
            if self.last_pos_err_norm < 1e-4 and self.last_rot_err_norm < 1e-3:
                break

            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.site_id)
            J = np.vstack([jacp[:, :6], self.rot_weight * jacr[:, :6]])
            err = np.concatenate([pos_err, self.rot_weight * rot_err])

            lam2 = self.lam ** 2
            mu2 = self.mu ** 2
            A = J.T @ J + (lam2 + mu2) * np.eye(6)
            b = J.T @ err + mu2 * (self.q_rest - q)
            dq = np.linalg.solve(A, b)
            q = q + dq
            q = np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])

        unclipped = q.copy()
        if self.max_dq_per_joint is not None:
            dq_total = np.clip(
                q - q_seed,
                -self.max_dq_per_joint,
                self.max_dq_per_joint,
            )
            q = q_seed + dq_total
        q = np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])
        self.last_limit_pressure = float(np.linalg.norm(unclipped - q))

        self._fk(q)
        return q
