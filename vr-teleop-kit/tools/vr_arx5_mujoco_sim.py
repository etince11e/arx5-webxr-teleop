"""VR-driven ARX5 Mujoco simulation.

Terminal 1:
    vr-teleop-relay

Terminal 2:
    python tools/vr_arx5_mujoco_sim.py

Quest browser:
    http://localhost:8443/  (USB adb reverse)
    Start Teleop, then hold the grip button to clutch-control the ARX5.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import ssl
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402
import numpy as np  # noqa: E402
import websockets  # noqa: E402

from vr_teleop_kit.core.pose_mapping import ClutchPoseMapper  # noqa: E402
from vr_teleop_kit.ik.arx5_ik import (  # noqa: E402
    ARX5_R_VR_TO_BASE,
    Arx5DLSIKSolver,
    DEFAULT_X5_Q_REST,
)


DEFAULT_URL = "ws://127.0.0.1:8443/ws"
DEFAULT_R_CALIB = ARX5_R_VR_TO_BASE.copy()

GRIP_BUTTON_INDEX = 1
TRIGGER_BUTTON_INDEX = 0
RESET_BUTTON_INDEX = 4
REST_RAMP_BUTTON_INDEX = 3
XR_FRAME_STALE_TIMEOUT_S = 0.2


def _yaw_from_quat_xyzw(q_xyzw) -> float | None:
    if q_xyzw is None:
        return None
    x, y, z, w = (float(v) for v in q_xyzw)
    return float(np.arctan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (y * y + z * z)))


def _R_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def parse_vec(raw: str, *, expected: int, name: str) -> np.ndarray:
    parts = [float(x.strip()) for x in raw.split(",") if x.strip()]
    if len(parts) != expected:
        raise argparse.ArgumentTypeError(
            f"{name} must contain {expected} comma-separated floats"
        )
    return np.asarray(parts, dtype=float)


def _ssl_context_for(url: str):
    if not url.startswith("wss://"):
        return None
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


class RelayClient:
    def __init__(self, url: str, connect_timeout_s: float = 5.0) -> None:
        self.url = url
        self.connect_timeout_s = connect_timeout_s
        self._lock = threading.Lock()
        self._latest_xr_frame: dict | None = None
        self._last_xr_frame_time = 0.0
        self._stop = threading.Event()
        self._connected = threading.Event()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ws = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._thread_main, name="arx5-vr-ws", daemon=True)
        self._thread.start()
        if not self._connected.wait(self.connect_timeout_s):
            raise RuntimeError(f"Timed out connecting to {self.url}. Start vr-teleop-relay first.")

    def stop(self) -> None:
        self._stop.set()
        if self._loop is not None and self._ws is not None:
            try:
                fut = asyncio.run_coroutine_threadsafe(self._ws.close(), self._loop)
                fut.result(timeout=1.0)
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def latest(self) -> tuple[dict | None, float]:
        with self._lock:
            return self._latest_xr_frame, self._last_xr_frame_time

    def send_json(self, payload: dict) -> None:
        if self._loop is None or self._ws is None:
            return
        text = json.dumps(payload)

        async def _send():
            try:
                await self._ws.send(text)
            except Exception:
                pass

        try:
            asyncio.run_coroutine_threadsafe(_send(), self._loop)
        except Exception:
            pass

    def _thread_main(self) -> None:
        asyncio.run(self._run())

    async def _run(self) -> None:
        self._loop = asyncio.get_running_loop()
        while not self._stop.is_set():
            try:
                async with websockets.connect(self.url, ssl=_ssl_context_for(self.url)) as ws:
                    self._ws = ws
                    self._connected.set()
                    while not self._stop.is_set():
                        try:
                            raw = await asyncio.wait_for(ws.recv(), timeout=0.1)
                        except asyncio.TimeoutError:
                            continue
                        try:
                            msg = json.loads(raw)
                        except Exception:
                            continue
                        if msg.get("type") == "xr_frame":
                            with self._lock:
                                self._latest_xr_frame = msg
                                self._last_xr_frame_time = time.time()
            except Exception as e:
                self._ws = None
                if not self._connected.is_set():
                    print(f"waiting for relay {self.url}: {e}")
                await asyncio.sleep(0.5)


def _button(ctrl: dict, index: int, field: str, default=0.0):
    buttons = ctrl.get("buttons") or []
    if len(buttons) <= index:
        return default
    return buttons[index].get(field, default)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws-url", default=DEFAULT_URL)
    ap.add_argument("--hand", choices=("left", "right"), default="right")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--freq", type=float, default=90.0)
    ap.add_argument("--qpos", default=None, help="initial six-joint qpos")
    ap.add_argument("--scale-translation", type=float, default=1.2)
    ap.add_argument("--scale-rotation", type=float, default=1.0)
    ap.add_argument("--pose-filter-alpha", type=float, default=0.8)
    ap.add_argument("--pos-reach", type=float, default=0.18)
    ap.add_argument("--rot-reach", type=float, default=0.5)
    ap.add_argument("--max-dq", type=float, default=0.08)
    ap.add_argument("--no-yaw-correction", action="store_true")
    args = ap.parse_args()

    qpos = (
        DEFAULT_X5_Q_REST.copy()
        if args.qpos is None else parse_vec(args.qpos, expected=6, name="--qpos")
    )
    solver = Arx5DLSIKSolver(
        args.urdf,
        q_rest=qpos,
        max_dq_per_joint=args.max_dq,
    )
    model, data = solver.model, solver.data
    tool_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tool0")
    j4_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "j4_anchor")
    mapper = ClutchPoseMapper(
        R=DEFAULT_R_CALIB.copy(),
        scale=args.scale_translation,
        scale_rotation=args.scale_rotation,
        pos_reach_limit=args.pos_reach,
        rot_reach_limit=args.rot_reach,
    )
    client = RelayClient(args.ws_url)
    client.start()

    pos_filt: np.ndarray | None = None
    quat_filt: np.ndarray | None = None
    last_grip = False
    last_reset = False
    last_rest = False
    engaged = False
    trigger_value = 0.0
    reset_active = False
    haptic = 0.0
    last_publish = 0.0
    last_status = 0.0
    period = 1.0 / max(args.freq, 1.0)
    next_tick = time.perf_counter()

    print("ARX5 VR Mujoco sim")
    print(f"  relay: {args.ws_url}")
    print(f"  hand: {args.hand}")
    print("  hold grip to clutch-control; A/X or thumbstick press resets to rest")
    print("  close the Mujoco viewer to exit")

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                xr, last_t = client.latest()
                stale = xr is None or (time.time() - last_t) > XR_FRAME_STALE_TIMEOUT_S
                if stale:
                    haptic *= 0.6
                else:
                    ctrls = xr.get("controllers") or {}
                    ctrl = ctrls.get(args.hand)
                    if ctrl is not None:
                        pos_raw = np.asarray(ctrl["position"], dtype=float)
                        ox, oy, oz, ow = ctrl["orientation"]
                        quat_raw = np.array([ow, ox, oy, oz], dtype=float)

                        alpha = float(args.pose_filter_alpha)
                        if pos_filt is None or quat_filt is None:
                            pos_filt = pos_raw.copy()
                            quat_filt = quat_raw.copy()
                        else:
                            pos_filt = (1.0 - alpha) * pos_filt + alpha * pos_raw
                            q_in = quat_raw if np.dot(quat_filt, quat_raw) >= 0.0 else -quat_raw
                            qf = (1.0 - alpha) * quat_filt + alpha * q_in
                            quat_filt = qf / np.linalg.norm(qf)

                        grip = bool(_button(ctrl, GRIP_BUTTON_INDEX, "p", False))
                        trigger_value = float(_button(ctrl, TRIGGER_BUTTON_INDEX, "v", 0.0))
                        reset = bool(_button(ctrl, RESET_BUTTON_INDEX, "p", False))
                        reset_active = reset
                        rest = bool(_button(ctrl, REST_RAMP_BUTTON_INDEX, "p", False))

                        if (reset and not last_reset) or (rest and not last_rest):
                            qpos = DEFAULT_X5_Q_REST.copy()
                            mapper.disengage()
                            engaged = False
                            last_grip = False
                            pos_filt = None
                            quat_filt = None
                        last_reset = reset
                        last_rest = rest

                        mapper.scale = args.scale_translation
                        mapper.scale_rotation = args.scale_rotation

                        if grip and not last_grip:
                            ee_pos, ee_quat = solver.fk(qpos)
                            j4_pos = solver.j4_anchor_xpos()
                            viewer_quat = (xr.get("viewer") or {}).get("orientation")
                            yaw_now = _yaw_from_quat_xyzw(viewer_quat)
                            if yaw_now is not None and not args.no_yaw_correction:
                                mapper.set_R(DEFAULT_R_CALIB @ _R_y(-yaw_now))
                            else:
                                mapper.set_R(DEFAULT_R_CALIB.copy())
                            mapper.engage(pos_filt, quat_filt, ee_pos, ee_quat, pivot_armbase=j4_pos)
                            engaged = True
                        elif not grip and last_grip:
                            mapper.disengage()
                            engaged = False
                            pos_filt = None
                            quat_filt = None
                        last_grip = grip

                        if engaged and pos_filt is not None and quat_filt is not None:
                            ee_pos, ee_quat = solver.fk(qpos)
                            target = mapper.target(pos_filt, quat_filt, ee_pos, ee_quat)
                            if target is not None:
                                qpos = solver.solve(target[0], target[1], qpos)
                                pressure = float(solver.last_limit_pressure)
                                pos_err = float(solver.last_pos_err_norm)
                                raw_haptic = max(
                                    min(1.0, max(0.0, (pressure - 0.03) / 0.15)),
                                    min(1.0, max(0.0, (pos_err - 0.03) / 0.12)),
                                )
                                haptic = 0.6 * haptic + 0.4 * raw_haptic
                        else:
                            haptic *= 0.6

                # Visual button feedback in the Mujoco scene:
                # - trigger fades tool0 from green to red (the X5 URDF has no
                #   actuated gripper joint to move yet).
                # - A/X reset button turns j4_anchor blue while pressed.
                if tool_site_id != -1:
                    model.site_rgba[tool_site_id] = [
                        max(0.05, trigger_value),
                        max(0.05, 1.0 - trigger_value),
                        0.15,
                        1.0,
                    ]
                if j4_site_id != -1:
                    model.site_rgba[j4_site_id] = (
                        [0.1, 0.35, 1.0, 1.0] if reset_active
                        else [1.0, 0.5, 0.0, 1.0]
                    )

                data.qpos[:6] = qpos
                mujoco.mj_forward(model, data)
                viewer.sync()

                now = time.perf_counter()
                if now - last_status > 0.5:
                    print(
                        f"\rhand={args.hand} grip={int(engaged)} "
                        f"trigger={trigger_value:.2f} reset={int(reset_active)} "
                        f"qpos={[round(float(v), 3) for v in qpos]}",
                        end="",
                        flush=True,
                    )
                    last_status = now
                if now - last_publish > 1.0 / 30.0:
                    selected_left = args.hand == "left"
                    payload = {
                        "type": "ik_state",
                        "left_qpos": [float(v) for v in qpos],
                        "right_qpos": [float(v) for v in qpos],
                        "qpos": [float(v) for v in qpos],
                        "left_engaged": bool(engaged and selected_left),
                        "right_engaged": bool(engaged and not selected_left),
                        "engaged": bool(engaged),
                        "left_haptic": float(haptic if selected_left else 0.0),
                        "right_haptic": float(haptic if not selected_left else 0.0),
                        "left_force_haptic": 0.0,
                        "right_force_haptic": 0.0,
                        "teleop_id": "arx5-mujoco-sim",
                        "server_time": time.time(),
                        "loop_hz": float(args.freq),
                    }
                    client.send_json(payload)
                    last_publish = now

                next_tick += period
                sleep_for = next_tick - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_tick = time.perf_counter()
    finally:
        client.stop()


if __name__ == "__main__":
    main()
