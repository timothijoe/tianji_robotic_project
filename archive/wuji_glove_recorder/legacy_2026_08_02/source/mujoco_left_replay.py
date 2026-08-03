"""Interactive, offline MuJoCo replay for a left Wuji Hand trajectory.

This module only loads local trajectory/model files. It has no ROS or hardware
device dependencies.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Lock

import numpy as np


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "mujoco-sim"
    / "wuji_hand_description"
    / "mjcf"
    / "left.xml"
)
SPEEDS = (0.25, 0.5, 1.0, 2.0, 4.0)


@dataclass(frozen=True)
class Trajectory:
    timestamps_ns: np.ndarray
    positions_rad: np.ndarray


def load_trajectory(path: Path) -> Trajectory:
    """Load one finite `(N, 20)` left-hand joint trajectory."""
    with np.load(path) as data:
        timestamps_ns = np.asarray(data["timestamps_ns"], dtype=np.int64)
        positions_rad = np.asarray(data["left_joint_positions_rad"], dtype=np.float64)
    if positions_rad.ndim != 2 or positions_rad.shape[0] < 1 or positions_rad.shape[1] != 20:
        raise ValueError("expected left_joint_positions_rad with shape (N, 20)")
    if timestamps_ns.shape != (positions_rad.shape[0],):
        raise ValueError("expected one timestamp per position frame")
    if not np.isfinite(positions_rad).all():
        raise ValueError("joint positions must be finite")
    return Trajectory(timestamps_ns=timestamps_ns, positions_rad=positions_rad)


def clamp_frame(index: int, frame_count: int) -> int:
    """Clamp a frame index to a non-empty trajectory."""
    if frame_count < 1:
        raise ValueError("frame_count must be positive")
    return min(max(index, 0), frame_count - 1)


@dataclass
class PlaybackState:
    frame_index: int = 0
    playing: bool = True
    speed_index: int = 2
    quit_requested: bool = False


def _joint_qpos_addresses(model, mujoco_module) -> list[int]:
    """Return qpos addresses in the model's 20 actuator order."""
    if model.nu != 20:
        raise ValueError(f"expected 20 MuJoCo actuators, got {model.nu}")
    addresses: list[int] = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            raise ValueError(f"actuator {actuator_id} is not attached to a joint")
        addresses.append(int(model.jnt_qposadr[joint_id]))
    return addresses


def _apply_frame(data, qpos_addresses: list[int], positions: np.ndarray, mujoco_module, model) -> None:
    for qpos_address, value in zip(qpos_addresses, positions):
        data.qpos[qpos_address] = value
    mujoco_module.mj_forward(model, data)


def _print_status(state: PlaybackState, trajectory: Trajectory) -> None:
    elapsed = (trajectory.timestamps_ns[state.frame_index] - trajectory.timestamps_ns[0]) / 1e9
    mode = "播放" if state.playing else "暂停"
    print(
        f"{mode} | 帧 {state.frame_index + 1}/{len(trajectory.positions_rad)} "
        f"| {elapsed:.2f} s | {SPEEDS[state.speed_index]:g}×"
    )


def run_viewer(trajectory: Trajectory) -> None:
    """Open a passive MuJoCo window and replay a trajectory without physics steps."""
    import mujoco
    import mujoco.viewer

    if not MODEL_PATH.is_file():
        raise FileNotFoundError(f"MuJoCo model not found: {MODEL_PATH}")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    qpos_addresses = _joint_qpos_addresses(model, mujoco)
    state = PlaybackState()
    lock = Lock()

    def key_callback(keycode: int) -> None:
        with lock:
            if keycode == 32:  # Space
                state.playing = not state.playing
            elif keycode == 263 and not state.playing:  # Left
                state.frame_index = clamp_frame(state.frame_index - 1, len(trajectory.positions_rad))
            elif keycode == 262 and not state.playing:  # Right
                state.frame_index = clamp_frame(state.frame_index + 1, len(trajectory.positions_rad))
            elif keycode in (82, 114):  # R/r
                state.frame_index = 0
                state.playing = False
            elif keycode in (61, 43):  # =/+ 
                state.speed_index = min(state.speed_index + 1, len(SPEEDS) - 1)
            elif keycode == 45:  # -
                state.speed_index = max(state.speed_index - 1, 0)
            elif keycode == 256:  # Escape
                state.quit_requested = True
            else:
                return
            _print_status(state, trajectory)

    _apply_frame(data, qpos_addresses, trajectory.positions_rad[0], mujoco, model)
    _print_status(state, trajectory)
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.azimuth = 180
        viewer.cam.elevation = -20
        viewer.cam.distance = 0.5
        viewer.cam.lookat[:] = [0, 0, 0.05]
        last_advance = time.monotonic()
        while viewer.is_running():
            with lock:
                if state.quit_requested:
                    break
                frame_index = state.frame_index
                playing = state.playing
                speed = SPEEDS[state.speed_index]
            if playing:
                next_index = (frame_index + 1) % len(trajectory.positions_rad)
                if next_index == 0:
                    interval = 1.0 / 120.0
                else:
                    interval = max(
                        (trajectory.timestamps_ns[next_index] - trajectory.timestamps_ns[frame_index]) / 1e9,
                        1e-4,
                    )
                if time.monotonic() - last_advance >= interval / speed:
                    with lock:
                        state.frame_index = next_index
                        frame_index = state.frame_index
                    last_advance = time.monotonic()
            _apply_frame(data, qpos_addresses, trajectory.positions_rad[frame_index], mujoco, model)
            viewer.sync()
            time.sleep(0.002)


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay a left Wuji Hand NPZ in MuJoCo.")
    parser.add_argument("trajectory", type=Path)
    args = parser.parse_args()
    trajectory = load_trajectory(args.trajectory)
    print(f"Loaded {trajectory.positions_rad.shape[0]} frames from {args.trajectory}")
    print("Controls: Space play/pause, ←/→ step when paused, R reset, +/- speed, Esc exit")
    run_viewer(trajectory)


if __name__ == "__main__":
    main()
