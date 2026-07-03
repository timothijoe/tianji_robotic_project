"""SDK-style backend selector for MuJoCo and the real Marvin SDK.

The goal of this module is to let demos use the same high-level calls as the
vendor ``DEMO_PYTHON`` examples while still being runnable in MuJoCo:

    robot = create_robot("mujoco", arm="B")
    robot.connect("mujoco")
    robot.set_position_state("B", 30, 30)
    robot.set_joint_position_cmd("B", joints_deg)
    robot.wait(2.0)
    robot.release_robot()

For real hardware, ``create_robot("real", sdk_root=...)`` returns a thin adapter
around the vendor ``Concise_Marvin_Robot`` object.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Any, Literal, Sequence

import numpy as np

from twin_control.controller import ControlMode
from twin_control.robot import (
    _DEFAULT_CART_D,
    _DEFAULT_CART_K,
    _DEFAULT_JOINT_D,
    _DEFAULT_JOINT_K,
    TwinRobot,
)
from twin_control.sdk_kine import MujocoKine
from twin_control.sdk_kine import FX_InvKineSolvePara as MujocoInvKineSolvePara


BackendName = Literal["mujoco", "real"]


class MujocoSdkRobot:
    """Subset of the vendor ``Concise_Marvin_Robot`` API backed by MuJoCo.

    Public joint values follow the SDK convention: degrees for joint position
    and degrees/s for joint velocity.  Arm ``"A"`` maps to the left MuJoCo arm;
    arm ``"B"`` maps to the right MuJoCo arm.
    """

    def __init__(
        self,
        *,
        arm: str = "B",
        viewer: bool = False,
        realtime: bool = True,
        model_path: str | Path | None = None,
        control_hz: float = 500.0,
        tcp_site_name: str | None = None,
    ) -> None:
        self.default_arm = _normalise_arm(arm)
        self.viewer = bool(viewer)
        self.realtime = bool(realtime)
        self.model_path = model_path
        self.control_hz = float(control_hz)
        self.tcp_site_name = tcp_site_name
        self.robot: TwinRobot | None = None
        self._vel_ratio = 0
        self._acc_ratio = 0
        self._imp_type = 0
        self._joint_cmd_pos = [0.0] * 7
        self._force_cmd = 0.0
        self._frame_serial = 0
        self._traj_state = b"\x00"

    def connect(self, robot_ip: str = "mujoco", log_switch: int = 0) -> bool:
        """Connect to the MuJoCo backend.

        ``robot_ip`` and ``log_switch`` are accepted for source compatibility
        with the vendor SDK; they are not used by the simulator.
        """
        del robot_ip, log_switch
        arm_name = _arm_name(self.default_arm)
        self.robot = TwinRobot(
            arm_name=arm_name,
            unit_mode="sdk",
            control_hz=self.control_hz,
            tcp_site_name=self.tcp_site_name,
        )
        self.robot.connect(
            model_path=self.model_path,
            viewer=self.viewer,
            realtime=self.realtime,
        )
        return True

    def release_robot(self) -> bool:
        if self.robot is not None:
            self.robot.close()
            self.robot = None
        return True

    def set_position_state(self, arm: str, velRatio: int, AccRatio: int) -> bool:
        self._check_arm(arm)
        self._remember_motion_params(velRatio, AccRatio, imp_type=0)
        self._require_robot().set_position_state(
            vel_ratio=_ratio(velRatio),
            acc_ratio=_ratio(AccRatio),
        )
        return True

    def set_imp_joint_state(
        self,
        arm: str,
        velRatio: int,
        AccRatio: int,
        K: Sequence[float] = _DEFAULT_JOINT_K,
        D: Sequence[float] = _DEFAULT_JOINT_D,
    ) -> bool:
        self._check_arm(arm)
        self._remember_motion_params(velRatio, AccRatio, imp_type=1)
        self._require_robot().set_joint_impedance_state(
            _ratio(velRatio),
            _ratio(AccRatio),
            K,
            D,
        )
        return True

    def set_imp_cart_state(
        self,
        arm: str,
        velRatio: int,
        AccRatio: int,
        K: Sequence[float] = _DEFAULT_CART_K,
        D: Sequence[float] = _DEFAULT_CART_D,
        rot_type: int = 0,
        cart_ctrl_para: Sequence[float] | None = None,
    ) -> bool:
        self._check_arm(arm)
        self._remember_motion_params(velRatio, AccRatio, imp_type=2)
        self._require_robot().set_cart_impedance_state(
            _ratio(velRatio),
            _ratio(AccRatio),
            K,
            D,
            rot_type=rot_type,
            cart_ctrl_para=cart_ctrl_para,
        )
        return True

    def set_imp_force_state(
        self,
        arm: str,
        fx_dir: Sequence[float],
        fc_adj_lmt: float,
        *,
        velRatio: int = 50,
        AccRatio: int = 50,
        K: Sequence[float] = _DEFAULT_CART_K,
        D: Sequence[float] = _DEFAULT_CART_D,
    ) -> bool:
        self._check_arm(arm)
        self._remember_motion_params(velRatio, AccRatio, imp_type=3)
        self._require_robot().set_force_state(
            _ratio(velRatio),
            _ratio(AccRatio),
            K,
            D,
            fx_dir,
            fc_adj_lmt,
        )
        return True

    def set_joint_position_cmd(
        self,
        arm: str,
        joint: Sequence[float],
        velocity: Sequence[float] | None = None,
    ) -> bool:
        self._check_arm(arm)
        robot = self._require_robot()
        q_cmd = np.asarray(joint, dtype=float).reshape(7)
        qd_cmd = None if velocity is None else np.asarray(velocity, dtype=float).reshape(7)
        self._joint_cmd_pos = q_cmd.tolist()
        if robot.state.name in ("CARTESIAN_IMPEDANCE", "FORCE"):
            pose = robot._kinematics.fk(q_cmd)[0]
            robot._controller.set_cart_cmd(pose)
            if robot._controller._mode == ControlMode.FORCE:
                robot._controller.set_joint_cmd(
                    np.deg2rad(q_cmd),
                    None if qd_cmd is None else np.deg2rad(qd_cmd),
                )
        else:
            robot.set_joint_position_cmd(q_cmd, qd_cmd)
        return True

    def set_force_cmd(self, arm: str, force: float) -> bool:
        self._check_arm(arm)
        self._force_cmd = float(force)
        self._require_robot().set_force_cmd(force)
        return True

    def disable(self, arm: str) -> bool:
        self._check_arm(arm)
        self._require_robot().disable()
        return True

    def wait(self, seconds: float, *, viewer_sync: bool = True) -> None:
        """Advance the simulation for ``seconds``.

        The real backend provides the same helper but sleeps wall-clock time.
        """
        robot = self._require_robot()
        steps = max(1, int(round(float(seconds) * robot.control_hz)))
        robot.spin(steps, viewer_sync=viewer_sync)
        self._frame_serial = (self._frame_serial + steps) % 1_000_000

    def step(self, *, viewer_sync: bool = True) -> None:
        self._require_robot().step(viewer_sync=viewer_sync)
        self._frame_serial = (self._frame_serial + 1) % 1_000_000

    def subscribe(self, dcss: Any = None) -> dict[str, Any]:
        """Return a vendor-shaped status dictionary for the selected arm."""
        del dcss
        robot = self._require_robot()
        q = robot.get_joint_positions().tolist()
        qd = robot.get_joint_velocities().tolist()
        wrench = robot.get_raw_wrench().tolist()
        torque = (
            robot._controller._previous_torque.tolist()
            if robot._controller is not None else [0.0] * 7
        )
        low_speed = b"\x01" if float(np.linalg.norm(qd, ord=np.inf)) < 0.5 else b"\x00"
        side_index = 0 if self.default_arm == "A" else 1
        states = [
            {"cur_state": 0, "cmd_state": 0, "err_code": 0},
            {"cur_state": 0, "cmd_state": 0, "err_code": 0},
        ]
        outputs = [
            _empty_output(),
            _empty_output(),
        ]
        states[side_index] = {
            "cur_state": _state_code(robot.control_mode),
            "cmd_state": _state_code(robot.control_mode),
            "err_code": 0,
        }
        outputs[side_index].update(
            {
                "frame_serial": self._frame_serial,
                "fb_joint_pos": q,
                "fb_joint_vel": qd,
                "fb_joint_cmd": list(self._joint_cmd_pos),
                "fb_joint_sToq": torque,
                "est_cart_fn": wrench,
                "low_speed_flag": low_speed,
                "traj_state": self._traj_state,
                "control_mode": robot.control_mode,
            }
        )
        inputs = [
            _empty_input(),
            _empty_input(),
        ]
        inputs[side_index].update(
            {
                "joint_vel_ratio": self._vel_ratio,
                "joint_acc_ratio": self._acc_ratio,
                "joint_cmd_pos": list(self._joint_cmd_pos),
                "imp_type": self._imp_type,
                "force_cmd": self._force_cmd,
            }
        )
        return {
            "states": states,
            "outputs": outputs,
            "inputs": inputs,
        }

    def get_joint_positions(self) -> np.ndarray:
        return self._require_robot().get_joint_positions()

    def get_joint_velocities(self) -> np.ndarray:
        return self._require_robot().get_joint_velocities()

    def get_tcp_pose(self) -> tuple[np.ndarray, np.ndarray]:
        return self._require_robot().get_tcp_pose()

    def _check_arm(self, arm: str) -> None:
        if _normalise_arm(arm) != self.default_arm:
            raise NotImplementedError(
                "MujocoSdkRobot controls one selected arm per instance; "
                f"this instance is for arm {self.default_arm}."
            )

    def _require_robot(self) -> TwinRobot:
        if self.robot is None:
            raise RuntimeError("robot is not connected; call connect() first")
        return self.robot

    def _remember_motion_params(self, vel_ratio: int, acc_ratio: int, imp_type: int) -> None:
        self._vel_ratio = int(np.clip(vel_ratio, 0, 100))
        self._acc_ratio = int(np.clip(acc_ratio, 0, 100))
        self._imp_type = int(imp_type)


class RealSdkRobotAdapter:
    """Thin wrapper that makes the real SDK easier to call like MuJoCo backend."""

    def __init__(self, robot: Any, dcss_factory: Any) -> None:
        self.robot = robot
        self._dcss_factory = dcss_factory
        self._dcss = dcss_factory()

    def __getattr__(self, name: str) -> Any:
        return getattr(self.robot, name)

    def connect(self, robot_ip: str, log_switch: int = 0) -> bool:
        return bool(self.robot.connect(robot_ip, log_switch=log_switch))

    def release_robot(self) -> Any:
        return self.robot.release_robot()

    def subscribe(self, dcss: Any = None) -> dict[str, Any] | None:
        return self.robot.subscribe(dcss if dcss is not None else self._dcss)

    def wait(self, seconds: float, *, viewer_sync: bool = True) -> None:
        del viewer_sync
        time.sleep(float(seconds))

    def step(self, *, viewer_sync: bool = True) -> None:
        del viewer_sync
        time.sleep(0.0)


def create_robot(
    backend: BackendName = "mujoco",
    *,
    arm: str = "B",
    viewer: bool = False,
    realtime: bool = True,
    model_path: str | Path | None = None,
    control_hz: float = 500.0,
    tcp_site_name: str | None = None,
    sdk_root: str | Path | None = None,
) -> Any:
    """Create a MuJoCo or real-hardware SDK-style robot object."""
    if backend == "mujoco":
        return MujocoSdkRobot(
            arm=arm,
            viewer=viewer,
            realtime=realtime,
            model_path=model_path,
            control_hz=control_hz,
            tcp_site_name=tcp_site_name,
        )
    if backend == "real":
        return _create_real_robot(sdk_root)
    raise ValueError(f"backend must be 'mujoco' or 'real', got {backend!r}")


def create_kine(
    backend: BackendName = "mujoco",
    *,
    arm_type: int = 1,
    model_path: str | Path | None = None,
    tcp_site_name: str | None = None,
    sdk_root: str | Path | None = None,
) -> Any:
    """Create a MuJoCo or real-hardware SDK-style kinematics object."""
    if backend == "mujoco":
        return MujocoKine(
            arm_type=arm_type,
            model_path=model_path,
            tcp_site_name=tcp_site_name,
        )
    if backend == "real":
        return _create_real_kine(sdk_root)
    raise ValueError(f"backend must be 'mujoco' or 'real', got {backend!r}")


def create_ik_param(
    backend: BackendName = "mujoco",
    *,
    sdk_root: str | Path | None = None,
) -> Any:
    """Create the backend-specific IK parameter/result structure."""
    if backend == "mujoco":
        return MujocoInvKineSolvePara()
    if backend == "real":
        return _create_real_ik_param(sdk_root)
    raise ValueError(f"backend must be 'mujoco' or 'real', got {backend!r}")


def _create_real_robot(sdk_root: str | Path | None) -> Any:
    root = Path(
        sdk_root
        or os.environ.get("TJ_FX_ROBOT_CONTRL_SDK", "")
        or "/home/zhoutong/docker_share/docker_mapping/cook_proj/TJ_FX_ROBOT_CONTRL_SDK"
    )
    if not root.exists():
        raise FileNotFoundError(
            "real backend requires the vendor SDK root; pass sdk_root or set "
            "TJ_FX_ROBOT_CONTRL_SDK"
        )
    sys.path.insert(0, str(root))
    from SDK_PYTHON.fx_robot import DCSS, Concise_Marvin_Robot

    robot = Concise_Marvin_Robot()
    return RealSdkRobotAdapter(robot, DCSS)


def _create_real_kine(sdk_root: str | Path | None) -> Any:
    root = Path(
        sdk_root
        or os.environ.get("TJ_FX_ROBOT_CONTRL_SDK", "")
        or "/home/zhoutong/docker_share/docker_mapping/cook_proj/TJ_FX_ROBOT_CONTRL_SDK"
    )
    if not root.exists():
        raise FileNotFoundError(
            "real backend requires the vendor SDK root; pass sdk_root or set "
            "TJ_FX_ROBOT_CONTRL_SDK"
        )
    sys.path.insert(0, str(root))
    from SDK_PYTHON.fx_kine import Marvin_Kine

    return Marvin_Kine()


def _create_real_ik_param(sdk_root: str | Path | None) -> Any:
    root = Path(
        sdk_root
        or os.environ.get("TJ_FX_ROBOT_CONTRL_SDK", "")
        or "/home/zhoutong/docker_share/docker_mapping/cook_proj/TJ_FX_ROBOT_CONTRL_SDK"
    )
    if not root.exists():
        raise FileNotFoundError(
            "real backend requires the vendor SDK root; pass sdk_root or set "
            "TJ_FX_ROBOT_CONTRL_SDK"
        )
    sys.path.insert(0, str(root))
    from SDK_PYTHON.fx_kine import FX_InvKineSolvePara

    return FX_InvKineSolvePara()


def _normalise_arm(arm: str) -> str:
    arm = str(arm).upper()
    if arm not in ("A", "B"):
        raise ValueError(f"arm must be 'A' or 'B', got {arm!r}")
    return arm


def _arm_name(arm: str) -> str:
    return "left" if _normalise_arm(arm) == "A" else "right"


def _ratio(value: float) -> float:
    return float(np.clip(float(value) / 100.0, 0.0, 1.0))


def _state_code(control_mode: str) -> int:
    if control_mode == "IDLE":
        return 0
    if control_mode == "POSITION":
        return 1
    return 3


def _empty_output() -> dict[str, Any]:
    return {
        "frame_serial": 0,
        "fb_joint_pos": [0.0] * 7,
        "fb_joint_vel": [0.0] * 7,
        "fb_joint_cmd": [0.0] * 7,
        "fb_joint_sToq": [0.0] * 7,
        "est_cart_fn": [0.0] * 6,
        "low_speed_flag": b"\x01",
        "traj_state": b"\x00",
        "control_mode": "IDLE",
    }


def _empty_input() -> dict[str, Any]:
    return {
        "joint_vel_ratio": 0,
        "joint_acc_ratio": 0,
        "joint_cmd_pos": [0.0] * 7,
        "imp_type": 0,
        "force_cmd": 0.0,
    }
