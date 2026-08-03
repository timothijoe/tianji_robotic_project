"""Wuji Hand backend implemented only by the existing MuJoCo simulator."""

import numpy as np

from tianji_robotics.wuji_hand.names import HAND_JOINT_NAMES
from twin_sim.robot import RightArmRobot
from twin_sim.wuji_hand_backend import SimWujiHand


class MujocoWujiHand:
    """Expose flat 20-joint domain vectors over ``SimWujiHand``."""

    def __init__(
        self, robot: RightArmRobot | None = None, *, viewer: bool = False
    ) -> None:
        self._hand = SimWujiHand(robot=robot, viewer=viewer)
        self.robot = self._hand.robot
        self.timestep_s = float(self.robot.sim.model.opt.timestep)
        self.realtime_paced = bool(viewer)
        self.range_tolerance_rad = 0.001

    @property
    def joint_ranges_rad(self) -> dict[str, tuple[float, float]]:
        ranges = self.robot.sim.model.actuator_ctrlrange[
            self.robot.sim.hand.actuator_ids
        ]
        return {
            name: (float(bounds[0]), float(bounds[1]))
            for name, bounds in zip(HAND_JOINT_NAMES, ranges, strict=True)
        }

    def read_position_rad(self) -> np.ndarray:
        return self._hand.read_joint_actual_position().reshape(20).copy()

    def read_target_position_rad(self) -> np.ndarray:
        return self._hand.read_joint_target_position().reshape(20).copy()

    def command_position_rad(self, target: np.ndarray) -> None:
        values = np.asarray(target, dtype=float)
        if values.shape != (20,) or not np.isfinite(values).all():
            raise ValueError("Wuji hand target must contain 20 finite radians")
        self._hand.write_joint_target_position(values.reshape(5, 4))

    def step(self, duration_s: float) -> None:
        self._hand.step(duration_s)

    def close(self) -> None:
        self._hand.close()
