"""Joint target replay models for MuJoCo-free low-level control debugging."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Callable, Literal

import numpy as np


ReplayModel = Literal["ideal", "lagged", "noisy", "lagged-noisy"]


@dataclass(frozen=True)
class JointReplayConfig:
    model: ReplayModel = "ideal"
    dt_s: float = 0.004
    tracking_alpha: float = 1.0
    joint_noise_std: float = 0.0
    command_delay_steps: int = 0
    joint_velocity_limit: float | None = None
    random_seed: int | None = None

    def __post_init__(self) -> None:
        if self.model not in ("ideal", "lagged", "noisy", "lagged-noisy"):
            raise ValueError("model must be ideal, lagged, noisy, or lagged-noisy")
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError("dt_s must be positive finite")
        if not math.isfinite(self.tracking_alpha) or not 0.0 < self.tracking_alpha <= 1.0:
            raise ValueError("tracking_alpha must be in (0, 1]")
        if not math.isfinite(self.joint_noise_std) or self.joint_noise_std < 0.0:
            raise ValueError("joint_noise_std must be non-negative finite")
        if int(self.command_delay_steps) < 0:
            raise ValueError("command_delay_steps must be non-negative")
        if self.joint_velocity_limit is not None:
            if not math.isfinite(self.joint_velocity_limit) or self.joint_velocity_limit <= 0.0:
                raise ValueError("joint_velocity_limit must be positive finite")


@dataclass(frozen=True)
class JointReplaySample:
    step_index: int
    time_s: float
    commanded_joints: np.ndarray
    delayed_target_joints: np.ndarray
    actual_joints: np.ndarray
    joint_velocities: np.ndarray
    tcp_pose: np.ndarray | None = None
    tcp_velocity: np.ndarray | None = None


class JointReplayExecutor:
    """Replay joint targets with optional lag, noise, delay, and velocity caps."""

    def __init__(
        self,
        initial_joints: np.ndarray,
        config: JointReplayConfig | None = None,
        *,
        fk: Callable[[np.ndarray], np.ndarray] | None = None,
        jacobian: Callable[[np.ndarray], np.ndarray] | None = None,
    ) -> None:
        self.config = config or JointReplayConfig()
        self._actual = np.asarray(initial_joints, dtype=float).reshape(-1).copy()
        if self._actual.size == 0 or not np.all(np.isfinite(self._actual)):
            raise ValueError("initial_joints must be a non-empty finite vector")
        self._previous = self._actual.copy()
        self._step_index = 0
        self._rng = np.random.default_rng(self.config.random_seed)
        self._fk = fk
        self._jacobian = jacobian
        self._delay = deque(
            [self._actual.copy() for _ in range(int(self.config.command_delay_steps))],
            maxlen=max(1, int(self.config.command_delay_steps) + 1),
        )

    @property
    def actual_joints(self) -> np.ndarray:
        return self._actual.copy()

    def step(self, commanded_joints: np.ndarray) -> JointReplaySample:
        command = np.asarray(commanded_joints, dtype=float).reshape(self._actual.shape)
        if not np.all(np.isfinite(command)):
            raise ValueError("commanded_joints must be finite")

        delayed = self._delayed_target(command)
        desired = self._response_target(delayed)
        desired = self._apply_noise(desired)
        actual = self._apply_velocity_limit(desired)
        qd = (actual - self._actual) / self.config.dt_s

        self._previous = self._actual
        self._actual = actual
        tcp_pose = self._fk(self._actual).copy() if self._fk is not None else None
        tcp_velocity = None
        if self._jacobian is not None:
            tcp_velocity = np.asarray(self._jacobian(self._actual), dtype=float).reshape(6, -1) @ qd

        sample = JointReplaySample(
            step_index=self._step_index,
            time_s=float((self._step_index + 1) * self.config.dt_s),
            commanded_joints=command.copy(),
            delayed_target_joints=delayed.copy(),
            actual_joints=self._actual.copy(),
            joint_velocities=qd.copy(),
            tcp_pose=tcp_pose,
            tcp_velocity=tcp_velocity.copy() if tcp_velocity is not None else None,
        )
        self._step_index += 1
        return sample

    def _delayed_target(self, command: np.ndarray) -> np.ndarray:
        if int(self.config.command_delay_steps) == 0:
            return command.copy()
        self._delay.append(command.copy())
        return self._delay.popleft().copy()

    def _response_target(self, delayed: np.ndarray) -> np.ndarray:
        if self.config.model in ("lagged", "lagged-noisy"):
            return self._actual + self.config.tracking_alpha * (delayed - self._actual)
        return delayed.copy()

    def _apply_noise(self, desired: np.ndarray) -> np.ndarray:
        if self.config.model not in ("noisy", "lagged-noisy") or self.config.joint_noise_std == 0.0:
            return desired
        return desired + self._rng.normal(0.0, self.config.joint_noise_std, size=desired.shape)

    def _apply_velocity_limit(self, desired: np.ndarray) -> np.ndarray:
        limit = self.config.joint_velocity_limit
        if limit is None:
            return desired.copy()
        max_delta = float(limit) * self.config.dt_s
        return self._actual + np.clip(desired - self._actual, -max_delta, max_delta)
