from collections import deque
from typing import Sequence

import numpy as np

from twin_sim.viewer_trail import _init_sphere_geom, _position


class GuardedChopTrace:
    planned_knife_color = np.asarray((0.1, 0.35, 1.0, 0.9))
    actual_knife_color = np.asarray((0.0, 0.9, 1.0, 0.9))
    actual_guard_color = np.asarray((0.75, 0.1, 0.9, 0.95))
    guard_target_color = np.asarray((1.0, 0.85, 0.1, 0.95))

    def __init__(
        self,
        viewer=None,
        *,
        max_points: int = 3000,
        marker_stride: int = 10,
    ) -> None:
        for name, value in (
            ("max_points", max_points),
            ("marker_stride", marker_stride),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, np.integer))
                or value <= 0
            ):
                raise ValueError(f"{name} must be a positive integer")
        self._viewer = viewer
        self._marker_stride = int(marker_stride)
        self._sample_index = 0
        self.planned_knife = deque(maxlen=int(max_points))
        self.actual_knife = deque(maxlen=int(max_points))
        self.actual_guard = deque(maxlen=int(max_points))
        self.guard_target = deque(maxlen=int(max_points))
        self.phase = ""
        self.cut_index = 0
        self.minimum_distance_m = float("inf")
        self.cut_allowed = False
        self.abort_reason = ""

    def append(
        self,
        *,
        planned_knife: Sequence[float],
        actual_knife: Sequence[float],
        actual_guard: Sequence[float],
        guard_target: Sequence[float],
        phase: str,
        cut_index: int,
        minimum_distance_m: float,
        cut_allowed: bool,
    ) -> None:
        points = tuple(
            _position(np.asarray(value, dtype=float))
            for value in (
                planned_knife,
                actual_knife,
                actual_guard,
                guard_target,
            )
        )
        self.planned_knife.append(points[0])
        self.actual_knife.append(points[1])
        self.actual_guard.append(points[2])
        self.guard_target.append(points[3])
        self.phase = str(phase)
        self.cut_index = int(cut_index)
        self.minimum_distance_m = min(
            self.minimum_distance_m, float(minimum_distance_m)
        )
        self.cut_allowed = bool(cut_allowed)
        if self._sample_index % self._marker_stride == 0:
            self._update_overlay()
            self._draw(
                (
                    (points[0], self.planned_knife_color, 0.006),
                    (points[1], self.actual_knife_color, 0.0045),
                    (points[2], self.actual_guard_color, 0.005),
                    (points[3], self.guard_target_color, 0.004),
                )
            )
        self._sample_index += 1

    def set_abort(self, reason: str) -> None:
        self.phase = "aborted"
        self.cut_allowed = False
        self.abort_reason = str(reason)
        self._update_overlay()

    def _update_overlay(self) -> None:
        viewer = self._viewer
        if viewer is None or not hasattr(viewer, "set_texts"):
            return
        status = (
            f"cut={self.cut_index}/5  phase={self.phase}  "
            f"distance={self.minimum_distance_m:.3f} m  "
            f"cut_allowed={self.cut_allowed}"
        )
        if self.abort_reason:
            status += f"  abort={self.abort_reason}"
        viewer.set_texts((None, None, "Guarded chop", status))

    def _draw(self, markers) -> None:
        viewer = self._viewer
        if viewer is None or getattr(viewer, "user_scn", None) is None:
            return
        with viewer.lock():
            scene = viewer.user_scn
            for position, color, size_m in markers:
                if scene.ngeom >= scene.maxgeom:
                    return
                _init_sphere_geom(
                    scene.geoms[scene.ngeom], position, size_m, color
                )
                scene.ngeom += 1
