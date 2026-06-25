from __future__ import annotations

import signal
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from cook_bringup.ros.conversions import (
    joint_state_data_from_msg,
    trajectory_data_to_msg,
)
from cook_bringup.ros.paths import default_urdf_path
from cook_bringup.ros.shutdown import (
    ignore_shutdown_signals,
    safe_destroy_node,
    safe_shutdown,
)
from cook_core.teaching import TeachPendantModel
from cook_description.models import load_robot_definition


class TeachPendantNode(Node):
    def __init__(self, *, context=None) -> None:
        super().__init__("teach_pendant_node", context=context)
        self.declare_parameter("urdf_path", str(default_urdf_path()))
        self.declare_parameter(
            "trajectory_topic",
            "/mujoco_controller_node/joint_trajectory",
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("duration_sec", 1.0)

        definition = load_robot_definition(
            self.get_parameter("urdf_path").get_parameter_value().string_value
        )
        self.model = TeachPendantModel(
            joint_names=definition.movable_joint_names,
            joint_limits=definition.joint_limits,
        )
        self.publisher = self.create_publisher(
            JointTrajectory,
            self.get_parameter("trajectory_topic").get_parameter_value().string_value,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").get_parameter_value().string_value,
            self._joint_state_callback,
            10,
        )
        self._latest_joint_state: dict[str, float] = self.model.current_positions()

    def publish_targets(self, target_positions: dict[str, float], duration_sec: float) -> None:
        trajectory = self.model.build_trajectory(
            target_positions=target_positions,
            duration_sec=duration_sec,
        )
        self.publisher.publish(trajectory_data_to_msg(trajectory, node=self))

    def latest_joint_state(self) -> dict[str, float]:
        return dict(self._latest_joint_state)

    def _joint_state_callback(self, message: JointState) -> None:
        state = joint_state_data_from_msg(message)
        self.model.update_current_state(state)
        self._latest_joint_state = self.model.current_positions()


class TeachPendantWindow:
    def __init__(self, node: TeachPendantNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Joint Teach Pendant")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._closed = False
        self._variables: dict[str, tk.DoubleVar] = {}
        self._duration = tk.DoubleVar(
            value=float(node.get_parameter("duration_sec").value)
        )
        self._build()
        self._sync_from_state()

    @property
    def closed(self) -> bool:
        return self._closed

    def run(self) -> None:
        self._tick()
        self.root.mainloop()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.root.quit()
            self.root.destroy()
        except (KeyboardInterrupt, tk.TclError):
            pass

    def request_close(self) -> None:
        if self._closed:
            return
        try:
            self.root.after_idle(self.close)
        except (KeyboardInterrupt, tk.TclError):
            self._closed = True
            pass

    def _build(self) -> None:
        container = ttk.Frame(self.root, padding=12)
        container.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        for row, name in enumerate(self.node.model.joint_names):
            lower, upper = self.node.model.joint_limits.get(name, (-3.14, 3.14))
            lower = -3.14 if lower is None else float(lower)
            upper = 3.14 if upper is None else float(upper)
            variable = tk.DoubleVar(value=0.0)
            self._variables[name] = variable
            ttk.Label(container, text=name, width=16).grid(row=row, column=0, sticky="w")
            ttk.Scale(
                container,
                variable=variable,
                from_=lower,
                to=upper,
                orient="horizontal",
                length=360,
            ).grid(row=row, column=1, sticky="ew", padx=8)
            ttk.Label(container, textvariable=variable, width=10).grid(
                row=row,
                column=2,
                sticky="e",
            )

        control_row = len(self._variables)
        ttk.Label(container, text="duration").grid(row=control_row, column=0, sticky="w")
        ttk.Spinbox(
            container,
            from_=0.1,
            to=30.0,
            increment=0.1,
            textvariable=self._duration,
            width=10,
        ).grid(row=control_row, column=1, sticky="w", padx=8)
        ttk.Button(container, text="Capture State", command=self._sync_from_state).grid(
            row=control_row + 1,
            column=0,
            pady=(10, 0),
            sticky="ew",
        )
        ttk.Button(container, text="Zero", command=self._zero).grid(
            row=control_row + 1,
            column=1,
            pady=(10, 0),
            sticky="ew",
        )
        ttk.Button(container, text="Send Trajectory", command=self._publish).grid(
            row=control_row + 1,
            column=2,
            pady=(10, 0),
            sticky="ew",
        )
        container.columnconfigure(1, weight=1)

    def _sync_from_state(self) -> None:
        state = self.node.latest_joint_state()
        for name, variable in self._variables.items():
            variable.set(float(state.get(name, 0.0)))

    def _zero(self) -> None:
        for variable in self._variables.values():
            variable.set(0.0)

    def _publish(self) -> None:
        self.node.publish_targets(
            {name: variable.get() for name, variable in self._variables.items()},
            duration_sec=float(self._duration.get()),
        )

    def _tick(self) -> None:
        if self._closed:
            return
        if not rclpy.ok():
            self.close()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.root.after(20, self._tick)
        except (KeyboardInterrupt, ExternalShutdownException):
            self.close()
        except tk.TclError:
            self._closed = True


def main(argv: list[str] | None = None) -> int:
    node = None
    window = None
    rclpy.init(args=argv)
    try:
        node = TeachPendantNode()
        window = TeachPendantWindow(node)
        def _handle_shutdown_signal(signum, frame):
            ignore_shutdown_signals()
            if window is not None:
                window.request_close()
            raise KeyboardInterrupt

        signal.signal(signal.SIGINT, _handle_shutdown_signal)
        signal.signal(signal.SIGTERM, _handle_shutdown_signal)
        window.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        ignore_shutdown_signals()
        if window is not None:
            window.close()
    finally:
        if window is not None:
            window.close()
        safe_destroy_node(node)
        safe_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
