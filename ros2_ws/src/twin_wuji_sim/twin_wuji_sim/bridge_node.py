from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from wujihand_msgs.msg import HandDiagnostics
from wujihand_msgs.srv import SetEnabled

from twin_sim.ros2_bridge import (
    LEFT_HAND_JOINT_NAMES,
    control_period,
    decode_joint_command,
    enabled_indices,
)
from twin_sim.wuji_hand_backend import SimWujiHand


class WujiSimBridge(Node):
    def __init__(self) -> None:
        super().__init__("wuji_sim_bridge")
        self.declare_parameter("viewer", False)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("start_enabled", True)

        self._hand: SimWujiHand | None = None
        try:
            self._hand = SimWujiHand(
                viewer=bool(self.get_parameter("viewer").value)
            )
            rate = float(self.get_parameter("publish_rate").value)
            self._period = control_period(
                rate, float(self._hand.robot.sim.model.opt.timestep)
            )
            self._hand.write_joint_enabled(
                bool(self.get_parameter("start_enabled").value)
            )
            self._state_pub = self.create_publisher(
                JointState, "joint_states", qos_profile_sensor_data
            )
            self._diagnostics_pub = self.create_publisher(
                HandDiagnostics, "hand_diagnostics", 10
            )
            self.create_subscription(
                JointState,
                "joint_commands",
                self._command_callback,
                qos_profile_sensor_data,
            )
            self.create_service(SetEnabled, "set_enabled", self._set_enabled)
            self.create_timer(self._period, self._tick)
        except Exception:
            if self._hand is not None:
                self._hand.close()
            super().destroy_node()
            raise

    def destroy_node(self) -> bool:
        if self._hand is not None:
            self._hand.close()
        return super().destroy_node()

    def _command_callback(self, message: JointState) -> None:
        try:
            current = self._hand.read_joint_target_position().reshape(20)
            target = decode_joint_command(message.name, message.position, current)
            self._hand.write_joint_target_position(target.reshape(5, 4))
        except (ValueError, RuntimeError) as error:
            self.get_logger().warning(f"rejected joint command: {error}")

    def _set_enabled(
        self, request: SetEnabled.Request, response: SetEnabled.Response
    ) -> SetEnabled.Response:
        try:
            indices = enabled_indices(request.finger_id, request.joint_id)
        except ValueError as error:
            response.success = False
            response.message = str(error)
            return response
        enabled = self._hand.read_joint_enabled()
        enabled.reshape(20)[list(indices)] = request.enabled
        self._hand.write_joint_enabled(enabled)
        response.success = True
        response.message = "simulation enable state updated"
        return response

    def _tick(self) -> None:
        self._hand.step(self._period)
        stamp = self.get_clock().now().to_msg()
        state = JointState()
        state.header.stamp = stamp
        state.name = list(LEFT_HAND_JOINT_NAMES)
        state.position = self._hand.read_joint_actual_position().reshape(20).tolist()
        state.velocity = (
            self._hand.robot.sim.data.qvel[
                self._hand.robot.sim.hand.dof_ids
            ].tolist()
        )
        state.effort = (
            self._hand.robot.sim.data.actuator_force[
                self._hand.robot.sim.hand.actuator_ids
            ].tolist()
        )
        self._state_pub.publish(state)

        diagnostics = HandDiagnostics()
        diagnostics.header.stamp = stamp
        diagnostics.handedness = "left"
        diagnostics.system_temperature = 0.0
        diagnostics.input_voltage = 0.0
        diagnostics.joint_temperatures = [0.0] * 20
        diagnostics.error_codes = [0] * 20
        diagnostics.enabled = self._hand.read_joint_enabled().reshape(20).tolist()
        diagnostics.effort_limits = [0.0] * 20
        self._diagnostics_pub.publish(diagnostics)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: WujiSimBridge | None = None
    try:
        node = WujiSimBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
