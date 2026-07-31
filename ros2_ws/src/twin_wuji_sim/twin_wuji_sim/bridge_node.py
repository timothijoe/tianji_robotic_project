from __future__ import annotations

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from wujihand_msgs.msg import HandDiagnostics
from wujihand_msgs.srv import SetEnabled

from twin_sim.ros2_bridge import LEFT_HAND_JOINT_NAMES, decode_joint_command
from twin_sim.wuji_hand_backend import SimWujiHand


class WujiSimBridge(Node):
    def __init__(self) -> None:
        super().__init__("wuji_sim_bridge")
        self.declare_parameter("viewer", False)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("start_enabled", True)

        rate = float(self.get_parameter("publish_rate").value)
        if not np.isfinite(rate) or rate <= 0.0:
            raise ValueError("publish_rate must be positive and finite")
        self._period = 1.0 / rate
        self._hand = SimWujiHand(
            viewer=bool(self.get_parameter("viewer").value)
        )
        self._hand.write_joint_enabled(
            bool(self.get_parameter("start_enabled").value)
        )
        self._state_pub = self.create_publisher(JointState, "joint_states", 10)
        self._diagnostics_pub = self.create_publisher(
            HandDiagnostics, "hand_diagnostics", 10
        )
        self.create_subscription(
            JointState, "joint_commands", self._command_callback, 10
        )
        self.create_service(SetEnabled, "set_enabled", self._set_enabled)
        self.create_timer(self._period, self._tick)

    def destroy_node(self) -> bool:
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
        if request.finger_id != 255 and request.finger_id > 4:
            response.success = False
            response.message = "finger_id must be 0-4 or 255"
            return response
        if request.joint_id != 255 and request.joint_id > 3:
            response.success = False
            response.message = "joint_id must be 0-3 or 255"
            return response
        enabled = self._hand.read_joint_enabled()
        fingers = range(5) if request.finger_id == 255 else (request.finger_id,)
        joints = range(4) if request.joint_id == 255 else (request.joint_id,)
        for finger in fingers:
            for joint in joints:
                enabled[finger, joint] = request.enabled
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
    node = WujiSimBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
