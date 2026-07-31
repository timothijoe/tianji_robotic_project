"""End-to-end ROS 2 smoke test for the headless MuJoCo hand bridge."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from wujihand_msgs.srv import SetEnabled

from twin_sim.ros2_bridge import LEFT_HAND_JOINT_NAMES


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("wuji_sim_smoke_probe")
        self.states: list[JointState] = []
        self.create_subscription(
            JointState,
            "/hand_left/joint_states",
            self.states.append,
            qos_profile_sensor_data,
        )
        self.commands = self.create_publisher(
            JointState,
            "/hand_left/joint_commands",
            qos_profile_sensor_data,
        )
        self.enabled = self.create_client(
            SetEnabled, "/hand_left/set_enabled"
        )


def wait_for(node: Node, predicate, timeout_s: float = 8.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError("timed out waiting for ROS 2 condition")


def call_enabled(probe: Probe, enabled: bool) -> None:
    assert probe.enabled.wait_for_service(timeout_sec=3.0)
    request = SetEnabled.Request()
    request.finger_id = 255
    request.joint_id = 255
    request.enabled = enabled
    future = probe.enabled.call_async(request)
    wait_for(probe, future.done)
    assert future.result().success


def publish_joint(probe: Probe, index: int, value: float) -> None:
    message = JointState()
    message.name = [LEFT_HAND_JOINT_NAMES[index]]
    message.position = [value]
    probe.commands.publish(message)


def main() -> None:
    repo = Path(__file__).resolve().parents[2]
    bridge = (
        repo
        / "ros2_ws/install/twin_wuji_sim/lib/twin_wuji_sim/bridge"
    )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = (
        f"{repo / 'src'}:{environment.get('PYTHONPATH', '')}"
    )
    process: subprocess.Popen | None = None
    probe: Probe | None = None
    ros_initialized = False
    try:
        process = subprocess.Popen(
            [str(bridge), "--ros-args", "-r", "__ns:=/hand_left"],
            env=environment,
        )
        rclpy.init()
        ros_initialized = True
        probe = Probe()
        wait_for(probe, lambda: len(probe.states) >= 2)
        state = probe.states[-1]
        assert tuple(state.name) == LEFT_HAND_JOINT_NAMES
        assert len(state.position) == 20
        assert np.isfinite(state.position).all()

        index = 1
        before = float(state.position[index])
        target = before + 0.12
        publish_joint(probe, index, target)
        wait_for(
            probe,
            lambda: probe.states[-1].position[index] > before + 0.03,
        )
        wait_for(
            probe,
            lambda: abs(probe.states[-1].position[index] - target) < 0.02,
        )

        call_enabled(probe, False)
        settled = float(probe.states[-1].position[index])
        publish_joint(probe, index, target + 0.12)
        start_count = len(probe.states)
        wait_for(probe, lambda: len(probe.states) >= start_count + 30)
        assert abs(probe.states[-1].position[index] - target) < 0.025
        assert abs(probe.states[-1].position[index] - (target + 0.12)) > 0.08

        call_enabled(probe, True)
        publish_joint(probe, index, target + 0.08)
        wait_for(
            probe,
            lambda: probe.states[-1].position[index] > settled + 0.03,
        )
        print("ROS2_WUJI_SIM_SMOKE_OK")
    finally:
        if probe is not None:
            probe.destroy_node()
        if ros_initialized and rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5.0)
        assert process is None or process.poll() is not None


if __name__ == "__main__":
    main()
