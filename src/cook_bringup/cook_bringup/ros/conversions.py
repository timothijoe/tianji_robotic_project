from __future__ import annotations

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cook_core.interfaces import (
    JointStateData,
    JointTrajectoryData,
    TrajectoryPointData,
)


def joint_state_data_from_msg(message: JointState) -> JointStateData:
    stamp = message.header.stamp
    timestamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return JointStateData(
        names=tuple(message.name),
        positions=tuple(message.position),
        velocities=tuple(message.velocity) if message.velocity else None,
        efforts=tuple(message.effort) if message.effort else None,
        timestamp_sec=timestamp_sec,
    )


def joint_state_data_to_msg(data: JointStateData, *, node=None) -> JointState:
    message = JointState()
    if node is not None:
        message.header.stamp = node.get_clock().now().to_msg()
    elif data.timestamp_sec is not None:
        sec = int(data.timestamp_sec)
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = int((data.timestamp_sec - sec) * 1e9)
    message.name = list(data.names)
    message.position = list(data.positions)
    message.velocity = [] if data.velocities is None else list(data.velocities)
    message.effort = [] if data.efforts is None else list(data.efforts)
    return message


def trajectory_data_from_msg(message: JointTrajectory) -> JointTrajectoryData:
    joint_names = tuple(message.joint_names)
    return JointTrajectoryData(
        joint_names=joint_names,
        points=tuple(_point_from_msg(point, joint_names) for point in message.points),
        trajectory_id=message.header.frame_id,
        frame_id=message.header.frame_id,
        source="ros",
    )


def trajectory_data_to_msg(data: JointTrajectoryData, *, node=None) -> JointTrajectory:
    message = JointTrajectory()
    if node is not None:
        message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = data.frame_id or data.trajectory_id
    message.joint_names = list(data.joint_names)
    message.points = [_point_to_msg(point, data.joint_names) for point in data.points]
    return message


def _point_from_msg(
    message: JointTrajectoryPoint,
    joint_names: tuple[str, ...],
) -> TrajectoryPointData:
    return TrajectoryPointData(
        positions=dict(zip(joint_names, message.positions)),
        velocities=(
            None if not message.velocities else dict(zip(joint_names, message.velocities))
        ),
        accelerations=(
            None
            if not message.accelerations
            else dict(zip(joint_names, message.accelerations))
        ),
        time_from_start_sec=duration_to_sec(message.time_from_start),
    )


def _point_to_msg(
    data: TrajectoryPointData,
    joint_names: tuple[str, ...],
) -> JointTrajectoryPoint:
    message = JointTrajectoryPoint()
    message.positions = list(data.ordered_positions(joint_names))
    if data.velocities is not None:
        message.velocities = [
            float(data.velocities.get(name, 0.0)) for name in joint_names
        ]
    if data.accelerations is not None:
        message.accelerations = [
            float(data.accelerations.get(name, 0.0)) for name in joint_names
        ]
    message.time_from_start = sec_to_duration(data.time_from_start_sec)
    return message


def duration_to_sec(duration: Duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def sec_to_duration(value: float) -> Duration:
    sec = int(value)
    nanosec = int(round((float(value) - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)
