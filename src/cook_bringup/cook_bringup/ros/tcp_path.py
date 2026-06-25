from __future__ import annotations

import math
from typing import Sequence

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

from cook_core.interfaces import JointTrajectoryData
from cook_core.trajectory import sample_trajectory_positions
from cook_mujoco.control.runtime import BodyPose


def compose_tcp_pose(
    parent_pose: BodyPose,
    *,
    offset_xyz: Sequence[float],
    offset_rpy: Sequence[float],
) -> BodyPose:
    parent_quaternion = _normalize_quaternion(parent_pose.quaternion)
    offset = _vector3(offset_xyz, "offset_xyz")
    offset_quaternion = _quaternion_from_rpy(*_vector3(offset_rpy, "offset_rpy"))
    world_offset = _rotate_vector(parent_quaternion, offset)
    return BodyPose(
        position=(
            float(parent_pose.position[0]) + world_offset[0],
            float(parent_pose.position[1]) + world_offset[1],
            float(parent_pose.position[2]) + world_offset[2],
        ),
        quaternion=_normalize_quaternion(
            _multiply_quaternions(parent_quaternion, offset_quaternion)
        ),
    )


def body_pose_to_pose_stamped(
    pose: BodyPose,
    *,
    frame_id: str,
    stamp=None,
) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = str(frame_id)
    if stamp is not None:
        message.header.stamp = stamp
    message.pose.position.x = float(pose.position[0])
    message.pose.position.y = float(pose.position[1])
    message.pose.position.z = float(pose.position[2])
    message.pose.orientation.w = float(pose.quaternion[0])
    message.pose.orientation.x = float(pose.quaternion[1])
    message.pose.orientation.y = float(pose.quaternion[2])
    message.pose.orientation.z = float(pose.quaternion[3])
    return message


def body_pose_to_transform_stamped(
    pose: BodyPose,
    *,
    parent_frame_id: str,
    child_frame_id: str,
    stamp=None,
) -> TransformStamped:
    message = TransformStamped()
    message.header.frame_id = str(parent_frame_id)
    if stamp is not None:
        message.header.stamp = stamp
    message.child_frame_id = str(child_frame_id)
    message.transform.translation.x = float(pose.position[0])
    message.transform.translation.y = float(pose.position[1])
    message.transform.translation.z = float(pose.position[2])
    message.transform.rotation.w = float(pose.quaternion[0])
    message.transform.rotation.x = float(pose.quaternion[1])
    message.transform.rotation.y = float(pose.quaternion[2])
    message.transform.rotation.z = float(pose.quaternion[3])
    return message


def body_poses_to_path(
    poses: Sequence[BodyPose],
    *,
    frame_id: str,
    stamp=None,
) -> Path:
    message = Path()
    message.header.frame_id = str(frame_id)
    if stamp is not None:
        message.header.stamp = stamp
    message.poses = [
        body_pose_to_pose_stamped(pose, frame_id=frame_id, stamp=stamp)
        for pose in poses
    ]
    return message


def append_body_pose_to_path(
    path: Path,
    pose: BodyPose,
    *,
    frame_id: str,
    stamp=None,
    max_points: int,
) -> None:
    path.header.frame_id = str(frame_id)
    if stamp is not None:
        path.header.stamp = stamp
    path.poses.append(body_pose_to_pose_stamped(pose, frame_id=frame_id, stamp=stamp))
    limit = max(1, int(max_points))
    if len(path.poses) > limit:
        del path.poses[: len(path.poses) - limit]


def build_predicted_tcp_path(
    *,
    runtime,
    trajectory: JointTrajectoryData,
    parent_body_name: str,
    frame_id: str,
    offset_xyz: Sequence[float],
    offset_rpy: Sequence[float],
    sample_count: int,
    stamp=None,
) -> Path:
    poses = []
    for elapsed_sec in _sample_times(trajectory, sample_count):
        positions = sample_trajectory_positions(trajectory, elapsed_sec)
        runtime.set_joint_positions(
            [positions[name] for name in trajectory.joint_names],
            forward=True,
        )
        poses.append(
            compose_tcp_pose(
                runtime.get_body_pose(parent_body_name),
                offset_xyz=offset_xyz,
                offset_rpy=offset_rpy,
            )
        )
    return body_poses_to_path(poses, frame_id=frame_id, stamp=stamp)


def _sample_times(
    trajectory: JointTrajectoryData,
    sample_count: int,
) -> tuple[float, ...]:
    if trajectory.is_empty:
        return ()
    end_time = max(0.0, trajectory.points[-1].time_from_start_sec)
    if end_time <= 0.0:
        return (0.0,)
    count = max(2, int(sample_count))
    step = end_time / float(count - 1)
    return tuple(step * index for index in range(count))


def _vector3(values: Sequence[float], label: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{label} must contain exactly 3 values")
    return (float(values[0]), float(values[1]), float(values[2]))


def _quaternion_from_rpy(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return _normalize_quaternion(
        (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    )


def _multiply_quaternions(
    lhs: Sequence[float],
    rhs: Sequence[float],
) -> tuple[float, float, float, float]:
    w1, x1, y1, z1 = lhs
    w2, x2, y2, z2 = rhs
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _rotate_vector(
    quaternion: Sequence[float],
    vector: Sequence[float],
) -> tuple[float, float, float]:
    q = _normalize_quaternion(quaternion)
    v = (0.0, float(vector[0]), float(vector[1]), float(vector[2]))
    rotated = _multiply_quaternions(
        _multiply_quaternions(q, v),
        (q[0], -q[1], -q[2], -q[3]),
    )
    return (rotated[1], rotated[2], rotated[3])


def _normalize_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    if len(quaternion) != 4:
        raise ValueError("quaternion must contain exactly 4 values")
    values = tuple(float(value) for value in quaternion)
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        raise ValueError("quaternion norm must be positive")
    return tuple(value / norm for value in values)
