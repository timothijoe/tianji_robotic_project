"""Deterministic tabletop placement and retreat correction for Wuji trajectories."""

from dataclasses import dataclass

import mujoco
import numpy as np

from tianji_robotics.wuji_hand.models import HandTrajectory


@dataclass(frozen=True)
class TableRetreatConfig:
    retreat_distance_m: float = 0.03
    place_duration_s: float = 1.0
    retreat_duration_s: float = 2.0
    hold_duration_s: float = 2.0
    fingertip_height_m: float = 0.0015
    thumb_clearance_m: float = 0.010
    candidate_stride: int = 10
    source_frame: int | None = None


@dataclass(frozen=True)
class CorrectedHandTrajectory:
    timestamps_ns: np.ndarray
    positions_rad: np.ndarray
    palm_positions_m: np.ndarray
    palm_quaternions_wxyz: np.ndarray
    phases: tuple[str, ...]
    source_frame: int


@dataclass(frozen=True)
class CorrectionReport:
    source_frame: int
    source_timestamp_ns: int
    candidate_score: float
    retreat_vector_m: tuple[float, float, float]
    actual_retreat_m: float
    minimum_thumb_clearance_m: float
    maximum_fingertip_height_error_m: float
    maximum_contact_slip_m: float
    maximum_hand_penetration_m: float
    maximum_retreat_fingertip_lift_m: float


def build_table_retreat(trajectory: HandTrajectory, backend, config: TableRetreatConfig = TableRetreatConfig()):
    _validate_config(config)
    source_frame, score = _select_frame(trajectory, backend, config)
    selected = trajectory.positions_rad[source_frame].copy()
    ranges = np.asarray(list(backend.joint_ranges_rad.values()))
    place_seed = selected.copy()
    place_seed[4:] = np.clip(0.0, ranges[4:, 0], ranges[4:, 1])
    quaternion = _fit_palm_down_quaternion(backend, place_seed)
    place_joints, place_palm = _solve_place(
        backend, place_seed, quaternion, config
    )
    backend.set_kinematic_pose(place_joints, place_palm, quaternion)
    contact_targets = backend.fingertip_positions_m().copy()
    if backend.thumb_position_m()[2] - backend.table_height_m < config.thumb_clearance_m:
        raise ValueError("selected pose cannot keep the thumb clear of the table")

    dt = backend.timestep_s
    place_count = _steps(config.place_duration_s, dt)
    retreat_count = _steps(config.retreat_duration_s, dt)
    hold_count = _steps(config.hold_duration_s, dt)
    joints_frames=[]; palm_frames=[]; phases=[]
    hover_palm = place_palm + np.array([0.0, 0.0, 0.05])
    for fraction in np.linspace(0.0, 1.0, place_count + 1):
        joints_frames.append(place_seed + fraction * (place_joints - place_seed))
        palm_frames.append(hover_palm + fraction * (place_palm - hover_palm))
        phases.append("PLACE")
    retreat_vector = np.array([config.retreat_distance_m, 0.0, 0.0])
    curl_target = place_joints.copy()
    ranges = np.asarray(list(backend.joint_ranges_rad.values()))
    for base in (4, 8, 12, 16):
        curl_target[base] += 0.35
        curl_target[base + 2] += 0.35
        curl_target[base + 3] += 0.25
    curl_target = np.clip(curl_target, ranges[:, 0], ranges[:, 1])
    previous = place_joints
    for fraction in np.linspace(0.0, 1.0, retreat_count + 1)[1:]:
        smooth = fraction * fraction * (3.0 - 2.0 * fraction)
        solved = place_joints + smooth * (curl_target - place_joints)
        previous = previous + np.clip(solved - previous, -0.1, 0.1)
        palm = place_palm + smooth * retreat_vector
        palm = _project_above_table(backend, previous, palm, quaternion)
        joints_frames.append(previous.copy()); palm_frames.append(palm); phases.append("RETREAT")
    final_palm = palm_frames[-1]
    for _ in range(hold_count):
        joints_frames.append(previous.copy()); palm_frames.append(final_palm.copy()); phases.append("HOLD")
    joints = np.asarray(joints_frames); palms=np.asarray(palm_frames)
    quaternions=np.tile(quaternion,(len(joints),1))
    diagnostics = _preflight(backend, joints, palms, quaternions, phases, contact_targets, config)
    timestamps = np.arange(len(joints), dtype=np.int64) * int(round(dt * 1e9))
    corrected=CorrectedHandTrajectory(timestamps,joints,palms,quaternions,tuple(phases),source_frame)
    report=CorrectionReport(source_frame,int(trajectory.timestamps_ns[source_frame]),score,tuple(retreat_vector),float(np.linalg.norm(retreat_vector)),*diagnostics)
    return corrected, report


def replay_table_retreat(corrected: CorrectedHandTrajectory, backend) -> None:
    for joints,palm,quaternion in zip(corrected.positions_rad,corrected.palm_positions_m,corrected.palm_quaternions_wxyz,strict=True):
        backend.command_pose(joints,palm,quaternion)
        backend.step(backend.timestep_s)


def _select_frame(trajectory, backend, config):
    candidates = [config.source_frame] if config.source_frame is not None else list(range(0, len(trajectory.positions_rad), config.candidate_stride))
    if candidates[-1] != len(trajectory.positions_rad)-1 and config.source_frame is None: candidates.append(len(trajectory.positions_rad)-1)
    best=None
    for index in candidates:
        if index is None or not 0 <= index < len(trajectory.positions_rad): raise ValueError("source frame is outside trajectory")
        backend.set_kinematic_pose(trajectory.positions_rad[index], np.zeros(3), [1,0,0,0])
        tips=backend.fingertip_positions_m(); thumb=backend.thumb_position_m()
        relative_thumb=float(thumb[2]-tips[:,2].mean()+config.fingertip_height_m)
        if relative_thumb < config.thumb_clearance_m: continue
        score=float(np.ptp(tips[:,2]) + 0.1*np.std(tips[:,:2]))
        candidate=(score,index)
        if best is None or candidate < best: best=candidate
    if best is None: raise ValueError("no feasible source frame keeps the thumb clear of the table")
    return best[1],best[0]


def _solve_contacts(backend, initial, palm, quaternion, targets, *, vertical_only):
    q=np.asarray(initial,dtype=float).copy(); active=np.arange(4,20)
    ranges=np.asarray(list(backend.joint_ranges_rad.values()))
    for _ in range(40):
        backend.set_kinematic_pose(q,palm,quaternion)
        current=backend.fingertip_positions_m(); error=(targets-current)
        if vertical_only:
            residual=error[:,2]; jacobian=backend.fingertip_position_jacobian()[2::3,active]
        else:
            residual=error.reshape(-1); jacobian=backend.fingertip_position_jacobian()[:,active]
        if np.max(np.abs(residual)) < 0.001: break
        lhs=jacobian.T@jacobian + 1e-4*np.eye(len(active))
        step=np.linalg.solve(lhs,jacobian.T@residual)
        q[active]+=np.clip(step,-0.04,0.04)
        q=np.clip(q,ranges[:,0],ranges[:,1])
    return q


def _fit_palm_down_quaternion(backend, joints) -> np.ndarray:
    backend.set_kinematic_pose(joints, np.array([0.0, 0.0, 0.2]), np.array([1.0, 0.0, 0.0, 0.0]))
    tips = backend.fingertip_positions_m()
    roots = backend.long_finger_root_positions_m()
    forward = tips.mean(axis=0) - roots.mean(axis=0)
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm <= 1e-8:
        raise ValueError("cannot fit palm frame from coincident finger landmarks")
    forward /= forward_norm
    lateral = roots[-1] - roots[0]
    lateral -= forward * float(np.dot(lateral, forward))
    lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm <= 1e-8:
        raise ValueError("cannot fit palm frame from collinear finger landmarks")
    lateral /= lateral_norm
    normal = np.cross(forward, lateral)
    local_basis = np.column_stack((forward, lateral, normal))
    # Fingers extend toward world -X, span toward +Y, and the palm-facing
    # normal points toward -Z so the knuckles remain above the table.
    world_basis = np.column_stack(([-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]))
    rotation = world_basis @ local_basis.T
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, rotation.reshape(-1))
    return quaternion


def _calibrated_palm_down_quaternion(backend, joints) -> np.ndarray:
    backend.set_kinematic_pose(joints, np.array([0.0, 0.0, 0.2]), np.array([1.0, 0.0, 0.0, 0.0]))
    tips = backend.fingertip_positions_m()
    roots = backend.long_finger_root_positions_m()
    forward = tips.mean(axis=0) - roots.mean(axis=0)
    forward /= np.linalg.norm(forward)
    lateral = roots[-1] - roots[0]
    lateral -= forward * float(np.dot(lateral, forward))
    lateral /= np.linalg.norm(lateral)
    normal = np.cross(forward, lateral)
    local_basis = np.column_stack((forward, lateral, normal))
    # The official left-hand palmar reference lies toward local -Z. Mapping
    # anatomical +normal to world +Z therefore places the palmar side down.
    world_basis = np.column_stack(([-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]))
    rotation = world_basis @ local_basis.T
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, rotation.reshape(-1))
    backend.set_kinematic_pose(joints, np.array([0.0, 0.0, 0.2]), quaternion)
    if backend.palmar_reference_position_m()[2] >= backend.dorsal_reference_position_m()[2]:
        raise ValueError("official Wuji palm-side calibration is inverted")
    return quaternion


def _solve_place(backend, initial, quaternion, config):
    q = np.asarray(initial, dtype=float).copy()
    backend.set_kinematic_pose(q, np.zeros(3), quaternion)
    tips = backend.fingertip_positions_m()
    provisional_height = 0.003
    palm = np.array([0.0, 0.0, provisional_height - float(tips[:, 2].mean())])
    provisional_targets = tips.copy()
    provisional_targets[:, 2] = backend.table_height_m + provisional_height
    q = _solve_contacts(backend, q, palm, quaternion, provisional_targets, vertical_only=True)
    backend.set_kinematic_pose(q, palm, quaternion)
    distances = backend.long_fingertip_table_distances_m()
    if not np.isfinite(distances).all():
        raise ValueError("selected pose cannot calibrate all four fingertip surfaces")
    surface_offsets = backend.fingertip_positions_m()[:, 2] - distances
    targets = backend.fingertip_positions_m().copy()
    targets[:, 2] = surface_offsets + 0.0002
    for _ in range(30):
        q = _solve_contacts(backend, q, palm, quaternion, targets, vertical_only=True)
        backend.set_kinematic_pose(q, palm, quaternion)
        penetration = backend.maximum_table_penetration_m()
        if penetration <= 0.0005:
            if backend.thumb_position_m()[2] - backend.table_height_m < config.thumb_clearance_m:
                raise ValueError("selected pose cannot keep the thumb clear of the table")
            return q, palm
        palm[2] += penetration + 0.0001
    raise ValueError("selected pose cannot place the whole hand above the table")


def _project_above_table(backend, joints, palm, quaternion):
    corrected = np.asarray(palm, dtype=float).copy()
    for _ in range(4):
        backend.set_kinematic_pose(joints, corrected, quaternion)
        penetration = backend.maximum_table_penetration_m()
        if penetration <= 0.0005:
            return corrected
        corrected[2] += penetration - 0.0005 + 0.0001
    raise ValueError("retreat frame cannot be projected above the table")


def _preflight(backend,joints,palms,quaternions,phases,targets,config):
    if not np.isfinite(joints).all() or np.max(np.abs(np.diff(joints,axis=0))) > 0.12: raise ValueError("corrected trajectory exceeds joint-step limit")
    min_thumb=float("inf"); max_height=0.0; max_slip=0.0; max_penetration=0.0; max_lift=0.0
    place_end=phases.index("RETREAT") if "RETREAT" in phases else len(phases)
    place_tip_heights = None
    for i,(q,palm,quat) in enumerate(zip(joints,palms,quaternions,strict=True)):
        backend.set_kinematic_pose(q,palm,quat)
        min_thumb=min(min_thumb,float(backend.thumb_position_m()[2]-backend.table_height_m))
        max_penetration=max(max_penetration,backend.maximum_table_penetration_m())
        if i == place_end - 1:
            place_tip_heights=backend.fingertip_positions_m()[:,2].copy()
        if i >= place_end:
            tips=backend.fingertip_positions_m(); max_height=max(max_height,float(np.max(np.abs(tips[:,2]-(backend.table_height_m+config.fingertip_height_m))))); max_slip=max(max_slip,float(np.max(np.linalg.norm(tips[:,:2]-targets[:,:2],axis=1))))
            if place_tip_heights is not None:
                max_lift=max(max_lift,float(np.max(tips[:,2]-place_tip_heights)))
    if min_thumb < config.thumb_clearance_m or max_penetration > 0.0005: raise ValueError(f"contact correction failed preflight: thumb={min_thumb}, penetration={max_penetration}")
    return min_thumb,max_height,max_slip,max_penetration,max_lift


def _steps(duration,dt):
    return max(1,int(round(duration/dt)))


def _validate_config(config):
    values=(config.retreat_distance_m,config.place_duration_s,config.retreat_duration_s,config.hold_duration_s,config.fingertip_height_m,config.thumb_clearance_m)
    if not np.isfinite(values).all() or any(value<=0 for value in values) or config.candidate_stride<1: raise ValueError("table-retreat configuration must be positive and finite")
