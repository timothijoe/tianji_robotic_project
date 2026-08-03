"""Deterministic tabletop placement and retreat correction for Wuji trajectories."""

from dataclasses import dataclass

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


def build_table_retreat(trajectory: HandTrajectory, backend, config: TableRetreatConfig = TableRetreatConfig()):
    _validate_config(config)
    source_frame, score = _select_frame(trajectory, backend, config)
    selected = trajectory.positions_rad[source_frame].copy()
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    origin = np.zeros(3)
    backend.set_kinematic_pose(selected, origin, quaternion)
    source_tips = backend.fingertip_positions_m()
    place_palm = np.array([0.0, 0.0, config.fingertip_height_m - float(source_tips[:, 2].mean())])
    place_targets = source_tips.copy()
    place_targets[:, 2] = backend.table_height_m + config.fingertip_height_m
    place_joints = _solve_contacts(backend, selected, place_palm, quaternion, place_targets, vertical_only=True)
    backend.set_kinematic_pose(place_joints, place_palm, quaternion)
    contact_targets = backend.fingertip_positions_m().copy()
    if backend.thumb_position_m()[2] - backend.table_height_m < config.thumb_clearance_m:
        raise ValueError("selected pose cannot keep the thumb clear of the table")

    dt = backend.timestep_s
    place_count = _steps(config.place_duration_s, dt)
    retreat_count = _steps(config.retreat_duration_s, dt)
    hold_count = _steps(config.hold_duration_s, dt)
    joints_frames=[]; palm_frames=[]; phases=[]
    for fraction in np.linspace(0.0, 1.0, place_count + 1):
        joints_frames.append(selected + fraction * (place_joints - selected))
        palm_frames.append(fraction * place_palm)
        phases.append("PLACE")
    retreat_vector = np.array([config.retreat_distance_m, 0.0, 0.0])
    previous = place_joints
    for fraction in np.linspace(0.0, 1.0, retreat_count + 1)[1:]:
        palm = place_palm + fraction * retreat_vector
        previous = _solve_contacts(backend, previous, palm, quaternion, contact_targets, vertical_only=False)
        joints_frames.append(previous.copy()); palm_frames.append(palm); phases.append("RETREAT")
    for _ in range(hold_count):
        joints_frames.append(previous.copy()); palm_frames.append(place_palm + retreat_vector); phases.append("HOLD")
    joints = np.asarray(joints_frames); palms=np.asarray(palm_frames)
    quaternions=np.tile(quaternion,(len(joints),1))
    diagnostics = _preflight(backend, joints, palms, quaternions, phases, contact_targets, config)
    timestamps = np.arange(len(joints), dtype=np.int64) * int(round(dt * 1e9))
    corrected=CorrectedHandTrajectory(timestamps,joints,palms,quaternions,tuple(phases),source_frame)
    report=CorrectionReport(source_frame,int(trajectory.timestamps_ns[source_frame]),score,tuple(retreat_vector),float(np.linalg.norm(palms[-1]-place_palm)),*diagnostics)
    return corrected, report


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


def _preflight(backend,joints,palms,quaternions,phases,targets,config):
    if not np.isfinite(joints).all() or np.max(np.abs(np.diff(joints,axis=0))) > 0.12: raise ValueError("corrected trajectory exceeds joint-step limit")
    min_thumb=float("inf"); max_height=0.0; max_slip=0.0
    place_end=phases.index("RETREAT") if "RETREAT" in phases else len(phases)
    for i,(q,palm,quat) in enumerate(zip(joints,palms,quaternions,strict=True)):
        backend.set_kinematic_pose(q,palm,quat)
        min_thumb=min(min_thumb,float(backend.thumb_position_m()[2]-backend.table_height_m))
        if i >= place_end:
            tips=backend.fingertip_positions_m(); max_height=max(max_height,float(np.max(np.abs(tips[:,2]-(backend.table_height_m+config.fingertip_height_m))))); max_slip=max(max_slip,float(np.max(np.linalg.norm(tips[:,:2]-targets[:,:2],axis=1))))
    if min_thumb < config.thumb_clearance_m or max_height > 0.002 or max_slip > 0.003: raise ValueError(f"contact correction failed preflight: thumb={min_thumb}, height={max_height}, slip={max_slip}")
    return min_thumb,max_height,max_slip


def _steps(duration,dt):
    return max(1,int(round(duration/dt)))


def _validate_config(config):
    values=(config.retreat_distance_m,config.place_duration_s,config.retreat_duration_s,config.hold_duration_s,config.fingertip_height_m,config.thumb_clearance_m)
    if not np.isfinite(values).all() or any(value<=0 for value in values) or config.candidate_stride<1: raise ValueError("table-retreat configuration must be positive and finite")
