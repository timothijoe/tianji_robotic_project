# Twin Control: Unified Kinematics & Control for Marvin CCS Robot in MuJoCo

**Date**: 2026-06-28
**Project**: tianji_robotic_project
**Model**: Marvin CCS dual-arm (7-DOF per arm), MJCF source `MarvinCCS/marvin_final_fixed.xml`

## 1. Goal

Implement, in pure Python, the kinematics and torque-mode control stack from the
TJ_FX_ROBOT_CONTRL_SDK (version 1003/1004) inside MuJoCo, then validate it
with four trajectory scenarios.

- **Kinematics**: MDH forward kinematics, damped-least-squares inverse kinematics,
  analytical Jacobian.
- **Control**: Joint impedance, Cartesian impedance, and force control — all behind
  a single SDK-aligned `TwinRobot` interface.
- **End-effector**: Force sensor site (flange + 95 mm along Z, the
  `right_force_sensor_site` / `left_force_sensor_site` position).

The implementation must be verifiable entirely in simulation (no hardware).

## 2. Architecture

```
src/twin_control/
  __init__.py
  kinematics.py       # MDH FK / IK / Jacobian, tool offset, unit_mode
  controller.py       # UnifiedController: joint / cartesian impedance, force control
  robot.py            # TwinRobot: SDK-style interface, MuJoCo glue layer
examples/
  trajectory_demo.py  # 4 trajectory scenarios
tests/
  test_kinematics.py
  test_controller.py
```

**Layering:**

```
kinematics.py   (numpy only)
       ↑
controller.py  (numpy + kinematics)
       ↑
robot.py       (numpy + mujoco + twin_mujoco.runtime + kinematics + controller)
```

- `kinematics.py` and `controller.py` have **no MuJoCo dependency**.
- `robot.py` is the **only** file that touches MuJoCo runtime (via `ArmView`).
- If file sizes become unwieldy, `controller.py` may split into submodules
  (e.g., `joint_imp.py`, `cart_imp.py`, `force_control.py`).

## 3. Kinematics Design (`kinematics.py`)

### 3.1 MDH Parameters

Source: `MarvinCCS/marvin_final_fixed.xml` and `ccs_m6_40.MvKDCfg`. CCS
configuration, 7 joints + 1 flange-to-sensor transform.

| Row | Joint/Sensor | α (rad) | a (m) | d (m) | θ-offset (rad) |
|-----|-------------|---------|-------|-------|-----------------|
| 1 | Joint 1 | 0 | 0 | 0.1745 | 0 |
| 2 | Joint 2 | +π/2 | 0 | 0 | 0 |
| 3 | Joint 3 | −π/2 | 0 | 0.287 | 0 |
| 4 | Joint 4 | +π/2 | 0.018 | 0 | π |
| 5 | Joint 5 | +π/2 | 0.018 | 0.314 | π |
| 6 | Joint 6 | +π/2 | 0 | 0 | π/2 |
| 7 | Joint 7 | +π/2 | 0 | 0 | π/2 |
| 8 | Flange→Sensor | +π/2 | 0 | 0.095 | π/2 |

Right-arm DH parameters will be derived from the MvKDCfg right-arm block
(rows 15–22 of `ccs_m6_40.MvKDCfg`). Key differences from left arm:
- Gravity direction: `(0, -9.81, 0)` vs left `(0, 9.81, 0)`.
- DH α signs may differ on joints 2, 3, 6, 7 (mirroring across YZ plane).
- Joint limits and PNVA are identical.
The exact right-arm DH table will be confirmed during implementation by
verifying FK round-trip consistency with MuJoCo body poses.

### 3.2 Forward Kinematics

Standard MDH product of exponentials (equivalent to DH matrix chain):

```
T_i = Rz(θ_i + offset_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)
T_0_N = ∏ T_i   for i = 1..8
```

Outputs:

- 4×4 homogeneous transform
- XYZABC (position in meters + Euler angles in radians for "si" mode;
  millimeters + degrees for "sdk" mode)

### 3.3 Inverse Kinematics

Damped Least Squares (DLS) with null-space projection:

1. **Initialize**: `q ← q_ref` (reference joint angles).
2. **Loop** (max 200 iterations, tolerance 1e-6 m / 1e-3 rad equiv):
   ```
   T_current = FK(q)
   e_pos = T_target[:3,3] - T_current[:3,3]         # 3-vector
   e_rot = rotation_error(T_current[:3,:3], T_target[:3,:3])  # 3-vector
   error = [e_pos; e_rot]                             # 6-vector
   if ‖error‖ < ε: break
   J = jacobian(q)
   Δq = J^T · (J·J^T + λ²·I)^(-1) · error            # DLS
   Δq += (I - J^T·J^+·J) · (q_ref - q) · α_null       # null-space projection
   q ← clamp(q + Δq, joint_limits)
   ```

3. **Return**: `IkResult` object with `joints_deg`, `success`, `iterations`,
   `residual`, `is_singular` fields.

Damping `λ` starts at 0.1 and adapts based on residual decrease. Null-space
gain `α_null` = 0.01 by default.

The DLS approach is chosen over analytical IK for portability across robot
configurations — it does not require deriving CCS-specific closed-form
solutions, and convergence is well-behaved given the 7-DOF redundancy.

### 3.4 Jacobian

Analytical 6×7 geometric Jacobian, computed column by column:

```
J_col_i = [ z_i × (p_tcp - p_i) ]
          [       z_i            ]
```

where `z_i` and `p_i` are the Z-axis and origin of each joint frame (from
the partial FK chain up to joint i), and `p_tcp` is the TCP position. All
expressed in base frame.

Alternative: use `mujoco.mj_jacSite()` from the `robot.py` layer when MuJoCo is
available; `kinematics.py` keeps the analytical version for standalone use.

### 3.5 Tool Offset

Internal tool matrix (4×4) defaults to identity. The FK chain always includes
row 8 (flange→sensor). Additional offset can be set via `set_tool(T_4x4)` which
appends another transform after the sensor frame. All inputs in DH row 8 units.

### 3.6 Unit Mode

```python
class MarvinKinematics:
    def __init__(self, arm: str, unit_mode: str = "si"):
        # unit_mode: "si" → rad, m, N·m; "sdk" → deg, mm, N·m
```

Internal calculations always use **SI units**. The `unit_mode` controls
only input/output conversion at the public API boundary:

| Parameter | SI mode | SDK mode |
|-----------|---------|----------|
| Joint angles | rad | deg |
| Positions | m | mm |
| Stiffness (joint) | N·m/rad | N·m/deg |
| Stiffness (translation) | N/m | N/mm |
| Damping (translation) | ratio | ratio |

Conversion factors are applied at the public method entry/exit points.
Current implementation uses "si" mode internally; SDK mode serves as a
display/parameter-consistency indicator for future hardware alignment.

## 4. Control Design (`controller.py`)

### 4.1 Control Modes

```python
class ControlMode(Enum):
    POSITION = 1           # stiff Cartesian impedance (no special params)
    JOINT_IMPEDANCE = 2    # τ = K·(q_des - q) + D·(qd_des - qd)
    CARTESIAN_IMPEDANCE = 3  # τ = J^T·(K·(x_des - x) + D·(xd_des - xd)) + nullspace
    FORCE = 4              # hybrid: Cartesian impedance in non-force axes,
                           #   admittance-based force tracking in force axis
```

### 4.2 Parameter Dataclasses

```python
@dataclass
class JointImpedanceParams:
    stiffness: tuple[float, ...]  # 7, N·m/rad (SI) or N·m/deg (SDK)
    damping: tuple[float, ...]    # 7, N·m/(rad/s) (SI) or N·m/(deg/s) (SDK)

@dataclass
class CartesianImpedanceParams:
    translational_stiffness: tuple[float, float, float]   # N/m (SI)
    translational_damping: tuple[float, float, float]     # N/(m/s) (SI)
    rotational_stiffness: tuple[float, float, float]      # N·m/rad (SI)
    rotational_damping: tuple[float, float, float]        # N·m/(rad/s) (SI)
    nullspace_stiffness: float                            # N·m/rad (SI)
    nullspace_damping: float                              # N·m/(rad/s) (SI)

@dataclass
class ForceControlParams:
    target_force_n: float           # target force, N
    direction: tuple[float, ...]    # 6D direction vector, e.g. [0,0,1,0,0,0] for Z force
    admittance_gain_m_per_ns: float # m/(N·s), position adjustment per force error
    max_position_offset_m: float    # maximum admittance adjustment, m
    feedback_alpha: float           # low-pass filter coefficient (0, 1]
```

### 4.3 Computation

**Joint Impedance**:
```
τ = K * (q_des - q)       # proportional stiffness
    + D * (0 - qd)        # velocity damping (target velocity = 0)
    + bias                 # gravity compensation
Result clamped by torque limits and rate limits.
```

**Cartesian Impedance**:
```
x, R = FK(q)
v = J @ qd
e_rot = orientation_error(R, R_des)          # 3-vector axis-angle
F_task = [Kp_trans * (x_des - x) - Kd_trans * v_linear;
          Kp_rot  * e_rot        - Kd_rot  * v_angular]
τ = J^T @ F_task
  + (I - J^T @ pinv(J^T)) @ (-K_null * (q - q_ref) - D_null * qd)
  + bias
```

**Force Control** (hybrid admittance):
- Non-force axes: Cartesian impedance tracking (same as above).
- Force axis: admittance adjusts the target position:
  ```
  F_filtered = α * F_raw + (1-α) * F_prev
  Δx_force ← Δx_force + gain * (F_target - F_filtered) * dt
  Δx_force ← clamp(Δx_force, ±max_offset)
  x_des_eff = x_des_nominal + Δx_force * tool_z_axis
  ```

### 4.4 Unified Controller API

```python
class UnifiedController:
    def __init__(self, kinematics: MarvinKinematics, joint_limits: np.ndarray,
                 unit_mode: str = "si"):
        ...

    # Mode & parameter setting
    def set_mode(self, mode: ControlMode) -> None
    def set_joint_impedance_params(self, params: JointImpedanceParams) -> None
    def set_cartesian_impedance_params(self, params: CartesianImpedanceParams) -> None
    def set_force_params(self, params: ForceControlParams) -> None

    # Target commands
    def set_joint_cmd(self, target_joints: np.ndarray) -> None
    def set_cart_cmd(self, target_pose_matrix: np.ndarray) -> None
    def set_force_cmd(self, force_n: float) -> None

    # Main compute (called each control cycle)
    def compute(self,
                current_joints: np.ndarray,      # rad or deg per unit_mode
                current_velocities: np.ndarray,
                jacobian: np.ndarray,            # 6×7
                wrench: np.ndarray | None = None,  # 6D, compensated
                bias_torque: np.ndarray | None = None,  # 7, gravity compensation
                ) -> np.ndarray:                 # → 7 joint torques (N·m, always SI)
```

### 4.5 Safety Constraints (inside `compute`)

- Joint torque limit: per-actuator `actuatorfrcrange` from MJCF.
- Torque rate limit: configurable, default 1500 N·m/s (same as existing tianji).
- Joint position limit clamp from MJCF `range`.
- NaN/Inf detection on all inputs and outputs.

## 5. Robot Interface (`robot.py`)

### 5.1 TwinRobot (SDK-aligned)

```python
class TwinRobot:
    """SDK-style unified interface for MuJoCo-based control."""

    # --- Lifecycle ---
    def connect(self, model_path: str) -> None
        # Load MJCF, create TwinMujocoRuntime, ArmView, kinematics, controller.
    def close(self) -> None        # Reset, optionally destroy runtime.

    # --- State switching (maps to SDK concise API) ---
    def set_position_state(self, vel_ratio: float = 0.5, acc_ratio: float = 0.5) -> None
    def set_joint_impedance_state(self, vel_ratio: float, acc_ratio: float,
                                   K, D) -> None
    def set_cart_impedance_state(self, vel_ratio: float, acc_ratio: float,
                                  K, D, rot_type: int = 0,
                                  cart_ctrl_para=None) -> None
    def set_force_state(self, vel_ratio: float, acc_ratio: float,
                         K, D, fx_dir, fc_adj_lmt: float) -> None
    def disable(self) -> None      # Disable torque output (set ctrl=0)

    # Note: vel_ratio and acc_ratio affect the trajectory interpolation speed
    # (how fast the controller moves the setpoint toward the target). They do
    # NOT affect the impedance parameters (stiffness/damping). For the initial
    # implementation, these ratios are accepted but the setpoint steps directly
    # to the commanded target; interpolation will be added later.

    # --- Command dispatch ---
    def set_joint_position_cmd(self, joints) -> None  # deg (sdk) or rad (si)
    def set_force_cmd(self, force: float) -> None      # N

    # --- Tool ---
    def set_tool(self, kine_para) -> None   # XYZABC mm/deg or m/rad per unit_mode
    def remove_tool(self) -> None

    # --- Data acquisition ---
    def get_joint_positions(self) -> np.ndarray
    def get_joint_velocities(self) -> np.ndarray
    def get_raw_wrench(self) -> np.ndarray      # Raw force+torque sensor
    def get_wrench(self) -> np.ndarray          # Compensated (tare + gravity)
    def get_tcp_pose(self) -> tuple[np.ndarray, np.ndarray]  # (position, rotation_matrix)
    def calibrate_wrench(self, samples: int = 1000) -> None

    # --- Stepping ---
    def step(self) -> None          # Compute torque → apply → mj_step, single cycle
    def spin(self, steps: int) -> None  # step() N times
```

### 5.2 Internal `step()` Logic

```
1. Read joint_positions, joint_velocities from ArmView.
2. If mode requires FK (cartesian / force): run FK → get TCP pose.
3. If mode requires Jacobian: build via MuJoCo mj_jacSite (7-DOF slice).
4. If mode requires wrench: read force+torque sensors, apply compensation.
5. Call UnifiedController.compute(...) → joint torques.
6. Apply torque via ArmView.apply_torque().
7. Call runtime.step() → advance MuJoCo.
```

## 6. Trajectory Scenarios

All trajectories use the **right arm** only, driven via `TwinRobot`.
**All trajectories open the MuJoCo Viewer by default** for real-time visual
inspection. Headless mode is available via `--headless` for automated testing.

Each trajectory type includes 3–4 distinct variants to exercise different
motion ranges and control behaviors.

### Trajectory 1 — Joint-Space Waypoints (4 variants)

**Mode**: JOINT_IMPEDANCE

| # | Name | Description | Hold (s) |
|---|------|-------------|----------|
| 1a | Small sweep | Home → 3 nearby configs (±10° per joint), return to home | 1.0 |
| 1b | Full range | Home → 4 configs spanning ±60° on joints 1-3, ±30° on 4-7 | 1.5 |
| 1c | Single joint | Move each joint individually (±30°) while others hold at home | 0.8 |
| 1d | Random walk | Home → 5 random feasible configs, random order | 1.0 |

**Validation**: Joint positions converge to targets within 0.5° within hold time.

### Trajectory 2 — Cartesian XYZ Translation (4 variants)

**Mode**: CARTESIAN_IMPEDANCE

| # | Name | Description | Hold (s) |
|---|------|-------------|----------|
| 2a | Axis-aligned | ±5 cm along X, Y, Z sequentially (6 segments) from home TCP | 1.0 |
| 2b | Diagonal | 5 cm along each of the 4 body diagonals (↗↘↖↙ in XZ+YZ planes) | 1.0 |
| 2c | Step response | 2 cm step in +Z, return; same for +X, return (tests overshoot) | 1.5 |
| 2d | Compound | Sequential translations forming a rectangular box (8 corners) | 0.8 |

**Validation**: TCP position error < 3 mm steady-state; no joint limit violations.

### Trajectory 3 — Cartesian Curves (3 variants)

**Mode**: CARTESIAN_IMPEDANCE

| # | Name | Description | Duration |
|---|------|-------------|----------|
| 3a | Circle XZ | Radius 8 cm, XZ plane, 3 full revolutions, constant speed | 5 s/rev |
| 3b | Circle XY | Radius 6 cm, XY plane (horizontal), 2 revolutions | 4 s/rev |
| 3c | Figure-8 | Lemniscate in XZ plane, width 12 cm × height 8 cm, 2 cycles | 6 s |

**Validation**: Position error < 5 mm from ideal path; orientation drift < 3°.

### Trajectory 4 — Hybrid Force Descent (3 variants)

**Mode**: CARTESIAN_IMPEDANCE → FORCE (Z-axis)

| # | Name | Description |
|---|------|-------------|
| 4a | Single press | Descend until contact (Fz > 2 N), hold 10 N for 1.5 s, retract |
| 4b | Repeated press | 3 press-release cycles at 10 N, 1 s hold each, 1 s between cycles |
| 4c | Variable force | Staircase: hold 5 N → 10 N → 15 N, each 1 s, then retract |

Uses the chopping board surface in `right_chopping_scene.xml` as the contact
target (board at Z ≈ 0.235 m in world frame).

**Validation**: Force tracking RMSE < 2 N during contact phases; contact detected
within 5 mm of board surface.

### CLI

```bash
python examples/trajectory_demo.py --trajectory 1a            # single variant
python examples/trajectory_demo.py --trajectory 2             # all variants in group 2
python examples/trajectory_demo.py --all                      # all 14 trajectories
python examples/trajectory_demo.py --all --headless --log /tmp/demo.csv  # headless + log
```

Default behavior: Viewer opens, trajectories run in sequence with a 2 s pause
between variants.

## 7. Testing

### Unit Tests (`test_kinematics.py`)

- FK at zero configuration → verify sensor-site pose matches expected translation
  (flange + 95 mm along flange Z).
- FK → IK → FK round-trip: `‖FK(IK(FK(q))) - FK(q)‖ < 1e-6` for 10 random
  configurations within joint limits.
- Jacobian consistency: J matches finite-difference of FK at 5 random points.
- Right-arm mirror: right FK output is symmetric with left FK (with appropriate
  sign flips).
- Unit mode conversion: same joint angles produce equivalent FK matrices in
  both "si" and "sdk" modes.
- FK → Matrix-to-XYZABC → XYZABC-to-Matrix roundtrip.

### Unit Tests (`test_controller.py`)

- Joint impedance: positive stiffness produces restoring torque toward target.
- Cartesian impedance: position error produces wrench in correct direction.
- Force control: admittance offset direction matches force error sign.
- Torque-rate limiting clips excessive changes.
- Torque magnitude limiting respects per-actuator bounds.

### Integration Validation

The four trajectory scenarios (Section 6) serve as integration validation.
Outputs include CSV logs with joint positions, TCP pose, wrench, and
control torques for manual inspection.

## 8. Design Decisions & Trade-offs

| Decision | Rationale |
|----------|-----------|
| DLS IK instead of analytical | Portable across configurations; avoids deriving CCS-specific closed forms; convergence is fast for 7-DOF |
| MDH parameters hardcoded from XML/MvKDCfg | Parameters are robot-fixed; avoids runtime dependency on .MvKDCfg parsing |
| Internal SI units, SDK mode as conversion layer | MuJoCo uses SI; avoids confusion from mixed units in algorithms; SDK mode purely for user-facing alignment |
| Separate kinematics/controller/robot layers | kinematics can be tested standalone; controller can swap kinematics implementations; robot.py is MuJoCo-only glue |
| ArmView + apply_torque reused from existing runtime | Avoids duplicating MuJoCo indexing logic; proven in existing chopping pipeline |
| Safety constraints inline in compute() | Per-tick enforcement; no separate safety layer needed for simulation phase |

## 9. Out of Scope (for this design)

- ROS 2 integration (pure MuJoCo phase, per tianji_robotic_project scope).
- Left-arm support for trajectories (kinematics supports it; trajectories use
  right arm only for consistency with existing chopping scene).
- PVT trajectory execution.
- Drag/teach mode.
- Hardware communication (UDP, 1KHz cycle).
- Dynamics parameter identification.

## 10. Future Extension Points

- **New robot models**: Replace DH table and joint limits in `kinematics.py`.
- **Left-arm trajectories**: Mirror the right-arm logic using `arm="left"`.
- **Hardware backend**: `TwinRobot` can be subclassed; replace `step()` with
  hardware communication while keeping the kinematics/controller layer intact.
- **Additional control modes**: Add new values to `ControlMode` enum and
  implement in `controller.py` behind the same `compute()` interface.
- **ROS integration**: `TwinRobot` can be wrapped as a ROS node (subscribe to
  joint commands, publish joint states) — same pattern as `cook_bringup`.
