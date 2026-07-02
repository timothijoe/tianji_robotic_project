# Marvin SDK Control Notes

This note records the current understanding of the vendor SDK, the MuJoCo
simulation adapter in this repository, and the relationship between kinematics,
targets, impedance parameters, feedback, and frequency. It is intended for
future agents working in this repository.

## Repository Context

The vendor SDK lives outside this repository:

```text
/home/zhoutong/docker_share/docker_mapping/cook_proj/TJ_FX_ROBOT_CONTRL_SDK
```

The current project provides a MuJoCo-based simulation and a compatibility
layer so SDK-style Python scripts can be run in simulation first and later
ported to the real robot.

Important local files:

```text
src/twin_control/sdk_compat.py
src/twin_control/sdk_kine.py
src/twin_control/robot.py
src/twin_control/controller.py
src/twin_control/chopping.py
examples/DEMO_PYTHON_STYLE/
```

Important vendor SDK files:

```text
TJ_FX_ROBOT_CONTRL_SDK/SDK_PYTHON/fx_robot.py
TJ_FX_ROBOT_CONTRL_SDK/SDK_PYTHON/fx_kine.py
TJ_FX_ROBOT_CONTRL_SDK/DEMO_PYTHON/
TJ_FX_ROBOT_CONTRL_SDK/python_doc_contrl.md
TJ_FX_ROBOT_CONTRL_SDK/python_doc_kine.md
TJ_FX_ROBOT_CONTRL_SDK/contrlSDK/MarvinSDK.h
TJ_FX_ROBOT_CONTRL_SDK/contrlSDK/FxRtCSDef.h
TJ_FX_ROBOT_CONTRL_SDK/kinematicsSDK/FxRobot.h
```

## Main Split: Control SDK vs Kinematics SDK

The vendor package has two logically separate SDKs.

The control SDK is exposed through:

```python
from SDK_PYTHON.fx_robot import Marvin_Robot, Concise_Marvin_Robot
```

It connects to the real controller over UDP, switches modes, sends targets,
sets impedance/force/tool parameters, and subscribes feedback.

The kinematics SDK is exposed through:

```python
from SDK_PYTHON.fx_kine import Marvin_Kine, FX_InvKineSolvePara
```

It does offline computation: load robot configuration, FK, IK, Jacobian,
MOVL/MOVLA planning, tool kinematic transform, and tool dynamic identification.
It does not itself drive the robot.

Do not assume `fx_robot.py` calls `fx_kine.py` in the Python process. In normal
vendor demos, the Python control demo sends parameters and targets to the real
controller; the controller firmware does the low-level closed loop.

## Kinematics SDK

Typical initialization:

```python
from SDK_PYTHON.fx_kine import Marvin_Kine, FX_InvKineSolvePara

kk = Marvin_Kine()
cfg = kk.load_config(arm_type=0, config_path="ccs_m6_40.MvKDCfg")
kk.initial_kine(
    robot_type=cfg["TYPE"][0],
    dh=cfg["DH"][0],
    pnva=cfg["PNVA"][0],
    j67=cfg["BD"][0],
)
```

`arm_type=0` means SDK arm A / left arm. `arm_type=1` means SDK arm B / right
arm.

The `.MvKDCfg` file is not URDF/MJCF. It contains vendor parameters for both
arms: robot type, gravity direction, DH/flange parameters, joint limits,
velocity/acceleration limits, link mass, center of mass, inertia, and special
6/7-axis interference limits.

### FK

```python
pose_mat = kk.fk(joints)
```

Input:

```python
joints = [j1, j2, j3, j4, j5, j6, j7]
```

Units: degrees.

Output: 4x4 homogeneous matrix. Translation is in millimetres. Rotation is a
rotation matrix.

If `kk.set_tool_kine(tool_mat)` has been called, FK is computed to the tool TCP.
Otherwise it is computed to the flange/end frame.

### IK

The vendor IK API uses a structure, not a direct `x, y, z, rpy` signature:

```python
sp = FX_InvKineSolvePara()
sp.set_input_ik_target_tcp(mat16)
sp.set_input_ik_ref_joint(ref_joints)
sp.set_input_ik_zsp_type(0)

result = kk.ik(sp)
```

Inputs:

```text
target_tcp: 4x4 target pose flattened to 16 values
ref_joint: 7 reference joint angles, degrees
zsp_type: zero-space selection mode
zsp_para: optional zero-space parameters
zsp_angle: optional zero-space arm-angle adjustment
```

Outputs are fields on the returned structure:

```python
result.m_Output_RetJoint.to_list()   # selected IK result, degrees
result.m_OutPut_AllJoint             # all candidate solutions
result.m_OutPut_Result_Num           # number of valid/candidate solutions
result.m_Output_IsOutRange           # target beyond reachable workspace
result.m_Output_IsDeg                # singularity flags
result.m_Output_IsJntExd             # joint-limit violation flag
result.m_Output_JntExdTags           # per-joint limit flags
```

The selected result is the one closest to the reference joints according to the
chosen zero-space rule. The reference joint matters because this is a 7-DOF
redundant arm.

### Jacobian

```python
J = kk.joints2JacobMatrix(joints)
```

Input: 7 joint angles, degrees.

Output: 6x7 Jacobian matrix.

This is available from the kinematics SDK, but the basic control demos do not
use it directly in Python.

### Motion Planning

Representative kinematics planning APIs:

```python
kk.movL(start_xyzabc, end_xyzabc, ref_joints, vel, acc, freq_hz, save_path)
kk.movLA(start_xyzabc, end_xyzabc, ref_joints, vel, acc, freq_hz)
kk.movL_KeepJ(start_joints, end_joints, vel, acc, freq_hz, save_path)
kk.movL_KeepJA(start_joints, end_joints, vel, acc, freq_hz)
```

Important units:

```text
XYZ: millimetres
ABC: degrees
joints: degrees
vel: mm/s
acc: mm/s^2
freq_hz: planning frequency parameter
```

The docs say the planner has a 1000 Hz base and may output 500 Hz if the
requested frequency is not a clean divisor. Treat `freq_hz` here as planning
sample frequency, not as a way to change the real controller's internal servo
frequency.

## Control SDK

The older control class:

```python
robot = Marvin_Robot()
```

The newer concise control class:

```python
robot = Concise_Marvin_Robot()
```

The old class requires many commands to be used between:

```python
robot.clear_set()
# set parameters/targets here
robot.send_cmd()
```

The docs explicitly state that the command buffer is refreshed at 1 kHz. The
Python code is not where that real controller frequency is configured.

### Real Controller Frequency

The real robot controller uses an internal 1 kHz UDP communication/subscription
and command buffer mechanism. There is no `set_control_hz()` style API in the
Python control SDK.

This is easy to confuse with other frequencies:

```text
1 kHz: real controller communication/buffer refresh, internal to controller
freq_hz: kinematics planner sample frequency for MOVL/MOVLA/PVT generation
control_hz: local MuJoCo simulation control-loop rate in this repository
time.sleep: Python task-level update/supervision pacing
```

For real hardware, Python can send target updates at a lower task rate. The
controller still runs its own internal loop.

### Feedback

Real robot feedback comes from:

```python
sub_data = robot.subscribe(dcss)
```

Relevant fields:

```python
sub_data["states"][0]["cur_state"]        # arm A current state
sub_data["states"][1]["cur_state"]        # arm B current state
sub_data["outputs"][0]["fb_joint_pos"]    # arm A joint position, degrees
sub_data["outputs"][0]["fb_joint_vel"]    # arm A joint velocity, deg/s
sub_data["outputs"][0]["fb_joint_cmd"]    # arm A last/current joint command
sub_data["outputs"][0]["fb_joint_sToq"]   # arm A sensed joint torque
sub_data["outputs"][0]["est_cart_fn"]     # arm A estimated end wrench
sub_data["outputs"][0]["low_speed_flag"]  # stopped/low-speed flag
```

Index `0` is arm A / left. Index `1` is arm B / right.

## What `showcase_torque_cart_impedance_arm_A.py` Actually Does

The vendor demo:

```text
DEMO_PYTHON/showcase_torque_cart_impedance_arm_A.py
```

does not contain a Python FK/IK/Jacobian loop and does not set a Python control
frequency. It sends mode and parameter changes to the controller:

```python
robot.clear_set()
robot.set_state(arm="A", state=3)          # torque mode
robot.set_impedance_type(arm="A", type=2)  # Cartesian impedance
robot.set_vel_acc(arm="A", velRatio=10, AccRatio=10)
robot.send_cmd()

robot.clear_set()
robot.set_cart_kd_params(
    arm="A",
    K=[2000, 2000, 2000, 40, 40, 40, 20],
    D=[0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 1],
    type=2,
)
robot.send_cmd()

robot.clear_set()
joint_cmd_1 = [10., 20., 30., 40., 50., 10., 10.]
robot.set_joint_cmd_pose(arm="A", joints=joint_cmd_1)
robot.send_cmd()
```

The important point: `joint_cmd_1` is a joint-space target in degrees, not an
end-effector XYZ/RPY target.

The likely internal controller interpretation is:

```text
Python sends:
    mode = torque + Cartesian impedance
    K/D parameters
    joint target q_cmd

Controller internally computes:
    x_des = FK(q_cmd)
    x_now = FK(q_now)
    x_dot = J(q_now) qd
    F = K (x_des - x_now) - D x_dot
    tau = J(q_now).T F + bias
```

This control law is inferred from the API semantics and documentation; the
actual low-level servo implementation is inside the controller system, not in
the Python demo. The open SDK source exposes communication wrappers and
parameter structures, not the firmware-level servo loop.

## Sending Only `joint_cmd_1`

When a script sends:

```python
robot.set_joint_cmd_pose(arm="A", joints=joint_cmd_1)
```

it provides a joint position target in degrees. Velocity and acceleration are
not in `joint_cmd_1`; they are set separately:

```python
robot.set_vel_acc(arm="A", velRatio=10, AccRatio=10)
```

Impedance is also set separately:

```python
robot.set_joint_kd_params(...)
robot.set_cart_kd_params(...)
robot.set_impedance_type(...)
```

A useful mental model:

```text
set_vel_acc          target transition speed/acceleration percentage
set_cart_kd_params   Cartesian stiffness/damping parameters
set_joint_kd_params  joint stiffness/damping parameters
set_state            high-level controller state
set_impedance_type   joint/cartesian/force impedance selection
set_joint_cmd_pose   target joint configuration
subscribe            measured feedback from real controller
```

For a single target, the controller moves toward it using its internal mode and
limits. For a smooth trajectory, do not simply send one target and sleep. Prefer
one of:

```text
setPln_joint / RunPlnJoint
MOVLA + setPln_Cart / RunPlnCart
PVT file upload + RunPVT
task-level loop that updates targets at a controlled lower frequency
```

## Tool Parameters

There are two relevant tool concepts.

Control SDK tool setting:

```python
robot.set_tool(arm, kineParams, dynamicParams)
```

`kineParams` is `[x, y, z, a, b, c]` in millimetres/degrees. `dynamicParams`
is 10 values:

```text
mass,
mcp_x, mcp_y, mcp_z,
ixx, ixy, ixz, iyy, iyz, izz
```

This affects controller-side tool kinematics/dynamics, especially torque mode
and force/impedance behaviour.

Kinematics SDK tool setting:

```python
tool_mat = kk.xyzabc_to_mat4x4(kineParams)
kk.set_tool_kine(tool_mat)
```

This affects offline FK/IK calculation in `fx_kine.py`. If you set a tool on
the controller but not in the kinematics SDK, offline FK/IK may refer to a
different TCP from the real controller.

## MuJoCo Compatibility Layer in This Repository

`src/twin_control/sdk_compat.py` provides a backend selector:

```python
from twin_control.sdk_compat import create_ik_param, create_kine, create_robot

robot = create_robot("mujoco", arm="B", viewer=True)
kine = create_kine("mujoco", arm_type=1)
sp = create_ik_param("mujoco")
robot.connect("mujoco")
...
robot.release_robot()
```

The MuJoCo backend supports a subset of the vendor concise API:

```text
connect
release_robot
set_position_state
set_imp_joint_state
set_imp_cart_state
set_imp_force_state
set_joint_position_cmd
set_force_cmd
disable
wait
step
subscribe
get_joint_positions
get_joint_velocities
get_tcp_pose
```

`src/twin_control/sdk_kine.py` provides a local MuJoCo-backed subset of the
vendor kinematics API:

```text
FX_InvKineSolvePara
create_ik_param
MujocoKine.load_config
MujocoKine.initial_kine
MujocoKine.fk
MujocoKine.ik
MujocoKine.joints2JacobMatrix
MujocoKine.mat4x4_to_xyzabc
MujocoKine.xyzabc_to_mat4x4
```

Boundary units intentionally follow the vendor SDK: joint angles are degrees,
Cartesian translation is millimetres, and homogeneous matrices are flattened in
the same 16-value shape used by `FX_InvKineSolvePara`. Internally the local
implementation converts to MuJoCo/SI units and calls `MarvinKinematics`.

The local `MujocoKine.load_config()` returns a vendor-shaped placeholder for
simulation. It does not parse `.MvKDCfg`; real hardware should still use the
vendor `Marvin_Kine` through `create_kine("real", sdk_root=...)`.

In the MuJoCo compatibility backend, `set_joint_position_cmd()` is mode-aware:

```text
POSITION / JOINT_IMPEDANCE:
    send q_des directly as joint target

CARTESIAN_IMPEDANCE:
    compute x_des = FK(q_des), then send x_des as Cartesian target

FORCE:
    compute x_des = FK(q_des), send x_des as Cartesian target,
    and keep q_des as the force-controller nullspace/reference joint target
```

This mirrors the practical interpretation of the vendor Cartesian impedance
demos, where Python still sends a joint target while the controller interprets
that target through the active impedance type.

The MuJoCo `subscribe()` response now exposes SDK-shaped feedback fields such
as `fb_joint_pos`, `fb_joint_vel`, `fb_joint_cmd`, `fb_joint_sToq`,
`est_cart_fn`, `low_speed_flag`, `traj_state`, and `control_mode`. These are
simulation estimates, not raw hardware telemetry.

The real backend uses:

```python
robot = create_robot("real", sdk_root=...)
```

which returns a thin `RealSdkRobotAdapter` around the vendor
`Concise_Marvin_Robot`. The adapter delegates normal SDK methods to the vendor
object, adds `wait(seconds)`, and owns a `DCSS()` instance so business scripts
can call `robot.subscribe(None)` in the same style as the MuJoCo backend.

The real kinematics backend uses:

```python
kine = create_kine("real", sdk_root=...)
sp = create_ik_param("real", sdk_root=...)
```

which returns the vendor `Marvin_Kine` object and the vendor
`FX_InvKineSolvePara` structure.

SDK arm mapping in the MuJoCo compatibility layer:

```text
A -> left arm
B -> right arm
```

The current chopping workflow usually uses arm B / right arm.

## Current Local Demo Style

New SDK-style examples are in:

```text
examples/DEMO_PYTHON_STYLE/
```

Run in MuJoCo:

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_position.py --backend mujoco --viewer
python3 examples/DEMO_PYTHON_STYLE/showcase_joint_impedance.py --backend mujoco --headless --no-realtime
python3 examples/DEMO_PYTHON_STYLE/showcase_ik_cart_impedance.py --backend mujoco --headless
```

Switch to real hardware by changing the backend:

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_position.py \
  --backend real \
  --robot-ip 192.168.1.190
```

The goal is to keep task/business logic written against SDK-style calls so it
can be simulated first and then ported to the real robot with minimal code
changes.

## Recommended Engineering Interpretation

When reviewing or writing control code, identify which layer is responsible:

```text
Offline planning/FK/IK/Jacobian:
    SDK_PYTHON/fx_kine.py or local twin_control.kinematics

Real robot mode/target/feedback:
    SDK_PYTHON/fx_robot.py / Concise_Marvin_Robot

Real low-level impedance/servo loop:
    controller firmware, not visible in Python demos

MuJoCo low-level impedance/force loop:
    src/twin_control/controller.py + src/twin_control/robot.py

Cross-backend SDK-style script interface:
    src/twin_control/sdk_compat.py + src/twin_control/sdk_kine.py
```

Do not infer that absence of Python FK/IK means absence of FK/IK in the real
control loop. In the vendor demos, those calculations are likely performed by
the controller internally after it receives joint targets, impedance parameters,
and mode settings.

Do not assume `control_hz` in the local MuJoCo code changes any real robot
frequency. It only controls how often the local simulator computes torques and
steps physics.
