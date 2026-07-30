# SDK Compat Minimum Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the MuJoCo backend support a vendor-SDK-style FK/IK plus Cartesian-impedance command loop.

**Architecture:** Keep the existing `TwinRobot`, `MarvinKinematics`, and `UnifiedController` modes unchanged. Extend the SDK compatibility layer so task code can call vendor-shaped APIs while the MuJoCo backend translates them into existing internal calls.

**Tech Stack:** Python, NumPy, pytest, MuJoCo-backed local robot model.

## Global Constraints

- Do not add a fifth internal control mode.
- Keep `TwinRobot` as the internal control implementation.
- SDK compatibility should preserve vendor units: joints in degrees, Cartesian translation in millimetres.
- Real backend should continue returning vendor `Concise_Marvin_Robot` without wrapping its core behavior.

---

### Task 1: Kinematics Compatibility

**Files:**
- Create: `src/twin_control/sdk_kine.py`
- Modify: `src/twin_control/sdk_compat.py`
- Modify: `tests/test_sdk_compat.py`

**Interfaces:**
- Produces: `create_kine("mujoco", arm_type=1)`
- Produces: `MujocoKine.fk(joints_deg) -> 4x4 list`
- Produces: `MujocoKine.ik(structure_data) -> structure_data`
- Produces: `MujocoKine.joints2JacobMatrix(joints_deg) -> 6x7 list`

- [ ] Write failing tests for `create_kine`, FK, IK, and Jacobian shape.
- [ ] Implement `MujocoKine` by delegating to `TwinMujocoRuntime` and `MarvinKinematics`.
- [ ] Run `pytest tests/test_sdk_compat.py -q`.

### Task 2: SDK-Style Cartesian Impedance Semantics

**Files:**
- Modify: `src/twin_control/sdk_compat.py`
- Modify: `tests/test_sdk_compat.py`

**Interfaces:**
- Consumes: `MujocoSdkRobot.set_imp_cart_state(...)`
- Changes: `MujocoSdkRobot.set_joint_position_cmd(...)` maps joint command through FK to a Cartesian target when current mode is Cartesian impedance or force.

- [ ] Write failing test showing Cartesian impedance receives `FK(q_des)` after `set_joint_position_cmd`.
- [ ] Implement mode-aware command translation.
- [ ] Run `pytest tests/test_sdk_compat.py -q`.

### Task 3: Feedback Compatibility and Debug Demo

**Files:**
- Modify: `src/twin_control/sdk_compat.py`
- Create: `examples/DEMO_PYTHON_STYLE/showcase_ik_cart_impedance.py`
- Modify: `tests/test_sdk_compat.py`

**Interfaces:**
- Produces: richer `subscribe()` fields: `frame_serial`, `fb_joint_pos`, `fb_joint_vel`, `fb_joint_cmd`, `fb_joint_sToq`, `est_cart_fn`, `low_speed_flag`, `traj_state`.
- Produces: `run_demo(...)` function callable from Python for debugging.

- [ ] Write failing tests for feedback fields and demo importability.
- [ ] Implement feedback fields and demo function.
- [ ] Run targeted tests and then full pytest.
