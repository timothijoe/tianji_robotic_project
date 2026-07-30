# Continuous Chop Trajectory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a first continuous Cartesian trajectory layer for chopping so the existing impedance controller tracks smoother targets.

**Architecture:** Add a small `twin_control.trajectory` module that generates minimum-jerk Cartesian pose points. Keep the existing controller unchanged; `TwinRobotChopper` will consume the trajectory points instead of linear interpolation and will ramp force targets during `FORCE_HOLD`.

**Tech Stack:** Python, NumPy, pytest, existing MuJoCo-backed `TwinRobotChopper`.

## Global Constraints

- Do not change the core impedance control equations in `UnifiedController`.
- Keep V1 scoped to Cartesian pose continuity and force target ramping.
- Preserve existing chopping public API and existing tests.

---

### Task 1: Minimum-Jerk Cartesian Trajectory

**Files:**
- Create: `src/twin_control/trajectory.py`
- Create: `tests/test_trajectory.py`

**Interfaces:**
- Produces: `minimum_jerk_scalar(r: float) -> float`
- Produces: `cartesian_minimum_jerk_trajectory(start_pos, target_pos, rotation, steps, phase=None, start_time_s=0.0, dt_s=0.002) -> list[CartesianTrajectoryPoint]`

- [ ] **Step 1: Write failing tests** for endpoint, midpoint, and constant target rotation.
- [ ] **Step 2: Run `pytest tests/test_trajectory.py -q`** and verify import failure.
- [ ] **Step 3: Implement dataclass and generator** in `src/twin_control/trajectory.py`.
- [ ] **Step 4: Re-run `pytest tests/test_trajectory.py -q`** and verify pass.

### Task 2: Chopper Integration

**Files:**
- Modify: `src/twin_control/chopping.py`
- Modify: `tests/test_twin_control_chopping.py`

**Interfaces:**
- Consumes: `cartesian_minimum_jerk_trajectory(...)`
- Produces: smoother target positions in `_move_tip_to(...)`
- Produces: ramped `target_force_n` samples during `FORCE_HOLD`

- [ ] **Step 1: Write failing test** that FORCE_HOLD target force starts below final force and ends at requested force.
- [ ] **Step 2: Run targeted chopping test** and verify it fails on current step target.
- [ ] **Step 3: Replace linear interpolation** in `_move_tip_to(...)` with minimum-jerk trajectory points.
- [ ] **Step 4: Add force target ramp** in `FORCE_HOLD`.
- [ ] **Step 5: Run targeted chopping tests** and verify pass.

### Task 3: Verification

**Files:**
- No new files.

**Interfaces:**
- Verifies all affected behavior through pytest.

- [ ] **Step 1: Run `pytest tests/test_trajectory.py tests/test_twin_control_chopping.py -q`**.
- [ ] **Step 2: Run broader relevant suite if targeted tests pass: `pytest tests/test_controller.py tests/test_robot.py tests/test_runtime_arm_view.py tests/test_trajectory.py tests/test_twin_control_chopping.py -q`**.
