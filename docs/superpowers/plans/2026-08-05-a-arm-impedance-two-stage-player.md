# A-Arm Impedance Two-Stage Player Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide a real-robot A-arm command-line player that plans a 20-second impedance entry from live feedback to an offline trajectory, then plays the trajectory at 0.1× speed.

**Architecture:** A small module in `real_robot_debug` loads and validates only the candidate NPZ's `left_arm_target_rad` and `time_s`, generates a quintic entry segment, and separates planning from hardware calls. The hardware runner follows the existing Marvin SDK joint-impedance sequence, but default mode remains read-only and `--execute` is rejected until an independent collision-preflight contract exists.

**Tech Stack:** Python 3, NumPy, pytest, existing `SDK_PYTHON.fx_robot` Marvin SDK.

## Global Constraints

- Target only SDK arm `A`, mapped to feedback arm index `0`.
- Load only `left_arm_target_rad` from `recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate.npz`.
- Entry is a 20-second, 200 Hz quintic smoothstep from measured feedback to the first target.
- Playback schedules source timestamps at 0.1× (5 ms source spacing becomes 50 ms wall time).
- Every plan target must be finite, seven-dimensional, and within explicit CLI joint limits.
- Default mode must neither enable the robot nor send a command.
- `--execute` must fail closed until a separately implemented collision-preflight approval is available.
- On any live-run failure, attempt `state=0` and release the SDK connection; never clear controller faults.

---

### Task 1: Pure trajectory planning and validation

**Files:**
- Create: `real_robot_debug/a_arm_impedance_two_stage.py`
- Test: `tests/hardware/test_a_arm_impedance_two_stage.py`

**Interfaces:**
- Produces `load_left_arm_trajectory(path: Path) -> OfflineTrajectory`.
- Produces `build_entry_trajectory(start_deg: np.ndarray, target_rad: np.ndarray, duration_s: float, control_hz: float) -> np.ndarray`.
- Produces `build_playback_schedule(time_s: np.ndarray, speed_scale: float) -> np.ndarray`.

- [ ] **Step 1: Write failing tests** for NPZ shape/timing validation, exact entry endpoints, and 0.1× schedule spacing.
- [ ] **Step 2: Run the focused tests** and confirm imports fail because the new module is absent.
- [ ] **Step 3: Implement the minimal pure data records, validation, quintic entry generation, and source-time schedule.**
- [ ] **Step 4: Rerun focused tests** and confirm they pass.

### Task 2: Fail-closed command-line and SDK runner

**Files:**
- Modify: `real_robot_debug/a_arm_impedance_two_stage.py`
- Modify: `tests/hardware/test_a_arm_impedance_two_stage.py`

**Interfaces:**
- Produces `PlayerConfig` parsed from CLI flags including `--execute`, K/D, limits, feedback tolerance, and source NPZ.
- Produces `run_player(config: PlayerConfig) -> dict`.

- [ ] **Step 1: Write failing tests** that default execution is read-only and `--execute` refuses before any SDK import or command without collision-preflight approval.
- [ ] **Step 2: Run the tests** and confirm the requested API is absent.
- [ ] **Step 3: Implement parser/config validation and a runner that imports the SDK only when a live run is requested.**
- [ ] **Step 4: Add the impedance command helpers** following `set_state(..., state=3)`, `set_impedance_type(..., type=1)`, `set_joint_kd_params`, and sampled `set_joint_position_cmd`/`set_joint_cmd_pose`; keep them unreachable while the preflight gate is false.
- [ ] **Step 5: Rerun the focused tests** and confirm they pass.

### Task 3: Operator documentation and verification

**Files:**
- Modify: `docs/wuji/hardware_interfaces.md`
- Test: `tests/hardware/test_a_arm_impedance_two_stage.py`

- [ ] **Step 1: Document the read-only invocation, expected planning output, and explicit safety reason execution is unavailable.**
- [ ] **Step 2: Run the focused test file and a read-only CLI smoke test against the local candidate NPZ.**
- [ ] **Step 3: Inspect git diff/status** to ensure no recording or unrelated user file changed.
