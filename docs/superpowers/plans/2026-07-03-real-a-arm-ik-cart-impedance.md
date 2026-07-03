# Real A-Arm IK Cartesian Impedance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a dedicated `real_robot_debug/` workflow for A-arm real-machine IK Cartesian impedance down-up lateral experiments.

**Architecture:** Keep the real-machine workflow separate from MuJoCo examples. Put pure trajectory/config helpers in the script so they can be tested without hardware, and keep all vendor SDK imports inside runtime functions where practical.

**Tech Stack:** Python 3, pytest, vendor `SDK_PYTHON.fx_robot`, vendor `SDK_PYTHON.fx_kine`, stdlib CSV/argparse/time/pathlib.

## Global Constraints

- Create a new folder named `real_robot_debug/`.
- Default real-machine arm is `A`.
- Start motion from current A-arm feedback pose, not from MuJoCo B-arm home.
- Default behavior is dry-run; real motion requires `--execute`.
- Do not modify `SDK_PYTHON/`, `test/`, or MuJoCo runtime code.

---

### Task 1: Real Debug Script and Tests

**Files:**
- Create: `real_robot_debug/real_ik_cart_impedance_lateral.py`
- Create: `real_robot_debug/README.md`
- Create: `tests/test_real_robot_debug_script.py`

**Interfaces:**
- Produces: `MotionConfig` dataclass
- Produces: `cut_cycle_progress(cycle_step: int, steps_per_cycle: int) -> tuple[float, float]`
- Produces: `build_relative_targets(start_pose_mm: np.ndarray, config: MotionConfig) -> list[np.ndarray]`
- Produces: `validate_config(config: MotionConfig) -> None`
- Produces: CLI entry point `main(argv: list[str] | None = None) -> int`

- [ ] **Step 1: Write failing tests for pure helpers**

Create `tests/test_real_robot_debug_script.py` with tests that import the script, assert default A-arm dry-run config, validate down-up/lateral target generation, and reject unsafe values.

- [ ] **Step 2: Run tests to verify RED**

Run: `python3 -m pytest tests/test_real_robot_debug_script.py -q`

Expected: FAIL because `real_robot_debug.real_ik_cart_impedance_lateral` does not exist.

- [ ] **Step 3: Implement script and README**

Create a hardware-safe script with default dry-run behavior, explicit `--execute`, config validation, SDK setup, Cartesian impedance setup, IK loop, optional CSV trace, and release in `finally`.

- [ ] **Step 4: Run tests to verify GREEN**

Run: `python3 -m pytest tests/test_real_robot_debug_script.py -q`

Expected: PASS.

- [ ] **Step 5: Run import syntax verification**

Run: `python3 -m py_compile real_robot_debug/real_ik_cart_impedance_lateral.py`

Expected: no output and exit code 0.
