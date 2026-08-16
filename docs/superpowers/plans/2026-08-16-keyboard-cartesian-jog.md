# Keyboard Cartesian Jog Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a conservative terminal keyboard jog program that moves one real robot arm by discrete, base-frame Cartesian steps only after explicit authorization and safety checks.

**Architecture:** `real_robot_debug/keyboard_cartesian_jog.py` separates pure configuration/key/pose/workspace helpers from an SDK runner. The runner uses DCSS feedback, `MOVLA`, `setPln_Cart`, idle confirmation, and cleanup using the established conventions in `real_ik_cart_impedance_lateral.py`. Tests use pure helpers and fakes only.

**Tech Stack:** Python 3, NumPy, argparse, POSIX `termios`/`tty`, existing `SDK_PYTHON`, pytest.

## Global Constraints

- Create `real_robot_debug/keyboard_cartesian_jog.py`; do not modify ROS 2, hand, MuJoCo, chopping, or recorded-playback code.
- Default arm A, dry-run, 2 mm step, velocity/acceleration ratios 10. No command is sent without `--execute`.
- Execute mode requires finite, strictly ordered `--workspace-min` and `--workspace-max` XYZ in millimetres; a step must be positive and at most 5 mm.
- Preserve TCP orientation, refresh feedback after every executed segment, and reject stale feedback, non-idle state, out-of-box poses, and planning failure.
- Space, Q, EOF, interruption, and errors clear commands, disable the arm unless `--keep-enabled`, then release the SDK. No test connects to a robot.

---

### Task 1: Pure key, pose, and configuration contract

**Files:**
- Create: `real_robot_debug/keyboard_cartesian_jog.py`
- Create: `tests/hardware/test_keyboard_cartesian_jog.py`

**Interfaces:**
- Produces `JogConfig`, `parse_xyz(value: str, flag_name: str) -> tuple[float, float, float]`, `validate_config(config: JogConfig) -> None`, `key_to_delta(key: str, step_mm: float) -> tuple[float, float, float] | None`, `candidate_pose(current_xyzabc: np.ndarray, delta_xyz: tuple[float, float, float]) -> np.ndarray`, and `inside_workspace(point_xyz: np.ndarray, lower_xyz: tuple[float, float, float], upper_xyz: tuple[float, float, float]) -> bool`.

- [ ] **Step 1: Write failing mapping and orientation-preserving pose tests**

```python
import numpy as np
from real_robot_debug.keyboard_cartesian_jog import candidate_pose, key_to_delta

def test_w_requests_one_positive_x_step():
    assert key_to_delta("w", 2.0) == (2.0, 0.0, 0.0)

def test_f_requests_one_negative_z_step():
    assert key_to_delta("F", 2.0) == (0.0, 0.0, -2.0)

def test_candidate_pose_preserves_orientation():
    current = np.array([100., 200., 300., 10., 20., 30.])
    expected = np.array([102., 198., 300., 10., 20., 30.])
    assert np.array_equal(candidate_pose(current, (2., -2., 0.)), expected)
```

- [ ] **Step 2: Verify the focused tests fail for the absent module**

Run: `pytest tests/hardware/test_keyboard_cartesian_jog.py -q`

Expected: FAIL during collection with `ModuleNotFoundError` for `real_robot_debug.keyboard_cartesian_jog`.

- [ ] **Step 3: Add the minimal pure mapping and pose implementation**

```python
KEY_DIRECTIONS = {
    "w": (1., 0., 0.), "s": (-1., 0., 0.),
    "a": (0., 1., 0.), "d": (0., -1., 0.),
    "r": (0., 0., 1.), "f": (0., 0., -1.),
}

def key_to_delta(key: str, step_mm: float):
    direction = KEY_DIRECTIONS.get(key.lower())
    return None if direction is None else tuple(step_mm * axis for axis in direction)

def candidate_pose(current_xyzabc: np.ndarray, delta_xyz: tuple[float, float, float]) -> np.ndarray:
    target = np.asarray(current_xyzabc, dtype=float).copy()
    target[:3] += np.asarray(delta_xyz, dtype=float)
    return target
```

- [ ] **Step 4: Verify mapping and pose tests pass**

Run: `pytest tests/hardware/test_keyboard_cartesian_jog.py -q`

Expected: PASS.

- [ ] **Step 5: Write failing execute/workspace validation tests**

```python
import pytest
from real_robot_debug.keyboard_cartesian_jog import JogConfig, inside_workspace, validate_config

def test_execute_requires_both_workspace_bounds():
    with pytest.raises(ValueError, match="workspace-min.*workspace-max"):
        validate_config(JogConfig(execute=True))

def test_step_above_five_mm_is_rejected():
    with pytest.raises(ValueError, match="step-mm must be in .*5"):
        validate_config(JogConfig(step_mm=5.1))

def test_workspace_includes_edges_but_rejects_outside_point():
    lower, upper = (0., 0., 0.), (10., 10., 10.)
    assert inside_workspace(np.array([0., 10., 5.]), lower, upper)
    assert not inside_workspace(np.array([10.1, 10., 5.]), lower, upper)
```

- [ ] **Step 6: Verify the validation tests fail for missing behavior**

Run: `pytest tests/hardware/test_keyboard_cartesian_jog.py -q`

Expected: FAIL because execute configuration is accepted or required APIs are absent.

- [ ] **Step 7: Implement strict configuration and workspace validation**

```python
@dataclass(frozen=True)
class JogConfig:
    arm: str = "A"; step_mm: float = 2.0; execute: bool = False
    workspace_min: tuple[float, float, float] | None = None
    workspace_max: tuple[float, float, float] | None = None
    vel_ratio: int = 10; acc_ratio: int = 10

def inside_workspace(point_xyz, lower_xyz, upper_xyz) -> bool:
    return bool(np.all(np.asarray(point_xyz) >= lower_xyz) and np.all(np.asarray(point_xyz) <= upper_xyz))

def validate_config(config: JogConfig) -> None:
    if not 0.0 < float(config.step_mm) <= 5.0:
        raise ValueError("step-mm must be in (0, 5]")
    if config.execute and (config.workspace_min is None or config.workspace_max is None):
        raise ValueError("--execute requires both --workspace-min and --workspace-max")
    if config.workspace_min is not None and config.workspace_max is not None and not np.all(np.asarray(config.workspace_min) < np.asarray(config.workspace_max)):
        raise ValueError("workspace-min must be strictly below workspace-max")
```

- [ ] **Step 8: Run all Task 1 tests and commit**

Run: `pytest tests/hardware/test_keyboard_cartesian_jog.py -q`

Expected: PASS.

```bash
git add real_robot_debug/keyboard_cartesian_jog.py tests/hardware/test_keyboard_cartesian_jog.py && git commit -m "feat: add safe keyboard jog helpers"
```

### Task 2: Guarded per-step SDK runner

**Files:**
- Modify: `real_robot_debug/keyboard_cartesian_jog.py`
- Modify: `tests/hardware/test_keyboard_cartesian_jog.py`

**Interfaces:**
- Consumes Task 1 helpers.
- Produces `run_jog_session(config: JogConfig, read_key: Callable[[], str], sdk_factory: Callable[[], tuple[object, object, object]]) -> list[np.ndarray]` and `plan_or_execute_step(...) -> np.ndarray`.

- [ ] **Step 1: Write failing dry-run test using fake robot/kinematics**

```python
def test_dry_run_plans_one_step_without_sending_robot_command():
    robot, dcss, kine = FakeRobot(), FakeDcss(), FakeKine()
    poses = run_jog_session(JogConfig(), iter(["w", "q"]).__next__, lambda: (robot, dcss, kine))
    assert np.array_equal(poses[-1][:3], np.array([2., 0., 0.]))
    assert robot.planned_commands == []
```

- [ ] **Step 2: Verify it fails for the missing runner**

Run: `pytest tests/hardware/test_keyboard_cartesian_jog.py::test_dry_run_plans_one_step_without_sending_robot_command -q`

Expected: FAIL with missing `run_jog_session`.

- [ ] **Step 3: Implement MOVLA planning with dry-run command suppression**

```python
def plan_or_execute_step(robot, dcss, kine, arm_index, joints, current_pose, target_pose, config):
    points, pset = kine.movLA(start_xyzabc=current_pose.tolist(), end_xyzabc=target_pose.tolist(), ref_joints=list(joints), vel=10.0, acc=100.0, freq_hz=100)
    if pset is None:
        raise RuntimeError("MOVLA planning failed")
    if not config.execute:
        return target_pose
    robot.setPln_Cart(arm=config.arm, pset=pset)
    _wait_until_traj_idle(robot, dcss, arm_index)
    return _read_feedback_pose(robot, dcss, kine, arm_index)
```

`run_jog_session` connects, confirms fresh feedback, converts current joints to TCP pose, maps keys, and always calls cleanup in `finally`.

- [ ] **Step 4: Verify the dry-run test passes**

Run: `pytest tests/hardware/test_keyboard_cartesian_jog.py::test_dry_run_plans_one_step_without_sending_robot_command -q`

Expected: PASS.

- [ ] **Step 5: Write failing execute refusal tests**

```python
def test_execute_outside_workspace_does_not_plan_or_send():
    cfg = JogConfig(execute=True, workspace_min=(0., 0., 0.), workspace_max=(1., 1., 1.))
    robot, dcss, kine = FakeRobot(), FakeDcss(), FakeKine()
    with pytest.raises(ValueError, match="outside workspace"):
        run_jog_session(cfg, iter(["w"]).__next__, lambda: (robot, dcss, kine))
    assert robot.planned_commands == []

def test_execute_planning_failure_does_not_send_command():
    cfg = JogConfig(execute=True, workspace_min=(-5., -5., -5.), workspace_max=(5., 5., 5.))
    robot, dcss, kine = FakeRobot(), FakeDcss(), FailingKine()
    with pytest.raises(RuntimeError, match="MOVLA planning failed"):
        run_jog_session(cfg, iter(["w"]).__next__, lambda: (robot, dcss, kine))
    assert robot.planned_commands == []
```

- [ ] **Step 6: Verify the refusal tests fail before guards are added**

Run: `pytest tests/hardware/test_keyboard_cartesian_jog.py -q`

Expected: FAIL because the request is not refused before planning/sending.

- [ ] **Step 7: Add explicit refusal and cleanup guards**

```python
if not inside_workspace(target_pose[:3], config.workspace_min, config.workspace_max):
    raise ValueError(f"requested pose is outside workspace: {target_pose[:3].tolist()}")
if not _feedback_is_fresh(robot, dcss, arm_index):
    raise RuntimeError("robot feedback frame did not update")
if not _trajectory_is_idle(robot, dcss, arm_index):
    raise RuntimeError("selected arm trajectory is not idle")
```

In `finally`, call `robot.clear_set()`, `robot.set_state(arm=config.arm, state=0)`, and the SDK send method unless `keep_enabled`; always call `robot.release_robot()`.

- [ ] **Step 8: Verify all runner tests pass and commit**

Run: `pytest tests/hardware/test_keyboard_cartesian_jog.py -q`

Expected: PASS with no network connection.

```bash
git add real_robot_debug/keyboard_cartesian_jog.py tests/hardware/test_keyboard_cartesian_jog.py && git commit -m "feat: add guarded keyboard cartesian jog runner"
```

### Task 3: CLI, raw terminal keys, documentation, and final verification

**Files:**
- Modify: `real_robot_debug/keyboard_cartesian_jog.py`
- Modify: `real_robot_debug/README.md`
- Modify: `tests/hardware/test_keyboard_cartesian_jog.py`

**Interfaces:**
- Produces `parse_args(argv: list[str] | None = None) -> JogConfig`, `raw_terminal_keys(stream) -> Iterator[str]`, and `main(argv: list[str] | None = None) -> int`.

- [ ] **Step 1: Write failing parser and emergency-key tests**

```python
def test_parse_execute_requires_workspace_values():
    with pytest.raises(ValueError, match="workspace-min.*workspace-max"):
        parse_args(["--execute"])

def test_space_stops_before_later_motion_key():
    robot, dcss, kine = FakeRobot(), FakeDcss(), FakeKine()
    run_jog_session(JogConfig(), iter([" ", "w"]).__next__, lambda: (robot, dcss, kine))
    assert robot.disabled
    assert robot.planned_commands == []
```

- [ ] **Step 2: Verify the CLI tests fail before implementation**

Run: `pytest tests/hardware/test_keyboard_cartesian_jog.py -q`

Expected: FAIL for absent parser or post-space key processing.

- [ ] **Step 3: Implement CLI and terminal lifecycle**

```python
parser.add_argument("--workspace-min", type=lambda text: parse_xyz(text, "--workspace-min"))
parser.add_argument("--workspace-max", type=lambda text: parse_xyz(text, "--workspace-max"))
parser.add_argument("--execute", action="store_true")
parser.add_argument("--step-mm", type=float, default=2.0)

@contextmanager
def raw_terminal_keys(stream=sys.stdin):
    previous = termios.tcgetattr(stream.fileno())
    try:
        tty.setraw(stream.fileno())
        yield iter(lambda: stream.read(1), "")
    finally:
        termios.tcsetattr(stream.fileno(), termios.TCSADRAIN, previous)
```

`Q` and Space terminate immediately. `main` prints arm, base frame, key mapping, step, dry-run/execute status, and a physical-E-stop warning before raw mode.

- [ ] **Step 4: Verify CLI tests pass**

Run: `pytest tests/hardware/test_keyboard_cartesian_jog.py -q`

Expected: PASS.

- [ ] **Step 5: Document commands that cannot accidentally be executed verbatim**

Add to `real_robot_debug/README.md`:

```bash
# Keyboard UI only; no motion is sent.
PYTHONPATH=. python3 real_robot_debug/keyboard_cartesian_jog.py --arm A

# Replace all six bounds with the current workcell's verified millimetre values.
PYTHONPATH=. python3 real_robot_debug/keyboard_cartesian_jog.py --arm A --execute --step-mm 2 \
  --workspace-min "XMIN,YMIN,ZMIN" --workspace-max "XMAX,YMAX,ZMAX"
```

State that bounds are mandatory site-specific values and Space is software disable, not a physical E-stop.

- [ ] **Step 6: Run final static and regression checks**

Run: `python3 -m py_compile real_robot_debug/keyboard_cartesian_jog.py && pytest tests/hardware/test_keyboard_cartesian_jog.py tests/hardware/test_a_arm_impedance_two_stage.py -q`

Expected: PASS, no physical connection.

- [ ] **Step 7: Inspect safety paths, then commit**

Run: `git diff develop_13_kinematic_branch...HEAD -- real_robot_debug/keyboard_cartesian_jog.py real_robot_debug/README.md tests/hardware/test_keyboard_cartesian_jog.py`

Expected: every `setPln_Cart` is dominated by `config.execute`, workspace checks precede planning, and all exits clean up.

```bash
git add real_robot_debug/keyboard_cartesian_jog.py real_robot_debug/README.md tests/hardware/test_keyboard_cartesian_jog.py && git commit -m "docs: document keyboard cartesian jog"
```
