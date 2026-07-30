# MuJoCo Position-Control Simulation Rebuild Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the legacy torque/impedance simulation with a Linux-only MuJoCo-native position-actuator simulator that supports model viewing, joint motion, Cartesian IK motion, one chopping cycle, and contact-force logging.

**Architecture:** A new `twin_sim` package owns model loading, a seven-joint right-arm view, SI-only MuJoCo kinematics, trajectory generation, force monitoring, CSV logging, and task orchestration. Cartesian paths are fully sampled and solved by continuous IK before execution; MuJoCo position actuators track the resulting joint targets. Legacy simulation code is archived and excluded from packaging/tests, while real-robot SDK files remain byte-identical.

**Tech Stack:** CPython 3.12, MuJoCo, NumPy, pytest, setuptools, XML/MJCF, project-local `.venv`

## Global Constraints

- Support Linux simulation only; do not connect to or run a real robot.
- Use CPython 3.12 and a repository-local `.venv`.
- Use SI units only inside `twin_sim`: radians, metres, seconds, and Newtons.
- Do not implement or expose joint impedance, Cartesian impedance, force control, or admittance control.
- A force threshold emits a warning and CSV flag but does not alter or stop the trajectory.
- NaN or infinite MuJoCo state stops execution immediately.
- Preserve `SDK_PYTHON/`, `test/`, `real_robot_debug/`, vendor binaries, configuration files, and their contents.
- Keep one copy of models, meshes, SDK binaries, and other large assets.
- Archive replaced simulation source, examples, tests, and documentation under `archive/legacy_simulation/`; archived Python is neither installed nor collected by pytest.
- Do not preserve compatibility with old simulation APIs, replay, or the full trajectory demo set.
- Use test-first implementation and commit after every task.

---

## Target File Map

### New runtime package

- `src/twin_sim/__init__.py`: small public API export list.
- `src/twin_sim/names.py`: immutable joint, actuator, site, geom, and sensor names.
- `src/twin_sim/paths.py`: repository and MJCF path resolution.
- `src/twin_sim/model.py`: model validation and `SimulationModel`.
- `src/twin_sim/robot.py`: `RightArmRobot`, position commands, stepping, and viewer lifecycle.
- `src/twin_sim/kinematics.py`: SI-only FK/Jacobian/DLS IK.
- `src/twin_sim/trajectory.py`: minimum-jerk joint and Cartesian-to-joint paths.
- `src/twin_sim/force_monitor.py`: raw/filtered force samples and warning threshold.
- `src/twin_sim/logging.py`: typed samples and CSV writer.
- `src/twin_sim/tasks/chop.py`: one-cycle chopping state machine.
- `src/twin_sim/cli.py`: `view`, `joint`, `cartesian`, and `chop` subcommands.

### Assets, tests, and docs

- `robot_assets/mujoco/right_chopping_scene.xml`: active position-actuator scene.
- `robot_assets/mujoco/meshes/*.obj`: active cleaver meshes.
- `MarvinCCS/`: remains the single source of Marvin arm meshes.
- `tests/simulation/`: only new simulator tests.
- `examples/simulation_demo.py`: minimal Python API example.
- `docs/simulation/{setup,architecture,usage,migration}.md`: new user documentation.
- `archive/legacy_simulation/README.md`: archive provenance and restoration notes.
- `requirements-sim.lock`: exact direct dependency pins.
- `.python-version`: `3.12`.

---

### Task 1: Reproducible Environment and Legacy Baseline

**Files:**
- Modify: `.gitignore`
- Create: `.python-version`
- Create: `requirements-sim.lock`
- Create: `docs/simulation/setup.md`
- Create: `docs/simulation/baseline.md`

**Interfaces:**
- Consumes: existing setuptools project and current test suite.
- Produces: `python3.12 -m venv .venv`, exact dependency installation, and a recorded pre-refactor baseline.

- [ ] **Step 1: Record protected-file hashes and the current baseline**

Run:

```bash
mkdir -p /tmp/tianji-rebuild
find SDK_PYTHON test real_robot_debug -type f -print0 | sort -z | xargs -0 sha256sum > /tmp/tianji-rebuild/protected-before.sha256
python3.12 -m pytest -q > /tmp/tianji-rebuild/legacy-pytest.txt
```

Expected: the hash manifest is non-empty; pytest output is captured even if legacy tests have known failures. Record the exact pass/fail counts and failing node IDs in `docs/simulation/baseline.md`.

- [ ] **Step 2: Add environment metadata**

Create `.python-version`:

```text
3.12
```

Create `requirements-sim.lock` with the current CPython 3.12 Linux releases verified on PyPI:

```text
mujoco==3.10.0
numpy==2.5.1
pytest==9.1.1
```

- [ ] **Step 3: Create and verify the clean environment**

Run:

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-sim.lock
.venv/bin/python -m pip install -e .
.venv/bin/python -c "import mujoco, numpy; print(mujoco.__version__, numpy.__version__)"
```

Expected: every command exits 0 and prints the pinned MuJoCo and NumPy versions.

- [ ] **Step 4: Update ignore rules**

Add to `.gitignore`:

```gitignore
.venv/
*.csv
logs/
```

- [ ] **Step 5: Verify configuration**

Run:

```bash
.venv/bin/python -m pip install -e .
.venv/bin/python -m pytest --collect-only -q
```

Expected: installation succeeds and pytest reports the existing legacy collection count recorded in `baseline.md`.

- [ ] **Step 6: Commit**

```bash
git add .gitignore .python-version requirements-sim.lock docs/simulation/setup.md docs/simulation/baseline.md
git commit -m "build: establish reproducible simulation environment"
```

---

### Task 2: Archive Legacy Simulation Without Touching Real-Robot Files

**Files:**
- Create: `archive/legacy_simulation/README.md`
- Move: `src/twin_control/` → `archive/legacy_simulation/src/twin_control/`
- Move: `src/twin_core/` → `archive/legacy_simulation/src/twin_core/`
- Move: `src/twin_mujoco/` → `archive/legacy_simulation/src/twin_mujoco/`
- Move: `src/twin_description/` → `archive/legacy_simulation/src/twin_description/`
- Move: existing simulation files under `examples/` → `archive/legacy_simulation/examples/`
- Move: existing `tests/*.py` → `archive/legacy_simulation/tests/`
- Move: old simulation plans/specs except the 2026-07-30 design and this plan → `archive/legacy_simulation/docs/`

**Interfaces:**
- Consumes: clean environment and `/tmp/tianji-rebuild/protected-before.sha256`.
- Produces: a non-importable historical snapshot and an empty active simulator surface ready for `twin_sim`.

- [ ] **Step 1: Write an archive-boundary test**

Create `tests/simulation/test_repository_boundaries.py`:

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

def test_legacy_simulation_is_archived():
    archive = ROOT / "archive" / "legacy_simulation"
    assert (archive / "src" / "twin_control").is_dir()
    assert not (ROOT / "src" / "twin_control").exists()

def test_pytest_does_not_collect_archive():
    assert "archive" not in {part for path in (ROOT / "tests").rglob("*.py") for part in path.parts}
```

- [ ] **Step 2: Run it and verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_repository_boundaries.py -v`

Expected: FAIL because `archive/legacy_simulation/src/twin_control` does not exist.

- [ ] **Step 3: Move files with Git-aware operations**

Use explicit `git mv` commands for the paths listed above. Do not move `SDK_PYTHON/`, `test/`, `real_robot_debug/`, `MarvinCCS/`, or `MarvinCCS_mujoco.zip`. Write `archive/legacy_simulation/README.md` with commit `a8cde84` as the design baseline and the actual pre-archive `git rev-parse HEAD` as the source snapshot.

- [ ] **Step 4: Verify archive isolation and protected hashes**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_repository_boundaries.py -v
sha256sum --check /tmp/tianji-rebuild/protected-before.sha256
```

Expected: test PASS and every protected file reports `OK`.

- [ ] **Step 5: Commit**

```bash
git add archive tests/simulation
git add -u src examples tests docs
git commit -m "refactor: archive legacy simulation"
```

---

### Task 3: Position-Actuator Scene and Model Validation

**Files:**
- Create: `robot_assets/mujoco/right_chopping_scene.xml`
- Create: `robot_assets/mujoco/meshes/right_cleaver.obj`
- Create: `robot_assets/mujoco/meshes/right_cleaver_handle.obj`
- Create: `src/twin_sim/__init__.py`
- Create: `src/twin_sim/names.py`
- Create: `src/twin_sim/paths.py`
- Create: `src/twin_sim/model.py`
- Modify: `pyproject.toml`
- Test: `tests/simulation/test_model.py`

**Interfaces:**
- Produces: `scene_path() -> Path`, `SimulationModel.load(path: Path | None = None) -> SimulationModel`, and validated right-arm index arrays.

- [ ] **Step 1: Write failing model tests**

```python
import mujoco
import pytest
from twin_sim.model import ModelValidationError, SimulationModel

def test_active_scene_has_fourteen_position_actuators():
    sim = SimulationModel.load()
    assert sim.model.njnt == 14
    assert sim.model.nu == 14
    assert sim.right.actuator_ids.shape == (7,)
    assert all(sim.model.actuator_biastype[i] == mujoco.mjtBias.mjBIAS_AFFINE
               for i in sim.right.actuator_ids)

def test_active_scene_has_task_objects():
    sim = SimulationModel.load()
    assert sim.require_site("right_tool_tip_site") >= 0
    assert sim.require_geom("chopping_board") >= 0
    assert sim.require_sensor("right_tool_force") >= 0

def test_missing_model_object_has_clear_error():
    sim = SimulationModel.load()
    with pytest.raises(ModelValidationError, match="missing site: absent"):
        sim.require_site("absent")
```

- [ ] **Step 2: Run tests and verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_model.py -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'twin_sim'`.

- [ ] **Step 3: Create the active scene**

Copy the archived chopping scene and cleaver OBJ files into `robot_assets/mujoco/`. Update Marvin mesh paths to `../../MarvinCCS/meshes/...`. Replace every actuator `<motor .../>` with a position actuator:

```xml
<position name="act_right_joint1" joint="right_joint1"
          kp="80" kv="18" ctrllimited="true" ctrlrange="-3.11 3.11"
          forcelimited="true" forcerange="-108 108"/>
```

Use joint ranges as `ctrlrange`, joint effort ranges as `forcerange`, and initial `kp/kv` vectors `(80,80,60,50,30,20,15)` / `(18,18,14,12,8,6,5)` for both arms. These are initial model constants, not a runtime API.

- [ ] **Step 4: Implement names, paths, and validation**

Define:

```python
@dataclass(frozen=True)
class ArmNames:
    joints: tuple[str, ...]
    actuators: tuple[str, ...]

@dataclass(frozen=True)
class ArmIndices:
    joint_ids: np.ndarray
    qpos_ids: np.ndarray
    dof_ids: np.ndarray
    actuator_ids: np.ndarray

class ModelValidationError(RuntimeError):
    pass

class SimulationModel:
    @classmethod
    def load(cls, path: Path | None = None) -> "SimulationModel": ...
    def require_site(self, name: str) -> int: ...
    def require_geom(self, name: str) -> int: ...
    def require_sensor(self, name: str) -> int: ...
```

`SimulationModel.load()` must call `mj_forward`, validate all 14 joint/actuator names plus `right_tool_tip_site`, `right_tool_force`, `right_tool_torque`, `right_knife_blade`, and `chopping_board`, and expose `left` and `right` `ArmIndices`.

- [ ] **Step 5: Switch packaging to the new simulator**

Replace the legacy package and script tables in `pyproject.toml` with:

```toml
[project.scripts]
twin-sim = "twin_sim.cli:main"

[tool.setuptools]
packages = ["twin_sim", "twin_sim.tasks"]

[tool.setuptools.package-dir]
twin_sim = "src/twin_sim"
"twin_sim.tasks" = "src/twin_sim/tasks"

[tool.pytest.ini_options]
testpaths = ["tests/simulation"]
pythonpath = ["src"]
```

- [ ] **Step 6: Run tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_model.py -v`

Expected: all tests PASS.

- [ ] **Step 7: Commit**

```bash
git add robot_assets src/twin_sim tests/simulation/test_model.py pyproject.toml
git commit -m "feat: add validated position-actuator scene"
```

---

### Task 4: Right-Arm Position Runtime

**Files:**
- Create: `src/twin_sim/robot.py`
- Test: `tests/simulation/test_robot.py`

**Interfaces:**
- Consumes: `SimulationModel` and `ArmIndices`.
- Produces: `RightArmRobot.reset(q)`, `command(q)`, `step(control_dt_s)`, `joint_positions`, `joint_velocities`, `tcp_pose()`, and `close()`.

- [ ] **Step 1: Write failing runtime tests**

```python
import numpy as np
import pytest
from twin_sim.robot import RIGHT_HOME_RAD, NumericalSafetyError, RightArmRobot

def test_position_target_is_tracked():
    robot = RightArmRobot()
    robot.reset(RIGHT_HOME_RAD)
    target = RIGHT_HOME_RAD + np.array([0.02, 0, 0, 0, 0, 0, 0])
    robot.command(target)
    for _ in range(250):
        robot.step(0.002)
    assert np.linalg.norm(robot.joint_positions - target) < 0.03

def test_invalid_command_is_rejected():
    robot = RightArmRobot()
    with pytest.raises(ValueError, match="7 finite"):
        robot.command([float("nan")] * 7)

def test_nonfinite_state_stops():
    robot = RightArmRobot()
    robot.sim.data.qpos[robot.sim.right.qpos_ids[0]] = np.nan
    with pytest.raises(NumericalSafetyError):
        robot.step(0.002)
```

- [ ] **Step 2: Verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_robot.py -v`

Expected: FAIL because `twin_sim.robot` does not exist.

- [ ] **Step 3: Implement the minimal runtime**

Implement:

```python
RIGHT_HOME_RAD = np.array((0.4, -1.3, 0.08, -1.606525, 0.057176, 0.79256, 1.5))

class NumericalSafetyError(RuntimeError):
    pass

class RightArmRobot:
    def __init__(self, model_path: Path | None = None, viewer: bool = False): ...
    def reset(self, joints_rad: Sequence[float] = RIGHT_HOME_RAD) -> None: ...
    def command(self, joints_rad: Sequence[float]) -> None: ...
    def step(self, control_dt_s: float) -> None: ...
    @property
    def joint_positions(self) -> np.ndarray: ...
    @property
    def joint_velocities(self) -> np.ndarray: ...
    def tcp_pose(self) -> np.ndarray: ...
    def close(self) -> None: ...
```

`step()` must require `control_dt_s` to be a positive integer multiple of the MJCF timestep, write only right-arm controls, hold left-arm controls at reset values, call `mj_step` for the required substeps, then verify `qpos`, `qvel`, and `ctrl` are finite.

- [ ] **Step 4: Run tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_robot.py -v`

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/robot.py tests/simulation/test_robot.py
git commit -m "feat: add right-arm position runtime"
```

---

### Task 5: SI-Only Kinematics and Continuous IK

**Files:**
- Create: `src/twin_sim/kinematics.py`
- Test: `tests/simulation/test_kinematics.py`

**Interfaces:**
- Consumes: `SimulationModel`, right-arm index arrays, and TCP site.
- Produces: `IkResult`, `Kinematics.fk(q)`, `jacobian(q)`, `ik(target, reference)`, and `solve_path(poses, seed)`.

- [ ] **Step 1: Write failing kinematics tests**

```python
import numpy as np
import pytest
from twin_sim.kinematics import Kinematics, PathIkError
from twin_sim.robot import RIGHT_HOME_RAD, RightArmRobot

def test_fk_ik_roundtrip():
    robot = RightArmRobot()
    kin = Kinematics(robot.sim)
    target = kin.fk(RIGHT_HOME_RAD)
    result = kin.ik(target, RIGHT_HOME_RAD)
    assert result.success
    assert np.allclose(kin.fk(result.joints_rad), target, atol=1e-4)

def test_unreachable_path_fails_before_execution():
    robot = RightArmRobot()
    kin = Kinematics(robot.sim)
    pose = np.eye(4)
    pose[:3, 3] = (10, 10, 10)
    with pytest.raises(PathIkError, match="sample 0"):
        kin.solve_path([pose], RIGHT_HOME_RAD)
```

- [ ] **Step 2: Verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_kinematics.py -v`

Expected: FAIL because `twin_sim.kinematics` does not exist.

- [ ] **Step 3: Implement DLS IK**

Define:

```python
@dataclass(frozen=True)
class IkResult:
    joints_rad: np.ndarray
    success: bool
    iterations: int
    residual: float

class PathIkError(RuntimeError):
    pass

class Kinematics:
    def fk(self, joints_rad: Sequence[float]) -> np.ndarray: ...
    def jacobian(self, joints_rad: Sequence[float]) -> np.ndarray: ...
    def ik(self, target: np.ndarray, reference: Sequence[float],
           *, max_iterations: int = 200, tolerance: float = 1e-5,
           damping: float = 1e-3) -> IkResult: ...
    def solve_path(self, poses: Sequence[np.ndarray], seed: Sequence[float],
                   *, max_joint_step_rad: float = 0.15) -> np.ndarray: ...
```

Use MuJoCo site FK/Jacobian, restore `MjData` after every query, use translation plus rotation-vector error, clip every iteration to MJCF joint limits, seed each path sample from the previous solution, and raise `PathIkError` with the failing sample index or joint-step violation.

- [ ] **Step 4: Run tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_kinematics.py -v`

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/kinematics.py tests/simulation/test_kinematics.py
git commit -m "feat: add continuous SI-only inverse kinematics"
```

---

### Task 6: Smooth Joint and Cartesian Trajectories

**Files:**
- Create: `src/twin_sim/trajectory.py`
- Test: `tests/simulation/test_trajectory.py`

**Interfaces:**
- Consumes: `Kinematics.solve_path`.
- Produces: `TrajectoryPoint`, `joint_trajectory()`, `cartesian_trajectory()`.

- [ ] **Step 1: Write failing trajectory tests**

```python
import numpy as np
from twin_sim.trajectory import joint_trajectory

def test_joint_trajectory_has_exact_endpoints_and_monotonic_time():
    start, goal = np.zeros(7), np.ones(7) * 0.1
    points = joint_trajectory(start, goal, duration_s=1.0, control_dt_s=0.01)
    assert np.allclose(points[0].joints_rad, start)
    assert np.allclose(points[-1].joints_rad, goal)
    assert all(a.time_s < b.time_s for a, b in zip(points, points[1:]))

def test_joint_trajectory_starts_and_ends_at_rest():
    points = joint_trajectory(np.zeros(7), np.ones(7) * 0.1, 1.0, 0.01)
    assert np.allclose(points[0].velocity_rad_s, 0)
    assert np.allclose(points[-1].velocity_rad_s, 0)
```

- [ ] **Step 2: Verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_trajectory.py -v`

Expected: FAIL because `twin_sim.trajectory` does not exist.

- [ ] **Step 3: Implement trajectory generation**

Define:

```python
@dataclass(frozen=True)
class TrajectoryPoint:
    time_s: float
    joints_rad: np.ndarray
    velocity_rad_s: np.ndarray
    target_pose: np.ndarray | None

def minimum_jerk(s: np.ndarray) -> tuple[np.ndarray, np.ndarray]: ...
def joint_trajectory(start, goal, duration_s, control_dt_s) -> list[TrajectoryPoint]: ...
def cartesian_trajectory(kinematics, start_pose, goal_pose, seed,
                         duration_s, control_dt_s) -> list[TrajectoryPoint]: ...
```

Use `10s³-15s⁴+6s⁵`, include exact endpoints, linearly interpolate translation, slerp orientation, solve the complete pose list before returning, and reject non-positive duration/dt or non-integral duration/dt ratios.

- [ ] **Step 4: Run tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_trajectory.py -v`

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/trajectory.py tests/simulation/test_trajectory.py
git commit -m "feat: add smooth IK-backed trajectories"
```

---

### Task 7: Force Monitoring and CSV Telemetry

**Files:**
- Create: `src/twin_sim/force_monitor.py`
- Create: `src/twin_sim/logging.py`
- Test: `tests/simulation/test_force_logging.py`

**Interfaces:**
- Produces: `ForceMonitor.sample() -> ForceSample`, `SimulationSample`, and `write_csv(path, samples)`.

- [ ] **Step 1: Write failing force/log tests**

```python
import csv
import warnings
from twin_sim.force_monitor import ForceMonitor
from twin_sim.logging import SimulationSample, write_csv

def test_threshold_warns_without_raising():
    monitor = ForceMonitor(alpha=1.0, warning_threshold_n=5.0)
    with warnings.catch_warnings(record=True) as seen:
        sample = monitor.update(raw_force_n=6.0)
    assert sample.over_threshold is True
    assert len(seen) == 1

def test_csv_contains_force_and_phase(tmp_path):
    path = tmp_path / "run.csv"
    write_csv(path, [SimulationSample.example()])
    row = next(csv.DictReader(path.open()))
    assert row["phase"] == "TEST"
    assert "filtered_force_n" in row
    assert "force_over_threshold" in row
```

- [ ] **Step 2: Verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_force_logging.py -v`

Expected: FAIL because the modules do not exist.

- [ ] **Step 3: Implement monitoring and logging**

Define:

```python
@dataclass(frozen=True)
class ForceSample:
    raw_force_n: float
    filtered_force_n: float
    over_threshold: bool

class ForceMonitor:
    def __init__(self, alpha: float, warning_threshold_n: float): ...
    def update(self, raw_force_n: float) -> ForceSample: ...

@dataclass(frozen=True)
class SimulationSample:
    time_s: float
    phase: str
    target_joints_rad: np.ndarray
    actual_joints_rad: np.ndarray
    target_pose: np.ndarray
    actual_pose: np.ndarray
    raw_force_n: float
    filtered_force_n: float
    force_over_threshold: bool
```

Read `right_tool_force` through MuJoCo sensor address/dimension, use the force-vector norm as the observed scalar, apply `filtered = alpha*raw + (1-alpha)*previous`, issue `warnings.warn(..., RuntimeWarning)` only on transition into over-threshold state, and validate the CSV parent before motion starts.

- [ ] **Step 4: Run tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_force_logging.py -v`

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/twin_sim/force_monitor.py src/twin_sim/logging.py tests/simulation/test_force_logging.py
git commit -m "feat: add observational force telemetry"
```

---

### Task 8: One-Cycle Chopping Task and Stable CLI

**Files:**
- Create: `src/twin_sim/tasks/__init__.py`
- Create: `src/twin_sim/tasks/chop.py`
- Create: `src/twin_sim/cli.py`
- Create: `examples/simulation_demo.py`
- Test: `tests/simulation/test_chop.py`
- Test: `tests/simulation/test_cli.py`

**Interfaces:**
- Consumes: robot, kinematics, trajectories, force monitor, and CSV writer.
- Produces: `ChopConfig`, `ChopResult`, `run_chop(config)`, and `twin-sim` CLI.

- [ ] **Step 1: Write failing chopping tests**

```python
import csv
from twin_sim.tasks.chop import ChopConfig, run_chop

def test_one_chop_completes_and_logs_all_phases(tmp_path):
    path = tmp_path / "chop.csv"
    result = run_chop(ChopConfig(control_dt_s=0.01), log_path=path)
    assert result.completed
    phases = {row["phase"] for row in csv.DictReader(path.open())}
    assert phases >= {"APPROACH", "DESCEND", "HOLD", "RETRACT", "COMPLETE"}
    assert result.final_tip_z_m >= result.safe_tip_z_m - 0.005

def test_force_warning_does_not_cancel_task(tmp_path):
    result = run_chop(
        ChopConfig(control_dt_s=0.01, force_warning_threshold_n=0.0),
        log_path=tmp_path / "warning.csv",
    )
    assert result.completed
    assert result.warning_count > 0
```

- [ ] **Step 2: Verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_chop.py -v`

Expected: FAIL because `twin_sim.tasks.chop` does not exist.

- [ ] **Step 3: Implement the task**

Define:

```python
@dataclass(frozen=True)
class ChopConfig:
    control_dt_s: float = 0.01
    approach_duration_s: float = 1.0
    descent_duration_s: float = 1.0
    hold_duration_s: float = 0.2
    retract_duration_s: float = 1.0
    penetration_m: float = 0.003
    force_filter_alpha: float = 0.2
    force_warning_threshold_n: float = 30.0

@dataclass(frozen=True)
class ChopResult:
    completed: bool
    samples: tuple[SimulationSample, ...]
    warning_count: int
    final_tip_z_m: float
    safe_tip_z_m: float

def run_chop(config: ChopConfig, *, log_path: Path,
             viewer: bool = False) -> ChopResult: ...
```

Derive board top from `chopping_board` geom position/size, derive blade bottom from `right_blade_edge_bot`, construct fixed-orientation Cartesian targets, solve every phase before executing the first phase, command each position point, sample after each step, and always close the viewer in `finally`.

- [ ] **Step 4: Add CLI tests and commands**

Test:

```python
from twin_sim.cli import main

def test_headless_chop_cli(tmp_path):
    path = tmp_path / "cli.csv"
    assert main(["chop", "--headless", "--log", str(path)]) == 0
    assert path.is_file()
```

Implement subcommands:

```text
twin-sim view
twin-sim joint --joint 1 --delta-rad 0.05 --headless
twin-sim cartesian --dz-m 0.02 --headless
twin-sim chop --headless --log logs/chop.csv
```

- [ ] **Step 5: Run task and CLI tests**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_chop.py tests/simulation/test_cli.py -v
.venv/bin/twin-sim joint --joint 1 --delta-rad 0.02 --headless
.venv/bin/twin-sim cartesian --dz-m 0.01 --headless
.venv/bin/twin-sim chop --headless --log /tmp/twin-sim-chop.csv
```

Expected: all tests PASS, all commands exit 0, and the chop CSV exists.

- [ ] **Step 6: Commit**

```bash
git add src/twin_sim/tasks src/twin_sim/cli.py examples/simulation_demo.py tests/simulation/test_chop.py tests/simulation/test_cli.py
git commit -m "feat: add position-controlled chopping workflow"
```

---

### Task 9: Documentation, Isolation, and Final Acceptance

**Files:**
- Modify: `README.md`
- Create: `docs/simulation/architecture.md`
- Create: `docs/simulation/usage.md`
- Create: `docs/simulation/migration.md`
- Modify: `tests/simulation/test_repository_boundaries.py`

**Interfaces:**
- Consumes: all completed simulator interfaces.
- Produces: final user documentation and repository-wide acceptance evidence.

- [ ] **Step 1: Strengthen isolation tests**

Add:

```python
import ast

def test_new_simulator_does_not_import_real_sdk():
    forbidden = {"SDK_PYTHON", "fx_robot", "fx_kine"}
    for path in (ROOT / "src" / "twin_sim").rglob("*.py"):
        tree = ast.parse(path.read_text())
        imported = {
            node.names[0].name.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
        } | {
            (node.module or "").split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
        }
        assert imported.isdisjoint(forbidden), path
```

- [ ] **Step 2: Write concise active documentation**

Document exact `.venv` commands, the four CLI commands, module boundaries, SI units, position-actuator semantics, warning-only force threshold, archived mappings, deferred admittance control, and the fact that real-robot code was not validated.

- [ ] **Step 3: Run complete verification**

Run:

```bash
.venv/bin/python -m pip install -e .
.venv/bin/python -m pytest -v
.venv/bin/twin-sim joint --joint 1 --delta-rad 0.02 --headless
.venv/bin/twin-sim cartesian --dz-m 0.01 --headless
.venv/bin/twin-sim chop --headless --log /tmp/twin-sim-final.csv
test -s /tmp/twin-sim-final.csv
sha256sum --check /tmp/tianji-rebuild/protected-before.sha256
git diff --check
```

Expected: all tests PASS; three smoke commands exit 0; CSV is non-empty; every protected file reports `OK`; `git diff --check` is silent.

- [ ] **Step 4: Check the force-trend acceptance rule**

Run a short Python reader over `/tmp/twin-sim-final.csv` that computes:

```python
baseline = median(filtered_force_n for phase == "APPROACH")
contact = median(filtered_force_n for second half of phase == "DESCEND")
lifted = median(filtered_force_n for second half of phase == "RETRACT")
assert contact > baseline
assert lifted < contact
```

Expected: both assertions pass. If they fail, adjust only MJCF contact/position-actuator constants or chopping depth and add a regression test; do not add an impedance controller.

- [ ] **Step 5: Commit**

```bash
git add README.md docs/simulation tests/simulation/test_repository_boundaries.py
git commit -m "docs: document rebuilt MuJoCo simulator"
```

- [ ] **Step 6: Record final evidence**

Run:

```bash
git status --short
git log --oneline -10
```

Expected: clean working tree and one focused commit per task.
