# Line Chop Trajectory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a five-cut line-chopping task with 3 cm blade-direction shifts, indexed CSV samples, and a dependency-free SVG trajectory plot.

**Architecture:** Reuse the calibrated single-chop geometry and position-control runtime, but preflight every descent, retract, and lateral shift before execution. A focused SVG renderer consumes immutable simulation samples and renders target/actual top and side views without adding plotting dependencies.

**Tech Stack:** CPython 3.12, MuJoCo 3.10, NumPy, standard-library XML/text output, pytest

## Global Constraints

- Default to 5 cuts spaced `0.03 m` along the blade XY projection.
- Preflight all Cartesian segments before commanding the robot.
- Keep the calibrated knife pose and native position actuators.
- Do not modify real-robot or vendor files.
- Add no runtime dependencies.
- Use test-first implementation.

---

### Task 1: Indexed Simulation Samples

**Files:**
- Modify: `src/twin_sim/logging.py`
- Modify: `src/twin_sim/tasks/chop.py`
- Test: `tests/simulation/test_force_logging.py`

**Interfaces:**
- Produces: `SimulationSample.cut_index: int = 0` and a `cut_index` CSV column.

- [ ] Write a failing CSV test asserting `cut_index == "3"`.
- [ ] Run `.venv/bin/python -m pytest tests/simulation/test_force_logging.py -q` and verify failure.
- [ ] Add the backward-compatible default field, validation for non-negative integers, and CSV serialization.
- [ ] Run the focused tests and commit.

### Task 2: Multi-Cut Preflight and Runtime

**Files:**
- Create: `src/twin_sim/tasks/line_chop.py`
- Modify: `src/twin_sim/tasks/__init__.py`
- Test: `tests/simulation/test_line_chop.py`

**Interfaces:**
- Produces: `LineChopConfig`, `LineChopResult`, `run_line_chop(...)`, and `_preflight_line_chop(...)`.
- Consumes: `ChopConfig`, `CHOP_READY_RAD`, `RightArmRobot`, `Kinematics`, `cartesian_trajectory`.

- [ ] Write failing configuration, phase-count, 3 cm spacing, board-boundary, and final-no-shift tests.
- [ ] Verify failures because `line_chop` does not exist.
- [ ] Generate safe/contact poses for all cuts from the blade-direction XY projection.
- [ ] Preflight all paths and validate every target before resetting to the first safe pose.
- [ ] Execute `READY`, five `DESCEND/HOLD/RETRACT`, four `SHIFT`, and `COMPLETE` phases with `cut_index`.
- [ ] Run focused tests and commit.

### Task 3: Dependency-Free SVG Plot

**Files:**
- Create: `src/twin_sim/trajectory_plot.py`
- Test: `tests/simulation/test_trajectory_plot.py`

**Interfaces:**
- Produces: `write_trajectory_svg(path: Path, samples: Sequence[SimulationSample]) -> None`.

- [ ] Write failing tests for destination validation and required SVG labels/paths.
- [ ] Verify failure because the renderer does not exist.
- [ ] Implement escaped SVG with XY top view, progress-Z side view, target/actual paths, numbered cut points, axes, and legend.
- [ ] Run focused tests and commit.

### Task 4: CLI Integration and Acceptance

**Files:**
- Modify: `src/twin_sim/cli.py`
- Modify: `docs/simulation/usage.md`
- Test: `tests/simulation/test_cli.py`

**Interfaces:**
- Produces: `twin-sim line-chop --cuts --spacing-m --log --plot [--slow] [--headless]`.

- [ ] Write a failing CLI mapping test for 5 cuts and 0.03 m spacing.
- [ ] Add parser and dispatch while preserving existing commands.
- [ ] Run headless acceptance and assert CSV/SVG creation.
- [ ] Run `.venv/bin/python -m pytest tests/simulation -q`.
- [ ] Run `sha256sum --check docs/simulation/protected-files.sha256`.
- [ ] Run a slow Viewer demo and commit.

