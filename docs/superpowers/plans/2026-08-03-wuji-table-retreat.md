# Wuji Table-Contact Retreat Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an offline command that derives and previews a four-fingertip tabletop retreat gesture from a Wuji glove MCAP.

**Architecture:** Build a self-owned MuJoCo scene from the vendored official hand, adding a mocap palm, table and fingertip sites. Select a source frame deterministically, solve PLACE and RETREAT with bounded projected damped least squares, preflight the entire corrected motion, then replay and optionally serialize it without changing raw replay.

**Tech Stack:** Python 3.12, NumPy 2.5, MuJoCo 3.10, pytest 9, JSON/NPZ.

## Global Constraints

- Never modify the vendored official MJCF or original MCAP.
- Use fingers 2–5 for table contact and keep finger 1 at least `0.010 m` clear.
- Default retreat is `0.030 m`; contact slip is at most `0.003 m`; height error and penetration are at most `0.002 m`.
- Maximum per-frame joint step is `0.12 rad`; palm retreat tolerance is `±0.002 m`.
- Add no optimization dependency and access no physical hardware or ROS command path.

---

### Task 1: Derived table scene and observable hand backend

**Files:**
- Create: `src/tianji_robotics/simulation/tabletop_wuji_hand.py`
- Create: `tests/simulation/test_tabletop_wuji_hand.py`

**Interfaces:**
- Produces: `TabletopWujiHand(viewer: bool = False, table_height_m: float = 0.0)`.
- Produces: `command_pose(joints_rad, palm_position_m, palm_quaternion_wxyz)`, `fingertip_positions_m`, `thumb_position_m`, `contact_diagnostics`, `step`, and `close`.

- [ ] Write failing tests requiring one plane, a mocap palm, 20 joints/actuators, four named long-finger sites, one thumb site, no arm names, finite pose validation and official hand camera.
- [ ] Run `.venv-wuji-teleop/bin/python -m pytest tests/simulation/test_tabletop_wuji_hand.py -q`; confirm RED because the backend does not exist.
- [ ] Implement runtime `MjSpec` derivation, mocap/table/sites, exact indexing and observable kinematics without editing assets.
- [ ] Rerun focused tests; confirm PASS.
- [ ] Commit as `feat: add Wuji tabletop simulation backend`.

### Task 2: Deterministic frame selection and correction solver

**Files:**
- Create: `src/tianji_robotics/workflows/wuji_table_retreat.py`
- Create: `tests/workflows/test_wuji_table_retreat.py`

**Interfaces:**
- Produces: `TableRetreatConfig`, `CorrectedHandTrajectory`, `CorrectionReport`.
- Produces: `build_table_retreat(trajectory, backend, config) -> tuple[CorrectedHandTrajectory, CorrectionReport]`.

- [ ] Write failing tests for deterministic earliest tie selection, explicit-frame validation, PLACE contact, thumb clearance, `0.030 ± 0.002 m` retreat, phase order, joint/step limits and infeasible-input failure before playback.
- [ ] Run `.venv-wuji-teleop/bin/python -m pytest tests/workflows/test_wuji_table_retreat.py -q`; confirm RED.
- [ ] Implement candidate scoring, fixed-iteration projected damped least squares, smooth phase sampling, diagnostics and complete preflight.
- [ ] Run focused workflow/backend tests; confirm PASS.
- [ ] Commit as `feat: derive Wuji tabletop retreat gestures`.

### Task 3: Safe output codecs and CLI

**Files:**
- Create: `src/tianji_robotics/data/table_retreat.py`
- Modify: `src/tianji_robotics/cli.py`
- Create: `tests/data/test_table_retreat_npz.py`
- Modify: `tests/cli/test_tianji_robot_cli.py`

**Interfaces:**
- Produces: `save_corrected_trajectory_npz`, `load_corrected_trajectory_npz`, `write_correction_report_json`.
- Produces: `tianji-robot sim wuji-table-retreat SOURCE` with documented overrides and optional output paths.

- [ ] Write failing codec round-trip and CLI routing tests, including rejection of hardware flags and missing optional dependencies.
- [ ] Run `.venv-wuji-teleop/bin/python -m pytest tests/data/test_table_retreat_npz.py tests/cli/test_tianji_robot_cli.py -q`; confirm RED.
- [ ] Implement pickle-free NPZ/JSON codecs and lazy CLI orchestration with full preflight before Viewer creation.
- [ ] Rerun data/CLI tests; confirm PASS.
- [ ] Commit as `feat: expose Wuji table-retreat workflow`.

### Task 4: Documentation and real-recording verification

**Files:**
- Create: `docs/wuji/table_retreat.md`
- Modify: `docs/wuji/development_status.md`
- Modify: `tests/documentation/test_wuji_docs.py`

**Interfaces:**
- Documents exact Headless/Viewer commands, outputs, success criteria, manual overrides and non-hardware guarantee.

- [ ] Write a failing documentation test for `wuji-table-retreat`, default thresholds and safety boundary.
- [ ] Add user documentation and current-progress evidence.
- [ ] Run the real recording Headless, save NPZ/report, and independently inspect all numeric acceptance criteria.
- [ ] Run Viewer and inspect four long fingers, raised thumb, backward palm motion and final HOLD.
- [ ] Run `.venv-wuji-teleop/bin/python -m pytest -q`, check official repos clean, and commit as `docs: verify Wuji tabletop retreat`.
