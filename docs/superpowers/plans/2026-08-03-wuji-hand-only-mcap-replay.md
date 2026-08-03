# Wuji Hand-Only MCAP Replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the existing Wuji MCAP replay command load and display only the official left Wuji Hand.

**Architecture:** Vendor the exact MIT-licensed official left-hand MJCF subset, then replace the arm-coupled MuJoCo adapter with a direct 20-joint MuJoCo backend. Keep data codecs, retargeting, replay scheduling, CLI, arm simulations and physical-device interfaces unchanged.

**Tech Stack:** Python 3.12, MuJoCo 3.10, NumPy 2.5, pytest 9, official Wuji MJCF/STL assets.

## Global Constraints

- All integration code lives in `tianji_robotic_project`; official source repositories remain unchanged.
- Vendor only `left.xml`, 52 left mesh files, MIT license and provenance metadata (about 3.2 MB).
- `tianji-robot sim wuji-replay` must contain no arm model and must never connect physical hardware.
- Existing `twin-sim` arm and arm-mounted-hand workflows remain unchanged.
- Use the official asset commit `63d2785932eb5b91523e44222f5d0bcffbce1e74`.

---

### Task 1: Vendor and validate the official left-hand model

**Files:**
- Create: `robot_assets/mujoco/wuji_hand_standalone/left.xml`
- Create: `robot_assets/mujoco/wuji_hand_standalone/meshes/left/*.STL`
- Create: `robot_assets/mujoco/wuji_hand_standalone/LICENSE`
- Create: `robot_assets/mujoco/wuji_hand_standalone/README.md`
- Create: `src/tianji_robotics/simulation/paths.py`
- Test: `tests/simulation/test_wuji_hand_only_assets.py`

**Interfaces:**
- Produces: `official_wuji_left_mjcf() -> pathlib.Path`.
- Produces: an unmodified official model with `nq == nv == nu == 20`.

- [ ] Write tests asserting path independence from CWD, missing-asset errors, exactly 20 DOFs/actuators, palm presence, and absence of arm names.
- [ ] Run `.venv-wuji-teleop/bin/python -m pytest tests/simulation/test_wuji_hand_only_assets.py -q` and confirm RED because the resolver/assets do not exist.
- [ ] Copy only the approved official subset, write source URL/commit/import date in README, and implement the project-root resolver.
- [ ] Compare SHA-256 of every copied model/mesh/license file with its official source, then rerun the focused test and confirm PASS.
- [ ] Commit as `assets: vendor official Wuji left-hand model`.

### Task 2: Replace the arm-coupled backend using TDD

**Files:**
- Modify: `src/tianji_robotics/simulation/wuji_hand.py`
- Modify: `tests/simulation/test_wuji_trajectory_replay.py`
- Create: `tests/simulation/test_wuji_hand_only_backend.py`

**Interfaces:**
- Consumes: `official_wuji_left_mjcf()`.
- Preserves: `MujocoWujiHand(viewer: bool = False)`, `joint_ranges_rad`, `read_position_rad`, `read_target_position_rad`, `command_position_rad`, `step`, and `close`.

- [ ] Write failing tests requiring no `RightArmRobot` import, exactly 20 hand actuators, canonical-to-official name mapping, finite/ranged command validation, stepping, idempotent close, and official Viewer camera configuration through an injected viewer launcher.
- [ ] Run `.venv-wuji-teleop/bin/python -m pytest tests/simulation/test_wuji_hand_only_backend.py tests/simulation/test_wuji_trajectory_replay.py -q` and confirm RED against the 42-body arm scene.
- [ ] Implement direct `MjModel`/`MjData` ownership, exact joint/actuator indexing, safe range intersection, `data.ctrl` commands, `mj_step`, passive Viewer sync and close.
- [ ] Rerun focused simulation tests and the Wuji workflow/CLI suites; confirm PASS.
- [ ] Commit as `fix: replay Wuji recordings on hand-only model`.

### Task 3: Document and verify the real workflow

**Files:**
- Modify: `docs/wuji/offline_replay.md`
- Modify: `docs/wuji/development_status.md`
- Modify: `tests/documentation/test_wuji_docs.py`

**Interfaces:**
- Public command remains `.venv-wuji-teleop/bin/tianji-robot sim wuji-replay SOURCE [--headless]`.

- [ ] Add a failing documentation test requiring the hand-only guarantee, official asset provenance and explicit statement that no arm model is loaded.
- [ ] Update docs with asset ownership, command, expected `1676 frames (13.958s)` result and manual Viewer acceptance.
- [ ] Run the real recording Headless and assert the runtime model has 20 DOFs/actuators and no arm names.
- [ ] Run the real recording in Viewer and visually verify only the left Wuji Hand appears.
- [ ] Run `.venv-wuji-teleop/bin/python -m pytest -q` and record the complete result.
- [ ] Check all official Git repositories remain clean and commit docs as `docs: verify hand-only Wuji replay`.
