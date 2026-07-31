# Left Wuji Hand Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Attach the original 20-DOF Wuji left hand to the simulated left wrist, expose safe position control, and preserve all existing chopping behavior.

**Architecture:** Vendor the MIT-licensed hand meshes and a composable MJCF fragment under `robot_assets/mujoco/wuji_hand`, mount that fragment below a single `left_hand_mount` transform, and extend `SimulationModel` with named hand indices. A focused `LeftHandController` validates and writes only the 20 hand controls while `RightArmRobot` continues to own the 14 arm controls.

**Tech Stack:** Python 3.12, MuJoCo 3.10.0 MJCF, NumPy 2.5.1, pytest 9.1.1

## Global Constraints

- Simulation-only changes; physical-robot files remain untouched.
- The project must not read either downloaded repository at runtime.
- Preserve the upstream original Wuji Hand's 20-joint names, limits, inertias, actuator gains, and order.
- Preserve all 14 existing arm joint and actuator names and right-arm chopping behavior.
- Keep `left_hand_mount` as the only flange-to-palm transform.
- Vendor the upstream MIT license and attribution with the assets.
- Do not add `wuji-mjlab`, NVIDIA, Isaac Lab, or RL dependencies.

---

### Task 1: Vendor and Compose the Left-Hand Asset

**Files:**
- Create: `robot_assets/mujoco/wuji_hand/LICENSE`
- Create: `robot_assets/mujoco/wuji_hand/README.md`
- Create: `robot_assets/mujoco/wuji_hand/left_hand.xml`
- Create: `robot_assets/mujoco/wuji_hand/meshes/left/*.STL`
- Modify: `robot_assets/mujoco/right_chopping_scene.xml`
- Modify: `tests/simulation/test_model.py`

**Interfaces:**
- Consumes: upstream files in `/home/linux/august_folder/wuji-description/hand/body/`
- Produces: body `left_hand_mount`, palm `left_palm_link`, 20 `left_fingerN_jointN` joints, and their 20 named position actuators

- [ ] **Step 1: Write failing composition assertions**

Update `tests/simulation/test_model.py` so the primary model test expects
`model.njnt == 34`, `model.nu == 34`, 14 arm actuators, and 20 hand actuators.
Add assertions that `left_hand_mount` and `left_palm_link` exist and that walking
`model.body_parentid` from the palm reaches `left_link7`.

- [ ] **Step 2: Run the focused test and confirm red**

Run: `.worktrees/mujoco-position-rebuild/.venv/bin/pytest tests/simulation/test_model.py -v`

Expected: FAIL because the active scene still contains only 14 joints and
actuators and has no `left_hand_mount`.

- [ ] **Step 3: Vendor upstream files with attribution**

Copy the 26 left-hand STL files byte-for-byte into
`robot_assets/mujoco/wuji_hand/meshes/left/`, copy the upstream `LICENSE`, and
write `README.md` identifying the source path, original project, selected
original-hand variant, and copy date. These copied binary assets are the only
non-`apply_patch` file operation in the task.

- [ ] **Step 4: Build a composable MJCF fragment**

Derive `left_hand.xml` from upstream `hand/body/mjcf/left.xml`. Remove
top-level `<mujoco>`, `<compiler>`, `<option>`, and `<worldbody>` ownership so it
can be composed without replacing the chopping scene's solver settings. Point
mesh files at `wuji_hand/meshes/left/`, preserve all body/joint/inertial/geom
definitions, and preserve all 20 `<position>` definitions.

- [ ] **Step 5: Attach through one mount**

In `right_chopping_scene.xml`, add the hand mesh assets, place:

```xml
<body name="left_hand_mount" pos="..." quat="...">
  <body name="left_palm_link">
    <!-- preserved hand body tree -->
  </body>
</body>
```

under `left_link7`, and append the 20 preserved hand position actuators to the
existing actuator block. Determine the explicit `pos` and `quat` by rendering
and inspecting the flange/palm frames; do not distribute correction transforms
through finger bodies.

- [ ] **Step 6: Load and inspect the composed model**

Run a short Python probe with `mujoco.MjModel.from_xml_path` and print `njnt`,
`nu`, mount/palm IDs, palm world pose, and all hand control ranges.

Expected: `njnt=34`, `nu=34`, all IDs non-negative, finite ranges, and no XML
compiler warnings.

- [ ] **Step 7: Run the model tests**

Run: `.worktrees/mujoco-position-rebuild/.venv/bin/pytest tests/simulation/test_model.py -v`

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add robot_assets/mujoco/wuji_hand robot_assets/mujoco/right_chopping_scene.xml tests/simulation/test_model.py
git commit -m "feat: mount Wuji hand on left wrist"
```

### Task 2: Add Named Hand Model Indices and Contracts

**Files:**
- Create: `src/twin_sim/hand_names.py`
- Modify: `src/twin_sim/model.py`
- Modify: `tests/simulation/test_model.py`

**Interfaces:**
- Produces: `HAND_JOINTS: tuple[str, ...]`, `HAND_ACTUATORS: tuple[str, ...]`,
  `HandIndices(joint_ids, qpos_ids, dof_ids, actuator_ids)`, and
  `SimulationModel.hand`

- [ ] **Step 1: Write failing hand-index tests**

Assert the exact upstream finger-major order:

```python
(
    "left_finger1_joint1", "left_finger1_joint2",
    "left_finger1_joint3", "left_finger1_joint4",
    # repeat joints 1..4 for fingers 2..5
)
```

Assert `sim.hand.*_ids.shape == (20,)`, every actuator transmits to its matching
joint with unit gear, and every hand actuator is a ranged position servo.

- [ ] **Step 2: Confirm red**

Run: `.worktrees/mujoco-position-rebuild/.venv/bin/pytest tests/simulation/test_model.py -v`

Expected: FAIL because `SimulationModel.hand` does not exist.

- [ ] **Step 3: Add constants and indices**

Create immutable ordered names in `hand_names.py`. Add `HandIndices` and
populate `hand` during `SimulationModel.load()` using name lookup, never numeric
offset assumptions.

- [ ] **Step 4: Generalize contract validation**

Extract the shared single-axis position-servo checks in `model.py` and apply
them to both seven-joint arms and the 20-joint hand. Preserve the arms'
force-range equality requirement and apply the equivalent upstream hand
joint/actuator force contract.

- [ ] **Step 5: Run model and repository-boundary tests**

Run:
`.worktrees/mujoco-position-rebuild/.venv/bin/pytest tests/simulation/test_model.py tests/simulation/test_repository_boundaries.py -v`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/twin_sim/hand_names.py src/twin_sim/model.py tests/simulation/test_model.py
git commit -m "feat: index Wuji hand joints by name"
```

### Task 3: Implement Atomic Left-Hand Position Control

**Files:**
- Create: `src/twin_sim/hand.py`
- Create: `tests/simulation/test_hand.py`
- Modify: `src/twin_sim/robot.py`

**Interfaces:**
- Produces:
  `DEFAULT_OPEN_RAD: np.ndarray`,
  `LeftHandController(sim: SimulationModel)`,
  `target: np.ndarray`,
  `command(joints_rad: Sequence[float]) -> None`,
  `apply() -> None`, and `open() -> None`
- `RightArmRobot.hand` exposes the controller and reset/step call it

- [ ] **Step 1: Write failing controller tests**

Cover construction, a 20-value in-range command, `open()`, and rejection of
shape `(19,)`, NaN, and one out-of-range element. Snapshot `data.ctrl` before
each invalid command and assert it remains byte-for-byte unchanged.

- [ ] **Step 2: Confirm red**

Run: `.worktrees/mujoco-position-rebuild/.venv/bin/pytest tests/simulation/test_hand.py -v`

Expected: FAIL because `twin_sim.hand` does not exist.

- [ ] **Step 3: Implement validation and atomic writes**

In `hand.py`, convert input to a float array, require shape `(20,)`, require all
finite values, and compare every value against
`model.actuator_ctrlrange[sim.hand.actuator_ids]`. Only after full validation
copy the target. `apply()` writes exactly `sim.hand.actuator_ids`.

Define `DEFAULT_OPEN_RAD` from a visually natural interior pose and assert at
module/test level that it remains inside all 20 ranges.

- [ ] **Step 4: Integrate reset and stepping**

Construct `self.hand` after loading the model. During `reset()`, initialize hand
qpos to `DEFAULT_OPEN_RAD`, zero its qvel, and apply the target. During every
`step()`, reapply the hand target alongside the two arm targets.

- [ ] **Step 5: Run hand and robot tests**

Run:
`.worktrees/mujoco-position-rebuild/.venv/bin/pytest tests/simulation/test_hand.py tests/simulation/test_robot.py -v`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/twin_sim/hand.py src/twin_sim/robot.py tests/simulation/test_hand.py
git commit -m "feat: add safe left hand position control"
```

### Task 4: Add Stability and Collision Regression Checks

**Files:**
- Create: `tests/simulation/test_hand_stability.py`
- Modify: `robot_assets/mujoco/right_chopping_scene.xml` only if a verified
  adjacent-body exclusion is required

**Interfaces:**
- Consumes: `RightArmRobot.hand`, MuJoCo contacts and body IDs
- Produces: regression coverage for finite rollout and initial clearance

- [ ] **Step 1: Write rollout and clearance tests**

Reset the robot, step 500 MuJoCo timesteps through the public control loop, and
assert all hand qpos/qvel values remain finite. Inspect active contacts at reset
and assert neither contact body belongs to both the hand subtree and
`chopping_board`; assert no non-adjacent hand/left-arm body pair penetrates.

- [ ] **Step 2: Run and diagnose red failures**

Run:
`.worktrees/mujoco-position-rebuild/.venv/bin/pytest tests/simulation/test_hand_stability.py -v`

Expected: PASS if calibration is correct, otherwise a focused failure naming
the colliding geoms/bodies.

- [ ] **Step 3: Correct only demonstrated attachment issues**

If needed, adjust only `left_hand_mount pos/quat`. Add a contact exclusion only
when the failing pair is an unavoidable directly adjacent wrist/palm pair.
Repeat the focused test after each adjustment.

- [ ] **Step 4: Run chopping regressions**

Run:
`.worktrees/mujoco-position-rebuild/.venv/bin/pytest tests/simulation/test_chop.py tests/simulation/test_line_chop.py tests/simulation/test_kinematics.py -v`

Expected: PASS with unchanged right-arm trajectory assertions.

- [ ] **Step 5: Commit**

```bash
git add tests/simulation/test_hand_stability.py robot_assets/mujoco/right_chopping_scene.xml
git commit -m "test: verify mounted hand stability and clearance"
```

### Task 5: Add a Visual Hand Demonstration and Final Verification

**Files:**
- Create: `src/twin_sim/tasks/hand_demo.py`
- Create: `tests/simulation/test_hand_demo.py`
- Modify: `src/twin_sim/cli.py`
- Modify: `tests/simulation/test_cli.py`
- Modify: `README.md`
- Modify: `docs/development_logs/2026-07-31-mujoco-position-line-chop-development-log.md`

**Interfaces:**
- Produces: CLI `twin-sim hand-demo [--headless] [--slow]`
- Demonstration: open → relaxed close → open using interpolated 20-joint targets

- [ ] **Step 1: Write failing CLI and trajectory tests**

Assert `hand-demo --headless` returns zero, sends only valid 20-value commands,
and completes open → relaxed-close → open. Assert `--slow` selects long viewer
holds and durations suitable for visual inspection.

- [ ] **Step 2: Confirm red**

Run:
`.worktrees/mujoco-position-rebuild/.venv/bin/pytest tests/simulation/test_hand_demo.py tests/simulation/test_cli.py -v`

Expected: FAIL because the command and task do not exist.

- [ ] **Step 3: Implement the demonstration**

Add a `HandDemoConfig` with control interval and phase durations. Generate
linear joint-space interpolation between `DEFAULT_OPEN_RAD` and a conservative
relaxed-close vector strictly inside the actuator ranges. Drive it through
`robot.hand.command()` and the existing `robot.step()` lifecycle.

- [ ] **Step 4: Document operation and provenance**

Add the exact commands:

```bash
.venv/bin/twin-sim hand-demo --headless
.venv/bin/twin-sim hand-demo --slow
```

Document that the original hand asset is MIT-licensed, where it is vendored,
and that RL/real-hand integration is not enabled. Append the implementation
outcome and any mount/collision issue to the existing development log.

- [ ] **Step 5: Run the full verification suite**

Run:

```bash
.worktrees/mujoco-position-rebuild/.venv/bin/pytest -q
.worktrees/mujoco-position-rebuild/.venv/bin/python tools/check_real_code_unchanged.py
.worktrees/mujoco-position-rebuild/.venv/bin/twin-sim hand-demo --headless
git diff --check
```

Expected: all tests pass; protected real-code check reports every protected file
unchanged; demo exits zero; no whitespace errors.

- [ ] **Step 6: Launch the slow visual demonstration**

Run:
`.worktrees/mujoco-position-rebuild/.venv/bin/twin-sim hand-demo --slow`

Inspect palm orientation, wrist clearance, table clearance, and all five
fingers through a complete slow cycle. Keep the viewer open long enough for the
user to inspect it.

- [ ] **Step 7: Commit**

```bash
git add src/twin_sim/tasks/hand_demo.py src/twin_sim/cli.py tests/simulation/test_hand_demo.py tests/simulation/test_cli.py README.md docs/development_logs/2026-07-31-mujoco-position-line-chop-development-log.md
git commit -m "feat: add Wuji hand visual demonstration"
```

