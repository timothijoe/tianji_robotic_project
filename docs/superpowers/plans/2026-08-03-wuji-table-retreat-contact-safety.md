# Wuji Table-Retreat Contact Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a visually and numerically valid Wuji tabletop gesture whose palm is parallel to the table, whose four long fingertips initially contact it, and whose complete collision geometry never crosses it during retreat.

**Architecture:** Extend `TabletopWujiHand` with anatomical landmarks and real hand/table collision clearance. Keep palm-frame fitting and phase generation in the pure workflow layer: PLACE solves simultaneous fingertip contact plus collision clearance, while RETREAT releases planted-tip constraints and projects each frame upward only when real geometry requires it. Preserve the existing CLI, official vendored model, raw replay, SDK boundary, and output formats.

**Tech Stack:** Python 3.12, NumPy, MuJoCo Python bindings, pytest, existing Wuji MCAP and official retargeter.

## Global Constraints

- Do not modify `../wuji-technology` or the vendored official MJCF/assets.
- Add no SciPy or other optimization dependency; use NumPy and MuJoCo Jacobians.
- PLACE long fingertips must be within `0.003 m` of the table.
- All hand/table collision distances must remain above `-0.0005 m`.
- Thumb clearance must remain at least `0.010 m`.
- Palm retreat must be `0.030 ± 0.002 m` by default.
- Joint steps must remain at or below `0.12 rad`.
- Raw `wuji-replay`, twin-arm simulations, SDK calls, and ROS 2 interfaces remain unchanged.

---

### Task 1: Real collision and anatomical landmark backend

**Files:**
- Modify: `src/tianji_robotics/simulation/tabletop_wuji_hand.py`
- Modify: `tests/simulation/test_tabletop_wuji_hand.py`

**Interfaces:**
- Consumes: existing `set_kinematic_pose(joints_rad, palm_position_m, palm_quaternion_wxyz)`.
- Produces: `long_finger_root_positions_m() -> np.ndarray`, `maximum_table_penetration_m() -> float`, and `minimum_hand_table_clearance_m() -> float`.

- [ ] **Step 1: Write failing backend tests**

```python
def test_anatomical_landmarks_return_four_world_positions():
    hand = TabletopWujiHand(viewer=False)
    try:
        hand.set_kinematic_pose(np.zeros(20), [0, 0, 0.2], [1, 0, 0, 0])
        assert hand.long_finger_root_positions_m().shape == (4, 3)
    finally:
        hand.close()


def test_real_collision_clearance_detects_table_crossing():
    hand = TabletopWujiHand(viewer=False)
    try:
        hand.set_kinematic_pose(np.zeros(20), [0, 0, -0.05], [1, 0, 0, 0])
        assert hand.maximum_table_penetration_m() > 0.01
        assert hand.minimum_hand_table_clearance_m() < -0.01
    finally:
        hand.close()
```

- [ ] **Step 2: Run tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_tabletop_wuji_hand.py`

Expected: FAIL because the landmark and clearance APIs do not both exist.

- [ ] **Step 3: Implement the backend APIs using MuJoCo IDs and contacts**

Cache body IDs for `finger2_link1` through `finger5_link1`, cache the table geom ID, return copies of `data.xpos`, and derive penetration from contacts involving the table:

```python
def long_finger_root_positions_m(self) -> np.ndarray:
    return self.data.xpos[self._long_finger_root_body_ids].copy()

def maximum_table_penetration_m(self) -> float:
    return max(0.0, -self.minimum_hand_table_clearance_m())
```

`minimum_hand_table_clearance_m()` returns the minimum table-contact distance, or `inf` when no hand geom is close enough to generate a contact.

- [ ] **Step 4: Run backend tests and verify GREEN**

Run: `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_tabletop_wuji_hand.py`

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/tianji_robotics/simulation/tabletop_wuji_hand.py tests/simulation/test_tabletop_wuji_hand.py
git commit -m "fix: expose real Wuji table collision diagnostics"
```

---

### Task 2: Anatomical palm-frame fitting and collision-safe PLACE

**Files:**
- Modify: `src/tianji_robotics/workflows/wuji_table_retreat.py`
- Modify: `tests/workflows/test_wuji_table_retreat.py`

**Interfaces:**
- Consumes: `fingertip_positions_m()`, `long_finger_root_positions_m()`, `fingertip_position_jacobian()`, and collision diagnostics from Task 1.
- Produces: `_fit_palm_down_quaternion(backend, joints) -> np.ndarray` and `_solve_place(...) -> tuple[np.ndarray, np.ndarray, np.ndarray]` returning joints, palm position, and palm quaternion.

- [ ] **Step 1: Write failing palm-frame and PLACE tests**

```python
def test_fitted_palm_frame_makes_extension_and_lateral_axes_horizontal():
    backend = TabletopWujiHand(viewer=False)
    try:
        quaternion = _fit_palm_down_quaternion(backend, FEASIBLE_POSE)
        backend.set_kinematic_pose(FEASIBLE_POSE, np.zeros(3), quaternion)
        roots = backend.long_finger_root_positions_m()
        tips = backend.fingertip_positions_m()
        assert abs((tips.mean(0) - roots.mean(0))[2]) <= 0.005
        assert abs((roots[-1] - roots[0])[2]) <= 0.005
    finally:
        backend.close()


def test_place_contacts_four_tips_without_any_hand_penetration():
    backend = TabletopWujiHand(viewer=False)
    try:
        corrected, report = build_table_retreat(
            _trajectory(), backend,
            TableRetreatConfig(place_duration_s=.1, retreat_duration_s=.2,
                               hold_duration_s=.1, candidate_stride=1),
        )
        place_end = corrected.phases.index("RETREAT") - 1
        backend.set_kinematic_pose(
            corrected.positions_rad[place_end],
            corrected.palm_positions_m[place_end],
            corrected.palm_quaternions_wxyz[place_end],
        )
        assert np.max(np.abs(backend.fingertip_positions_m()[:, 2])) <= .003
        assert backend.maximum_table_penetration_m() <= .0005
    finally:
        backend.close()
```

- [ ] **Step 2: Run workflow tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py`

Expected: FAIL because palm fitting and simultaneous collision-safe placement are absent.

- [ ] **Step 3: Implement anatomical frame fitting**

Build a right-handed local basis from the mean root-to-tip direction, the index-to-little-finger root direction, and their cross product. Map it to a world basis whose finger-forward and lateral axes are horizontal and whose palm normal points away from the table. Convert the rotation matrix with `mujoco.mju_mat2Quat`; reject degenerate landmarks.

- [ ] **Step 4: Implement collision-safe PLACE as one convergent loop**

Within each bounded iteration:

1. solve the four fingertip vertical residuals with the existing damped least-squares Jacobian;
2. run `mj_forward`;
3. raise the palm by `penetration + 0.0001 m` when penetration exceeds `0.0005 m`;
4. repeat fingertip IK after every lift instead of returning early;
5. accept only when all four tip heights are within `0.003 m`, thumb clearance is at least `0.010 m`, and whole-hand penetration is at most `0.0005 m`.

If no candidate converges, reject it and continue deterministic source-frame selection. Do not hard-code an Euler-angle quaternion.

- [ ] **Step 5: Run workflow tests and verify GREEN**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py`

Expected: all tests pass with a horizontal anatomical palm frame and collision-safe PLACE.

- [ ] **Step 6: Commit**

```bash
git add src/tianji_robotics/workflows/wuji_table_retreat.py tests/workflows/test_wuji_table_retreat.py
git commit -m "fix: solve horizontal collision-safe Wuji placement"
```

---

### Task 3: Released-contact RETREAT with per-frame safety projection

**Files:**
- Modify: `src/tianji_robotics/workflows/wuji_table_retreat.py`
- Modify: `src/tianji_robotics/data/table_retreat.py`
- Modify: `tests/workflows/test_wuji_table_retreat.py`
- Modify: `tests/data/test_table_retreat_npz.py`

**Interfaces:**
- Consumes: collision-safe PLACE pose from Task 2.
- Produces: `_project_above_table(...) -> np.ndarray`, plus `maximum_hand_penetration_m` and `maximum_retreat_fingertip_lift_m` in `CorrectionReport`.

- [ ] **Step 1: Write failing RETREAT safety tests**

```python
def test_retreat_releases_tip_contact_but_never_crosses_table():
    backend = TabletopWujiHand(viewer=False)
    try:
        corrected, report = build_table_retreat(_trajectory(), backend, SHORT_CONFIG)
        retreat = [i for i, phase in enumerate(corrected.phases) if phase == "RETREAT"]
        heights = []
        for index in retreat:
            backend.set_kinematic_pose(corrected.positions_rad[index],
                                       corrected.palm_positions_m[index],
                                       corrected.palm_quaternions_wxyz[index])
            assert backend.maximum_table_penetration_m() <= .0005
            heights.append(backend.fingertip_positions_m()[:, 2].copy())
        assert np.max(heights[-1] - heights[0]) > 0
        assert report.maximum_hand_penetration_m <= .0005
    finally:
        backend.close()
```

- [ ] **Step 2: Run focused tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py tests/data/test_table_retreat_npz.py`

Expected: FAIL because RETREAT still attempts planted contacts or lacks the new report field.

- [ ] **Step 3: Implement released-contact retreat**

Resolve retreat from the fitted palm-forward axis. Interpolate palm translation and a bounded long-finger curl target with smooth endpoints. For every generated frame, call `_project_above_table`: repeatedly run kinematics and add only the required world-Z correction until penetration is at most `0.0005 m`. Clamp per-frame joint changes to `0.10 rad`, below the `0.12 rad` preflight limit.

- [ ] **Step 4: Update report serialization**

Add the two float fields to `CorrectionReport`; the existing dataclass-to-JSON path remains pickle-free. Update constructor fixtures and assert both keys round-trip as plain JSON numbers.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py tests/data/test_table_retreat_npz.py`

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/tianji_robotics/workflows/wuji_table_retreat.py src/tianji_robotics/data/table_retreat.py tests/workflows/test_wuji_table_retreat.py tests/data/test_table_retreat_npz.py
git commit -m "fix: keep Wuji retreat trajectory above table"
```

---

### Task 4: Real MCAP verification, Viewer acceptance, and documentation

**Files:**
- Modify: `src/tianji_robotics/simulation/tabletop_wuji_hand.py`
- Modify: `src/tianji_robotics/cli.py`
- Modify: `tests/cli/test_tianji_robot_cli.py`
- Modify: `docs/wuji/table_retreat.md`
- Modify: `docs/development_status.md`

**Interfaces:**
- Consumes: corrected trajectory and report from Tasks 2–3.
- Produces: clear CLI summary including penetration and fingertip lift, plus an oblique default Viewer camera suitable for judging table clearance.

- [ ] **Step 1: Write failing CLI assertion**

Extend the CLI test to require `penetration_m` and `tip_lift_m` in the successful table-retreat summary. Keep device and ROS mocks asserting no hardware access.

- [ ] **Step 2: Run CLI and documentation tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/cli/test_tianji_robot_cli.py tests/documentation/test_wuji_docs.py`

Expected: FAIL because the CLI does not print the new diagnostics.

- [ ] **Step 3: Update CLI, camera, and usage documentation**

Print both safety values, use an oblique camera close enough to inspect the four fingertips and palm/table gap, document initial-only contact semantics, and retain the existing command:

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_162909_764.mcap
```

- [ ] **Step 4: Run focused and full automated verification**

Run:

```bash
.venv-wuji-teleop/bin/pytest -q tests/simulation/test_tabletop_wuji_hand.py tests/workflows/test_wuji_table_retreat.py tests/data/test_table_retreat_npz.py tests/cli/test_tianji_robot_cli.py tests/documentation/test_wuji_docs.py
.venv-wuji-teleop/bin/pytest -q
```

Expected: focused tests pass; full suite passes with no new warnings.

- [ ] **Step 5: Run real MCAP headless preflight**

Run:

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_162909_764.mcap \
  --headless --report recordings/verification/wuji_table_retreat_safe.json
```

Expected: exit 0; JSON reports penetration at most `0.0005 m`, initial tip height at most `0.003 m`, and retreat `0.030 ± 0.002 m`.

- [ ] **Step 6: Perform two-view manual acceptance**

Launch the Viewer, capture oblique PLACE/HOLD images, then rotate to a side view and capture the table gap. Reject and return to Task 2 if the palm is vertical, any mesh crosses the table, four long fingertips do not initially contact, or the thumb points through the table.

- [ ] **Step 7: Commit verified documentation and presentation changes**

```bash
git add src/tianji_robotics/simulation/tabletop_wuji_hand.py src/tianji_robotics/cli.py tests/cli/test_tianji_robot_cli.py docs/wuji/table_retreat.md docs/development_status.md
git commit -m "docs: verify collision-safe Wuji table retreat"
```

