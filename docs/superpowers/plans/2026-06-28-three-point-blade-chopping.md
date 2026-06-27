# Two-Point Blade Chopping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Change the MuJoCo right-arm chopping demo so target planning uses `right_blade_edge_top` and `right_blade_edge_bot` as the cutting edge, while `right_tool_tip_site` remains the controlled site.

**Architecture:** Add a two-point blade-geometry layer inside `twin_mujoco.chopping` that reads blade-edge positions, computes a horizontal blade target rotation, and predicts blade-edge target positions from the commanded tool-tip pose. Keep the existing `CartesianForceController` and `right_tool_tip_site` Jacobian for the quick version.

**Tech Stack:** Python 3.10+, numpy, mujoco Python package, pytest, stdlib dataclasses/csv/argparse.

## Global Constraints

- Keep this phase MuJoCo-only; do not add ROS2, launch files, RViz, or real robot SDK integration.
- Do not modify `MarvinCCS/`, `MarvinCCS_mujoco.zip`, or `MarvinCCS/marvin_final_fixed.xml`.
- Keep `twin-chop --cycles 1 --headless` and `twin-chop --cycles 1 --viewer` working.
- Keep the existing CSV schema unchanged.
- Tests must be written and observed failing before production code changes.
- The controller continues to command `right_tool_tip_site`; the new behavior is target planning and sample geometry based on the two blade-edge sites.

---

## File Structure

- Modify `src/twin_mujoco/twin_mujoco/chopping.py`: add blade edge constants, a `BladeGeometry` dataclass, target rotation/position helper methods, and sample fields for two blade-edge positions.
- Modify `tests/test_description_scene.py`: assert both blade-edge sites and the controlled tip site exist in the MuJoCo scene.
- Modify `tests/test_chopping.py`: add tests for geometry helpers, horizontal posture prediction, safe/descent target prediction, and sample data.

---

### Task 1: Blade Edge Geometry And Horizontal Target Prediction

**Files:**
- Modify: `src/twin_mujoco/twin_mujoco/chopping.py`
- Modify: `tests/test_description_scene.py`
- Modify: `tests/test_chopping.py`

**Interfaces:**
- Consumes: `TwinMujocoRuntime.site_pose(name: str) -> tuple[np.ndarray, np.ndarray]`
- Consumes: `RightArmChopper._board_top() -> float`
- Produces: `BLADE_EDGE_SITE_NAMES: tuple[str, str]`
- Produces: `BladeGeometry(edge_positions: tuple[tuple[float, float, float], ...], tip_position: tuple[float, float, float], tip_rotation: tuple[tuple[float, float, float], ...])`
- Produces: `BladeGeometry.z_values -> tuple[float, float]`
- Produces: `BladeGeometry.min_z -> float`
- Produces: `BladeGeometry.max_z -> float`
- Produces: `BladeGeometry.clearances(board_top: float) -> tuple[float, float]`
- Produces: `RightArmChopper._blade_geometry() -> BladeGeometry`
- Produces: `RightArmChopper._horizontal_blade_rotation(geometry: BladeGeometry) -> np.ndarray`
- Produces: `RightArmChopper._predict_blade_edge_positions(tip_target: Sequence[float], target_rotation: np.ndarray, geometry: BladeGeometry) -> tuple[tuple[float, float, float], ...]`
- Produces: `RightArmChopper._tip_target_for_blade_clearance(tip_position: np.ndarray, target_rotation: np.ndarray, geometry: BladeGeometry, board_top: float, clearance_m: float) -> np.ndarray`
- Produces: `RightArmChopper._tip_target_for_blade_on_board(tip_position: np.ndarray, target_rotation: np.ndarray, geometry: BladeGeometry, board_top: float) -> np.ndarray`

- [ ] **Step 1: Write scene-site guard test**

Add to `tests/test_description_scene.py`:

```python
def test_right_chopping_scene_exposes_blade_edge_and_control_sites():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))

    assert _id(model, mujoco.mjtObj.mjOBJ_SITE, "right_blade_edge_top") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_SITE, "right_blade_edge_bot") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_SITE, "right_tool_tip_site") >= 0
```

- [ ] **Step 2: Run scene-site guard test**

Run: `python3 -m pytest tests/test_description_scene.py::test_right_chopping_scene_exposes_blade_edge_and_control_sites -v`

Expected: PASS, because the sites already exist. This is a guard over the existing scene.

- [ ] **Step 3: Write failing blade-geometry tests**

Add to `tests/test_chopping.py`:

```python
def test_blade_geometry_reads_two_finite_edge_positions_and_tip_pose():
    chopper = RightArmChopper()

    geometry = chopper._blade_geometry()

    assert len(geometry.edge_positions) == 2
    assert len(geometry.z_values) == 2
    assert geometry.min_z <= geometry.max_z
    assert len(geometry.tip_position) == 3
    assert np.asarray(geometry.tip_rotation).shape == (3, 3)
    for position in geometry.edge_positions + (geometry.tip_position,):
        assert len(position) == 3
        assert np.all(np.isfinite(position))


def test_horizontal_blade_rotation_predicts_edge_points_at_equal_height():
    chopper = RightArmChopper()
    chopper.runtime.reset()
    chopper.runtime.set_arm_positions("left", LEFT_HOME_Q)
    chopper.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
    geometry = chopper._blade_geometry()

    rotation = chopper._horizontal_blade_rotation(geometry)
    predicted = chopper._predict_blade_edge_positions(geometry.tip_position, rotation, geometry)

    assert abs(predicted[0][2] - predicted[1][2]) <= 1e-9


def test_two_point_safe_target_keeps_both_blade_edges_above_board():
    chopper = RightArmChopper()
    chopper.runtime.reset()
    chopper.runtime.set_arm_positions("left", LEFT_HOME_Q)
    chopper.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
    board_top = chopper._board_top()
    geometry = chopper._blade_geometry()
    rotation = chopper._horizontal_blade_rotation(geometry)

    target = chopper._tip_target_for_blade_clearance(
        np.asarray(geometry.tip_position),
        rotation,
        geometry,
        board_top,
        clearance_m=0.08,
    )
    predicted = chopper._predict_blade_edge_positions(target, rotation, geometry)

    assert all(position[2] >= board_top + 0.08 - 1e-9 for position in predicted)


def test_two_point_descend_target_predicts_both_blade_edges_on_board():
    chopper = RightArmChopper()
    chopper.runtime.reset()
    chopper.runtime.set_arm_positions("left", LEFT_HOME_Q)
    chopper.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
    board_top = chopper._board_top()
    geometry = chopper._blade_geometry()
    rotation = chopper._horizontal_blade_rotation(geometry)

    target = chopper._tip_target_for_blade_on_board(
        np.asarray(geometry.tip_position),
        rotation,
        geometry,
        board_top,
    )
    predicted = chopper._predict_blade_edge_positions(target, rotation, geometry)

    for position in predicted:
        assert abs(position[2] - board_top) <= 1e-9
```

- [ ] **Step 4: Run blade-geometry tests to verify RED**

Run:

```bash
python3 -m pytest \
  tests/test_chopping.py::test_blade_geometry_reads_two_finite_edge_positions_and_tip_pose \
  tests/test_chopping.py::test_horizontal_blade_rotation_predicts_edge_points_at_equal_height \
  tests/test_chopping.py::test_two_point_safe_target_keeps_both_blade_edges_above_board \
  tests/test_chopping.py::test_two_point_descend_target_predicts_both_blade_edges_on_board \
  -v
```

Expected: FAIL with `AttributeError` for `_blade_geometry` or missing helper methods.

- [ ] **Step 5: Implement blade geometry and target helpers**

In `src/twin_mujoco/twin_mujoco/chopping.py`, add `BLADE_EDGE_SITE_NAMES`, `BladeGeometry`, `_blade_geometry`, `_horizontal_blade_rotation`, `_predict_blade_edge_positions`, `_tip_target_for_blade_clearance`, and `_tip_target_for_blade_on_board` using the exact interfaces above.

- [ ] **Step 6: Run blade-geometry tests to verify GREEN**

Run:

```bash
python3 -m pytest \
  tests/test_description_scene.py::test_right_chopping_scene_exposes_blade_edge_and_control_sites \
  tests/test_chopping.py::test_blade_geometry_reads_two_finite_edge_positions_and_tip_pose \
  tests/test_chopping.py::test_horizontal_blade_rotation_predicts_edge_points_at_equal_height \
  tests/test_chopping.py::test_two_point_safe_target_keeps_both_blade_edges_above_board \
  tests/test_chopping.py::test_two_point_descend_target_predicts_both_blade_edges_on_board \
  -v
```

Expected: PASS.

- [ ] **Step 7: Commit Task 1**

```bash
git add src/twin_mujoco/twin_mujoco/chopping.py tests/test_description_scene.py tests/test_chopping.py
git commit -m "feat: add two-point blade geometry"
```

---

### Task 2: Use Two-Point Blade Targets In Chopping Samples

**Files:**
- Modify: `src/twin_mujoco/twin_mujoco/chopping.py`
- Modify: `tests/test_chopping.py`

**Interfaces:**
- Consumes: `BladeGeometry`
- Consumes: `RightArmChopper._blade_geometry() -> BladeGeometry`
- Consumes: `RightArmChopper._horizontal_blade_rotation(...) -> np.ndarray`
- Consumes: `RightArmChopper._tip_target_for_blade_clearance(...) -> np.ndarray`
- Consumes: `RightArmChopper._tip_target_for_blade_on_board(...) -> np.ndarray`
- Produces: `ForceControlSample.blade_edge_positions: tuple[tuple[float, float, float], ...]`
- Produces: `ForceControlSample.target_blade_edge_positions: tuple[tuple[float, float, float], ...]`

- [ ] **Step 1: Write failing sample and run-behavior tests**

Add tests that assert active samples include two actual and target blade-edge positions, `FORCE_HOLD` target blade edges are both at `board_top`, and the first safe target keeps both blade edges at least `safe_height_m` above the board.

- [ ] **Step 2: Run new sample tests to verify RED**

Run the three new `tests/test_chopping.py` tests directly.

Expected: FAIL with `AttributeError` because `ForceControlSample` does not expose blade edge position fields yet.

- [ ] **Step 3: Add sample fields and record blade edge geometry**

Extend `ForceControlSample` with `blade_edge_positions` and `target_blade_edge_positions`. Update `_sample` and `_record_phase` to carry predicted target blade-edge positions while recording actual blade-edge positions from `_blade_geometry()`.

- [ ] **Step 4: Change run target calculation to use horizontal two-point blade targets**

In `RightArmChopper.run`, compute `geometry = self._blade_geometry()`, `rotation = self._horizontal_blade_rotation(geometry)`, and derive `safe`/`descend` from the two-point helpers. Pass `rotation` and predicted safe/descend blade-edge positions through every phase.

- [ ] **Step 5: Run new sample tests to verify GREEN**

Run the three new `tests/test_chopping.py` tests directly.

Expected: PASS.

- [ ] **Step 6: Run existing chopping tests**

Run: `python3 -m pytest tests/test_chopping.py -v`

Expected: PASS.

- [ ] **Step 7: Run full test suite**

Run: `python3 -m pytest -v`

Expected: PASS.

- [ ] **Step 8: Commit Task 2**

```bash
git add src/twin_mujoco/twin_mujoco/chopping.py tests/test_chopping.py
git commit -m "feat: plan chopping targets from blade edge"
```

---

## Self-Review

- Spec coverage: required blade-edge sites are validated, target planning uses the two blade-edge sites, target rotation makes the blade edge horizontal, descent is world `-Z`, CSV remains unchanged, and MuJoCo-only scope is preserved.
- Placeholder scan: no TBD/TODO/fill-in steps remain.
- Type consistency: `BladeGeometry`, `blade_edge_positions`, and `target_blade_edge_positions` names are consistent across both tasks.

## Revision Task: Three-Point Completion And World-Z Admittance

- Add tests for controller world-Z force axis support.
- Add tests that FORCE_HOLD target geometry includes all three blade reference points at `board_top`.
- Keep two blade-edge target fields for compatibility and add three-point reference target fields.
- Change FORCE_HOLD controller input from fixed commanded wrench to measured MuJoCo wrench feedback.
- Set the chopper force axis to world `-Z` during run setup.
- Preserve existing CSV schema.
