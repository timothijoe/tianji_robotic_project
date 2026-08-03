# Wuji Recorded Palm-Down Retreat Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive the tabletop retreat from the user's 499-frame recording with the palm facing down, preserving recorded finger coordination while applying only collision and continuity corrections.

**Architecture:** Add a topic-detecting joint-state MCAP source beside the existing glove-skeleton source. Split recorded-gesture analysis from MuJoCo placement: the workflow detects the active interval, preserves recorded joints/timestamps, applies an explicit official-model palm-side calibration, and projects only the palm height for table safety. The existing CLI chooses the correct source adapter without relying on filenames.

**Tech Stack:** Python 3.12, NumPy, MuJoCo, MCAP JSON schema messages, pytest, existing official Wuji retargeter.

## Global Constraints

- Primary acceptance input: `recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap`.
- Preserve all 499 joint samples and relative timestamps.
- Never zero long-finger joints or generate a common synthetic curl target.
- Palmar reference must be below dorsal reference at PLACE.
- Thumb clearance must be at least `0.010 m`.
- Real hand/table penetration tolerance is `0.0005 m`.
- Default palm retreat is `0.030 ± 0.002 m` and occurs only in the detected active interval.
- Joint steps remain at most `0.12 rad`.
- Middle/ring correlation remains at least `0.85`; ring/little remains at least `0.75` when the source satisfies those thresholds.
- Do not modify or track recording files or `../wuji-technology`.

---

### Task 1: Direct canonical JointState MCAP loading

**Files:**
- Modify: `src/tianji_robotics/data/mcap.py`
- Modify: `tests/data/test_mcap.py`

**Interfaces:**
- Consumes: `/joint_states` JSON messages written by `write_joint_state_mcap`.
- Produces: `detect_hand_mcap_kind(path: Path) -> Literal["joint_states", "right_glove_skeleton"]` and `JointStateMcapSource(path).trajectory() -> HandTrajectory`.

- [ ] **Step 1: Write failing direct-load tests**

```python
def test_joint_state_source_round_trips_canonical_trajectory(tmp_path):
    expected = trajectory()
    path = write_joint_state_mcap(expected, tmp_path / "left.mcap")
    assert detect_hand_mcap_kind(path) == "joint_states"
    actual = JointStateMcapSource(path).trajectory()
    np.testing.assert_allclose(actual.positions_rad, expected.positions_rad)
    np.testing.assert_array_equal(actual.timestamps_ns, expected.timestamps_ns)
    assert actual.joint_names == HAND_JOINT_NAMES


def test_detects_right_glove_skeleton_fixture(skeleton_mcap):
    assert detect_hand_mcap_kind(skeleton_mcap) == "right_glove_skeleton"
```

- [ ] **Step 2: Run the tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/data/test_mcap.py`

Expected: import or attribute failure for the new detector/source.

- [ ] **Step 3: Implement topic detection and strict JointState decoding**

Inspect summary channel topics once. For `/joint_states`, decode JSON, require exactly the canonical names, reorder positions by names when all names are unique, validate 20 finite positions, and return `HandTrajectory`. For `/right_glove/hand_skeleton`, return the existing kind. Reject ambiguous files containing both or neither supported topic.

- [ ] **Step 4: Run data tests and verify GREEN**

Run: `.venv-wuji-teleop/bin/pytest -q tests/data/test_mcap.py`

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/tianji_robotics/data/mcap.py tests/data/test_mcap.py
git commit -m "feat: load canonical Wuji joint-state MCAP"
```

---

### Task 2: Recorded motion analysis and preservation

**Files:**
- Create: `src/tianji_robotics/workflows/recorded_hand_motion.py`
- Create: `tests/workflows/test_recorded_hand_motion.py`

**Interfaces:**
- Consumes: canonical `HandTrajectory`.
- Produces: `MotionInterval(start_frame: int, end_frame: int)`, `detect_motion_interval(trajectory)`, `finger_displacement_correlations(positions_rad)`, and `smooth_joint_step_outliers(positions_rad, limit_rad=0.12)`.

- [ ] **Step 1: Write failing interval and correlation tests**

```python
def test_detects_dominant_interval_without_changing_samples():
    trajectory = synthetic_recording(active_start=320, active_end=457, frames=499)
    interval = detect_motion_interval(trajectory)
    assert abs(interval.start_frame - 320) <= 15
    assert abs(interval.end_frame - 457) <= 15


def test_correlations_keep_index_separate_and_long_fingers_coupled():
    correlations = finger_displacement_correlations(recorded_positions())
    assert correlations["middle_ring"] >= .85
    assert correlations["ring_little"] >= .75
    assert correlations["index_middle"] < correlations["middle_ring"]
```

- [ ] **Step 2: Run tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_recorded_hand_motion.py`

Expected: module import failure.

- [ ] **Step 3: Implement deterministic analysis**

Compute per-frame motion energy from the norm of 20-joint differences, smooth with a bounded window, and select the dominant contiguous interval using a relative threshold and hysteresis. Correlations use per-finger displacement magnitude from the first frame. The smoothing API copies input and modifies only steps exceeding `0.12 rad`.

- [ ] **Step 4: Add the real NPZ evidence test when the ignored fixture exists**

Load `session_20260802_174440_936_right_to_left_wuji_hand.npz` with `pytest.skip` when absent. Assert 499 frames, interval boundaries within 20 frames of `320–457`, middle/ring at least `.85`, and ring/little at least `.75`.

- [ ] **Step 5: Run tests and verify GREEN**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_recorded_hand_motion.py`

Expected: all tests pass, including the local real-data evidence test.

- [ ] **Step 6: Commit**

```bash
git add src/tianji_robotics/workflows/recorded_hand_motion.py tests/workflows/test_recorded_hand_motion.py
git commit -m "feat: analyze recorded Wuji finger motion"
```

---

### Task 3: Explicit palmar-side calibration

**Files:**
- Modify: `src/tianji_robotics/simulation/tabletop_wuji_hand.py`
- Modify: `src/tianji_robotics/workflows/wuji_table_retreat.py`
- Modify: `tests/simulation/test_tabletop_wuji_hand.py`
- Modify: `tests/workflows/test_wuji_table_retreat.py`

**Interfaces:**
- Consumes: official fixed left-hand model and recorded opening pose.
- Produces: `palmar_reference_position_m()`, `dorsal_reference_position_m()`, and `_calibrated_palm_down_quaternion(...)` with an automated palmar-below-dorsal assertion.

- [ ] **Step 1: Write the failing orientation regression test**

```python
def test_calibrated_place_has_palmar_side_below_dorsal_side():
    backend = TabletopWujiHand(viewer=False)
    try:
        quaternion = _calibrated_palm_down_quaternion(backend, FEASIBLE_POSE)
        backend.set_kinematic_pose(FEASIBLE_POSE, [0, 0, .2], quaternion)
        assert backend.palmar_reference_position_m()[2] < backend.dorsal_reference_position_m()[2]
    finally:
        backend.close()
```

- [ ] **Step 2: Run the test and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_tabletop_wuji_hand.py tests/workflows/test_wuji_table_retreat.py`

Expected: missing reference-site or calibration API failure.

- [ ] **Step 3: Add fixed model reference sites and calibrated orientation**

Add two named sites on opposite known sides of `palm_link`, derived from the fixed official asset coordinate convention. Fit horizontal extension/lateral axes, evaluate the two 180-degree normal candidates, and select only the candidate that places the palmar site below the dorsal site. Remove the misleading hard-coded `normal -> -Z` assumption.

- [ ] **Step 4: Run orientation tests and verify GREEN**

Run: `.venv-wuji-teleop/bin/pytest -q tests/simulation/test_tabletop_wuji_hand.py tests/workflows/test_wuji_table_retreat.py`

Expected: orientation regression passes and existing table collision tests remain green.

- [ ] **Step 5: Commit**

```bash
git add src/tianji_robotics/simulation/tabletop_wuji_hand.py src/tianji_robotics/workflows/wuji_table_retreat.py tests/simulation/test_tabletop_wuji_hand.py tests/workflows/test_wuji_table_retreat.py
git commit -m "fix: calibrate Wuji palmar side toward table"
```

---

### Task 4: Recording-driven table-retreat workflow

**Files:**
- Modify: `src/tianji_robotics/workflows/wuji_table_retreat.py`
- Modify: `src/tianji_robotics/cli.py`
- Modify: `src/tianji_robotics/data/table_retreat.py`
- Modify: `tests/workflows/test_wuji_table_retreat.py`
- Modify: `tests/cli/test_tianji_robot_cli.py`
- Modify: `tests/data/test_table_retreat_npz.py`

**Interfaces:**
- Consumes: direct/fallback trajectories, `MotionInterval`, and palm calibration.
- Produces: corrected trajectory retaining source frames/timing and an expanded `CorrectionReport` containing interval, source kind, correction norms, correlations, and safety metrics.

- [ ] **Step 1: Write failing recording-preservation tests**

```python
def test_recorded_workflow_preserves_samples_and_moves_palm_only_in_active_interval():
    corrected, report = build_recorded_table_retreat(recording, backend)
    assert len(corrected.positions_rad) == len(recording.positions_rad)
    np.testing.assert_array_equal(np.diff(corrected.timestamps_ns), np.diff(recording.timestamps_ns))
    assert report.motion_start_frame > 0
    before = corrected.palm_positions_m[:report.motion_start_frame, :2]
    assert np.max(np.ptp(before, axis=0)) < 1e-9
    assert report.actual_retreat_m == pytest.approx(.03, abs=.002)


def test_recorded_workflow_does_not_zero_or_replace_finger_motion():
    corrected, report = build_recorded_table_retreat(recording, backend)
    assert report.maximum_joint_correction_rad < .12
    assert report.corrected_middle_ring_correlation >= .85
    assert report.corrected_ring_little_correlation >= .75
```

- [ ] **Step 2: Run focused tests and verify RED**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py tests/cli/test_tianji_robot_cli.py tests/data/test_table_retreat_npz.py`

Expected: missing recording-driven workflow/report fields.

- [ ] **Step 3: Replace synthetic seed/curl generation**

Delete `place_seed[4:] = 0` and the common `curl_target` loop. Copy recorded samples, apply only range/step correction, compute the active interval, and generate a smooth 30 mm palm translation over that interval. Project palm Z per frame for collision safety without solving planted fingertip positions during retreat.

- [ ] **Step 4: Route CLI by MCAP topics**

For `joint_states`, load directly. For `right_glove_skeleton`, call the existing official retargeter. Pass `source_kind` into the report. Preserve the same public command and hardware-free boundary.

- [ ] **Step 5: Expand report/NPZ serialization tests**

Assert motion boundaries, source kind, overall/per-finger correction norms, source/corrected correlations, penetration, thumb clearance, and retreat distance serialize as plain JSON/NPZ values without pickle.

- [ ] **Step 6: Run focused tests and verify GREEN**

Run: `.venv-wuji-teleop/bin/pytest -q tests/workflows/test_wuji_table_retreat.py tests/cli/test_tianji_robot_cli.py tests/data/test_table_retreat_npz.py`

Expected: all focused tests pass.

- [ ] **Step 7: Commit**

```bash
git add src/tianji_robotics/workflows/wuji_table_retreat.py src/tianji_robotics/cli.py src/tianji_robotics/data/table_retreat.py tests/workflows/test_wuji_table_retreat.py tests/cli/test_tianji_robot_cli.py tests/data/test_table_retreat_npz.py
git commit -m "feat: drive Wuji table retreat from recording"
```

---

### Task 5: Real-recording and visual acceptance

**Files:**
- Modify: `docs/wuji/table_retreat.md`
- Modify: `docs/wuji/development_status.md`
- Modify: `tests/documentation/test_wuji_docs.py`

**Interfaces:**
- Consumes: completed CLI and the two user-provided recordings.
- Produces: reproducible commands, numerical report, and PREPARE/mid-RETREAT/HOLD screenshots.

- [ ] **Step 1: Run real left-hand MCAP Headless validation**

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap \
  --headless --report recordings/verification/wuji_recorded_retreat.json
```

Expected: 499 frames, palm-down check true, motion interval near `320–457`, retreat `0.030 ± 0.002 m`, penetration at most `0.0005 m`, thumb clearance at least `0.010 m`, and preserved correlations.

- [ ] **Step 2: Run raw right-glove fallback validation**

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_174440_936.mcap \
  --headless --report recordings/verification/wuji_recorded_retreat_from_raw.json
```

Expected: source kind is `right_glove_skeleton`; safety and correlation thresholds pass.

- [ ] **Step 3: Perform three-stage two-angle Viewer acceptance**

Capture PREPARE, mid-RETREAT, and HOLD. Reject the result if the palm faces up, if the table intersects the hand, if the thumb touches, or if the recorded asymmetric finger coordination is visibly replaced by uniform curling.

- [ ] **Step 4: Update documentation and its test**

Publish the new preferred command, source/fallback semantics, measured interval and metrics, and the three screenshot paths. Remove the older `162909_764` recording as the visual baseline.

- [ ] **Step 5: Run focused and full regression**

```bash
.venv-wuji-teleop/bin/pytest -q tests/data/test_mcap.py tests/workflows/test_recorded_hand_motion.py tests/simulation/test_tabletop_wuji_hand.py tests/workflows/test_wuji_table_retreat.py tests/cli/test_tianji_robot_cli.py tests/data/test_table_retreat_npz.py tests/documentation/test_wuji_docs.py
.venv-wuji-teleop/bin/pytest -q
```

Expected: focused and full suites pass with no new warning category.

- [ ] **Step 6: Commit evidence**

```bash
git add docs/wuji/table_retreat.md docs/wuji/development_status.md tests/documentation/test_wuji_docs.py
git commit -m "docs: verify recorded palm-down Wuji retreat"
```

