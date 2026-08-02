# Wuji Offline Simulation Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the complete self-developed right-glove MCAP to left Wuji Hand MuJoCo workflow into `tianji_robotic_project`, establish the new `tianji_robotics` package boundary, and preserve explicit SDK/ROS 2 real-device interfaces without connecting hardware.

**Architecture:** Domain types and validation live under `tianji_robotics.wuji_hand`; file codecs, official retargeting, simulation, hardware interfaces, and workflow orchestration depend inward on those types. The existing `twin_sim` implementation remains operational while thin `tianji_robotics.simulation` adapters reuse it during this first migration phase. ROS 2 stays in the same Git repository under `ros2_ws`, while official Wuji repositories remain external and unchanged.

**Tech Stack:** Python 3.12, NumPy 2.5.1, MuJoCo 3.10.0, MCAP Python, official `wuji-sdk`, pytest 9.1.1, setuptools, ROS 2 Jazzy interfaces.

## Global Constraints

- Do not modify any independent official Git repository under `../wuji-technology`.
- Do not connect to, discover, enable, or command any physical glove, hand, or arm during implementation or verification.
- `simulation` must not import `wujihandpy`, real-device `wuji_sdk` APIs, Tianji `SDK_PYTHON`, or `rclpy`.
- Official `RetargetSession` must be imported lazily and creating it must not discover hardware.
- All Wuji joint vectors use 20 finite radians in `left_finger1_joint1` through `left_finger5_joint4` order.
- Raw recordings and virtual environments stay Git-ignored.
- Preserve existing `twin-sim` behavior and the existing simulation test suite during this phase.
- Use test-first red-green-refactor cycles for every behavior change.

---

## File Map

**New domain package**

- `src/tianji_robotics/__init__.py`: package marker and version-neutral public namespace.
- `src/tianji_robotics/wuji_hand/models.py`: immutable `SkeletonFrame` and `HandTrajectory` value objects.
- `src/tianji_robotics/wuji_hand/interfaces.py`: `SkeletonSource`, `Retargeter`, `WujiHandBackend`, and `PhysicalWujiHandBackend` protocols.
- `src/tianji_robotics/wuji_hand/names.py`: authoritative joint names.
- `src/tianji_robotics/wuji_hand/transforms.py`: right-to-left wrist coordinate conversion.
- `src/tianji_robotics/wuji_hand/validation.py`: trajectory timing, range, and step validation.

**New adapters and workflow**

- `src/tianji_robotics/data/mcap.py`: Studio skeleton reader and JointState MCAP writer.
- `src/tianji_robotics/data/npz.py`: validated trajectory persistence.
- `src/tianji_robotics/wuji_sdk/retargeter.py`: lazy official `RetargetSession` adapter only.
- `src/tianji_robotics/simulation/wuji_hand.py`: adapter around existing `SimWujiHand`.
- `src/tianji_robotics/simulation/replay.py`: deterministic headless and Viewer replay.
- `src/tianji_robotics/hardware/wuji_hand/sdk.py`: guarded SDK lifecycle interface with injected runtime.
- `src/tianji_robotics/workflows/wuji_glove_replay.py`: offline pipeline orchestration.
- `src/tianji_robotics/cli.py`: `tianji-robot sim wuji-replay` and hardware preflight command tree.

**Packaging, migration, and docs**

- `pyproject.toml`: discover both packages; add `tianji-robot`; add optional `wuji-offline` dependencies.
- `.gitignore`: ignore `.venv-wuji-teleop/` and all `recordings/` content.
- `scripts/setup_wuji_teleop_env.sh`: reproducible environment creation.
- `docs/wuji/offline_replay.md`: local usage and troubleshooting.
- `docs/wuji/hardware_interfaces.md`: SDK versus ROS 2 ownership and safety boundary.
- `recordings/wuji/`: moved local data, ignored by Git.
- `archive/wuji_glove_recorder/`: retained legacy snapshot after verified migration.

---

### Task 1: Establish the installable package and enforced boundaries

**Files:**
- Create: `src/tianji_robotics/__init__.py`
- Create: `src/tianji_robotics/wuji_hand/__init__.py`
- Create: `src/tianji_robotics/simulation/__init__.py`
- Create: `src/tianji_robotics/hardware/__init__.py`
- Create: `src/tianji_robotics/data/__init__.py`
- Create: `src/tianji_robotics/workflows/__init__.py`
- Create: `src/tianji_robotics/wuji_sdk/__init__.py`
- Modify: `pyproject.toml`
- Modify: `.gitignore`
- Create: `tests/architecture/test_package_boundaries.py`

**Interfaces:**
- Produces: importable `tianji_robotics` namespace and the `tianji-robot` console entry point target.
- Produces: static import rules used by every later task.

- [ ] **Step 1: Write failing packaging and boundary tests**

```python
def test_new_package_is_importable():
    import tianji_robotics
    assert tianji_robotics.__name__ == "tianji_robotics"

def test_simulation_never_imports_hardware_packages():
    forbidden = {"SDK_PYTHON", "fx_robot", "fx_kine", "wujihandpy", "rclpy"}
    assert imported_roots(ROOT / "src/tianji_robotics/simulation").isdisjoint(forbidden)

def test_runtime_directories_are_ignored():
    text = (ROOT / ".gitignore").read_text()
    assert "recordings/" in text
    assert ".venv-wuji-teleop/" in text
```

- [ ] **Step 2: Run the tests and confirm RED**

Run: `.venv/bin/python -m pytest tests/architecture/test_package_boundaries.py -q`

Expected: FAIL because `tianji_robotics` and `.venv-wuji-teleop/` do not exist in the package/configuration.

- [ ] **Step 3: Add minimal package markers, package discovery, CLI declaration, and ignore rule**

Use setuptools discovery rather than a manually maintained package list:

```toml
[project.scripts]
twin-sim = "twin_sim.cli:main"
tianji-robot = "tianji_robotics.cli:main"

[project.optional-dependencies]
wuji-offline = ["mcap>=1.3,<2", "wuji-sdk>=2026.7.21"]

[tool.setuptools.packages.find]
where = ["src"]
include = ["twin_sim*", "tianji_robotics*"]
```

Add `.venv-wuji-teleop/` to `.gitignore`; keep the existing `recordings/` rule.

- [ ] **Step 4: Run the focused and existing package tests**

Run: `.venv/bin/python -m pytest tests/architecture/test_package_boundaries.py tests/simulation/test_repository_boundaries.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add .gitignore pyproject.toml src/tianji_robotics tests/architecture
git commit -m "refactor: establish tianji robotics package boundaries"
```

---

### Task 2: Define Wuji domain models, contracts, transforms, and validation

**Files:**
- Create: `src/tianji_robotics/wuji_hand/models.py`
- Create: `src/tianji_robotics/wuji_hand/interfaces.py`
- Create: `src/tianji_robotics/wuji_hand/names.py`
- Create: `src/tianji_robotics/wuji_hand/transforms.py`
- Create: `src/tianji_robotics/wuji_hand/validation.py`
- Create: `tests/wuji_hand/test_models.py`
- Create: `tests/wuji_hand/test_transforms.py`
- Create: `tests/wuji_hand/test_validation.py`

**Interfaces:**
- Produces: `SkeletonFrame(timestamp_ns: int, frame_id: str, side: Literal["left", "right"], keypoints_m: np.ndarray)`.
- Produces: `HandTrajectory(timestamps_ns: np.ndarray, positions_rad: np.ndarray, joint_names: tuple[str, ...], metadata: Mapping[str, object])`.
- Produces: `validate_trajectory(trajectory, joint_ranges_rad, *, max_step_rad) -> HandTrajectory`.
- Produces: `mirror_right_to_left(keypoints_m) -> np.ndarray`.

- [ ] **Step 1: Write failing value-object and mirror tests**

```python
def test_mirror_flips_only_y_and_copies_input():
    source = np.arange(63, dtype=np.float32).reshape(21, 3)
    result = mirror_right_to_left(source)
    np.testing.assert_array_equal(result[:, 0], source[:, 0])
    np.testing.assert_array_equal(result[:, 1], -source[:, 1])
    np.testing.assert_array_equal(result[:, 2], source[:, 2])
    assert not np.shares_memory(result, source)

def test_trajectory_requires_strictly_increasing_timestamps():
    with pytest.raises(ValueError, match="strictly increasing"):
        HandTrajectory(np.array([10, 10]), np.zeros((2, 20)), HAND_JOINT_NAMES, {})
```

- [ ] **Step 2: Run and confirm RED**

Run: `.venv/bin/python -m pytest tests/wuji_hand -q`

Expected: collection FAIL because the domain modules do not exist.

- [ ] **Step 3: Implement immutable validated models and protocols**

Use frozen dataclasses that copy arrays in `__post_init__`; require `(21, 3)` finite skeletons, `(N, 20)` finite trajectories, non-empty frames, exactly 20 unique canonical names, and strictly increasing timestamps. Define protocols without importing any concrete backend.

```python
@runtime_checkable
class Retargeter(Protocol):
    def step(self, keypoints_m: np.ndarray) -> np.ndarray: ...

@runtime_checkable
class WujiHandBackend(Protocol):
    def read_position_rad(self) -> np.ndarray: ...
    def command_position_rad(self, target: np.ndarray) -> None: ...
    def step(self, duration_s: float) -> None: ...
    def close(self) -> None: ...
```

- [ ] **Step 4: Add failing range and step-limit tests**

```python
def test_validation_reports_first_out_of_range_joint():
    positions = valid_positions()
    positions[0, 3] = 2.0
    trajectory = trajectory_with(positions)
    with pytest.raises(ValueError, match="frame 0.*left_finger1_joint4"):
        validate_trajectory(trajectory, ranges(), max_step_rad=0.2)

def test_validation_reports_excessive_frame_step():
    positions = valid_positions(frame_count=2)
    positions[1, 7] += 0.3
    trajectory = trajectory_with(positions)
    with pytest.raises(ValueError, match="frame 1.*max step"):
        validate_trajectory(trajectory, ranges(), max_step_rad=0.2)
```

- [ ] **Step 5: Implement minimal range and step validation, then run tests**

Run: `.venv/bin/python -m pytest tests/wuji_hand -q`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/tianji_robotics/wuji_hand tests/wuji_hand
git commit -m "feat: add validated Wuji hand domain contracts"
```

---

### Task 3: Migrate MCAP/NPZ codecs and the official retarget adapter

**Files:**
- Create: `src/tianji_robotics/data/mcap.py`
- Create: `src/tianji_robotics/data/npz.py`
- Create: `src/tianji_robotics/wuji_sdk/retargeter.py`
- Create: `tests/data/test_wuji_mcap.py`
- Create: `tests/data/test_hand_npz.py`
- Create: `tests/wuji_sdk/test_retargeter.py`

**Interfaces:**
- Consumes: `SkeletonFrame`, `HandTrajectory`, `Retargeter`, and `HAND_JOINT_NAMES` from Task 2.
- Produces: `StudioMcapSkeletonSource(path: Path).frames() -> Iterator[SkeletonFrame]`.
- Produces: `write_joint_state_mcap(trajectory, destination) -> Path`.
- Produces: `save_trajectory_npz(trajectory, destination) -> Path` and `load_trajectory_npz(path) -> HandTrajectory`.
- Produces: `OfficialWujiRetargeter.create_left_first_generation() -> OfficialWujiRetargeter`.

- [ ] **Step 1: Write a failing generated-MCAP parser test**

Generate two `/right_glove/hand_skeleton` JSON messages and one unrelated message with `mcap.writer.Writer`; assert timestamps become nanoseconds, frame id is `r_wrist`, side is `right`, unrelated topics are ignored, and malformed frames name the source path.

- [ ] **Step 2: Run and confirm RED**

Run: `.venv-wuji-teleop/bin/python -m pytest tests/data/test_wuji_mcap.py -q`

Expected: FAIL because `StudioMcapSkeletonSource` does not exist.

- [ ] **Step 3: Implement MCAP parsing by adapting the proven recorder logic**

Parse only `SKELETON_TOPIC = "/right_glove/hand_skeleton"`; convert `timestamp_us * 1_000`; create validated `SkeletonFrame` objects. Do not import any SDK.

- [ ] **Step 4: Write failing NPZ and JointState round-trip tests**

Assert timestamps, positions, canonical names, and metadata survive NPZ load/save. Read the generated JointState MCAP and assert 20 names and positions on `/joint_states`.

- [ ] **Step 5: Implement codecs and run their tests**

Run: `.venv-wuji-teleop/bin/python -m pytest tests/data -q`

Expected: PASS.

- [ ] **Step 6: Write the failing lazy-import retargeter test**

```python
def test_adapter_uses_left_first_generation_without_device_manager(monkeypatch):
    fake = fake_wuji_sdk_module()
    monkeypatch.setitem(sys.modules, "wuji_sdk", fake)
    adapter = OfficialWujiRetargeter.create_left_first_generation()
    result = adapter.step(np.zeros((21, 3), dtype=np.float32))
    assert result.shape == (20,)
    assert fake.calls == [(fake.HandModel.WujiHand, fake.Handedness.Left)]
```

- [ ] **Step 7: Implement lazy official adapter and verify all focused tests**

The module top level must not import `wuji_sdk`; the factory imports only `HandModel`, `Handedness`, and `RetargetSession`, then validates every returned command.

Run: `.venv-wuji-teleop/bin/python -m pytest tests/data tests/wuji_sdk -q`

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add src/tianji_robotics/data src/tianji_robotics/wuji_sdk tests/data tests/wuji_sdk
git commit -m "feat: migrate Wuji recording and retarget adapters"
```

---

### Task 4: Build the offline pipeline and MuJoCo replay backend

**Files:**
- Create: `src/tianji_robotics/workflows/wuji_glove_replay.py`
- Create: `src/tianji_robotics/simulation/wuji_hand.py`
- Create: `src/tianji_robotics/simulation/replay.py`
- Create: `tests/workflows/test_wuji_glove_replay.py`
- Create: `tests/simulation/test_wuji_trajectory_replay.py`

**Interfaces:**
- Consumes: Tasks 2–3 domain models, source, codecs, and retargeter.
- Produces: `retarget_recording(source, retargeter) -> HandTrajectory`.
- Produces: `MujocoWujiHand(robot: RightArmRobot | None = None, viewer: bool = False)` implementing `WujiHandBackend`.
- Produces: `replay_trajectory(trajectory, backend, *, realtime: bool) -> ReplaySummary`.

- [ ] **Step 1: Write a failing pure pipeline test with fakes**

```python
def test_pipeline_mirrors_every_frame_before_retargeting():
    source = FakeSource(two_right_frames())
    retargeter = RecordingRetargeter()
    trajectory = retarget_recording(source, retargeter)
    assert trajectory.positions_rad.shape == (2, 20)
    np.testing.assert_array_equal(retargeter.inputs[0][:, 1], -source.items[0].keypoints_m[:, 1])
    assert trajectory.metadata["input_transform"] == "mirror_y_right_wrist_to_left_wrist"
```

- [ ] **Step 2: Run and confirm RED**

Run: `.venv/bin/python -m pytest tests/workflows/test_wuji_glove_replay.py -q`

Expected: FAIL because the workflow does not exist.

- [ ] **Step 3: Implement the pure pipeline**

Reject non-right input, mirror every frame, require 20 finite values, and preserve source timestamps. Do not perform file writes or backend calls inside `retarget_recording`.

- [ ] **Step 4: Write a failing headless replay test**

Use a two-frame in-range `HandTrajectory`, construct `MujocoWujiHand(viewer=False)`, replay without wall-clock sleeping, and assert simulation time increases, two targets are accepted, and summary fields are `frame_count == 2` and `duration_s > 0`.

- [ ] **Step 5: Implement the adapter and timestamp-based replay**

Wrap existing `twin_sim.wuji_hand_backend.SimWujiHand`; expose flat 20-element radians while preserving its internal `(5, 4)` SDK shape. For each interval, split duration into valid MuJoCo control steps. Viewer mode optionally sleeps to match timestamps; headless mode never sleeps.

- [ ] **Step 6: Run workflow, simulation, and existing hand tests**

Run: `.venv/bin/python -m pytest tests/workflows tests/simulation/test_wuji_trajectory_replay.py tests/simulation/test_wuji_hand_backend.py -q`

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/tianji_robotics/workflows src/tianji_robotics/simulation tests/workflows tests/simulation/test_wuji_trajectory_replay.py
git commit -m "feat: replay Wuji glove trajectories in MuJoCo"
```

---

### Task 5: Add explicit CLI domains and inert real-device interfaces

**Files:**
- Create: `src/tianji_robotics/cli.py`
- Create: `src/tianji_robotics/hardware/wuji_hand/__init__.py`
- Create: `src/tianji_robotics/hardware/wuji_hand/sdk.py`
- Create: `tests/cli/test_tianji_robot_cli.py`
- Create: `tests/hardware/test_wuji_sdk_lifecycle.py`
- Modify: `ros2_ws/src/twin_wuji_sim/README.md` or its existing package documentation file.

**Interfaces:**
- Consumes: offline pipeline and replay from Task 4.
- Produces: `tianji-robot sim wuji-replay SOURCE [--headless] [--npz PATH] [--joint-state-mcap PATH]`.
- Produces: `SdkWujiHand(runtime)` with explicit `connect()`, `arm()`, `command_position_rad()`, `disarm()`, `close()`.
- Produces: `tianji-robot hardware wuji-sdk preflight TRAJECTORY`; no command in this task permits physical motion.

- [ ] **Step 1: Write failing CLI routing tests**

Assert `sim wuji-replay --headless` calls the offline workflow with viewer disabled; assert `hardware wuji-sdk preflight` only loads and validates a trajectory; assert `--arm` is rejected because physical execution is outside this phase; assert simulation parsers expose no hardware options.

- [ ] **Step 2: Run and confirm RED**

Run: `.venv/bin/python -m pytest tests/cli/test_tianji_robot_cli.py -q`

Expected: FAIL because the CLI does not exist.

- [ ] **Step 3: Implement the CLI with dependency errors that identify the missing optional extra**

Return exit code 2 and a concise message when `mcap` or `wuji_sdk` is unavailable. Keep `twin-sim` unchanged.

- [ ] **Step 4: Write failing guarded lifecycle tests**

```python
def test_constructing_sdk_backend_does_not_connect():
    runtime = FakeRuntime()
    SdkWujiHand(runtime)
    assert runtime.calls == []

def test_command_requires_connect_and_arm():
    hand = SdkWujiHand(FakeRuntime())
    with pytest.raises(RuntimeError, match="not connected"):
        hand.command_position_rad(np.zeros(20))
```

Also cover `connect → arm → command → disarm → close`, failure cleanup, and shape validation using only fakes.

- [ ] **Step 5: Implement the injected SDK lifecycle without importing vendor packages**

This task defines the stable safety contract only. A future hardware plan supplies the official runtime implementation after on-device review.

- [ ] **Step 6: Document ROS 2 versus SDK ownership and run focused tests**

Run: `.venv/bin/python -m pytest tests/cli tests/hardware tests/architecture -q`

Expected: PASS and no physical SDK import or connection.

- [ ] **Step 7: Commit**

```bash
git add src/tianji_robotics/cli.py src/tianji_robotics/hardware tests/cli tests/hardware ros2_ws/src
git commit -m "feat: separate simulation and Wuji hardware entry points"
```

---

### Task 6: Create the reproducible environment and migrate local data/docs safely

**Files:**
- Create: `scripts/setup_wuji_teleop_env.sh`
- Create: `docs/wuji/offline_replay.md`
- Create: `docs/wuji/hardware_interfaces.md`
- Modify: `README.md`
- Move locally: `../wuji-record-data/*` to `recordings/wuji/`
- Archive in repository: relevant source/document history from `../wuji-technology/wuji-glove-recorder`, `../wuji-technology/doc_zt`, and `../wuji-technology/docs/superpowers` only after new tests pass.
- Create: `tests/documentation/test_wuji_docs.py`

**Interfaces:**
- Produces: one idempotent setup command and one documented replay command.
- Produces: a recoverable migration record without modifying official nested Git repositories.

- [ ] **Step 1: Write failing documentation/static script tests**

Assert the setup script uses project-relative paths, creates `.venv-wuji-teleop`, installs the project with `[wuji-offline,test]`, contains no `/home/zhoutong`, and the docs reference `tianji-robot sim wuji-replay`.

- [ ] **Step 2: Run and confirm RED**

Run: `.venv/bin/python -m pytest tests/documentation/test_wuji_docs.py -q`

Expected: FAIL because the files do not exist.

- [ ] **Step 3: Implement the POSIX setup script and docs**

The script must run `python3.12 -m venv "$PROJECT_ROOT/.venv-wuji-teleop"`, upgrade the declared build tools, and install `-e "$PROJECT_ROOT[wuji-offline,test]"`. Do not activate or alter the caller shell.

- [ ] **Step 4: Move recordings into the ignored destination with read-only verification**

Before moving, record file count, byte total, and SHA-256 list. Move to `recordings/wuji/`, then verify the same count, byte total, and hashes. Do not stage recording files.

- [ ] **Step 5: Archive self-developed Wuji sources and docs after parity tests**

Use a recoverable move into `archive/wuji_glove_recorder/legacy_2026_08_02/` for source history that is not represented by Git. Do not move or edit directories containing an official `.git`. Update documentation links to the new locations.

- [ ] **Step 6: Run documentation tests and confirm Git ignores local artifacts**

Run: `.venv/bin/python -m pytest tests/documentation/test_wuji_docs.py -q`

Run: `git status --short --ignored recordings .venv-wuji-teleop`

Expected: docs test PASS; recordings/environment appear only as ignored entries.

- [ ] **Step 7: Commit only source, script, docs, tests, and archived text/code**

```bash
git add README.md scripts/setup_wuji_teleop_env.sh docs/wuji tests/documentation archive/wuji_glove_recorder
git commit -m "docs: consolidate Wuji workflow and environment"
```

---

### Task 7: Verify the complete local workflow

**Files:**
- No source files; this task verifies the committed deliverable.

**Interfaces:**
- Verifies the deliverable; produces no new feature surface.

- [ ] **Step 1: Run the complete automated suite**

Run: `.venv/bin/python -m pytest -q`

Expected: all existing and new default-environment tests PASS.

- [ ] **Step 2: Run the Wuji environment suite**

Run: `.venv-wuji-teleop/bin/python -m pytest tests/wuji_hand tests/data tests/wuji_sdk tests/workflows tests/cli tests/hardware -q`

Expected: PASS.

- [ ] **Step 3: Run a real-recording conversion and headless MuJoCo replay**

Run:

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-replay \
  recordings/wuji/august_02/session_20260802_162909_764.mcap \
  --headless \
  --npz recordings/wuji/august_02/session_20260802_162909_764_refactored.npz \
  --joint-state-mcap recordings/wuji/august_02/session_20260802_162909_764_refactored.mcap
```

Expected: exit 0, nonzero frame count, finite validated 20-joint trajectory, generated ignored outputs, and completed Headless MuJoCo replay.

- [ ] **Step 4: Run hardware-side-effect audit**

Run static import tests plus `tianji-robot hardware wuji-sdk preflight` against the generated trajectory. Confirm no USB discovery, ROS node, device connection, enable, or command occurs.

- [ ] **Step 5: Run Viewer acceptance when a display is available**

Run the same replay without `--headless`; visually confirm correct left-hand flexion direction, timing, and no sustained joint-limit saturation. If no display is available, record Viewer verification as not run rather than claiming it passed.

- [ ] **Step 6: Check repository and official-repo integrity**

Run: `git status --short`

Run `git -C` status and remote checks for each official repository under `../wuji-technology`; expect no changes introduced by this work and unchanged official remotes.

---

## Deferred Follow-up Plans

The following independently reviewable safety-critical work is intentionally not mixed into this first delivery:

1. `real_robot_debug` decomposition into production Tianji arm domain, SDK adapter, and hardware workflows.
2. Official Wuji SDK physical runtime implementation and on-device lifecycle verification.
3. Wuji ROS 2 physical replay adapter and SDK/ROS ownership lock integration.
4. Removal of the `twin_sim` compatibility package after all imports and ROS packages migrate.

This first plan still leaves complete, typed, guarded interfaces for those follow-ups and delivers the requested end-to-end offline simulation locally.
