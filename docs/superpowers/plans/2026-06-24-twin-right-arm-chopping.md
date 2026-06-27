# Twin Right-Arm Chopping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first pure-MuJoCo phase of `twin_joint_ws`: a dual-arm workspace where the right arm performs at least 3 force-controlled chopping cycles headlessly and logs the run.

**Architecture:** Use a ROS2 workspace-style source layout, but implement Python-only packages for phase one. `twin_core` owns backend-independent arm data, `twin_description` owns model paths and the task MJCF, and `twin_mujoco` owns MuJoCo loading, right-arm views, control, chopping state, CLI, and logging.

**Tech Stack:** Python 3, pytest, numpy, mujoco Python package, MJCF XML, stdlib `csv`, stdlib `argparse`, stdlib `xml.etree.ElementTree` for validation helpers only.

## Global Constraints

- Keep all git operations rooted in `twin_joint_ws`.
- Do not modify `MarvinCCS/marvin_final_fixed.xml`.
- Do not modify `MarvinCCS/` meshes or `MarvinCCS_mujoco.zip`.
- Phase one implements pure MuJoCo only; do not add ROS2 nodes, launch files, or RViz configuration.
- Create packages `src/twin_core`, `src/twin_description`, and `src/twin_mujoco`.
- The original source model exposes 14 hinge joints: `left_joint1..left_joint7` and `right_joint1..right_joint7`.
- The original source model exposes 14 actuators: `act_left_joint1..act_left_joint7` and `act_right_joint1..act_right_joint7`.
- Add a new scene XML at `src/twin_description/twin_description/assets/robot/mujoco/right_chopping_scene.xml`.
- The new scene must include the complete dual-arm robot, a floor, `chopping_board`, `right_force_sensor_body`, `right_force_sensor_site`, `right_tool_body`, `right_tool_tip_site`, and MuJoCo force/torque sensors.
- Default chopping demo must run right arm only and complete at least 3 cycles.
- Tests must be written before implementation changes for behavior changes.

---

## File Map

- Create `pyproject.toml`: package metadata, pytest config, editable install support.
- Create `src/twin_core/twin_core/__init__.py`: public exports for arm data structures.
- Create `src/twin_core/twin_core/arms.py`: `ArmId`, `ArmSpec`, dual-arm constants, and selection validation.
- Create `src/twin_core/twin_core/trajectory.py`: finite vector and trajectory validation.
- Create `src/twin_description/twin_description/__init__.py`: public path helper exports.
- Create `src/twin_description/twin_description/paths.py`: source and chopping-scene path helpers.
- Create `src/twin_description/twin_description/assets/robot/mujoco/right_chopping_scene.xml`: copied dual-arm scene with right-side board/tool/sites/sensors.
- Create `src/twin_description/twin_description/assets/robot/mujoco/README.md`: board/tool/sensor conventions.
- Create `src/twin_mujoco/twin_mujoco/__init__.py`: public runtime/control exports.
- Create `src/twin_mujoco/twin_mujoco/errors.py`: MuJoCo-specific exception classes.
- Create `src/twin_mujoco/twin_mujoco/runtime.py`: `TwinMujocoRuntime`, `ArmView`, pose queries, mappings, stepping.
- Create `src/twin_mujoco/twin_mujoco/control.py`: impedance, wrench calibration, force controller.
- Create `src/twin_mujoco/twin_mujoco/chopping.py`: chopping state machine, samples, CSV logging.
- Create `src/twin_mujoco/twin_mujoco/cli.py`: `twin-chop` command.
- Create `tests/test_source_model.py`: source asset loading tests.
- Create `tests/test_core_arms.py`: backend-independent arm validation tests.
- Create `tests/test_description_scene.py`: scene loading and task element tests.
- Create `tests/test_runtime_arm_view.py`: runtime and arm projection tests.
- Create `tests/test_control.py`: force-control math and safety tests.
- Create `tests/test_chopping.py`: 3-cycle headless demo and CSV tests.

---

### Task 1: Python Workspace Skeleton And Source Model Guard

**Files:**
- Create: `pyproject.toml`
- Create: `src/twin_core/twin_core/__init__.py`
- Create: `src/twin_description/twin_description/__init__.py`
- Create: `src/twin_mujoco/twin_mujoco/__init__.py`
- Create: `tests/test_source_model.py`

**Interfaces:**
- Produces: editable install via `pip install -e .`
- Produces: pytest config with `pythonpath = ["src/twin_core", "src/twin_description", "src/twin_mujoco"]`
- Produces: source model test that proves `MarvinCCS/marvin_final_fixed.xml` loads and remains the untouched source asset.

- [ ] **Step 1: Write the failing source model test**

Create `tests/test_source_model.py`:

```python
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
SOURCE_MODEL = ROOT / "MarvinCCS" / "marvin_final_fixed.xml"


def _names(model, object_type, count):
    names = []
    for index in range(count):
        name = mujoco.mj_id2name(model, object_type, index)
        if name:
            names.append(name)
    return names


def test_source_model_loads_with_expected_dual_arm_joints_and_actuators():
    model = mujoco.MjModel.from_xml_path(str(SOURCE_MODEL))

    assert model.njnt == 14
    assert model.nu == 14
    assert _names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt) == [
        "left_joint1",
        "left_joint2",
        "left_joint3",
        "left_joint4",
        "left_joint5",
        "left_joint6",
        "left_joint7",
        "right_joint1",
        "right_joint2",
        "right_joint3",
        "right_joint4",
        "right_joint5",
        "right_joint6",
        "right_joint7",
    ]
    assert _names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu) == [
        "act_left_joint1",
        "act_left_joint2",
        "act_left_joint3",
        "act_left_joint4",
        "act_left_joint5",
        "act_left_joint6",
        "act_left_joint7",
        "act_right_joint1",
        "act_right_joint2",
        "act_right_joint3",
        "act_right_joint4",
        "act_right_joint5",
        "act_right_joint6",
        "act_right_joint7",
    ]
```

- [ ] **Step 2: Run the test to verify it fails before package setup exists**

Run: `pytest tests/test_source_model.py -v`

Expected: pytest collects the test and either passes if MuJoCo is installed and source XML already loads, or fails with a concrete environment/model error. If it passes, continue; this test is a guard over existing source behavior rather than a missing-code test.

- [ ] **Step 3: Add package metadata and empty package initializers**

Create `pyproject.toml`:

```toml
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "twin-joint-ws"
version = "0.1.0"
description = "Pure MuJoCo dual-arm chopping MVP for twin_joint_ws"
requires-python = ">=3.10"
dependencies = [
  "mujoco>=3.1",
  "numpy>=1.24",
]

[project.optional-dependencies]
test = [
  "pytest>=7.4",
]

[project.scripts]
twin-chop = "twin_mujoco.cli:main"

[tool.setuptools.packages.find]
where = ["src/twin_core", "src/twin_description", "src/twin_mujoco"]

[tool.pytest.ini_options]
testpaths = ["tests"]
pythonpath = ["src/twin_core", "src/twin_description", "src/twin_mujoco"]
```

Create `src/twin_core/twin_core/__init__.py`:

```python
"""Backend-independent dual-arm primitives for twin_joint_ws."""
```

Create `src/twin_description/twin_description/__init__.py`:

```python
"""Asset path helpers for twin_joint_ws."""
```

Create `src/twin_mujoco/twin_mujoco/__init__.py`:

```python
"""Pure MuJoCo runtime and demos for twin_joint_ws."""
```

- [ ] **Step 4: Run the source model test**

Run: `pytest tests/test_source_model.py -v`

Expected: `1 passed`.

- [ ] **Step 5: Commit**

```bash
git add pyproject.toml src tests/test_source_model.py
git commit -m "test: guard twin source model loading"
```

---

### Task 2: Core Dual-Arm Specs And Trajectory Validation

**Files:**
- Modify: `src/twin_core/twin_core/__init__.py`
- Create: `src/twin_core/twin_core/arms.py`
- Create: `src/twin_core/twin_core/trajectory.py`
- Create: `tests/test_core_arms.py`

**Interfaces:**
- Produces: `ArmId = Literal["left", "right"]`
- Produces: `ArmSpec(name: ArmId, joint_names: tuple[str, ...], actuator_names: tuple[str, ...], home: tuple[float, ...] | None = None)`
- Produces: `arm_spec(name: str) -> ArmSpec`
- Produces: `all_arm_specs() -> dict[str, ArmSpec]`
- Produces: `finite_vector(values: Sequence[float], size: int, label: str) -> np.ndarray`
- Produces: `validate_waypoints(waypoints: Sequence[Sequence[float]], dof: int) -> np.ndarray`

- [ ] **Step 1: Write failing arm and trajectory tests**

Create `tests/test_core_arms.py`:

```python
import numpy as np
import pytest

from twin_core import arm_spec, all_arm_specs, finite_vector, validate_waypoints


def test_arm_specs_select_exactly_seven_joints_and_actuators_per_arm():
    left = arm_spec("left")
    right = arm_spec("right")

    assert left.joint_names == tuple(f"left_joint{i}" for i in range(1, 8))
    assert right.joint_names == tuple(f"right_joint{i}" for i in range(1, 8))
    assert left.actuator_names == tuple(f"act_left_joint{i}" for i in range(1, 8))
    assert right.actuator_names == tuple(f"act_right_joint{i}" for i in range(1, 8))
    assert len(all_arm_specs()) == 2


def test_unknown_arm_name_is_rejected():
    with pytest.raises(ValueError, match="unknown arm"):
        arm_spec("center")


def test_finite_vector_rejects_wrong_size_and_nan():
    assert finite_vector([1, 2, 3], 3, "point").shape == (3,)
    with pytest.raises(ValueError, match="point must contain 3 finite values"):
        finite_vector([1, 2], 3, "point")
    with pytest.raises(ValueError, match="point must contain 3 finite values"):
        finite_vector([1, np.nan, 3], 3, "point")


def test_validate_waypoints_returns_2d_array():
    result = validate_waypoints([[0, 1], [2, 3]], dof=2)

    assert result.shape == (2, 2)
    np.testing.assert_allclose(result, [[0, 1], [2, 3]])
```

- [ ] **Step 2: Run tests to verify they fail because `twin_core` exports are missing**

Run: `pytest tests/test_core_arms.py -v`

Expected: FAIL with `ImportError` or missing names from `twin_core`.

- [ ] **Step 3: Implement core arm specs**

Create `src/twin_core/twin_core/arms.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


ArmId = Literal["left", "right"]


@dataclass(frozen=True)
class ArmSpec:
    name: ArmId
    joint_names: tuple[str, ...]
    actuator_names: tuple[str, ...]
    home: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        if len(self.joint_names) != 7:
            raise ValueError(f"{self.name} arm must define 7 joints")
        if len(self.actuator_names) != 7:
            raise ValueError(f"{self.name} arm must define 7 actuators")
        if self.home is not None and len(self.home) != 7:
            raise ValueError(f"{self.name} arm home must contain 7 values")


_SPECS: dict[str, ArmSpec] = {
    "left": ArmSpec(
        name="left",
        joint_names=tuple(f"left_joint{i}" for i in range(1, 8)),
        actuator_names=tuple(f"act_left_joint{i}" for i in range(1, 8)),
    ),
    "right": ArmSpec(
        name="right",
        joint_names=tuple(f"right_joint{i}" for i in range(1, 8)),
        actuator_names=tuple(f"act_right_joint{i}" for i in range(1, 8)),
    ),
}


def arm_spec(name: str) -> ArmSpec:
    try:
        return _SPECS[str(name)]
    except KeyError as exc:
        allowed = ", ".join(sorted(_SPECS))
        raise ValueError(f"unknown arm '{name}'; expected one of: {allowed}") from exc


def all_arm_specs() -> dict[str, ArmSpec]:
    return dict(_SPECS)
```

Create `src/twin_core/twin_core/trajectory.py`:

```python
from __future__ import annotations

from typing import Sequence

import numpy as np


def finite_vector(values: Sequence[float], size: int, label: str) -> np.ndarray:
    result = np.asarray(values, dtype=float).reshape(-1)
    if result.shape != (int(size),) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain {int(size)} finite values")
    return result


def validate_waypoints(waypoints: Sequence[Sequence[float]], dof: int) -> np.ndarray:
    result = np.asarray(waypoints, dtype=float)
    if result.ndim != 2 or result.shape[1] != int(dof) or not np.all(np.isfinite(result)):
        raise ValueError(f"waypoints must be a finite N x {int(dof)} array")
    if result.shape[0] == 0:
        raise ValueError("waypoints must contain at least one row")
    return result
```

Modify `src/twin_core/twin_core/__init__.py`:

```python
"""Backend-independent dual-arm primitives for twin_joint_ws."""

from twin_core.arms import ArmId, ArmSpec, all_arm_specs, arm_spec
from twin_core.trajectory import finite_vector, validate_waypoints

__all__ = [
    "ArmId",
    "ArmSpec",
    "all_arm_specs",
    "arm_spec",
    "finite_vector",
    "validate_waypoints",
]
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_core_arms.py -v`

Expected: `4 passed`.

- [ ] **Step 5: Commit**

```bash
git add src/twin_core tests/test_core_arms.py
git commit -m "feat: add dual arm core specs"
```

---

### Task 3: Description Path Helpers

**Files:**
- Modify: `src/twin_description/twin_description/__init__.py`
- Create: `src/twin_description/twin_description/paths.py`
- Create: `tests/test_description_scene.py`

**Interfaces:**
- Produces: `workspace_root() -> Path`
- Produces: `source_model_path() -> Path`
- Produces: `right_chopping_scene_path() -> Path`

- [ ] **Step 1: Write failing path helper tests**

Create `tests/test_description_scene.py`:

```python
from pathlib import Path

from twin_description import right_chopping_scene_path, source_model_path, workspace_root


def test_description_paths_resolve_inside_twin_workspace():
    root = workspace_root()

    assert root.name == "twin_joint_ws"
    assert source_model_path() == root / "MarvinCCS" / "marvin_final_fixed.xml"
    assert right_chopping_scene_path() == (
        root
        / "src"
        / "twin_description"
        / "twin_description"
        / "assets"
        / "robot"
        / "mujoco"
        / "right_chopping_scene.xml"
    )
    assert source_model_path().is_file()
    assert isinstance(right_chopping_scene_path(), Path)
```

- [ ] **Step 2: Run the path helper test to verify it fails**

Run: `pytest tests/test_description_scene.py::test_description_paths_resolve_inside_twin_workspace -v`

Expected: FAIL because `twin_description.paths` exports do not exist.

- [ ] **Step 3: Implement path helpers**

Create `src/twin_description/twin_description/paths.py`:

```python
from __future__ import annotations

from pathlib import Path


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[3]


def source_model_path() -> Path:
    return workspace_root() / "MarvinCCS" / "marvin_final_fixed.xml"


def right_chopping_scene_path() -> Path:
    return (
        workspace_root()
        / "src"
        / "twin_description"
        / "twin_description"
        / "assets"
        / "robot"
        / "mujoco"
        / "right_chopping_scene.xml"
    )
```

Modify `src/twin_description/twin_description/__init__.py`:

```python
"""Asset path helpers for twin_joint_ws."""

from twin_description.paths import right_chopping_scene_path, source_model_path, workspace_root

__all__ = ["right_chopping_scene_path", "source_model_path", "workspace_root"]
```

- [ ] **Step 4: Run the path helper test**

Run: `pytest tests/test_description_scene.py::test_description_paths_resolve_inside_twin_workspace -v`

Expected: `1 passed`.

- [ ] **Step 5: Commit**

```bash
git add src/twin_description tests/test_description_scene.py
git commit -m "feat: add twin description path helpers"
```

---

### Task 4: Right Chopping Scene XML

**Files:**
- Modify: `tests/test_description_scene.py`
- Create: `src/twin_description/twin_description/assets/robot/mujoco/right_chopping_scene.xml`
- Create: `src/twin_description/twin_description/assets/robot/mujoco/README.md`

**Interfaces:**
- Produces: MJCF scene with source dual-arm robot copied from `MarvinCCS/marvin_final_fixed.xml`.
- Produces: MuJoCo objects named `chopping_board`, `right_force_sensor_body`, `right_force_sensor_site`, `right_tool_body`, `right_tool_tip_site`, `right_tool_force`, `right_tool_torque`.
- Produces: right-side board at world `y < 0`, initially `pos="0 -0.82 0.055"` and `size="0.24 0.18 0.025"` so board top is `0.08`.
- Produces: wrist-mounted tool under `right_link7` using:

```xml
<body name="right_force_sensor_body" pos="0 -0.095 0" quat="0.5 0.5 -0.5 0.5">
  <geom name="right_force_sensor_housing" type="cylinder" pos="0 0 -0.0105" size="0.026 0.0105" rgba="0.18 0.18 0.2 1" contype="0" conaffinity="0" />
  <site name="right_force_sensor_site" size="0.012" rgba="1 0.7 0 1" />
  <body name="right_tool_body">
    <inertial pos="0 0 0.07" mass="0.22" diaginertia="0.00022 0.00022 0.00003" />
    <geom name="right_tool_shaft" type="cylinder" pos="0 0 0.065" size="0.008 0.065" rgba="0.75 0.75 0.78 1" contype="0" conaffinity="0" />
    <geom name="right_tool_tip" type="sphere" pos="0 0 0.135" size="0.012" rgba="0.9 0.2 0.1 1" friction="0.9 0.02 0.002" solref="0.04 1" solimp="0.85 0.95 0.005" />
    <site name="right_tool_tip_site" pos="0 0 0.135" size="0.009" rgba="0 1 1 0.7" />
  </body>
</body>
```

- [ ] **Step 1: Extend scene tests before creating the scene**

Append to `tests/test_description_scene.py`:

```python
import mujoco


def _id(model, object_type, name: str) -> int:
    return int(mujoco.mj_name2id(model, object_type, name))


def test_right_chopping_scene_loads_with_task_objects():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))

    assert model.njnt == 14
    assert model.nu == 14
    assert _id(model, mujoco.mjtObj.mjOBJ_GEOM, "chopping_board") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_force_sensor_body") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_BODY, "right_tool_body") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_SITE, "right_force_sensor_site") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_SITE, "right_tool_tip_site") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_SENSOR, "right_tool_force") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_SENSOR, "right_tool_torque") >= 0


def test_chopping_board_is_on_right_arm_side():
    model = mujoco.MjModel.from_xml_path(str(right_chopping_scene_path()))
    geom_id = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "chopping_board")

    assert model.geom_pos[geom_id, 1] < 0.0
    assert model.geom_pos[geom_id, 2] + model.geom_size[geom_id, 2] == 0.08
```

- [ ] **Step 2: Run scene tests to verify missing scene failure**

Run: `pytest tests/test_description_scene.py -v`

Expected: existing path test passes; new scene tests FAIL because `right_chopping_scene.xml` does not exist.

- [ ] **Step 3: Create the scene XML**

Create `src/twin_description/twin_description/assets/robot/mujoco/right_chopping_scene.xml` by copying the full contents of `MarvinCCS/marvin_final_fixed.xml`, then make these exact edits in the copied file only:

1. Change the root `<mujoco>` model name to `Twin right-arm chopping scene`.
2. Ensure `<option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81" />` exists.
3. Add under `<worldbody>` before `robot_base`:

```xml
<light pos="0 -1 1.5" dir="0 1 -1" />
<geom name="floor" type="plane" size="1.5 1.5 0.05" rgba="0.2 0.25 0.3 1" />
<geom name="chopping_board" type="box" pos="0 -0.82 0.055" size="0.24 0.18 0.025" rgba="0.55 0.32 0.12 1" friction="1.0 0.03 0.003" solref="0.04 1" solimp="0.85 0.95 0.005" />
```

4. Add the `right_force_sensor_body` XML block from this task under `right_link7`, after the existing right wrist geom.
5. Add before `</mujoco>`:

```xml
<sensor>
  <force name="right_tool_force" site="right_force_sensor_site" />
  <torque name="right_tool_torque" site="right_force_sensor_site" />
</sensor>
```

Do not edit `MarvinCCS/marvin_final_fixed.xml`.

- [ ] **Step 4: Document scene conventions**

Create `src/twin_description/twin_description/assets/robot/mujoco/README.md`:

```markdown
# Right Chopping Scene

`right_chopping_scene.xml` is the phase-one MuJoCo task scene. It copies the
dual-arm model from `MarvinCCS/marvin_final_fixed.xml` and adds only task
objects: floor, right-side chopping board, right wrist force sensor body, tool,
tool-tip site, and force/torque sensors.

The source XML in `MarvinCCS/` is treated as an upstream asset and is not edited.
The board is placed on the right-arm side of the workspace (`y < 0`). Runtime
tests verify the final safe home pose, tool Z direction, sensor-site placement,
and board clearance before the chopping demo is accepted.
```

- [ ] **Step 5: Run scene tests**

Run: `pytest tests/test_description_scene.py -v`

Expected: all tests in this file pass.

- [ ] **Step 6: Verify the source XML is unchanged**

Run: `git diff -- MarvinCCS/marvin_final_fixed.xml`

Expected: no output.

- [ ] **Step 7: Commit**

```bash
git add src/twin_description tests/test_description_scene.py
git commit -m "feat: add right chopping mujoco scene"
```

---

### Task 5: Twin MuJoCo Runtime And ArmView

**Files:**
- Modify: `src/twin_mujoco/twin_mujoco/__init__.py`
- Create: `src/twin_mujoco/twin_mujoco/errors.py`
- Create: `src/twin_mujoco/twin_mujoco/runtime.py`
- Create: `tests/test_runtime_arm_view.py`

**Interfaces:**
- Produces: `TwinMujocoRuntime.load(model_path: str | Path | None = None) -> TwinMujocoRuntime`
- Produces: `TwinMujocoRuntime.arm_view(name: str) -> ArmView`
- Produces: `ArmView.joint_positions`, `ArmView.joint_velocities`, `ArmView.effort_limits`
- Produces: `ArmView.site_pose(name: str) -> tuple[np.ndarray, np.ndarray]`
- Produces: `ArmView.site_jacobian(name: str) -> np.ndarray`
- Produces: `ArmView.apply_torque(torque_nm: Sequence[float]) -> np.ndarray`

- [ ] **Step 1: Write failing runtime tests**

Create `tests/test_runtime_arm_view.py`:

```python
import numpy as np

from twin_description import right_chopping_scene_path
from twin_mujoco import TwinMujocoRuntime


def test_runtime_loads_full_dual_arm_scene_and_exposes_arm_views():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())

    assert runtime.joint_names == tuple(
        [f"left_joint{i}" for i in range(1, 8)]
        + [f"right_joint{i}" for i in range(1, 8)]
    )
    assert runtime.actuator_names == tuple(
        [f"act_left_joint{i}" for i in range(1, 8)]
        + [f"act_right_joint{i}" for i in range(1, 8)]
    )
    assert runtime.arm_view("right").joint_names == tuple(f"right_joint{i}" for i in range(1, 8))


def test_right_arm_view_exposes_state_limits_and_tool_jacobian():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    right = runtime.arm_view("right")

    runtime.reset()
    assert right.joint_positions.shape == (7,)
    assert right.joint_velocities.shape == (7,)
    assert right.effort_limits.shape == (7,)
    assert np.all(right.effort_limits > 0)
    assert right.site_jacobian("right_tool_tip_site").shape == (6, 7)


def test_right_arm_torque_application_does_not_write_left_actuators():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    right = runtime.arm_view("right")

    applied = right.apply_torque(np.ones(7))

    assert applied.shape == (7,)
    np.testing.assert_allclose(runtime.data.ctrl[:7], np.zeros(7))
    np.testing.assert_allclose(runtime.data.ctrl[7:], np.ones(7))
```

- [ ] **Step 2: Run runtime tests to verify missing runtime failure**

Run: `pytest tests/test_runtime_arm_view.py -v`

Expected: FAIL because `TwinMujocoRuntime` is not exported.

- [ ] **Step 3: Implement runtime errors**

Create `src/twin_mujoco/twin_mujoco/errors.py`:

```python
class TwinMujocoError(RuntimeError):
    pass


class MujocoModelError(TwinMujocoError):
    pass


class SafetyStop(TwinMujocoError):
    pass
```

- [ ] **Step 4: Implement runtime and arm views**

Create `src/twin_mujoco/twin_mujoco/runtime.py`:

```python
from __future__ import annotations

from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np

from twin_core import ArmSpec, arm_spec
from twin_description import right_chopping_scene_path
from twin_mujoco.errors import MujocoModelError, SafetyStop


class TwinMujocoRuntime:
    def __init__(self, model, data) -> None:
        self.model = model
        self.data = data
        self.joint_names = self._names(mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
        self.actuator_names = self._names(mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
        self.timestep = float(model.opt.timestep)

    @classmethod
    def load(cls, model_path: str | Path | None = None) -> "TwinMujocoRuntime":
        path = Path(model_path) if model_path is not None else right_chopping_scene_path()
        if not path.is_file():
            raise MujocoModelError(f"MuJoCo model file does not exist: {path}")
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
            data = mujoco.MjData(model)
        except Exception as exc:
            raise MujocoModelError(f"failed to load MuJoCo model: {path}") from exc
        return cls(model, data)

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            raise SafetyStop("MuJoCo state contains NaN or infinity")

    def arm_view(self, name: str) -> "ArmView":
        return ArmView(self, arm_spec(name))

    def body_pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        body_id = self._id(mujoco.mjtObj.mjOBJ_BODY, name)
        return self.data.xpos[body_id].copy(), self.data.xmat[body_id].reshape(3, 3).copy()

    def site_pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        site_id = self._id(mujoco.mjtObj.mjOBJ_SITE, name)
        return self.data.site_xpos[site_id].copy(), self.data.site_xmat[site_id].reshape(3, 3).copy()

    def _id(self, object_type, name: str) -> int:
        value = int(mujoco.mj_name2id(self.model, object_type, name))
        if value < 0:
            raise MujocoModelError(f"MuJoCo object not found: {name}")
        return value

    def _names(self, object_type, count: int) -> tuple[str, ...]:
        return tuple(mujoco.mj_id2name(self.model, object_type, i) or "" for i in range(count))


class ArmView:
    def __init__(self, runtime: TwinMujocoRuntime, spec: ArmSpec) -> None:
        self.runtime = runtime
        self.spec = spec
        self.joint_names = spec.joint_names
        self.actuator_names = spec.actuator_names
        self._joint_ids = np.array([runtime._id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in spec.joint_names], dtype=int)
        self._qpos = np.array([runtime.model.jnt_qposadr[i] for i in self._joint_ids], dtype=int)
        self._dofs = np.array([runtime.model.jnt_dofadr[i] for i in self._joint_ids], dtype=int)
        self._actuators = np.array([runtime._id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in spec.actuator_names], dtype=int)
        self.effort_limits = np.max(np.abs(runtime.model.actuator_ctrlrange[self._actuators]), axis=1)

    @property
    def joint_positions(self) -> np.ndarray:
        return self.runtime.data.qpos[self._qpos].copy()

    @property
    def joint_velocities(self) -> np.ndarray:
        return self.runtime.data.qvel[self._dofs].copy()

    @property
    def bias_torque(self) -> np.ndarray:
        mujoco.mj_forward(self.runtime.model, self.runtime.data)
        return self.runtime.data.qfrc_bias[self._dofs].copy()

    def site_pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        return self.runtime.site_pose(name)

    def site_jacobian(self, name: str) -> np.ndarray:
        site_id = self.runtime._id(mujoco.mjtObj.mjOBJ_SITE, name)
        jacp = np.zeros((3, self.runtime.model.nv))
        jacr = np.zeros((3, self.runtime.model.nv))
        mujoco.mj_jacSite(self.runtime.model, self.runtime.data, jacp, jacr, site_id)
        return np.vstack((jacp[:, self._dofs], jacr[:, self._dofs]))

    def apply_torque(self, torque_nm: Sequence[float]) -> np.ndarray:
        torque = np.asarray(torque_nm, dtype=float).reshape(-1)
        if torque.shape != (7,) or not np.all(np.isfinite(torque)):
            raise ValueError("torque_nm must contain 7 finite values")
        applied = np.clip(torque, -self.effort_limits, self.effort_limits)
        self.runtime.data.ctrl[:] = 0.0
        self.runtime.data.ctrl[self._actuators] = applied
        return applied.copy()
```

Modify `src/twin_mujoco/twin_mujoco/__init__.py`:

```python
"""Pure MuJoCo runtime and demos for twin_joint_ws."""

from twin_mujoco.errors import MujocoModelError, SafetyStop, TwinMujocoError
from twin_mujoco.runtime import ArmView, TwinMujocoRuntime

__all__ = [
    "ArmView",
    "MujocoModelError",
    "SafetyStop",
    "TwinMujocoError",
    "TwinMujocoRuntime",
]
```

- [ ] **Step 5: Run runtime tests**

Run: `pytest tests/test_runtime_arm_view.py -v`

Expected: `3 passed`.

- [ ] **Step 6: Commit**

```bash
git add src/twin_mujoco tests/test_runtime_arm_view.py
git commit -m "feat: add twin mujoco runtime arm views"
```

---

### Task 6: Right-Arm Force Controller

**Files:**
- Modify: `src/twin_mujoco/twin_mujoco/__init__.py`
- Create: `src/twin_mujoco/twin_mujoco/control.py`
- Create: `tests/test_control.py`

**Interfaces:**
- Produces: `CartesianImpedanceConfig`, `ForceControlConfig`, `WrenchCalibrator`, `CartesianForceController`
- Produces: `downward_tool_rotation(x_axis_world=(1.0, 0.0, 0.0)) -> np.ndarray`
- Produces: `CartesianForceController.compute(compensated_wrench_tool: Sequence[float]) -> np.ndarray`

- [ ] **Step 1: Write failing force controller tests**

Create `tests/test_control.py`:

```python
import numpy as np

from twin_description import right_chopping_scene_path
from twin_mujoco import TwinMujocoRuntime
from twin_mujoco.control import CartesianForceController, WrenchCalibrator, downward_tool_rotation


def test_downward_tool_rotation_has_tool_z_aligned_to_world_negative_z():
    rotation = downward_tool_rotation()

    np.testing.assert_allclose(rotation[:, 2], [0.0, 0.0, -1.0])
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)


def test_wrench_calibrator_removes_stationary_bias():
    calibrator = WrenchCalibrator(tool_mass_kg=0.22)
    rotation = np.eye(3)
    raw = np.array([0.1, 0.2, 2.3, 0.01, 0.02, 0.03])

    calibrator.calibrate([raw, raw], rotation)
    compensated = calibrator.compensate(raw, rotation)

    np.testing.assert_allclose(compensated, np.zeros(6), atol=1e-9)


def test_cartesian_controller_produces_finite_limited_right_arm_torque():
    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    right = runtime.arm_view("right")
    controller = CartesianForceController(right)
    position, rotation = right.site_pose("right_tool_tip_site")

    controller.set_target(position, rotation)
    torque = controller.compute(np.zeros(6))

    assert torque.shape == (7,)
    assert np.all(np.isfinite(torque))
    assert np.all(np.abs(torque) <= right.effort_limits + 1e-9)
```

- [ ] **Step 2: Run controller tests to verify missing control module failure**

Run: `pytest tests/test_control.py -v`

Expected: FAIL because `twin_mujoco.control` does not exist.

- [ ] **Step 3: Implement force control**

Create `src/twin_mujoco/twin_mujoco/control.py` using the proven structure from `cook_ws/src/cook_mujoco/cook_mujoco/control/force_control.py`, adapted to consume an `ArmView` instead of a single-arm runtime. Include:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from twin_mujoco.runtime import ArmView


@dataclass(frozen=True)
class CartesianImpedanceConfig:
    translational_stiffness: tuple[float, float, float] = (2500.0, 2500.0, 2800.0)
    translational_damping: tuple[float, float, float] = (105.0, 105.0, 115.0)
    rotational_stiffness: tuple[float, float, float] = (45.0, 45.0, 35.0)
    rotational_damping: tuple[float, float, float] = (5.5, 5.5, 4.5)
    nullspace_stiffness: float = 4.0
    nullspace_damping: float = 1.5
    torque_rate_limit: float = 1500.0


@dataclass(frozen=True)
class ForceControlConfig:
    target_force_n: float = 10.0
    feedback_alpha: float = 0.18
    admittance_gain_m_per_ns: float = 0.03
    maximum_position_offset_m: float = 0.04


class WrenchCalibrator:
    def __init__(self, *, tool_mass_kg: float = 0.22, gravity: Sequence[float] = (0.0, 0.0, -9.81)):
        self.tool_mass_kg = float(tool_mass_kg)
        self.gravity_world = _vector(gravity, 3, "gravity")
        self._residual = np.zeros(6)
        self._calibrated = False

    def modeled_gravity_wrench(self, sensor_rotation_world: np.ndarray) -> np.ndarray:
        rotation = np.asarray(sensor_rotation_world, dtype=float).reshape(3, 3)
        force = -(rotation.T @ (self.tool_mass_kg * self.gravity_world))
        return np.concatenate((force, np.zeros(3)))

    def calibrate(self, samples: Iterable[Sequence[float]], sensor_rotation_world: np.ndarray) -> None:
        values = np.asarray(list(samples), dtype=float)
        if values.ndim != 2 or values.shape[1] != 6 or len(values) == 0 or not np.all(np.isfinite(values)):
            raise ValueError("calibration requires finite six-axis samples")
        self._residual = np.mean(values, axis=0) - self.modeled_gravity_wrench(sensor_rotation_world)
        self._calibrated = True

    def compensate(self, raw_wrench: Sequence[float], sensor_rotation_world: np.ndarray) -> np.ndarray:
        if not self._calibrated:
            raise ValueError("wrench sensor has not been calibrated")
        raw = _vector(raw_wrench, 6, "raw_wrench")
        return raw - self._residual - self.modeled_gravity_wrench(sensor_rotation_world)


class CartesianForceController:
    def __init__(
        self,
        arm: ArmView,
        *,
        impedance: CartesianImpedanceConfig | None = None,
        force: ForceControlConfig | None = None,
        tool_site: str = "right_tool_tip_site",
    ) -> None:
        self.arm = arm
        self.tool_site = tool_site
        self.impedance = impedance or CartesianImpedanceConfig()
        self.force = force or ForceControlConfig()
        self.desired_position, self.desired_rotation = arm.site_pose(tool_site)
        self.nullspace_reference = arm.joint_positions
        self.force_enabled = False
        self._filtered_force = 0.0
        self._force_position_offset = 0.0
        self._previous_torque = np.zeros(7)

    def set_target(self, position: Sequence[float], rotation: np.ndarray) -> None:
        self.desired_position = _vector(position, 3, "position")
        self.desired_rotation = np.asarray(rotation, dtype=float).reshape(3, 3).copy()

    def enable_force(self, enabled: bool, *, target_force_n: float | None = None) -> None:
        if target_force_n is not None:
            if target_force_n < 0:
                raise ValueError("target force must be non-negative")
            self.force = ForceControlConfig(
                target_force_n=float(target_force_n),
                feedback_alpha=self.force.feedback_alpha,
                admittance_gain_m_per_ns=self.force.admittance_gain_m_per_ns,
                maximum_position_offset_m=self.force.maximum_position_offset_m,
            )
        self.force_enabled = bool(enabled)
        if not self.force_enabled:
            self._force_position_offset = 0.0

    def compute(self, compensated_wrench_tool: Sequence[float]) -> np.ndarray:
        wrench_tool = _vector(compensated_wrench_tool, 6, "compensated_wrench_tool")
        position, rotation = self.arm.site_pose(self.tool_site)
        jacobian = self.arm.site_jacobian(self.tool_site)
        velocity = jacobian @ self.arm.joint_velocities
        orientation_error = rotation_error(rotation, self.desired_rotation)
        timestep = self.arm.runtime.timestep
        tool_axis_world = rotation[:, 2]
        alpha = self.force.feedback_alpha
        self._filtered_force = (1.0 - alpha) * self._filtered_force + alpha * float(wrench_tool[2])
        effective_target = self.desired_position.copy()
        if self.force_enabled:
            error = self.force.target_force_n - self._filtered_force
            self._force_position_offset = float(np.clip(
                self._force_position_offset + self.force.admittance_gain_m_per_ns * error * timestep,
                -self.force.maximum_position_offset_m,
                self.force.maximum_position_offset_m,
            ))
            effective_target += self._force_position_offset * tool_axis_world
        kp = np.asarray(self.impedance.translational_stiffness)
        kd = np.asarray(self.impedance.translational_damping)
        kr = np.asarray(self.impedance.rotational_stiffness)
        dr = np.asarray(self.impedance.rotational_damping)
        task_wrench = np.concatenate((
            kp * (effective_target - position) - kd * velocity[:3],
            kr * orientation_error - dr * velocity[3:],
        ))
        torque = jacobian.T @ task_wrench + self.arm.bias_torque
        null_projector = np.eye(7) - jacobian.T @ np.linalg.pinv(jacobian.T)
        torque += null_projector @ (
            -self.impedance.nullspace_stiffness * (self.arm.joint_positions - self.nullspace_reference)
            - self.impedance.nullspace_damping * self.arm.joint_velocities
        )
        max_delta = self.impedance.torque_rate_limit * timestep
        torque = np.clip(torque, self._previous_torque - max_delta, self._previous_torque + max_delta)
        torque = np.clip(torque, -self.arm.effort_limits, self.arm.effort_limits)
        self._previous_torque = torque.copy()
        return torque


def downward_tool_rotation(x_axis_world: Sequence[float] = (1.0, 0.0, 0.0)) -> np.ndarray:
    z_axis = np.array((0.0, 0.0, -1.0))
    x_axis = _vector(x_axis_world, 3, "x_axis_world")
    x_axis = x_axis - z_axis * np.dot(z_axis, x_axis)
    norm = np.linalg.norm(x_axis)
    if norm < 1e-9:
        raise ValueError("tool x axis cannot be parallel to tool z axis")
    x_axis /= norm
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))
```

Also include the helper functions `rotation_error`, `_matrix_to_quaternion`, `_quaternion_multiply`, and `_vector` from the cook control file with the same behavior and error messages adapted to `ValueError`.

- [ ] **Step 4: Export controller classes**

Append to `src/twin_mujoco/twin_mujoco/__init__.py`:

```python
from twin_mujoco.control import CartesianForceController, CartesianImpedanceConfig, ForceControlConfig, WrenchCalibrator
```

Add those four names to `__all__`.

- [ ] **Step 5: Run controller tests**

Run: `pytest tests/test_control.py -v`

Expected: `3 passed`.

- [ ] **Step 6: Commit**

```bash
git add src/twin_mujoco tests/test_control.py
git commit -m "feat: add right arm cartesian force controller"
```

---

### Task 7: Chopping State Machine, CSV Logging, And CLI

**Files:**
- Modify: `src/twin_mujoco/twin_mujoco/__init__.py`
- Create: `src/twin_mujoco/twin_mujoco/chopping.py`
- Create: `src/twin_mujoco/twin_mujoco/cli.py`
- Create: `tests/test_chopping.py`

**Interfaces:**
- Produces: `ChoppingPhase` enum with `APPROACH`, `DESCEND`, `FORCE_HOLD`, `RETRACT`, `SHIFT`, `COMPLETE`, `FAULT`
- Produces: `ChoppingConfig(cycles=3, target_force_n=10.0, force_hold_s=0.15, control_hz=500.0, ...)`
- Produces: `ForceControlSample`
- Produces: `RightArmChopper(runtime: TwinMujocoRuntime | None = None)`
- Produces: `RightArmChopper.run(config: ChoppingConfig, log_path: str | Path | None = None) -> list[ForceControlSample]`
- Produces: CLI `twin-chop --cycles 3 --headless --log /tmp/twin_chop.csv`

- [ ] **Step 1: Write failing chopping tests**

Create `tests/test_chopping.py`:

```python
import csv

from twin_mujoco.chopping import ChoppingConfig, ChoppingPhase, RightArmChopper


def test_headless_chopping_completes_three_cycles_and_logs_csv(tmp_path):
    log_path = tmp_path / "right_chop.csv"
    chopper = RightArmChopper()

    samples = chopper.run(
        ChoppingConfig(cycles=3, target_force_n=6.0, force_hold_s=0.05),
        log_path=log_path,
    )

    assert samples
    assert samples[-1].phase == ChoppingPhase.COMPLETE
    phases = {sample.phase for sample in samples}
    assert ChoppingPhase.APPROACH in phases
    assert ChoppingPhase.DESCEND in phases
    assert ChoppingPhase.FORCE_HOLD in phases
    assert ChoppingPhase.RETRACT in phases
    assert ChoppingPhase.SHIFT in phases
    with log_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows
    assert rows[0].keys() >= {
        "time_s",
        "phase",
        "control_mode",
        "target_x",
        "actual_x",
        "target_force_n",
        "measured_force_n",
        "raw_wrench_0",
        "wrench_0",
        "q_0",
        "qd_0",
        "tau_0",
        "fault",
    }


def test_chopping_rejects_invalid_cycle_count():
    chopper = RightArmChopper()

    try:
        chopper.run(ChoppingConfig(cycles=0))
    except ValueError as exc:
        assert "cycles must be at least 1" in str(exc)
    else:
        raise AssertionError("expected invalid cycle count to fail")
```

- [ ] **Step 2: Run chopping tests to verify missing state machine failure**

Run: `pytest tests/test_chopping.py -v`

Expected: FAIL because `twin_mujoco.chopping` does not exist.

- [ ] **Step 3: Implement chopping data structures**

Create `src/twin_mujoco/twin_mujoco/chopping.py` with the enum, config, sample dataclass, and CSV field list:

```python
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np

from twin_mujoco.control import CartesianForceController
from twin_mujoco.runtime import TwinMujocoRuntime


class ChoppingPhase(str, Enum):
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    FORCE_HOLD = "FORCE_HOLD"
    RETRACT = "RETRACT"
    SHIFT = "SHIFT"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"


@dataclass(frozen=True)
class ChoppingConfig:
    cycles: int = 3
    safe_height_m: float = 0.08
    spacing_m: float = 0.02
    descent_speed_m_s: float = 0.05
    retract_speed_m_s: float = 0.08
    force_hold_s: float = 0.15
    target_force_n: float = 10.0
    control_hz: float = 500.0


@dataclass(frozen=True)
class ForceControlSample:
    time_s: float
    phase: ChoppingPhase
    control_mode: str
    target_position: tuple[float, float, float]
    actual_position: tuple[float, float, float]
    target_force_n: float
    measured_force_n: float
    raw_wrench: tuple[float, ...]
    compensated_wrench: tuple[float, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    joint_torques: tuple[float, ...]
    contact: bool
    fault: str = ""
```

- [ ] **Step 4: Implement `RightArmChopper` with deterministic headless phase execution**

Continue in `chopping.py`:

```python
class RightArmChopper:
    def __init__(self, runtime: TwinMujocoRuntime | None = None) -> None:
        self.runtime = runtime or TwinMujocoRuntime.load()
        self.right = self.runtime.arm_view("right")
        self.controller = CartesianForceController(self.right)
        self.samples: list[ForceControlSample] = []

    def run(self, config: ChoppingConfig | None = None, log_path: str | Path | None = None) -> list[ForceControlSample]:
        cfg = config or ChoppingConfig()
        if cfg.cycles < 1:
            raise ValueError("cycles must be at least 1")
        self.runtime.reset()
        self.samples.clear()
        position, rotation = self.right.site_pose("right_tool_tip_site")
        board_top = self._board_top()
        safe = position.copy()
        safe[2] = max(safe[2], board_top + cfg.safe_height_m)
        descend = safe.copy()
        descend[2] = board_top + 0.02
        elapsed = 0.0
        torque = np.zeros(7)
        for cycle in range(cfg.cycles):
            shifted_safe = safe.copy()
            shifted_safe[0] += cycle * cfg.spacing_m
            shifted_descend = descend.copy()
            shifted_descend[0] += cycle * cfg.spacing_m
            elapsed = self._record_phase(elapsed, ChoppingPhase.APPROACH if cycle == 0 else ChoppingPhase.SHIFT, shifted_safe, rotation, 8, 0.0, torque)
            elapsed = self._record_phase(elapsed, ChoppingPhase.DESCEND, shifted_descend, rotation, 8, 0.0, torque)
            self.controller.enable_force(True, target_force_n=cfg.target_force_n)
            elapsed = self._record_phase(elapsed, ChoppingPhase.FORCE_HOLD, shifted_descend, rotation, max(2, int(cfg.force_hold_s * cfg.control_hz)), cfg.target_force_n, torque)
            self.controller.enable_force(False)
            elapsed = self._record_phase(elapsed, ChoppingPhase.RETRACT, shifted_safe, rotation, 8, 0.0, torque)
        self.samples.append(self._sample(elapsed, ChoppingPhase.COMPLETE, "IDLE", safe, 0.0, np.zeros(7), ""))
        if log_path is not None:
            self.write_csv(log_path)
        return list(self.samples)
```

Use this simple deterministic phase runner first so the CLI and CSV are testable. Then replace `_record_phase` internals with physical stepping and controller torque in the same task once the tests are green. `_record_phase` must call `self.controller.set_target(target, rotation)`, compute torque with a six-axis zero wrench for free-space phases and `[0, 0, target_force_n, 0, 0, 0]` during `FORCE_HOLD`, apply torque to the right arm, call `runtime.step()`, and record each sample.

- [ ] **Step 5: Implement CSV writer and CLI**

Create `src/twin_mujoco/twin_mujoco/cli.py`:

```python
from __future__ import annotations

import argparse
from pathlib import Path

from twin_mujoco.chopping import ChoppingConfig, RightArmChopper


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="twin-chop")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--force", type=float, default=10.0)
    parser.add_argument("--hold", type=float, default=0.15)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--log", type=Path, default=None)
    args = parser.parse_args(argv)
    chopper = RightArmChopper()
    chopper.run(
        ChoppingConfig(cycles=args.cycles, target_force_n=args.force, force_hold_s=args.hold),
        log_path=args.log,
    )
    return 0
```

Modify `src/twin_mujoco/twin_mujoco/__init__.py` to export `ChoppingConfig`, `ChoppingPhase`, `ForceControlSample`, and `RightArmChopper`.

- [ ] **Step 6: Run chopping tests**

Run: `pytest tests/test_chopping.py -v`

Expected: `2 passed`.

- [ ] **Step 7: Commit**

```bash
git add src/twin_mujoco tests/test_chopping.py
git commit -m "feat: add right arm chopping demo"
```

---

### Task 8: Physical Scene Validation And Final Acceptance

**Files:**
- Modify: `tests/test_description_scene.py`
- Modify: `tests/test_runtime_arm_view.py`
- Modify: `tests/test_chopping.py`
- Modify: `src/twin_mujoco/twin_mujoco/chopping.py`
- Create: `README.md`

**Interfaces:**
- Produces: validation that right tool local Z is approximately world `-Z` at the demo safe pose.
- Produces: validation that `right_force_sensor_site` is above `right_tool_tip_site` at the demo safe pose.
- Produces: validation that the right tool tip stays above `chopping_board` at safe height.
- Produces: README instructions for running tests and `twin-chop`.

- [ ] **Step 1: Add physical acceptance tests**

Append to `tests/test_runtime_arm_view.py`:

```python
def test_safe_home_geometry_is_valid_for_right_chopping():
    from twin_mujoco.chopping import LEFT_HOME_Q, RIGHT_CHOPPING_HOME_Q

    runtime = TwinMujocoRuntime.load(right_chopping_scene_path())
    runtime.reset()
    runtime.set_arm_positions("left", LEFT_HOME_Q)
    runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
    right = runtime.arm_view("right")
    tool_position, tool_rotation = right.site_pose("right_tool_tip_site")
    sensor_position, _ = right.site_pose("right_force_sensor_site")
    board_id = runtime._id(__import__("mujoco").mjtObj.mjOBJ_GEOM, "chopping_board")
    board_top = runtime.model.geom_pos[board_id, 2] + runtime.model.geom_size[board_id, 2]

    assert tool_position[2] > board_top + 0.05
    assert tool_rotation[2, 2] < -0.90
    assert sensor_position[2] > tool_position[2]
```

- [ ] **Step 2: Run the physical test and inspect failure**

Run: `pytest tests/test_runtime_arm_view.py::test_safe_home_geometry_is_valid_for_right_chopping -v`

Expected: FAIL if the initial copied tool transform or zero joint pose does not meet the safe pose requirements.

- [ ] **Step 3: Make the demo initialize a right-arm safe pose**

Add `RIGHT_CHOPPING_HOME_Q` to `src/twin_mujoco/twin_mujoco/chopping.py`:

```python
RIGHT_CHOPPING_HOME_Q = np.array((0.0, 0.45, 0.0, -1.25, 0.0, 0.65, 0.0), dtype=float)
LEFT_HOME_Q = np.zeros(7, dtype=float)
```

Add `TwinMujocoRuntime.set_arm_positions(arm_name: str, joint_positions: Sequence[float]) -> None` in `runtime.py`, using the selected `ArmView` qpos and dof indexes, zeroing selected velocities, and calling `mujoco.mj_forward`.

In `RightArmChopper.run`, after `self.runtime.reset()`, call:

```python
self.runtime.set_arm_positions("left", LEFT_HOME_Q)
self.runtime.set_arm_positions("right", RIGHT_CHOPPING_HOME_Q)
```

- [ ] **Step 4: Re-run physical tests**

Run: `pytest tests/test_runtime_arm_view.py -v`

Expected: all runtime tests pass. If `tool_rotation[2, 2] < -0.90` fails, adjust only the copied `right_force_sensor_body` quaternion in `right_chopping_scene.xml`, re-run this exact command, and keep `MarvinCCS/marvin_final_fixed.xml` unchanged.

- [ ] **Step 5: Add README**

Create `README.md`:

```markdown
# twin_joint_ws

Pure MuJoCo phase-one workspace for a dual-arm Marvin model. The source robot
asset is kept under `MarvinCCS/marvin_final_fixed.xml`; phase-one task geometry
lives in `src/twin_description/twin_description/assets/robot/mujoco/right_chopping_scene.xml`.

## Setup

```bash
python3 -m pip install -e .[test]
```

## Test

```bash
pytest -v
```

## Headless right-arm chopping

```bash
twin-chop --cycles 3 --headless --log /tmp/twin_right_chop.csv
```

The first phase is pure MuJoCo. ROS2 nodes and launch files are intentionally
out of scope.
```

- [ ] **Step 6: Run final verification**

Run: `pytest -v`

Expected: all tests pass.

Run: `twin-chop --cycles 3 --headless --log /tmp/twin_right_chop.csv`

Expected: command exits `0` and `/tmp/twin_right_chop.csv` contains rows with `APPROACH`, `DESCEND`, `FORCE_HOLD`, `RETRACT`, `SHIFT`, and `COMPLETE`.

Run: `git diff -- MarvinCCS/marvin_final_fixed.xml`

Expected: no output.

- [ ] **Step 7: Commit**

```bash
git add README.md src tests
git commit -m "test: verify right chopping acceptance"
```

---

## Self-Review

### Spec Coverage

- Workspace packages `twin_core`, `twin_description`, `twin_mujoco`: covered by Tasks 1-3.
- Original source MJCF unchanged: covered by Global Constraints and Task 4/8 `git diff` checks.
- New chopping scene XML: covered by Task 4.
- Full 14-joint, 14-actuator dual-arm runtime: covered by Tasks 1, 4, and 5.
- Dual-arm abstraction before right-arm demo: covered by Task 2 `ArmSpec` and Task 5 `ArmView`.
- Right-arm force-control pattern: covered by Task 6 and Task 7.
- Default at least 3 cycles: covered by Task 7 default `ChoppingConfig(cycles=3)` and Task 8 CLI verification.
- CSV logging with expected fields: covered by Task 7 tests.
- Tool Z, sensor site, and safe height validation: covered by Task 8.
- No ROS2 nodes in phase one: covered by Global Constraints and README.

### Placeholder Scan

No task uses placeholder language or unspecified file paths. The only adaptive instruction is the Task 8 quaternion correction path, and it is bounded by an exact failing assertion, exact file, exact command, and the source-MJCF no-edit constraint.

### Type Consistency

The plan consistently uses `TwinMujocoRuntime`, `ArmView`, `CartesianForceController`, `ChoppingConfig`, `ChoppingPhase`, `ForceControlSample`, and `RightArmChopper` across tasks. Site and sensor names use the `right_` prefix consistently: `right_tool_tip_site`, `right_force_sensor_site`, `right_tool_force`, and `right_tool_torque`.
