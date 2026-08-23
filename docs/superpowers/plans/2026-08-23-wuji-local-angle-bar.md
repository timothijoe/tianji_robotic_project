# Wuji Local Angle-Bar Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS-free desktop panel that switches between official left and right Wuji MuJoCo hands and drives their 20 position actuators with bounded radian sliders.

**Architecture:** Extend stable asset lookup to choose an official first-generation MJCF by hand side. A focused MuJoCo backend owns a single selected model and exposes metadata, validated target assignment, stepping, and viewer lifecycle. A Tkinter panel renders model-derived sliders and invokes only that backend; the CLI starts the panel without importing hardware or ROS modules.

**Tech Stack:** Python 3.12, MuJoCo 3.10, NumPy, Tkinter standard library, pytest.

## Global Constraints

- Do not import or call ROS, RViz, `wujihandpy`, USB, networking, or real hardware SDKs.
- Support first-generation official Wuji left and right MJCF assets only; exactly one hand is loaded at a time.
- All commands use a finite `(20,)` radian vector in the active model actuator order.
- Slider bounds come from the active model's position-actuator `ctrlrange`; backend validation repeats those checks.
- Default and reset poses must be explicit, finite, and validated against the active model. Maintain separate left/right 20-value open-pose constants; never infer either from zero or from control-range midpoints.
- Preserve existing user changes outside files named in each task.

---

## File structure

| Path | Responsibility |
|---|---|
| `src/tianji_robotics/simulation/paths.py` | Locate left/right official first-generation MJCF assets. |
| `src/tianji_robotics/simulation/local_angle_bar.py` | Side-safe MuJoCo backend, explicit per-side open poses, Tkinter control panel, and local application loop. |
| `src/tianji_robotics/cli.py` | Register `tianji-robot sim wuji-angle-bar`. |
| `tests/simulation/test_local_angle_bar.py` | Model side, ranges, validation, reset, and panel-independent unit coverage. |
| `tests/cli/test_tianji_robot_cli.py` | CLI routing and side argument coverage. |
| `docs/simulation/wuji_local_angle_bar.md` | Local operator instructions and explicit non-ROS/non-hardware boundary. |

### Task 1: Side-selectable model metadata and backend

**Files:**
- Modify: `src/tianji_robotics/simulation/paths.py`
- Create: `src/tianji_robotics/simulation/local_angle_bar.py`
- Create: `tests/simulation/test_local_angle_bar.py`

**Interfaces:**
- Consumes: `mujoco.MjModel`, `mujoco.MjData`, and the official model asset tree.
- Produces: `official_wuji_hand_mjcf(side: str) -> Path`; `LocalWujiHand(side: str, *, viewer: bool = False)` with `side`, `joint_names`, `control_ranges_rad`, `target_rad`, `command(target_rad)`, `reset_open()`, `step()`, and `close()`.

- [ ] **Step 1: Write failing side/validation tests**

```python
import numpy as np
import pytest

from tianji_robotics.simulation.local_angle_bar import LocalWujiHand
from tianji_robotics.simulation.paths import official_wuji_hand_mjcf

@pytest.mark.parametrize("side", ["left", "right"])
def test_each_official_hand_has_twenty_bounded_position_actuators(side):
    backend = LocalWujiHand(side, viewer=False)
    try:
        assert backend.side == side
        assert len(backend.joint_names) == 20
        assert backend.control_ranges_rad.shape == (20, 2)
        assert np.all(np.isfinite(backend.control_ranges_rad))
        assert np.all(backend.control_ranges_rad[:, 0] < backend.control_ranges_rad[:, 1])
        assert official_wuji_hand_mjcf(side).is_file()
    finally:
        backend.close()

def test_backend_rejects_wrong_shape_nan_and_out_of_range_targets():
    backend = LocalWujiHand("left", viewer=False)
    try:
        with pytest.raises(ValueError, match="20 finite"):
            backend.command(np.zeros(19))
        with pytest.raises(ValueError, match="20 finite"):
            backend.command(np.full(20, np.nan))
        bad = backend.target_rad.copy(); bad[0] = backend.control_ranges_rad[0, 1] + 0.01
        with pytest.raises(ValueError, match="outside range"):
            backend.command(bad)
    finally:
        backend.close()
```

- [ ] **Step 2: Run the focused tests to verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_local_angle_bar.py -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'tianji_robotics.simulation.local_angle_bar'`.

- [ ] **Step 3: Implement asset lookup and backend**

```python
# paths.py
def official_wuji_hand_mjcf(side: str) -> Path:
    if side not in {"left", "right"}:
        raise ValueError("hand side must be 'left' or 'right'")
    path = _project_root() / "robot_assets/mujoco/wuji_hand_standalone/mjcf" / f"{side}.xml"
    if not path.is_file():
        raise FileNotFoundError(f"official Wuji {side}-hand model is missing: {path}")
    return path
```

```python
# local_angle_bar.py: core behavior
class LocalWujiHand:
    def __init__(self, side: str, *, viewer: bool = False) -> None:
        self.side = _validate_side(side)
        self.model = mujoco.MjModel.from_xml_path(str(official_wuji_hand_mjcf(self.side)))
        self.data = mujoco.MjData(self.model)
        self._actuator_ids = np.arange(self.model.nu, dtype=np.int32)
        if self.model.nu != 20:
            raise RuntimeError("official Wuji hand must expose 20 actuators")
        self.joint_names = tuple(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in self._actuator_ids)
        self.control_ranges_rad = self.model.actuator_ctrlrange[self._actuator_ids].copy()
        self._open_target_rad = OPEN_TARGET_RAD[self.side].copy()
        if np.any(self._open_target_rad < self.control_ranges_rad[:, 0]) or np.any(self._open_target_rad > self.control_ranges_rad[:, 1]):
            raise RuntimeError(f"{self.side} open target is outside model control ranges")
        self.command(self._open_target_rad)

    def command(self, target_rad: np.ndarray) -> None:
        target = np.asarray(target_rad, dtype=float)
        if target.shape != (20,) or not np.isfinite(target).all():
            raise ValueError("hand target must contain 20 finite radians")
        if np.any(target < self.control_ranges_rad[:, 0]) or np.any(target > self.control_ranges_rad[:, 1]):
            raise ValueError("hand target is outside range")
        self.data.ctrl[self._actuator_ids] = target
```

Implement `target_rad` as a copied view of active controls, `reset_open()` as `command(self._open_target_rad)`, `step()` as exactly one `mujoco.mj_step`, and idempotent `close()`.

- [ ] **Step 4: Run focused backend tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_local_angle_bar.py -v`

Expected: PASS.

- [ ] **Step 5: Commit the backend boundary**

```bash
git add src/tianji_robotics/simulation/paths.py src/tianji_robotics/simulation/local_angle_bar.py tests/simulation/test_local_angle_bar.py
git commit -m "feat: add local Wuji hand control backend"
```

### Task 2: Tkinter angle-bar panel and hand switching

**Files:**
- Modify: `src/tianji_robotics/simulation/local_angle_bar.py`
- Modify: `tests/simulation/test_local_angle_bar.py`

**Interfaces:**
- Consumes: `LocalWujiHand` from Task 1.
- Produces: `WujiAngleBarPanel(root, backend_factory, initial_side)` and `run_local_angle_bar(side: str) -> int`.

- [ ] **Step 1: Write failing panel-independent behavior tests**

```python
def test_reset_open_commands_active_hand_open_target():
    backend = LocalWujiHand("right", viewer=False)
    try:
        target = backend.control_ranges_rad[:, 1]
        backend.command(target)
        backend.reset_open()
        np.testing.assert_allclose(backend.target_rad, backend.open_target_rad)
        assert np.all(backend.target_rad >= backend.control_ranges_rad[:, 0])
        assert np.all(backend.target_rad <= backend.control_ranges_rad[:, 1])
    finally:
        backend.close()

def test_switching_hand_constructs_a_fresh_side_backend():
    created = []
    def factory(side, *, viewer):
        created.append((side, viewer)); return LocalWujiHand(side, viewer=False)
    # Call the panel's side-change method with a minimal fake Tk root.
    # Assert factory sees left then right and the original backend was closed.
```

- [ ] **Step 2: Run the focused tests to verify failure**

Run: `.venv/bin/python -m pytest tests/simulation/test_local_angle_bar.py::test_reset_open_commands_active_hand_open_target -v`

Expected: FAIL because `open_target_rad` and `reset_open` are not exposed.

- [ ] **Step 3: Implement panel lifecycle and model-derived sliders**

```python
class WujiAngleBarPanel:
    def __init__(self, root, backend_factory=LocalWujiHand, initial_side="left") -> None:
        self._root, self._backend_factory = root, backend_factory
        self._backend = backend_factory(initial_side, viewer=True)
        self._side = tk.StringVar(value=initial_side)
        self._targets = [tk.DoubleVar(value=value) for value in self._backend.target_rad]
        self._build_controls()
        self._root.after(0, self._tick)

    def _apply_slider(self, index: int) -> None:
        target = self._backend.target_rad
        target[index] = self._targets[index].get()
        self._backend.command(target)

    def switch_side(self, side: str) -> None:
        old = self._backend
        self._backend = self._backend_factory(side, viewer=True)
        old.close()
        self._targets = [tk.DoubleVar(value=value) for value in self._backend.target_rad]
        self._rebuild_slider_groups()
```

Build five labeled finger groups, four `tk.Scale` rows per group, a left/right `tk.Radiobutton` selector, one reset button, and per-slider radian labels. Obtain every range and name from `self._backend.control_ranges_rad` and `self._backend.joint_names`; do not hard-code side-specific limits. In `_tick`, call `self._backend.step()`, update the passive viewer if open, and reschedule only while both Tk and Viewer remain open.

- [ ] **Step 4: Run focused backend/panel tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_local_angle_bar.py -v`

Expected: PASS.

- [ ] **Step 5: Commit the local control panel**

```bash
git add src/tianji_robotics/simulation/local_angle_bar.py tests/simulation/test_local_angle_bar.py
git commit -m "feat: add Wuji local angle-bar panel"
```

### Task 3: Explicit local CLI and operator documentation

**Files:**
- Modify: `src/tianji_robotics/cli.py`
- Modify: `tests/cli/test_tianji_robot_cli.py`
- Create: `docs/simulation/wuji_local_angle_bar.md`

**Interfaces:**
- Consumes: `run_local_angle_bar(side: str) -> int` from Task 2.
- Produces: `tianji-robot sim wuji-angle-bar [--hand left|right]`.

- [ ] **Step 1: Write failing CLI routing tests**

```python
def test_angle_bar_routes_with_default_left_hand(monkeypatch):
    calls = []
    monkeypatch.setattr(cli, "_run_wuji_angle_bar", lambda args: calls.append(args) or 0)
    assert cli.main(["sim", "wuji-angle-bar"]) == 0
    assert calls[0].hand == "left"

def test_angle_bar_accepts_right_hand(monkeypatch):
    calls = []
    monkeypatch.setattr(cli, "_run_wuji_angle_bar", lambda args: calls.append(args) or 0)
    assert cli.main(["sim", "wuji-angle-bar", "--hand", "right"]) == 0
    assert calls[0].hand == "right"
```

- [ ] **Step 2: Run CLI tests to verify failure**

Run: `.venv/bin/python -m pytest tests/cli/test_tianji_robot_cli.py -k angle_bar -v`

Expected: FAIL because `wuji-angle-bar` is not a recognized command.

- [ ] **Step 3: Add a lazy local-only CLI route and document it**

```python
# _parser(), under `sim_commands`
angle_bar = sim_commands.add_parser("wuji-angle-bar", help="locally control one simulated Wuji hand with angle bars")
angle_bar.add_argument("--hand", choices=("left", "right"), default="left")
angle_bar.set_defaults(handler=_run_wuji_angle_bar)

def _run_wuji_angle_bar(args: argparse.Namespace) -> int:
    from tianji_robotics.simulation.local_angle_bar import run_local_angle_bar
    return run_local_angle_bar(args.hand)
```

Document the command, the left/right switch, radian units, reset action, dependencies (`.venv` plus a graphical session), and the hard boundary: it is MuJoCo-only and never opens ROS, USB, or a real hand SDK.

- [ ] **Step 4: Run CLI, documentation, and focused simulation tests**

Run: `.venv/bin/python -m pytest tests/cli/test_tianji_robot_cli.py tests/simulation/test_local_angle_bar.py -v`

Expected: PASS.

- [ ] **Step 5: Commit user-facing entry points**

```bash
git add src/tianji_robotics/cli.py tests/cli/test_tianji_robot_cli.py docs/simulation/wuji_local_angle_bar.md
git commit -m "feat: expose local Wuji angle-bar control"
```

### Task 4: Final regression and manual GUI acceptance

**Files:**
- Verify: `src/tianji_robotics/simulation/local_angle_bar.py`
- Verify: `src/tianji_robotics/cli.py`
- Verify: `docs/simulation/wuji_local_angle_bar.md`

**Interfaces:**
- Consumes: completed Tasks 1–3.
- Produces: verified local-only Wuji angle-bar controller.

- [ ] **Step 1: Run static import and full relevant tests**

Run: `.venv/bin/python -m pytest tests/simulation/test_local_angle_bar.py tests/simulation/test_wuji_hand_only_backend.py tests/cli/test_tianji_robot_cli.py tests/architecture/test_package_boundaries.py -v`

Expected: PASS with no ROS, hardware SDK, or `wujihandpy` import required.

- [ ] **Step 2: Run manual left-hand GUI acceptance**

Run: `.venv/bin/tianji-robot sim wuji-angle-bar --hand left`

Expected: one MuJoCo Viewer and one angle-bar window open; every slider stays within its shown radian range and visibly updates the corresponding left-hand joint; Reset returns to the documented open pose.

- [ ] **Step 3: Run manual right-hand switch acceptance**

Run: `.venv/bin/tianji-robot sim wuji-angle-bar --hand right`

Expected: the right-hand model loads; all 20 slider ranges/names are rebuilt from that model; no left-hand target or stale viewer remains after switching back and forth.

- [ ] **Step 4: Inspect the final diff**

Run: `git diff --check HEAD~3..HEAD && git status --short`

Expected: no whitespace errors; only known pre-existing user changes remain outside committed feature files.

- [ ] **Step 5: Commit any acceptance-only documentation correction**

```bash
git add docs/simulation/wuji_local_angle_bar.md
git commit -m "docs: record Wuji angle-bar verification"
```

Only perform this commit if manual acceptance required a documentation correction; otherwise leave history unchanged.
