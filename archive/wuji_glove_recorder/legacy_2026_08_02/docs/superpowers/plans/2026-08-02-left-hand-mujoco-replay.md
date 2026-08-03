# Left Hand MuJoCo Replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Interactively replay an offline `(N, 20)` left Wuji Hand trajectory in the existing MuJoCo left-hand model.

**Architecture:** A single standalone Python program validates the NPZ input, derives a frame index from playback state, maps the file's firmware order to MuJoCo actuators by finger-major order, and sets only simulator `qpos`. The passive viewer owns user camera interaction and receives keyboard callbacks for playback controls.

**Tech Stack:** Python 3, NumPy, MuJoCo Python bindings, pytest.

## Global Constraints

- Input fields are `timestamps_ns` shaped `(N,)` and `left_joint_positions_rad` shaped `(N, 20)` with finite values.
- Use `mujoco-sim/wuji_hand_description/mjcf/left.xml` only.
- Do not import ROS or Wuji SDK; do not scan, connect, enable, or command hardware.
- Only set `MjData.qpos` and call `mujoco.mj_forward`; do not invoke `mj_step`.
- Keyboard controls are Space, Left/Right, R, +/-, and Esc as defined in the approved design.

---

### Task 1: Implement and test NPZ loading plus frame-index controls

**Files:**
- Create: `wuji-glove-recorder/mujoco_left_replay.py`
- Create: `wuji-glove-recorder/tests/test_mujoco_left_replay.py`

**Interfaces:**
- Produces `load_trajectory(path: Path) -> Trajectory` with `timestamps_ns: np.ndarray` and `positions_rad: np.ndarray`.
- Produces `clamp_frame(index: int, frame_count: int) -> int`.

- [ ] **Step 1: Write failing tests**

```python
import numpy as np
import pytest
from mujoco_left_replay import clamp_frame, load_trajectory


def test_load_trajectory_accepts_20_joint_data(tmp_path):
    path = tmp_path / 'left.npz'
    np.savez(path, timestamps_ns=[10, 20], left_joint_positions_rad=np.zeros((2, 20)))
    trajectory = load_trajectory(path)
    assert trajectory.positions_rad.shape == (2, 20)


def test_load_trajectory_rejects_nonfinite_joint_data(tmp_path):
    path = tmp_path / 'bad.npz'
    np.savez(path, timestamps_ns=[10], left_joint_positions_rad=np.full((1, 20), np.nan))
    with pytest.raises(ValueError, match='finite'):
        load_trajectory(path)


def test_clamp_frame_stays_inside_trajectory():
    assert clamp_frame(-1, 4) == 0
    assert clamp_frame(8, 4) == 3
```

- [ ] **Step 2: Verify RED**

Run: `cd wuji-glove-recorder && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pytest tests/test_mujoco_left_replay.py -v`

Expected: FAIL because `mujoco_left_replay` does not exist.

- [ ] **Step 3: Implement the pure data layer**

Implement an immutable `Trajectory` dataclass and functions that require a non-empty `(N, 20)` finite array and one timestamp per row. `clamp_frame` returns `min(max(index, 0), frame_count - 1)` and rejects `frame_count < 1`.

- [ ] **Step 4: Verify GREEN**

Run the Task 1 command. Expected: 3 tests PASS.

### Task 2: Add passive MuJoCo playback and document use

**Files:**
- Modify: `wuji-glove-recorder/mujoco_left_replay.py`
- Modify: `wuji-glove-recorder/requirements.txt`
- Modify: `wuji-glove-recorder/README.md`

**Interfaces:**
- Consumes `Trajectory` and `clamp_frame` from Task 1.
- Produces executable `main()` accepting the trajectory file as its only positional argument.

- [ ] **Step 1: Write a failing test for a mismatched timestamp count**

```python
def test_load_trajectory_rejects_mismatched_timestamps(tmp_path):
    path = tmp_path / 'bad-time.npz'
    np.savez(path, timestamps_ns=[10], left_joint_positions_rad=np.zeros((2, 20)))
    with pytest.raises(ValueError, match='one timestamp'):
        load_trajectory(path)
```

- [ ] **Step 2: Verify RED**

Run: `cd wuji-glove-recorder && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pytest tests/test_mujoco_left_replay.py::test_load_trajectory_rejects_mismatched_timestamps -v`

Expected: FAIL because timestamp count validation is absent.

- [ ] **Step 3: Implement viewer**

Load `../mujoco-sim/wuji_hand_description/mjcf/left.xml`, resolve 20 actuator joint addresses via `model.actuator_trnid`, and copy each selected trajectory frame into the matching `data.qpos` entry before `mujoco.mj_forward`. Launch `mujoco.viewer.launch_passive` with a key callback: Space toggles play, Left/Right change a paused frame, R resets and pauses, +/- change speed through `[0.25, 0.5, 1.0, 2.0, 4.0]`, and Esc requests exit. Advance according to recording timestamp deltas divided by speed; loop after the last frame. Print current status at state changes.

- [ ] **Step 4: Add dependency and README launch command**

Append `mujoco>=3.0` to requirements and document:

```bash
cd /home/zhoutong/catkin_robotic_ws/august_ws/wuji-technology/wuji-glove-recorder
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pip install -r requirements.txt
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python mujoco_left_replay.py /home/zhoutong/catkin_robotic_ws/august_ws/wuji-record-data/august_02/session_20260802_162909_764_left_wuji_hand.npz
```

- [ ] **Step 5: Verify complete test suite and launch**

Run: `cd wuji-glove-recorder && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pytest tests -v`

Expected: all tests PASS.

Run the documented viewer command with the desktop available. Expected: a left mechanical-hand window opens; the controls work and no hardware is accessed.
