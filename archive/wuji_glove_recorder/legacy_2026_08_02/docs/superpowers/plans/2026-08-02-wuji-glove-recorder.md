# Wuji Glove Recorder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a two-button desktop application that records right Wuji Glove skeleton frames to a portable NPZ file without controlling the robot hand.

**Architecture:** One Python module keeps the small application self-contained. A pure `Recording` data class validates and serializes frames, `GloveSession` owns the SDK connection/subscription, and `RecorderApp` runs the Tkinter interface while a daemon worker receives frames. The worker communicates with the UI only through a queue and never imports ROS.

**Tech Stack:** Python 3, Tkinter, `wuji-sdk`, NumPy, pytest.

## Global Constraints

- Device endpoint is `192.168.10.151:50001` by default; the program must never change device network settings.
- The program must only subscribe to `hand_skeleton`; it must not import `rclpy`, publish ROS messages, connect to, enable, or command the robot hand.
- Each valid frame consists of exactly 21 Cartesian keypoints in meters, stored as `float64` in shape `(N, 21, 3)`.
- Files are compressed NPZ files under `wuji-glove-recorder/recordings/` and include `timestamps_ns`, `keypoints_m`, and JSON metadata.
- The UI has exactly the user-facing recording actions “开始记录” and “停止并保存”.

---

## File Structure

- `wuji-glove-recorder/glove_recorder.py`: application entry point, frame extraction, recording serialization, SDK lifecycle, and UI.
- `wuji-glove-recorder/requirements.txt`: external Python dependencies for the isolated recorder environment.
- `wuji-glove-recorder/tests/test_glove_recorder.py`: pure recording-data behavior tests; no hardware or GUI required.
- `wuji-glove-recorder/README.md`: setup, launch, output format, and safety statement.

### Task 1: Implement and test the pure recording data layer

**Files:**
- Create: `wuji-glove-recorder/glove_recorder.py`
- Create: `wuji-glove-recorder/tests/test_glove_recorder.py`

**Interfaces:**
- Produces `extract_keypoints(frame: object) -> tuple[tuple[float, float, float], ...]`, raising `ValueError` unless a frame yields 21 finite 3D points.
- Produces `Recording(endpoint: str, serial_number: str, hand_side: str)` with `append(timestamp_ns: int, points: Sequence[Sequence[float]]) -> None` and `save(output_dir: pathlib.Path) -> pathlib.Path`.

- [ ] **Step 1: Write failing tests for frame validation and NPZ content**

```python
from pathlib import Path
import numpy as np
import pytest
from glove_recorder import Recording, extract_keypoints


def test_extract_keypoints_accepts_list_position_frames():
    class Position:
        def __init__(self, xyz): self.xyz = xyz
        def __iter__(self): return iter(self.xyz)
    class Pose:
        def __init__(self, xyz): self.position = Position(xyz)
    class Joint:
        def __init__(self, xyz): self.pose = Pose(xyz)
    class Frame:
        joints = [Joint((i, i + 1, i + 2)) for i in range(21)]

    points = extract_keypoints(Frame())
    assert len(points) == 21
    assert points[20] == (20.0, 21.0, 22.0)


def test_save_writes_expected_arrays_and_metadata(tmp_path: Path):
    recording = Recording('192.168.10.151:50001', 'SN1', 'right')
    recording.append(123, [(0.0, 0.0, 0.0)] * 21)
    path = recording.save(tmp_path)

    with np.load(path) as data:
        assert data['timestamps_ns'].tolist() == [123]
        assert data['keypoints_m'].shape == (1, 21, 3)
        assert 'SN1' in str(data['metadata'])


def test_save_rejects_an_empty_recording(tmp_path: Path):
    with pytest.raises(ValueError, match='no valid frames'):
        Recording('endpoint', 'SN1', 'right').save(tmp_path)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd wuji-glove-recorder && pytest tests/test_glove_recorder.py -v`

Expected: FAIL because `glove_recorder` does not exist.

- [ ] **Step 3: Implement the minimal data layer**

```python
def extract_keypoints(frame):
    points = []
    for joint in frame.joints:
        xyz = tuple(float(value) for value in joint.pose.position)
        if len(xyz) != 3 or not all(math.isfinite(value) for value in xyz):
            raise ValueError('invalid keypoint')
        points.append(xyz)
    if len(points) != 21:
        raise ValueError('expected 21 keypoints')
    return tuple(points)


@dataclass
class Recording:
    endpoint: str
    serial_number: str
    hand_side: str
    timestamps_ns: list[int] = field(default_factory=list)
    frames: list[tuple[tuple[float, float, float], ...]] = field(default_factory=list)

    def append(self, timestamp_ns, points):
        if len(points) != 21:
            raise ValueError('expected 21 keypoints')
        self.timestamps_ns.append(int(timestamp_ns))
        self.frames.append(tuple(tuple(map(float, point)) for point in points))

    def save(self, output_dir):
        if not self.frames:
            raise ValueError('no valid frames to save')
        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / f'glove_{datetime.now():%Y%m%d_%H%M%S}.npz'
        metadata = json.dumps({'endpoint': self.endpoint, 'serial_number': self.serial_number, 'hand_side': self.hand_side})
        np.savez_compressed(path, timestamps_ns=np.asarray(self.timestamps_ns, dtype=np.int64), keypoints_m=np.asarray(self.frames, dtype=np.float64), metadata=np.asarray(metadata))
        return path
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd wuji-glove-recorder && pytest tests/test_glove_recorder.py -v`

Expected: PASS with three tests.

### Task 2: Add safe SDK session and the two-button UI

**Files:**
- Modify: `wuji-glove-recorder/glove_recorder.py`
- Modify: `wuji-glove-recorder/tests/test_glove_recorder.py`

**Interfaces:**
- Consumes `Recording` and `extract_keypoints` from Task 1.
- Produces `GloveSession.connect()`, `GloveSession.recv_points()`, `GloveSession.close()`, and executable `RecorderApp`.

- [ ] **Step 1: Write a failing test for a rejected malformed keypoint**

```python
def test_append_rejects_malformed_keypoint():
    recording = Recording('endpoint', 'SN1', 'right')
    with pytest.raises(ValueError, match='invalid keypoint'):
        recording.append(1, [(0.0, 0.0, 0.0)] * 20 + [(1.0, 2.0)])
```

- [ ] **Step 2: Run the focused test to verify it fails**

Run: `cd wuji-glove-recorder && pytest tests/test_glove_recorder.py::test_append_rejects_malformed_keypoint -v`

Expected: FAIL because `append` currently permits a point that does not contain three values.

- [ ] **Step 3: Implement lifecycle and UI behavior**

Implement `GloveSession` using only `SdkManager.instance().connect(address=endpoint, device_name='glove_recorder')`, `glove.hand_skeleton().subscribe()`, and `manager.disconnect(glove)`. Obtain serial number and side after connection. In `RecorderApp`, create a status label, frame-count label, duration label, and `开始记录` / `停止并保存` buttons. Start a daemon worker only after the Start button; it calls `recv()`, runs `extract_keypoints`, and appends `time.monotonic_ns()` data while a `threading.Event` is set. Use `root.after(100, ...)` to refresh labels and handle queued errors. Disable Start during recording, disable Stop otherwise. On Stop, join the worker briefly, call `Recording.save(recordings_dir)`, show the file path, and retain no recording buffer. Bind window close to stop recording, close the SDK session, then destroy the root window.

The module's `main()` creates `tk.Tk()`, creates `RecorderApp(root, endpoint=DEFAULT_ENDPOINT)`, calls `app.connect()`, then starts `root.mainloop()`. It must contain no `rclpy` import and no ROS topic name.

- [ ] **Step 4: Run all pure tests**

Run: `cd wuji-glove-recorder && pytest tests/test_glove_recorder.py -v`

Expected: PASS with four tests.

### Task 3: Add install and operator documentation, then perform live verification

**Files:**
- Create: `wuji-glove-recorder/requirements.txt`
- Create: `wuji-glove-recorder/README.md`

**Interfaces:**
- Consumes `glove_recorder.py` from Tasks 1–2.
- Produces repeatable setup and launch instructions for the existing virtual environment.

- [ ] **Step 1: Write the dependency file**

```text
wuji-sdk>=2026.7.21
numpy>=1.24
pytest>=8
```

- [ ] **Step 2: Write the operator README**

Document these exact commands:

```bash
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pip install -r requirements.txt
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python glove_recorder.py
```

State that the program only reads `hand_skeleton`, has no ROS dependency, does not command the robot, and writes output below `recordings/`. Explain `timestamps_ns`, `keypoints_m`, and `metadata`.

- [ ] **Step 3: Install the non-system dependency and run tests**

Run: `/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pip install -r wuji-glove-recorder/requirements.txt && /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pytest wuji-glove-recorder/tests/test_glove_recorder.py -v`

Expected: NumPy installs into the existing isolated venv; all four tests PASS.

- [ ] **Step 4: Perform a manual hardware check**

Run: `cd wuji-glove-recorder && /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python glove_recorder.py`

Expected: window reports successful glove connection. Click 开始记录, make a 5-second open/close gesture, click 停止并保存, and confirm the status gives a new `recordings/glove_*.npz` file.

- [ ] **Step 5: Verify the saved recording shape and metadata**

Run: `cd wuji-glove-recorder && /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -c "import glob, numpy as n; p=sorted(glob.glob('recordings/*.npz'))[-1]; d=n.load(p); print(p, d['timestamps_ns'].shape, d['keypoints_m'].shape, d['metadata'])"`

Expected: output includes `(N,)`, `(N, 21, 3)`, and the right glove serial number, where `N > 0`.
