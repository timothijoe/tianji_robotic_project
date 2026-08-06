# 左臂离线轨迹接入段 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 生成 20 秒、200 Hz 的左臂离线接入 NPZ，从明确给出的当前关节角平滑过渡到来源抓取轨迹的左臂首帧。

**Architecture:** 纯 NumPy 插值、输入校验和 NPZ 序列化置于独立数据模块；CLI 只解析参数、调用模块并打印审核摘要。模块不得导入 SDK、网络或实时控制代码，产物不含任何右臂或右手命令。

**Tech Stack:** Python 3、NumPy、pytest。

## Global Constraints

- 工具只能读写本地 NPZ；不得连接 `192.168.1.190`、导入 `SDK_PYTHON` 或发送机械臂指令。
- 默认采样率固定为 200 Hz，默认时长固定为 20 秒；时间数组包含 0 和 20 秒，共 4001 帧。
- 采用五次平滑插值 `10s^3 - 15s^4 + 6s^5`，两端速度和加速度为零。
- 输出仅包含左臂数据和审核元数据，文件名必须含 `offline_entry_only`。
- 起点必须由 CLI 的七个显式角度（度）提供；不得在生成阶段读取真机反馈。

---

### Task 1: 离线接入轨迹数据模块

**Files:**
- Create: `src/tianji_robotics/data/left_arm_entry.py`
- Test: `tests/data/test_left_arm_entry.py`

**Interfaces:**
- Produces: `generate_left_arm_entry(start_rad: np.ndarray, destination_rad: np.ndarray, *, duration_s: float = 20.0, sample_rate_hz: float = 200.0) -> EntryTrajectory`.
- Produces: `EntryTrajectory.time_s`, `EntryTrajectory.left_arm_target_rad`, `EntryTrajectory.peak_velocity_rad_s`, `EntryTrajectory.peak_acceleration_rad_s2`.
- Produces: `validate_joint_vector(values: np.ndarray, *, name: str) -> np.ndarray`.

- [ ] **Step 1: Write the failing endpoint and timing test**

```python
import numpy as np
from tianji_robotics.data.left_arm_entry import generate_left_arm_entry

def test_generates_200hz_20_second_quintic_entry_with_exact_endpoints():
    start = np.deg2rad(np.array([-90, -90, 90, -90, 0, 0, 0], dtype=float))
    target = np.deg2rad(np.array([71.431, -66.107, -49.733, -128.492, 94.122, 45.648, -49.911]))
    result = generate_left_arm_entry(start, target)
    assert result.time_s.shape == (4001,)
    assert result.left_arm_target_rad.shape == (4001, 7)
    assert result.time_s[0] == 0.0
    assert result.time_s[-1] == 20.0
    assert np.allclose(np.diff(result.time_s), 0.005)
    assert np.array_equal(result.left_arm_target_rad[0], start)
    assert np.array_equal(result.left_arm_target_rad[-1], target)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests/data/test_left_arm_entry.py`

Expected: FAIL with `ModuleNotFoundError: No module named 'tianji_robotics.data.left_arm_entry'`.

- [ ] **Step 3: Write minimal interpolation implementation**

```python
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class EntryTrajectory:
    time_s: np.ndarray
    left_arm_target_rad: np.ndarray
    peak_velocity_rad_s: np.ndarray
    peak_acceleration_rad_s2: np.ndarray

def validate_joint_vector(values, *, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (7,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain seven finite joint values")
    return result.copy()

def generate_left_arm_entry(start_rad, destination_rad, *, duration_s=20.0, sample_rate_hz=200.0):
    start = validate_joint_vector(start_rad, name="start_rad")
    destination = validate_joint_vector(destination_rad, name="destination_rad")
    if not np.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be positive and finite")
    if not np.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
        raise ValueError("sample_rate_hz must be positive and finite")
    intervals = int(round(float(duration_s) * float(sample_rate_hz)))
    if intervals < 1 or not np.isclose(intervals / sample_rate_hz, duration_s):
        raise ValueError("duration_s * sample_rate_hz must be a positive integer")
    time_s = np.linspace(0.0, float(duration_s), intervals + 1)
    phase = time_s / float(duration_s)
    blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
    joints = start + blend[:, None] * (destination - start)
    joints[0], joints[-1] = start, destination
    velocity = np.gradient(joints, time_s, axis=0)
    acceleration = np.gradient(velocity, time_s, axis=0)
    return EntryTrajectory(time_s, joints, np.max(np.abs(velocity), axis=0), np.max(np.abs(acceleration), axis=0))
```

- [ ] **Step 4: Add validation and smooth-boundary tests**

```python
import pytest

@pytest.mark.parametrize("bad", [np.zeros(6), np.full(7, np.nan)])
def test_rejects_nonfinite_or_nonseven_joint_vector(bad):
    with pytest.raises(ValueError, match="seven finite"):
        generate_left_arm_entry(bad, np.zeros(7))

def test_has_near_zero_boundary_velocity_and_acceleration():
    result = generate_left_arm_entry(np.zeros(7), np.ones(7), duration_s=2.0, sample_rate_hz=200.0)
    velocity = np.gradient(result.left_arm_target_rad, result.time_s, axis=0)
    acceleration = np.gradient(velocity, result.time_s, axis=0)
    assert np.all(np.abs(velocity[[0, -1]]) < 2e-4)
    assert np.all(np.abs(acceleration[[0, -1]]) < 0.05)
```

- [ ] **Step 5: Run module tests and commit**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests/data/test_left_arm_entry.py`

Expected: PASS.

```bash
git add src/tianji_robotics/data/left_arm_entry.py tests/data/test_left_arm_entry.py
git commit -m "feat: generate offline left arm entry trajectories"
```

### Task 2: NPZ 写入和离线 CLI

**Files:**
- Modify: `src/tianji_robotics/data/left_arm_entry.py`
- Create: `scripts/generate_left_arm_entry_trajectory.py`
- Modify: `tests/data/test_left_arm_entry.py`

**Interfaces:**
- Consumes: `generate_left_arm_entry(...) -> EntryTrajectory`.
- Produces: `save_left_arm_entry_npz(trajectory: EntryTrajectory, *, source_npz: Path, destination: Path, start_deg: np.ndarray, duration_s: float, sample_rate_hz: float) -> Path`.
- Produces: a direct CLI accepting `--source-npz`, `--start-deg`, `--output`, optional `--duration-s`, and optional `--sample-rate-hz`.

- [ ] **Step 1: Write the failing NPZ content test**

```python
def test_save_writes_left_arm_only_entry_metadata(tmp_path):
    source = tmp_path / "source.npz"
    np.savez(source, left_arm_target_rad=np.vstack((np.zeros(7), np.ones(7))))
    output = tmp_path / "left_offline_entry_only.npz"
    trajectory = generate_left_arm_entry(np.zeros(7), np.ones(7), duration_s=1.0, sample_rate_hz=10.0)
    saved = save_left_arm_entry_npz(trajectory, source_npz=source, destination=output, start_deg=np.zeros(7), duration_s=1.0, sample_rate_hz=10.0)
    with np.load(saved, allow_pickle=False) as data:
        assert set(data.files) == {"format_version", "time_s", "left_arm_target_rad", "source_npz_path", "entry_start_deg", "entry_destination_rad", "duration_s", "sample_rate_hz", "interpolation", "peak_velocity_rad_s", "peak_acceleration_rad_s2"}
        assert "right_arm_target_rad" not in data.files
        assert "right_hand_target_rad" not in data.files
        assert data["interpolation"].item() == "quintic_smoothstep"
```

- [ ] **Step 2: Run it to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests/data/test_left_arm_entry.py::test_save_writes_left_arm_only_entry_metadata`

Expected: FAIL with `NameError: name 'save_left_arm_entry_npz' is not defined`.

- [ ] **Step 3: Implement NPZ serialization and CLI**

```python
def save_left_arm_entry_npz(trajectory, *, source_npz, destination, start_deg, duration_s, sample_rate_hz):
    destination = Path(destination)
    if "offline_entry_only" not in destination.stem:
        raise ValueError("output filename must contain 'offline_entry_only'")
    np.savez(destination, format_version=np.asarray(1, dtype=np.int64),
        time_s=trajectory.time_s, left_arm_target_rad=trajectory.left_arm_target_rad,
        source_npz_path=np.asarray(str(Path(source_npz))), entry_start_deg=np.asarray(start_deg, dtype=float),
        entry_destination_rad=trajectory.left_arm_target_rad[-1], duration_s=np.asarray(duration_s, dtype=float),
        sample_rate_hz=np.asarray(sample_rate_hz, dtype=float), interpolation=np.asarray("quintic_smoothstep"),
        peak_velocity_rad_s=trajectory.peak_velocity_rad_s, peak_acceleration_rad_s2=trajectory.peak_acceleration_rad_s2)
    return destination
```

CLI must load only `left_arm_target_rad` from `--source-npz`, reject a missing or malformed `(N, 7)` array, parse seven comma-separated `--start-deg` values with `np.deg2rad`, call the two module APIs, and print path, frame count, duration, per-joint total change, peak velocity and peak acceleration. It must not import `SDK_PYTHON`, `socket`, or a real-robot debug module.

- [ ] **Step 4: Add subprocess CLI test**

```python
def test_cli_writes_offline_entry_file(tmp_path):
    source = tmp_path / "source.npz"
    output = tmp_path / "result_offline_entry_only.npz"
    np.savez(source, left_arm_target_rad=np.vstack((np.ones(7), np.zeros(7))))
    result = subprocess.run([sys.executable, "scripts/generate_left_arm_entry_trajectory.py",
        "--source-npz", str(source), "--start-deg", "0,0,0,0,0,0,0", "--output", str(output),
        "--duration-s", "1", "--sample-rate-hz", "10"], check=False, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert output.exists()
```

- [ ] **Step 5: Run focused tests**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests/data/test_left_arm_entry.py`

Expected: PASS.

- [ ] **Step 6: Generate the requested offline artifact**

Run:

```bash
python3 scripts/generate_left_arm_entry_trajectory.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate.npz \
  --start-deg=-90.001,-90.000,90.000,-89.996,-0.000,0.000,-0.001 \
  --output recordings/left_arm_offline_entry_only_20s_200hz.npz
```

Expected: exits 0, reports 4001 frames, and writes only a local NPZ file.

- [ ] **Step 7: Verify generated NPZ is structurally safe**

Run:

```bash
python3 -c "import numpy as np; d=np.load('recordings/left_arm_offline_entry_only_20s_200hz.npz', allow_pickle=False); assert d['time_s'].shape == (4001,); assert d['left_arm_target_rad'].shape == (4001, 7); assert 'right_arm_target_rad' not in d.files; assert 'right_hand_target_rad' not in d.files; print(d.files)"
```

Expected: exits 0 and lists only the documented left-arm fields.

- [ ] **Step 8: Commit the CLI and verified offline artifact**

```bash
git add src/tianji_robotics/data/left_arm_entry.py tests/data/test_left_arm_entry.py scripts/generate_left_arm_entry_trajectory.py recordings/left_arm_offline_entry_only_20s_200hz.npz
git commit -m "feat: export offline left arm entry trajectory"
```

