# 左手 MCAP 实体回放 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a fail-closed ROS 2 replay tool that sends a validated converted left-hand MCAP to the existing Wuji Hand driver only when the user explicitly supplies `--arm`.

**Architecture:** Keep MCAP parsing, named-joint normalization, MJCF-range validation, timing validation, and ramp generation in a pure-Python module that never imports ROS at module load time. The armed runtime imports `rclpy` only after offline preflight has passed, waits for actual left-hand joint feedback, ramps to the first frame, and publishes named `JointState` commands at a capped playback speed. A POSIX shell wrapper preserves the ROS environment while exposing the existing virtual environment’s MCAP package.

**Tech Stack:** Python 3.12, `mcap`, NumPy, ROS 2 Jazzy `rclpy`, `sensor_msgs/msg/JointState`, existing left-hand MJCF XML, `pytest`.

## Global Constraints

- Accept only existing `.mcap` paths whose filename includes `_right_to_left_wuji_hand`.
- Default execution is offline preflight only; no ROS initialization or command publication without `--arm`.
- `--speed` defaults to `0.1`, accepts only `(0, 1]`; never exceed recorded speed.
- `--ramp-seconds` defaults to `3.0` and must be positive; `--max-step-rad` defaults to `0.08` and must be positive.
- Publish to `/<hand-name>/joint_commands`, defaulting to `/hand_0/joint_commands`; wait for `/<hand-name>/joint_states` before any command.
- Publish all 20 canonical names `left_finger{1..5}_joint{1..4}`; never rely on implicit source order.
- Validate every source value against the left MJCF joint ranges and reject invalid trajectories; never clip or offer a force/ignore flag.
- On normal completion, `Ctrl+C`, feedback timeout/loss, or a runtime validation failure: stop publishing and issue no additional target pose.
- The tool must not start/stop/enable/reset/home the hand driver, connect to a glove, or call the hand SDK directly.
- The workspace root is not a Git repository; do not run commit commands. Record verification outputs in the final handoff instead.

---

### Task 1: Implement pure MCAP parsing and fail-closed preflight

**Files:**
- Create: `wuji-glove-recorder/replay_left_hand_mcap.py`
- Create: `wuji-glove-recorder/tests/test_replay_left_hand_mcap.py`

**Interfaces:**
- Produces `CANONICAL_LEFT_JOINT_NAMES: tuple[str, ...]` with exactly 20 names in finger-major order.
- Produces `Frame(timestamp_ns: int, positions_rad: np.ndarray)` and `Trajectory(frames: tuple[Frame, ...])`.
- Produces `load_validated_trajectory(path: Path, max_step_rad: float) -> Trajectory`.
- Produces `joint_limits_from_mjcf(path: Path) -> dict[str, tuple[float, float]]`.
- Raises `ValueError` with a specific reason for malformed paths, topic data, names, positions, timestamps, limits, or frame discontinuities.

- [ ] **Step 1: Write failing tests for canonical reordering and filename protection**

```python
from pathlib import Path
import numpy as np
import pytest

from replay_left_hand_mcap import CANONICAL_LEFT_JOINT_NAMES, normalize_joint_state


def test_normalize_joint_state_reorders_named_left_joints():
    names = list(reversed(CANONICAL_LEFT_JOINT_NAMES))
    positions = list(range(20))
    normalized = normalize_joint_state(names, positions)
    assert np.array_equal(normalized, np.arange(19, -1, -1, dtype=float))


@pytest.mark.parametrize("path", [Path("capture.mcap"), Path("right_to_left.mcap")])
def test_replay_filename_must_identify_right_to_left_output(path):
    from replay_left_hand_mcap import validate_replay_path
    with pytest.raises(ValueError, match="right_to_left_wuji_hand"):
        validate_replay_path(path)
```

- [ ] **Step 2: Run the focused tests and verify they fail**

Run:

```bash
cd /home/zhoutong/catkin_robotic_ws/august_ws/wuji-technology/wuji-glove-recorder
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pytest tests/test_replay_left_hand_mcap.py -v
```

Expected: FAIL during import because `replay_left_hand_mcap` does not exist.

- [ ] **Step 3: Implement canonical names, path validation, and named-joint normalization**

```python
CANONICAL_LEFT_JOINT_NAMES = tuple(
    f"left_finger{finger}_joint{joint}"
    for finger in range(1, 6)
    for joint in range(1, 5)
)

def validate_replay_path(path: Path) -> Path:
    resolved = path.resolve()
    if not resolved.is_file() or resolved.suffix != ".mcap":
        raise ValueError(f"expected an existing .mcap file: {resolved}")
    if "_right_to_left_wuji_hand" not in resolved.stem:
        raise ValueError("refusing a file not named *_right_to_left_wuji_hand.mcap")
    return resolved

def normalize_joint_state(names: Sequence[str], positions: Sequence[float]) -> np.ndarray:
    if len(names) != 20 or len(positions) != 20 or len(set(names)) != 20:
        raise ValueError("expected 20 unique named left-hand joints")
    by_name = dict(zip(names, positions))
    if set(by_name) != set(CANONICAL_LEFT_JOINT_NAMES):
        raise ValueError("joint names do not match the 20 canonical left-hand joints")
    result = np.asarray([by_name[name] for name in CANONICAL_LEFT_JOINT_NAMES], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError("joint positions must be finite")
    return result
```

- [ ] **Step 4: Run the focused tests and verify they pass**

Run the command from Step 2.

Expected: PASS for canonical reordering and rejected filenames.

- [ ] **Step 5: Add failing MCAP/MJCF validation tests**

Use `mcap.writer.Writer` to generate temporary JSON `/joint_states` MCAPs. Cover a valid two-frame trajectory, a missing Topic, non-monotonic timestamps, a duplicate/missing joint name, a non-finite position, an out-of-range value, and a delta larger than `0.08`.

```python
def test_load_validated_trajectory_rejects_a_joint_outside_mjcf_range(tmp_path):
    source = write_joint_state_mcap(
        tmp_path / "case_right_to_left_wuji_hand.mcap",
        frames=[(100, CANONICAL_LEFT_JOINT_NAMES, [99.0] * 20)],
    )
    with pytest.raises(ValueError, match="outside MJCF range"):
        load_validated_trajectory(source, max_step_rad=0.08)
```

- [ ] **Step 6: Implement JSON-MCAP loading and MJCF validation**

Implement the following behavior exactly:

```python
def joint_limits_from_mjcf(path: Path) -> dict[str, tuple[float, float]]:
    root = ElementTree.parse(path).getroot()
    limits = {}
    for joint in root.findall(".//joint"):
        name, raw_range = joint.get("name"), joint.get("range")
        if name and raw_range:
            low, high = map(float, raw_range.split())
            limits[f"left_{name}"] = (low, high)
    if set(limits) != set(CANONICAL_LEFT_JOINT_NAMES):
        raise ValueError("left MJCF does not define exactly 20 canonical joint ranges")
    return limits

def load_validated_trajectory(path: Path, max_step_rad: float) -> Trajectory:
    if max_step_rad <= 0:
        raise ValueError("max_step_rad must be positive")
    # validate path; iterate /joint_states with make_reader; parse JSON;
    # normalize every frame; require strictly increasing message.log_time.
    # Check each normalized vector against the corresponding MJCF range and
    # every adjacent absolute joint difference against max_step_rad.
```

Set `MODEL_PATH` using `Path(__file__).resolve().parents[1] / "mujoco-sim" / "wuji_hand_description" / "mjcf" / "left.xml"`; do not import `mujoco` merely to read XML ranges.

- [ ] **Step 7: Run all parser/preflight tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pytest tests/test_replay_left_hand_mcap.py -v
```

Expected: PASS; no test imports `rclpy`, opens a ROS graph, or touches hardware.

---

### Task 2: Implement preflight reporting, parameter parsing, and safe ramp mathematics

**Files:**
- Modify: `wuji-glove-recorder/replay_left_hand_mcap.py`
- Modify: `wuji-glove-recorder/tests/test_replay_left_hand_mcap.py`

**Interfaces:**
- Produces `ReplayConfig(arm: bool, speed: float, ramp_seconds: float, hand_name: str, state_timeout: float, max_step_rad: float, path: Path)`.
- Produces `parse_args(argv: Sequence[str] | None) -> ReplayConfig`.
- Produces `preflight_report(trajectory: Trajectory) -> str`.
- Produces `ramp_positions(start: np.ndarray, goal: np.ndarray, duration_s: float, rate_hz: float, max_step_rad: float) -> tuple[np.ndarray, ...]`.

- [ ] **Step 1: Write failing tests for CLI defaults/invalid values and ramp safety**

```python
def test_parse_args_defaults_to_offline_ten_percent_replay(tmp_path):
    config = parse_args([str(tmp_path / "sample_right_to_left_wuji_hand.mcap")])
    assert config.arm is False
    assert config.speed == 0.1
    assert config.ramp_seconds == 3.0
    assert config.hand_name == "hand_0"


def test_ramp_has_exact_endpoints_and_bounded_steps():
    frames = ramp_positions(
        np.zeros(20), np.full(20, 0.3), duration_s=1.0, rate_hz=50.0, max_step_rad=0.08
    )
    assert np.array_equal(frames[0], np.zeros(20))
    assert np.array_equal(frames[-1], np.full(20, 0.3))
    assert np.abs(np.diff(np.asarray(frames), axis=0)).max() <= 0.08
```

- [ ] **Step 2: Run the new tests and verify they fail**

Run the Task 1 pytest command with the two test names.

Expected: FAIL because configuration/ramp interfaces are absent.

- [ ] **Step 3: Implement exact parameter and ramp validation**

Use `argparse` with these flags: `--arm`, `--speed`, `--ramp-seconds`, `--hand-name`, `--state-timeout`, `--max-step-rad`, and one required `mcap` path. Reject speed outside `(0, 1]`, nonpositive ramp/timeout/max-step, and blank hand names via `parser.error`.

Implement ramp frame count as the maximum of the requested 50 Hz duration and the number required to keep every joint increment within `max_step_rad`:

```python
steps_by_time = max(1, math.ceil(duration_s * rate_hz))
steps_by_delta = max(1, math.ceil(float(np.max(np.abs(goal - start))) / max_step_rad))
steps = max(steps_by_time, steps_by_delta)
return tuple(start + (goal - start) * (index / steps) for index in range(steps + 1))
```

`preflight_report` must include frame count, duration seconds, all 20 min/max pairs, largest interframe delta, and first-frame positions.

- [ ] **Step 4: Run all replay unit tests and verify they pass**

Run the Task 1 pytest command.

Expected: PASS; execute `python replay_left_hand_mcap.py <mcap>` only after Task 3 adds `main`.

---

### Task 3: Add an armed ROS runtime with feedback wait, ramp, and timestamp replay

**Files:**
- Modify: `wuji-glove-recorder/replay_left_hand_mcap.py`
- Modify: `wuji-glove-recorder/tests/test_replay_left_hand_mcap.py`

**Interfaces:**
- Produces `run_offline(config: ReplayConfig) -> int`, which loads, validates, reports, and exits with 0 without importing `rclpy` when `config.arm` is false.
- Produces `run_armed(config: ReplayConfig, trajectory: Trajectory) -> int`, which imports ROS lazily and returns 0 on normal final-frame completion and nonzero on feedback timeout/runtime safety failure.
- Produces `main(argv: Sequence[str] | None = None) -> int`.

- [ ] **Step 1: Write a failing test proving dry-run has no ROS dependency**

```python
def test_run_offline_never_imports_rclpy(monkeypatch, valid_mcap):
    monkeypatch.setitem(sys.modules, "rclpy", None)
    config = ReplayConfig(path=valid_mcap, arm=False, speed=0.1, ramp_seconds=3.0,
                          hand_name="hand_0", state_timeout=5.0, max_step_rad=0.08)
    assert run_offline(config) == 0
```

Also test that a replay `JointState` message construction contains all 20 canonical names and exactly the supplied position vector. Keep this helper independent of an active ROS node by accepting `joint_state_type` as an injected constructor in the test.

- [ ] **Step 2: Run the focused tests and verify they fail**

Run the Task 1 pytest command with the Task 3 test names.

Expected: FAIL because `run_offline` and message construction do not exist.

- [ ] **Step 3: Implement lazy ROS runtime without hardware side effects at import time**

Implement these runtime rules:

```python
def main(argv=None) -> int:
    config = parse_args(argv)
    trajectory = load_validated_trajectory(config.path, config.max_step_rad)
    print(preflight_report(trajectory))
    if not config.arm:
        print("Preflight passed. Add --arm only when the physical hand is ready.")
        return 0
    return run_armed(config, trajectory)
```

Within `run_armed`, import `rclpy`, `Node`, `qos_profile_sensor_data`, and `JointState` locally. Create one node, publisher, and state subscriber. Keep the latest feedback only when it has 20 finite positions. Spin until feedback arrives or the monotonic timeout expires. Do not publish before feedback exists.

Publish the ramp frames at 50 Hz, skipping the duplicate start frame so the first command is a bounded movement. Replay every trajectory frame after the ramp. For each frame, wait the timestamp delta divided by `speed` using a monotonic deadline; before each publish check finite values, MJCF ranges, and delta from the last command. Build every message as:

```python
message = JointState()
message.name = list(CANONICAL_LEFT_JOINT_NAMES)
message.position = positions.astype(float).tolist()
publisher.publish(message)
```

Handle `KeyboardInterrupt` in `main` by printing `Stopped; no additional pose was published.` and return 130. In `finally`, destroy the node and call `rclpy.shutdown()` only if initialized. Never issue a compensating, zero, home, enable, or reset command.

- [ ] **Step 4: Run all pure-Python unit tests**

Run the Task 1 pytest command.

Expected: PASS. Do not run `--arm` during automated tests.

- [ ] **Step 5: Run the real sample in offline preflight mode**

Run:

```bash
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python replay_left_hand_mcap.py \
  /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

Expected: a preflight report for 499 frames and about 4.15 seconds, followed by the explicit statement that `--arm` is required. It must not import ROS, create a publisher, or command hardware.

---

### Task 4: Add a POSIX launcher, documentation, and final non-hardware verification

**Files:**
- Create: `wuji-glove-recorder/replay_left_hand_mcap.sh`
- Modify: `wuji-glove-recorder/README.md`
- Modify: `doc_zt/README.md`
- Modify: `doc_zt/运行与排障.md`
- Modify: `doc_zt/问题与决策日志.md`
- Modify: `wuji-glove-recorder/tests/test_replay_left_hand_mcap.py`

**Interfaces:**
- `replay_left_hand_mcap.sh [arguments...]` preserves existing ROS paths, adds only the MCAP virtual-environment site-packages path, and delegates to `python3 replay_left_hand_mcap.py`.
- All docs label `--arm` as a physical-action command and list the first-run command at `--speed 0.05 --ramp-seconds 5`.

- [ ] **Step 1: Write a failing launcher test**

```python
def test_launcher_dry_run_preserves_ros_pythonpath_and_calls_replayer(tmp_path):
    source = tmp_path / "sample_right_to_left_wuji_hand.mcap"
    source.touch()
    result = subprocess.run(
        ["sh", str(LAUNCHER), "--dry-run", str(source)],
        capture_output=True, text=True, check=True,
    )
    assert "replay_left_hand_mcap.py" in result.stdout
    assert "PYTHONPATH" in result.stdout
```

Define launcher `--dry-run` as a shell-only mode that prints the exact command and environment addition without invoking Python. It is separate from Python’s normal offline preflight.

- [ ] **Step 2: Run the launcher test and verify it fails**

Run the Task 1 pytest command with the launcher test name.

Expected: FAIL because the launcher does not exist.

- [ ] **Step 3: Implement the POSIX launcher**

Use this structure:

```sh
#!/bin/sh
set -eu
SCRIPT_DIR="$(CDPATH= cd "$(dirname "$0")" && pwd -P)"
MCAP_SITE="/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/lib/python3.12/site-packages"

if [ "${1:-}" = "--dry-run" ]; then
  shift
  printf 'PYTHONPATH=%s:${PYTHONPATH:-} python3 %s/replay_left_hand_mcap.py' "$MCAP_SITE" "$SCRIPT_DIR"
  for argument in "$@"; do printf ' %s' "$argument"; done
  printf '\n'
  exit 0
fi

export PYTHONPATH="$MCAP_SITE${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$SCRIPT_DIR/replay_left_hand_mcap.py" "$@"
```

Make it executable with `chmod +x`.

- [ ] **Step 4: Update user documentation**

Add a clearly marked “实体左手回放（危险：会运动硬件）” section to `wuji-glove-recorder/README.md` with:

```bash
# 只读检查，不动硬件
sh replay_left_hand_mcap.sh /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap

# 首次实体执行：先在另一个终端启动 /hand_0 驱动；有人监看且保持急停可用
sh replay_left_hand_mcap.sh --arm --speed 0.05 --ramp-seconds 5 \
  /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

State that `Ctrl+C` only stops this publisher; it is not a hardware emergency stop. Add a short dated record to each `doc_zt` file that links the design spec and describes the `--arm`, preflight, MJCF checks, ramp, and no-auto-home policy.

- [ ] **Step 5: Verify shell, tests, and offline command**

Run:

```bash
cd /home/zhoutong/catkin_robotic_ws/august_ws/wuji-technology/wuji-glove-recorder
sh -n replay_left_hand_mcap.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pytest tests -v
sh replay_left_hand_mcap.sh --dry-run /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
sh replay_left_hand_mcap.sh /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

Expected: shell syntax valid; all tests pass; dry-run prints but does not execute; normal command prints the 499-frame offline preflight success report and does not touch hardware.

- [ ] **Step 6: Do not execute the armed command automatically**

Do not run a command containing `--arm` as part of implementation or verification. In the final handoff, give the user the first-run command and explicitly request that they start the existing driver, clear the workspace, maintain direct visual supervision, and run it themselves.

---

## Plan self-review

- **Spec coverage:** Tasks 1–2 implement all offline validation, fixed limits, naming, paths, speed/ramp/delta configuration and reporting. Task 3 implements the explicit `--arm` gate, feedback wait, bounded ramp, timestamp replay, and stop-without-extra-pose behavior. Task 4 supplies the POSIX environment bridge, operator documentation, and non-hardware verification.
- **No bypass:** No task adds clipping, force flags, direct SDK control, auto-home, auto-enable, or automatic armed execution.
- **Type consistency:** `Trajectory` contains named-normalized 20-value `Frame` objects from Task 1; Tasks 2–3 consume only that form. `ReplayConfig` is introduced in Task 2 before Task 3 uses it.
- **Dependency isolation:** Tests use the existing `wuji-teleop-venv` and pure Python interfaces. ROS is imported only in `run_armed`, which automated verification will not call.
- **Repository state:** There is no Git repository at the workspace root, so the plan intentionally substitutes exact verification commands for commit steps.
