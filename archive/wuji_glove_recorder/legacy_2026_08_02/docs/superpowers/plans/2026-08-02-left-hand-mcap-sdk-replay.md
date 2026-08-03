# 左手 MCAP SDK 实体回放 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fail-closed SDK-direct replay command for validated converted left-hand MCAP trajectories.

**Architecture:** Reuse `replay_left_hand_mcap.py` for offline MCAP validation and ramp math. Keep SDK lifecycle in a new module with injected manager/process probes so tests never access USB; only the `--arm` path performs SDK discovery, left-side verification, enable, realtime streaming, disable and disconnect.

**Tech Stack:** Python 3.12, existing `wuji-sdk 2026.7.21`, NumPy, MCAP, pytest, POSIX sh.

## Global Constraints

- No `--arm`: never scan, connect, enable, or command hardware.
- Reject `--arm` when ROS driver process/node or another SDK replay is active; do not kill it.
- Require exactly one first-generation `DeviceType.WujiHand` and `handedness_name() == "Left"`.
- Reuse existing converted-MCAP/MJCF/maximum-step validation; do not clip or add a force flag.
- Default speed `0.1`, permitted `(0, 1]`; ramp `3.0` seconds; effort `1.5 A`; LowPass cutoff `5 Hz`.
- Cleanup closes publisher, disables hand, and disconnects all SDK devices on every armed exit.
- Do not execute `--arm` in automated verification. No Git repository exists, so do not commit.

---

### Task 1: Create testable SDK safety primitives

**Files:**
- Create: `wuji-glove-recorder/replay_left_hand_mcap_sdk.py`
- Create: `wuji-glove-recorder/tests/test_replay_left_hand_mcap_sdk.py`

**Interfaces:**
- `SdkReplayConfig(path: Path, arm: bool, speed: float, ramp_seconds: float, max_step_rad: float, effort_limit: float, cutoff_hz: float)`.
- `parse_args(argv) -> SdkReplayConfig`.
- `assert_sdk_control_available(node_names: Sequence[str], process_lines: Sequence[str]) -> None`.
- `interpolated_positions(start, end, max_step_rad, hz=100) -> tuple[np.ndarray, ...]`.

- [ ] **Step 1: Write failing tests**

```python
def test_control_check_rejects_ros_driver():
    with pytest.raises(RuntimeError, match="stop ROS driver"):
        assert_sdk_control_available(["/hand_0/wujihand_driver"], [])

def test_interpolation_never_exceeds_step_limit():
    frames = interpolated_positions(np.zeros(20), np.full(20, .2), .08)
    assert np.abs(np.diff(np.asarray(frames), axis=0)).max() <= .08
```

- [ ] **Step 2: Run and verify RED**

```bash
cd /home/zhoutong/catkin_robotic_ws/august_ws/wuji-technology/wuji-glove-recorder
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pytest tests/test_replay_left_hand_mcap_sdk.py -v
```

Expected: module import failure.

- [ ] **Step 3: Implement minimal pure functions**

Implement argparse defaults/ranges exactly from the spec; reject a node name ending `wujihand_driver`, process lines containing `wujihand_driver_node` or `replay_left_hand_mcap_sdk.py`, and any nonfinite/wrong-shape interpolation input. Import neither `wuji_sdk` nor `subprocess` at module import time.

- [ ] **Step 4: Run focused tests**

Run the Step 2 command. Expected: PASS.

### Task 2: Implement injected SDK lifecycle and cleanup

**Files:**
- Modify: `wuji-glove-recorder/replay_left_hand_mcap_sdk.py`
- Modify: `wuji-glove-recorder/tests/test_replay_left_hand_mcap_sdk.py`

**Interfaces:**
- `run_sdk_replay(config, trajectory, manager, sdk_types, sleep=time.sleep) -> int`.
- `send_positions(publisher, joint_command_type, positions) -> None`.

- [ ] **Step 1: Write failing fake-SDK tests**

Create fake manager/device/hand/controller/publisher classes. Assert that success sends 20 `JointCommand(position, 0.0, 0.0)` values per call; wrong side raises; and `publisher.close`, `hand.disable`, `manager.disconnect_all` occur both after success and after a publisher exception.

```python
def test_wrong_side_is_disabled_and_disconnected(valid_trajectory):
    hand = FakeHand(side="right")
    with pytest.raises(RuntimeError, match="left"):
        run_sdk_replay(config, valid_trajectory, FakeManager(hand), FakeSdkTypes(), sleep=lambda _: None)
    assert hand.disabled and hand.manager.disconnected
```

- [ ] **Step 2: Run and verify RED**

Run the Task 1 pytest command. Expected: missing `run_sdk_replay` or failing fake lifecycle assertions.

- [ ] **Step 3: Implement lifecycle**

Within `run_sdk_replay`: scan/filter exactly one `DeviceType.WujiHand`; connect by SN; verify left; read 20 finite actual positions; set effort limit; enable; enter `realtime_controller(LowPass(cutoff_hz=...))`; publish the 100 Hz ramp then timestamp/speed trajectory with interpolation. Use `try/finally` so publisher closes if constructed, `hand.disable()` is attempted after enable, and `manager.disconnect_all()` always runs. Never send a zero/home command in cleanup.

- [ ] **Step 4: Run focused tests**

Run the Task 1 pytest command. Expected: PASS; no test scans USB.

### Task 3: Add arm-gated main, launcher, and SDK operator docs

**Files:**
- Modify: `wuji-glove-recorder/replay_left_hand_mcap_sdk.py`
- Create: `wuji-glove-recorder/replay_left_hand_mcap_sdk.sh`
- Modify: `wuji-glove-recorder/tests/test_replay_left_hand_mcap_sdk.py`
- Modify: `wuji-glove-recorder/README.md`
- Modify: `doc_zt/04_MCAP实体左手回放.md`
- Modify: `doc_zt/05_新Agent交接与复现清单.md`

- [ ] **Step 1: Write failing arm-gate and shell tests**

```python
def test_main_without_arm_never_loads_sdk(monkeypatch, valid_path):
    monkeypatch.setitem(sys.modules, "wuji_sdk", None)
    assert main([str(valid_path)]) == 0

def test_sdk_launcher_dry_run_prints_sdk_script(tmp_path):
    result = subprocess.run(["sh", str(LAUNCHER), "--dry-run", str(tmp_path / "x_right_to_left_wuji_hand.mcap")], capture_output=True, text=True, check=True)
    assert "replay_left_hand_mcap_sdk.py" in result.stdout
```

- [ ] **Step 2: Run and verify RED**

Run the Task 1 pytest command. Expected: missing main/launcher behavior.

- [ ] **Step 3: Implement main and POSIX launcher**

`main` loads the existing validated trajectory and reports it. Without `--arm`, print that SDK was not loaded and return 0. With `--arm`, collect ROS node names with `ros2 node list` only when available, collect `pgrep -af` output, call control check, then lazily import `wuji_sdk` and call lifecycle. Launcher must use `/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python`, support shell-only `--dry-run`, and never source ROS.

- [ ] **Step 4: Document exact control handoff**

Document: stop ROS launch → prove `/hand_0/wujihand_driver` absent → SDK preflight → `--arm --speed 0.05 --ramp-seconds 5` → on exit the SDK disables hand → restart ROS before returning to ROS tools.

- [ ] **Step 5: Verify without hardware**

```bash
sh -n replay_left_hand_mcap_sdk.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python -m pytest tests -v
sh replay_left_hand_mcap_sdk.sh --dry-run /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
sh replay_left_hand_mcap_sdk.sh /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

Expected: all tests pass; dry run prints only; normal command passes offline preflight without scanning SDK or moving hardware. Never run `--arm` automatically.

## Plan self-review

- Task 1 covers CLI, conflict gate and safe interpolation.
- Task 2 covers first-generation SDK discovery, left-side check, realtime command format and all cleanup paths.
- Task 3 covers lazy arm gating, POSIX invocation, docs and full non-hardware verification.
- No task adds a bypass, ROS shutdown action, direct hardware test, or automatic armed command.
