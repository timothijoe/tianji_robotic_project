# Wuji Right Index-Finger Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in command that tests one small return motion of the fixed Wuji right hand's index MCP.

**Architecture:** Put preflight, planning, and execution cleanup in a testable hardware module; leave a thin script wrapper to lazily construct the official SDK hand. The default path is read-only dry-run; every execution path is gated by fixed thermal, fault, handedness, and limit checks.

**Tech Stack:** Python 3.12, NumPy, wujihandpy, argparse, pytest.

## Global Constraints

- Control only `finger(1).joint(0)`, the index MCP of the Wuji right hand.
- Default to dry-run; require `--execute` for any write.
- Require right-hand code `0`, twenty zero error codes, and all reported temperatures `<= 40.0°C`.
- Plan exactly `current - 0.05`, `current + 0.05`, then `current` rad.
- Require each target to retain `0.01` rad margin from hardware limits.
- Do not change effort limits/control mode, run ROS 2, control Tianji arms, or implement a thermal/fault bypass.
- If enabled, call `hand.write_joint_enabled(False)` in `finally`, including on an SDK exception.

## File Structure

- Create `src/tianji_robotics/hardware/wuji_hand/index_mcp_test.py`: pure preflight/planning plus guarded execution.
- Create `scripts/test_wuji_right_index_mcp.py`: standalone CLI.
- Create `tests/hardware/test_wuji_index_mcp_test.py`: fake-SDK unit coverage; no automated test touches USB.

### Task 1: Preflight and target planning

**Files:**
- Create: `src/tianji_robotics/hardware/wuji_hand/index_mcp_test.py`
- Test: `tests/hardware/test_wuji_index_mcp_test.py`

**Interfaces:**
- `IndexMcpTestPlan(current_rad: float, targets_rad: tuple[float, float, float], max_temperature_c: float)`
- `build_index_mcp_test_plan(hand) -> IndexMcpTestPlan`

- [ ] **Step 1: Write failing tests**

```python
def test_builds_one_small_index_mcp_return_motion():
    plan = build_index_mcp_test_plan(FakeHand(position=0.4))
    assert plan.targets_rad == (0.35, 0.45, 0.4)

@pytest.mark.parametrize("hand, message", [
    (FakeHand(side=1), "right hand"),
    (FakeHand(errors=np.ones((5, 4), dtype=np.uint32)), "error codes"),
    (FakeHand(temperatures=np.full((5, 4), 40.1)), "temperature"),
])
def test_rejects_unsafe_preflight(hand, message):
    with pytest.raises(ValueError, match=message):
        build_index_mcp_test_plan(hand)
```

`FakeHand` supplies 5×4 zero errors, 25°C temperatures, position/limits, and the SDK read methods; each test overrides only the unsafe input.

- [ ] **Step 2: Verify RED**

Run: `.venv-wujihand/bin/python -m pytest -q tests/hardware/test_wuji_index_mcp_test.py`

Expected: import failure because the module is absent.

- [ ] **Step 3: Implement minimal planning**

```python
RIGHT_HANDEDNESS, INDEX_FINGER, MCP_JOINT = 0, 1, 0
MAX_TEMPERATURE_C, AMPLITUDE_RAD, LIMIT_MARGIN_RAD = 40.0, 0.05, 0.01

@dataclass(frozen=True)
class IndexMcpTestPlan:
    current_rad: float
    targets_rad: tuple[float, float, float]
    max_temperature_c: float

def build_index_mcp_test_plan(hand) -> IndexMcpTestPlan:
    if int(hand.read_handedness()) != RIGHT_HANDEDNESS:
        raise ValueError("connected device is not a right hand")
    errors = np.asarray(hand.read_joint_error_code())
    if errors.shape != (5, 4) or np.any(errors != 0):
        raise ValueError("joint error codes must all be zero")
    temperatures = np.asarray(hand.read_joint_temperature(), dtype=float)
    if temperatures.shape != (5, 4) or not np.isfinite(temperatures).all():
        raise ValueError("joint temperature matrix is invalid")
    maximum = float(np.max(temperatures))
    if maximum > MAX_TEMPERATURE_C:
        raise ValueError(f"joint temperature {maximum:.1f}°C exceeds 40.0°C")
    positions = np.asarray(hand.read_joint_actual_position(), dtype=float)
    lower = np.asarray(hand.read_joint_lower_limit(), dtype=float)
    upper = np.asarray(hand.read_joint_upper_limit(), dtype=float)
    if any(values.shape != (5, 4) or not np.isfinite(values).all()
           for values in (positions, lower, upper)):
        raise ValueError("index MCP state or limits are invalid")
    current = float(positions[INDEX_FINGER, MCP_JOINT])
    low, high = float(lower[INDEX_FINGER, MCP_JOINT]), float(upper[INDEX_FINGER, MCP_JOINT])
    targets = (current - AMPLITUDE_RAD, current + AMPLITUDE_RAD, current)
    if low >= high or any(t < low + LIMIT_MARGIN_RAD or t > high - LIMIT_MARGIN_RAD for t in targets):
        raise ValueError("index MCP target lacks hardware-limit margin")
    return IndexMcpTestPlan(current, targets, maximum)
```

Convert every SDK array with `np.asarray`, validate every 5×4 read shape, and index only `[1, 0]`.

- [ ] **Step 4: Verify GREEN**

Run: `.venv-wujihand/bin/python -m pytest -q tests/hardware/test_wuji_index_mcp_test.py`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/tianji_robotics/hardware/wuji_hand/index_mcp_test.py tests/hardware/test_wuji_index_mcp_test.py
git commit -m "feat: guard Wuji index MCP test planning"
```

### Task 2: Opt-in execution and cleanup

**Files:**
- Modify: `src/tianji_robotics/hardware/wuji_hand/index_mcp_test.py`
- Modify: `tests/hardware/test_wuji_index_mcp_test.py`

**Interface:** `run_index_mcp_test(hand, plan, *, execute: bool, dwell_s: float, sleep=time.sleep) -> bool`.

- [ ] **Step 1: Write failing execution tests**

```python
def test_dry_run_never_enables_or_writes():
    hand, plan = FakeHand(), build_index_mcp_test_plan(FakeHand())
    assert run_index_mcp_test(hand, plan, execute=False, dwell_s=0) is False
    assert hand.calls == []

def test_execution_writes_three_targets_then_disables():
    hand = FakeHand()
    assert run_index_mcp_test(hand, build_index_mcp_test_plan(hand), execute=True, dwell_s=0)
    assert hand.calls == [("enabled", True), ("target", 0.35), ("target", 0.45), ("target", 0.4), ("enabled", False)]

def test_write_error_still_disables():
    hand = FakeHand(fail_on_target=2)
    with pytest.raises(RuntimeError, match="write failed"):
        run_index_mcp_test(hand, build_index_mcp_test_plan(hand), execute=True, dwell_s=0)
    assert hand.calls[-1] == ("enabled", False)
```

Extend the fake with nested `finger().joint().write_joint_target_position()`, `write_joint_enabled()`, a call log, and optional second-write failure.

- [ ] **Step 2: Verify RED**

Run: `.venv-wujihand/bin/python -m pytest -q tests/hardware/test_wuji_index_mcp_test.py -k 'dry_run or execution or write_error'`

Expected: missing `run_index_mcp_test`.

- [ ] **Step 3: Implement execution**

```python
def run_index_mcp_test(hand, plan, *, execute, dwell_s, sleep=time.sleep):
    if not execute:
        return False
    if not np.isfinite(dwell_s) or dwell_s < 0:
        raise ValueError("dwell_s must be finite and non-negative")
    enabled = False
    try:
        hand.write_joint_enabled(True)
        enabled = True
        joint = hand.finger(INDEX_FINGER).joint(MCP_JOINT)
        for target in plan.targets_rad:
            joint.write_joint_target_position(target)
            sleep(dwell_s)
        return True
    finally:
        if enabled:
            hand.write_joint_enabled(False)
```

- [ ] **Step 4: Verify GREEN**

Run: `.venv-wujihand/bin/python -m pytest -q tests/hardware/test_wuji_index_mcp_test.py`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/tianji_robotics/hardware/wuji_hand/index_mcp_test.py tests/hardware/test_wuji_index_mcp_test.py
git commit -m "feat: add guarded Wuji index MCP execution"
```

### Task 3: CLI and no-motion hardware check

**Files:**
- Create: `scripts/test_wuji_right_index_mcp.py`
- Modify: `tests/hardware/test_wuji_index_mcp_test.py`

**Interface:** required `--serial-number`, optional `--execute`, and `--dwell-s 0.5`; return 0 for a successful dry-run/execution and 1 on any preflight or SDK error.

- [ ] **Step 1: Write a failing CLI test**

```python
def test_cli_defaults_to_dry_run(monkeypatch, capsys):
    captured = {}
    monkeypatch.setattr(cli, "_create_hand", lambda serial: FakeHand())
    monkeypatch.setattr(cli, "run_index_mcp_test",
                        lambda hand, plan, **kw: captured.update(kw) or False)
    assert cli.main(["--serial-number", "365939643134"]) == 0
    assert captured == {"execute": False, "dwell_s": 0.5}
    assert "dry-run" in capsys.readouterr().out
```

- [ ] **Step 2: Verify RED**

Run: `.venv-wujihand/bin/python -m pytest -q tests/hardware/test_wuji_index_mcp_test.py -k cli`

Expected: CLI-module import failure.

- [ ] **Step 3: Implement CLI**

The script imports `build_index_mcp_test_plan` and `run_index_mcp_test`; `_create_hand(serial_number)` performs a lazy `import wujihandpy` then calls `wujihandpy.Hand(serial_number=serial_number)`. Print current angle, three targets, and max temperature before calling execution. Catch `Exception`, print `preflight failed: <message>`, and return 1.

- [ ] **Step 4: Verify**

Run: `.venv-wujihand/bin/python -m pytest -q tests/hardware/test_wuji_index_mcp_test.py`

Expected: PASS with no USB access.

Run: `.venv-wujihand/bin/python scripts/test_wuji_right_index_mcp.py --serial-number 365939643134`

Expected now: exit 1 with an over-temperature preflight message; no command write. Do not run `--execute` in automated verification.

- [ ] **Step 5: Commit**

```bash
git add scripts/test_wuji_right_index_mcp.py tests/hardware/test_wuji_index_mcp_test.py
git commit -m "feat: add safe Wuji right index test CLI"
```

## Self-Review

- Spec coverage: Task 1 covers hand/fault/thermal/limit gates; Task 2 covers dry-run and unconditional cleanup; Task 3 covers the approved command and a no-motion real-device check.
- Placeholder scan: no unresolved sections or omitted error paths.
- Type consistency: Tasks 2–3 use only `IndexMcpTestPlan`, `build_index_mcp_test_plan`, and `run_index_mcp_test` defined above.
