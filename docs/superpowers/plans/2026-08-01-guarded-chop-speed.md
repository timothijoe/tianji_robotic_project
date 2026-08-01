# Guarded Chop Speed Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the default guarded-chop demonstration about 1.5 times faster without changing motion geometry, controller type, safety thresholds, or phase ordering.

**Architecture:** Keep the existing state machine and MuJoCo joint-position servo architecture intact. Pin the approved timing contract in a focused unit test, then update only the `GuardedChopConfig` defaults so the existing launcher automatically uses the faster profile.

**Tech Stack:** Python 3.12, pytest, MuJoCo, Bash launcher

## Global Constraints

- Keep both robot arms and the Wuji Hand on MuJoCo joint-position actuators.
- Do not change trajectory geometry, controller limits, safety thresholds, phase ordering, ROS 2 interfaces, or physical-hardware code.
- Use these exact defaults: ready 1.2 s, cut 0.7 s, knife-up component 0.7 s, hand-open 0.4 s, hand-shift 2.0 s, hand-close 0.4 s, final hold 3.0 s.
- Keep `control_dt_s=0.01`, `interlock_settle_s=0.3`, and `stability_timeout_s=4.0`.

---

### Task 1: Pin and implement the faster default timing profile

**Files:**
- Modify: `tests/simulation/test_guarded_chop_state_machine.py`
- Modify: `src/twin_sim/tasks/guarded_chop.py`

**Interfaces:**
- Consumes: `GuardedChopConfig()` public dataclass constructor.
- Produces: the same `GuardedChopConfig` API with approved faster defaults; explicit caller overrides remain supported.

- [ ] **Step 1: Write the failing default-profile test**

Replace the partial timing assertion in `test_guarded_chop_defaults_match_approved_motion` with:

```python
    assert config.control_dt_s == 0.01
    assert config.guard_ready_duration_s == 1.2
    assert config.cut_duration_s == 0.7
    assert config.knife_up_duration_s == 0.7
    assert config.hand_open_duration_s == 0.4
    assert config.hand_shift_duration_s == 2.0
    assert config.hand_close_duration_s == 0.4
    assert config.interlock_settle_s == 0.3
    assert config.stability_timeout_s == 4.0
    assert config.final_hold_s == 3.0
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
.venv/bin/pytest tests/simulation/test_guarded_chop_state_machine.py::test_guarded_chop_defaults_match_approved_motion -v
```

Expected: FAIL because the current ready duration is `2.0`, proving the test detects the old timing profile.

- [ ] **Step 3: Apply the minimal configuration change**

Set these `GuardedChopConfig` defaults in `src/twin_sim/tasks/guarded_chop.py`:

```python
    guard_ready_duration_s: float = 1.2
    cut_duration_s: float = 0.7
    knife_up_duration_s: float = 0.7
    hand_open_duration_s: float = 0.4
    hand_shift_duration_s: float = 2.0
    hand_close_duration_s: float = 0.4
    interlock_settle_s: float = 0.3
    stability_timeout_s: float = 4.0
    final_hold_s: float = 3.0
```

- [ ] **Step 4: Run focused tests and verify GREEN**

Run:

```bash
.venv/bin/pytest tests/simulation/test_guarded_chop_state_machine.py -v
```

Expected: all tests in the file PASS.

- [ ] **Step 5: Commit the timing change**

```bash
git add tests/simulation/test_guarded_chop_state_machine.py src/twin_sim/tasks/guarded_chop.py
git commit -m "feat: speed up guarded chop defaults"
```

### Task 2: Verify simulation behavior and document the result

**Files:**
- Modify: `docs/superpowers/specs/2026-08-01-guarded-chop-speed-design.md`

**Interfaces:**
- Consumes: `GuardedChopConfig` faster defaults and `./scripts/run_guarded_chop.sh`.
- Produces: reproducible automated and visual verification evidence in the design record.

- [ ] **Step 1: Run guarded-chop regression tests**

Run:

```bash
.venv/bin/pytest tests/simulation/test_guarded_chop_state_machine.py tests/simulation/test_guarded_chop_integration.py tests/simulation/test_guarded_chop_startup.py -v
```

Expected: all selected tests PASS, including plane/object execution and startup ordering.

- [ ] **Step 2: Run the complete test suite**

Run:

```bash
.venv/bin/pytest
```

Expected: no failures and no new warnings relative to the existing contact-force observation warning.

- [ ] **Step 3: Launch the Viewer for human acceptance**

Run:

```bash
./scripts/run_guarded_chop.sh
```

Expected: the first frame is the ready cutting scene; the five-cut sequence visibly follows cut down, left-hand retreat, and diagonal knife lift/shift; the Viewer exits after an approximately 3-second final hold.

- [ ] **Step 4: Record exact verification results**

Append a dated `## 实现进展` section to the design document containing the focused-test count, full-suite count, headless result and the pending/completed human Viewer verdict. Do not record a visual pass until the user confirms it.

- [ ] **Step 5: Commit verification documentation**

```bash
git add docs/superpowers/specs/2026-08-01-guarded-chop-speed-design.md
git commit -m "docs: record guarded chop speed verification"
```
