# Continuous PIP-Led Guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace five visibly resetting recorded-hand loops with one continuous, small robot-left retreat whose long fingers keep MCP joints mostly straight and use PIP joints for the dominant bend.

**Architecture:** Add a pure hand-trajectory shaping function that redistributes long-finger flexion without changing timing or thumb behavior. Build five cycle anchors from one monotonic wrist carrier, replace arm RESET paths with compensated transition paths, and preflight actual long-finger pad world positions across the complete sequence.

**Tech Stack:** Python 3.12, NumPy, MuJoCo, pytest.

## Global Constraints

- Long fingers are fingers 2-5; `joint1=MCP`, `joint2=abduction`, `joint3=PIP`, `joint4=DIP`.
- During active retreat, MCP `<= 0.30 rad` and PIP `>= MCP + 0.25 rad`.
- Thumb joints and timestamps remain unchanged.
- Hand joint step remains `<= 0.12 rad`.
- Wrist moves monotonically along robot-left `+Y`, totaling `0.025-0.040 m` over five cycles.
- Every long-finger pad may move toward the knife by at most `0.0005 m` per sample and ends farther robot-left than it starts.
- There is no arm RESET toward robot-right after cuts 1-4.
- Preserve all table, thumb, knife-clearance, old-task, and simulation-only constraints.

---

### Task 1: PIP-led hand shaping

**Files:**
- Modify: `src/twin_sim/recorded_hand_guard.py`
- Modify: `tests/simulation/test_recorded_hand_guard.py`

**Interfaces:**
- Produces: `shape_pip_led_guard_hand(positions_rad: np.ndarray, phases: tuple[str, ...], control_ranges: np.ndarray) -> np.ndarray`.

- [x] **Step 1: Write failing anatomical redistribution tests**

Load the real corrected cycle and model actuator ranges. Assert output shape and
timestamps are unchanged; all thumb columns `0:4`, abduction columns
`5,9,13,17`, and their timing remain equal to input. For active `RETREAT`
samples assert long-finger MCP columns `4,8,12,16 <= 0.30`, PIP columns
`6,10,14,18 >= MCP + 0.25`, all values stay in actuator ranges, and maximum
joint step is `<= 0.12 rad`. Assert middle/ring PIP timing correlation remains
positive and index is not forced equal to them.

- [x] **Step 2: Run RED**

```bash
.venv-wuji-teleop/bin/pytest -q tests/simulation/test_recorded_hand_guard.py
```

Expected: import failure for `shape_pip_led_guard_hand`.

- [x] **Step 3: Implement pure redistribution**

For fingers 2-5, clamp MCP to `0.30 rad`; transfer the positive removed MCP
flexion into PIP, enforcing `PIP >= MCP + 0.25`, then clip to the PIP actuator
range. Preserve joint2, use the recorded joint4 unchanged, and apply a bounded
forward/backward smoothing pass only if required to enforce `0.12 rad` steps.
Raise `ValueError` if the contract cannot be met without changing protected
columns.

- [x] **Step 4: Run GREEN**

Run the adapter tests and verify real MCAP invariants.

---

### Task 2: Continuous compensated five-cycle carrier

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Modify: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`
- Modify: `tests/simulation/test_recorded_hand_guarded_chop.py`
- Modify: `docs/simulation/recorded_hand_guarded_chop.md`
- Modify: `docs/wuji/development_status.md`

**Interfaces:**
- Adds: `RecordedHandGuardedChopConfig.total_wrist_retreat_m = 0.032`.
- Produces: `RecordedHandGuardedChopPlan.left_transitions` instead of `left_resets`.
- Adds metrics: `minimum_pad_step_y_m` and `pad_net_retreats_m` to the plan/result.

- [x] **Step 1: Write failing monotonic-carrier tests**

On the real plan assert five cycle-start palm `Y` values are nondecreasing,
the first-to-final palm `Y` displacement is `0.025-0.040 m`, four transitions
exist, and every transition's final palm target equals the next cycle's first
target without a robot-right jump. Assert there is no `RESET` event or phase.

- [x] **Step 2: Write failing physical pad-motion tests**

Evaluate `left_finger2_pad` through `left_finger5_pad` for every cycle and
transition sample. Assert minimum consecutive `delta Y >= -0.0005 m`, all four
net displacements are positive, and the existing knife/table/thumb metrics
still pass.

- [x] **Step 3: Run RED**

Expected: old five independent anchors and RESET paths move robot-right between
cycles and expose `left_resets` instead of `left_transitions`.

- [x] **Step 4: Implement monotonic wrist carrier and compensated transitions**

Scale the recorded relative palm translation so each cycle contributes one
fifth of `total_wrist_retreat_m`. The first cycle starts at the existing safe
robot-left offset; each following cycle starts at the preceding cycle's final
palm target. Generate at least 0.5-second minimum-jerk transitions while the
hand moves from its prior end pose toward the next shaped start pose. At each
transition sample, increase wrist `+Y` only as much as necessary to keep the
minimum long-finger-pad `delta Y >= -0.0005 m`; solve sequential left-arm IK.
Reject plans outside the configured total retreat range or existing safety
limits.

- [x] **Step 5: Update runtime phases**

Execute `HAND_MOTION -> HAND_SAFE -> CUT_DOWN -> KNIFE_RETRACT/SHIFT ->
CONTINUOUS_TRANSITION`. Remove runtime `RESET`; keep completed cycle/cut counts
and the knife interlock unchanged.

- [x] **Step 6: Run focused and old-task regression**

```bash
.venv-wuji-teleop/bin/pytest -q \
  tests/simulation/test_recorded_hand_guard.py \
  tests/simulation/test_recorded_hand_guarded_chop_preflight.py \
  tests/simulation/test_recorded_hand_guarded_chop.py \
  tests/simulation/test_guarded_chop_preflight.py \
  tests/simulation/test_guarded_chop_state_machine.py
```

- [x] **Step 7: Document and verify Headless/Viewer**

Record MCP/PIP limits, total wrist retreat, finger-pad monotonicity, and safety
metrics. Viewer acceptance must show no visible arm return between rounds and
PIP-led rather than MCP-led finger bending.

- [x] **Step 8: Commit**

```bash
git add src/twin_sim/recorded_hand_guard.py \
  src/twin_sim/tasks/recorded_hand_guarded_chop.py \
  tests/simulation/test_recorded_hand_guard.py \
  tests/simulation/test_recorded_hand_guarded_chop_preflight.py \
  tests/simulation/test_recorded_hand_guarded_chop.py \
  docs/simulation/recorded_hand_guarded_chop.md docs/wuji/development_status.md
git commit -m "fix: make recorded guard continuous and PIP-led"
```
