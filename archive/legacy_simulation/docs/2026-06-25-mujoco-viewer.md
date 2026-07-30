# MuJoCo Viewer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a MuJoCo viewer mode to the existing right-arm chopping CLI while preserving headless behavior.

**Architecture:** Keep `RightArmChopper.run()` as the simulation owner and add an optional viewer sync hook that is called after each control step and once at completion. The CLI owns launching `mujoco.viewer.launch_passive(...)` so tests can monkeypatch the launcher without opening a real window.

**Tech Stack:** Python 3.12, MuJoCo Python `mujoco.viewer`, pytest monkeypatch.

## Global Constraints

- Preserve `twin-chop --cycles 3 --headless --log /tmp/twin_right_chop.csv`.
- Do not modify `MarvinCCS/`, `MarvinCCS_mujoco.zip`, or `MarvinCCS/marvin_final_fixed.xml`.
- Automated tests must not open a real GUI window.

---

### Task 1: Viewer Mode For Chopping CLI

**Files:**
- Modify: `src/twin_mujoco/twin_mujoco/chopping.py`
- Modify: `src/twin_mujoco/twin_mujoco/cli.py`
- Modify: `tests/test_chopping.py`
- Modify: `README.md`

**Interfaces:**
- Consumes: `RightArmChopper.run(config: ChoppingConfig | None, log_path: str | Path | None)`
- Produces: `RightArmChopper.run(..., viewer_sync: Callable[[], None] | None = None)`
- Produces: `twin-chop --viewer` launches a passive MuJoCo viewer and syncs it during chopping.

- [ ] **Step 1: Write failing tests**

Add tests that assert `RightArmChopper.run()` invokes a viewer sync callback and `cli.main(["--viewer"])` launches a monkeypatched viewer instead of raising `SystemExit`.

- [ ] **Step 2: Verify RED**

Run: `python3 -m pytest tests/test_chopping.py::test_chopper_syncs_viewer_callback_during_run tests/test_chopping.py::test_cli_viewer_mode_launches_passive_viewer -v`

Expected: FAIL because `viewer_sync` is not accepted and `--viewer` is currently rejected.

- [ ] **Step 3: Implement minimal viewer integration**

Add an optional `viewer_sync` callback to `RightArmChopper.run()`, call it after each control step and after COMPLETE. In `cli.main`, when `--viewer` is set, launch `mujoco.viewer.launch_passive(chopper.runtime.model, chopper.runtime.data)` as a context manager and pass `viewer.sync` to `run()`.

- [ ] **Step 4: Verify GREEN**

Run the two viewer tests, then `python3 -m pytest -v`.

- [ ] **Step 5: Document usage**

Add `twin-chop --cycles 3 --viewer --log /tmp/twin_right_chop.csv` to README.

- [ ] **Step 6: Commit**

Commit with message: `feat: add mujoco viewer mode`.
