# Depth-Aligned Subtle Guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Align the knife and guarding fingertips in robot-depth while restoring recording-scale PIP/DIP motion and selecting the safest feasible close lateral spacing without deforming the gesture.

**Architecture:** Hand shaping is completed once, independently of knife clearance. Preflight then moves the right knife path to track the long-finger contact depth in world `X` and searches lateral spacing in two immutable clearance tiers: 20 mm first, then 10 mm. The selected synchronized plan reports depth, clearance, spacing, amplitude, and contact evidence.

**Tech Stack:** Python 3, NumPy, MuJoCo, Tianji left/right IK, Wuji MCAP loader, pytest.

## Global Constraints

- Preserve the recorded timing and finger coordination; clearance search must not reshape hand joints.
- Blade-reference and mean long-finger-pad world `X` differ by at most `0.010 m`.
- Prefer complete-geometry clearance `0.020 m`; use `0.010 m` only when no 20 mm candidate exists.
- Knife/hand intersection and planned hand/table penetration are forbidden.
- Scale current PIP peak-to-peak motion to approximately 80 percent.
- Bound every long-finger DIP peak-to-peak motion to `0.25-0.50 rad`.
- RETREAT/HOLD distal axes remain `25-50 degrees` from world `-Z` for palmar-pad-biased contact.

---

### Task 1: Restore recording-scale palmar-pad motion

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Test: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`

**Interfaces:**
- Consumes: shaped 20-joint MCAP trajectory, phase labels, palm rotation, actuator ranges.
- Produces: `_shape_palmar_pad_guard(...) -> np.ndarray` with bounded PIP/DIP amplitude and contact angle.

- [ ] Add failing assertions that long-finger PIP peak-to-peak values are 75–85 percent of the current baseline `[0.384813, 0.260536, 0.310028, 0.358330]`, every DIP range is `0.25-0.50 rad`, and active distal angles are `25-50 degrees`.
- [ ] Run the real-plan preflight test and verify it fails because current DIP ranges reach `1.323 rad` and distal angles are below 15 degrees.
- [ ] Replace vertical-DIP maximization with an offset-plus-amplitude shaping pass that retains MCAP temporal variation, targets the `25-50 degree` band, and never changes joints during clearance search.
- [ ] Run `tests/simulation/test_recorded_hand_guarded_chop_preflight.py` and verify all tests pass.
- [ ] Commit with `feat: restore recording-scale guard motion`.

### Task 2: Align knife and pad depth

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Test: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`

**Interfaces:**
- Consumes: complete left-arm/hand samples and knife TCP-to-blade-reference transform.
- Produces: synchronized knife targets whose blade-reference `X` tracks mean long-finger-pad `X`, plus `maximum_depth_mismatch_m`.

- [ ] Add a failing real-plan assertion `plan.maximum_depth_mismatch_m <= .010` and remove acceptance of the old `-0.180 m` hand-only longitudinal offset.
- [ ] Verify RED: current measured average depth mismatch is about `0.182 m`.
- [ ] Set the hand anchor back to its reachable recorded placement and compute every knife TCP target `X` from mean pad world `X` minus the blade-reference/TCP offset.
- [ ] Measure depth mismatch on every synchronized sample and reject values above `0.010 m`.
- [ ] Run the focused preflight suite and commit with `feat: align knife with guard depth`.

### Task 3: Search 20 mm then 10 mm clearance without reshaping

**Files:**
- Modify: `src/twin_sim/tasks/recorded_hand_guarded_chop.py`
- Modify: `src/twin_sim/cli.py`
- Test: `tests/simulation/test_recorded_hand_guarded_chop_preflight.py`
- Test: `tests/simulation/test_recorded_hand_guarded_chop.py`
- Test: `tests/simulation/test_cli.py`

**Interfaces:**
- Consumes: fixed hand plan, lateral candidates from `0.030 m` upward, clearance tiers `(0.020, 0.010)`.
- Produces: `selected_clearance_tier_m`, `selected_lateral_spacing_m`, and a synchronized collision-free plan.

- [ ] Add failing tests that prefer 20 mm when feasible, fall back to 10 mm only after exhausting preferred candidates, report selected values, and preserve identical hand arrays across candidate searches.
- [ ] Verify RED on missing selection fields/search helper.
- [ ] Implement deterministic tier-first search; build only right-arm synchronized targets per candidate, measure complete geometry, reject intersections/penetration, and never call the hand shaper inside the search loop.
- [ ] Update CLI output and runtime result fields; run focused tests and the real Headless command.
- [ ] Update `docs/simulation/recorded_hand_guarded_chop.md`, `docs/simulation/current_version_handoff.md`, and `docs/wuji/development_status.md` with actual selected tier and measurements.
- [ ] Run `tests/simulation -q`, launch Viewer with a 60-second final hold, and commit with `feat: align subtle guard with knife`.
