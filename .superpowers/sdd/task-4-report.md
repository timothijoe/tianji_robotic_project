# Task 4 Report: Meaningful Guarded-Chop Trails

## Delivered

- Replaced the per-sample blue/cyan/purple/yellow spheres with capsule
  segments: a static blue planned blade trail and sampled cyan actual-blade
  and purple actual-guard trails.
- Added `GuardedChopTrace.set_plan()` and five compact blue vertical cut marks
  (`1.2 mm` radius, `10 mm` total length).
- Removed the yellow visual guard target and its deque. `run_guarded_chop()`
  now sends the five planned blade contact points once after preflight, while
  `_observe_sample()` appends only actual blade and guard positions.
- Preserved `marker_stride` downsampling, bounded `user_scn.maxgeom` writes,
  cumulative minimum-distance reporting, overlay throttling, and abort text.
- Reused `viewer_trail._position()` and `_init_sphere_geom()` for validation
  and base Viewer geometry initialization, then used MuJoCo's
  `mjv_connector()` for capsule geometry.

## TDD evidence

RED was observed after changing the visualization test first:

```text
.venv/bin/pytest tests/simulation/test_guarded_chop_visualization.py -q
F.F
AttributeError: 'GuardedChopTrace' object has no attribute 'set_plan'
TypeError: GuardedChopTrace.append() missing ... 'planned_knife' and 'guard_target'
2 failed, 1 passed in 0.20s
```

GREEN after the minimal capsule/trail implementation:

```text
.venv/bin/pytest tests/simulation/test_guarded_chop_visualization.py -q
3 passed in 0.16s
```

## Verification

```text
.venv/bin/pytest tests/simulation/test_guarded_chop_visualization.py \
  tests/simulation/test_guarded_chop_integration.py -q -s
7 passed in 58.13s

GuardedChopTrace-injected headless run_guarded_chop smoke check:
trace integration: 5 3000 3000

Real mujoco.MjvGeom capsule smoke check:
real MjvGeom capsule initialized: [0.0012..., 0.0012..., 0.004999...]

.venv/bin/python -m compileall -q <modified Python files>
exit 0

git diff --check
exit 0
```

## Commit

- `6e4a61d feat: draw meaningful guarded chop trails`

## Self-review and concerns

- Confirmed there is no `guard_target_color`, yellow marker draw, large sphere
  marker, or planned-target computation in the runtime sample path.
- Confirmed the real MuJoCo connector produces a capsule with the requested
  radius and half-length; the fake-Viewer fallback is covered by unit tests.
- The passive Viewer's custom scene has a finite geometry budget. Rendering
  deliberately stops when `user_scn.maxgeom` is reached, matching the bounded
  behavior requested; very long viewer sessions can therefore show a
  truncated tail rather than overwrite existing custom geometry.
- No action-state-machine, CLI, or physical-robot code was changed.

## Important review fixes

The review reproduced two root causes in commit `6e4a61d`:

```text
Low-budget diagnostic (user_scn.maxgeom=4):
ngeom: 4
half lengths: [0.01, 0.01, 0.01, 0.01]

append signature:
(self, *, actual_knife, actual_guard, ..., planned_knife=None)
```

The first four greedy geometry writes were planned-trail segments, so no cut
mark survived a four-slot budget. The public append signature also explicitly
kept and silently accepted the removed planned-path argument.

### Review-fix TDD evidence

Budget-priority RED, before reordering plan geometry:

```text
.venv/bin/pytest \
  tests/simulation/test_guarded_chop_visualization.py::\
test_plan_uses_every_limited_scene_slot_for_cut_marks_first -q
FAILED: geom.size[0] was 0.0015, expected cut_mark_radius_m 0.0012
1 failed in 0.18s
```

Budget-priority GREEN after drawing the five cut marks before optional plan
segments:

```text
1 passed in 0.16s
```

Removed-argument RED, before deleting the compatibility parameter:

```text
.venv/bin/pytest \
  tests/simulation/test_guarded_chop_visualization.py::\
test_append_rejects_removed_planned_knife_argument -q
Failed: DID NOT RAISE <class 'TypeError'>
1 failed in 0.18s
```

Removed-argument GREEN after deleting `planned_knife` from the public
signature and implementation:

```text
1 passed in 0.15s
```

### Review-fix verification

```text
.venv/bin/pytest tests/simulation/test_guarded_chop_visualization.py -q
5 passed in 0.17s

.venv/bin/pytest tests/simulation/test_guarded_chop_visualization.py \
  tests/simulation/test_guarded_chop_integration.py -q -s
9 passed in 58.12s

inspect.signature(GuardedChopTrace.append)
(self, *, actual_knife: Sequence[float], actual_guard: Sequence[float],
 phase: str, cut_index: int, minimum_distance_m: float,
 cut_allowed: bool) -> None

.venv/bin/python -m compileall -q \
  src/twin_sim/guarded_chop_visualization.py \
  tests/simulation/test_guarded_chop_visualization.py
exit 0

git diff --cached --check
exit 0
```

Review-fix commit:

- `345dfe6 fix: prioritize guarded chop cut marks`
