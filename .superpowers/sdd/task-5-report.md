# Task 5 Report: CLI, Documentation, and Final Verification

## Delivered

- Added `twin-sim guarded-chop --scene {plane,object}` with default
  `plane`.
- Passed the selected scene to
  `GuardedChopConfig(scene_mode=args.scene, final_hold_s=args.final_hold)`.
- Preserved the existing result summary fields: success, cuts, shifts, total
  shift, minimum distance, and reason.
- Documented the default plane demonstration and the retained, non-default
  object contact-calibration scene.
- Documented the right-to-left cutting direction, blue planned knife path,
  cyan actual knife trail, purple actual guard trail, five compact cut marks,
  and explicit `HAND_OPEN` → `HAND_SHIFT` → `HAND_CLOSE` sequence.
- Did not modify real-robot, ROS, or SDK files.

## TDD evidence

The parser tests were added before production changes. The required RED run
failed for the intended missing feature:

```text
.venv/bin/pytest tests/simulation/test_cli.py -q
FAILED test_guarded_chop_cli_defaults_to_plane_scene
  AttributeError: 'Namespace' object has no attribute 'scene'
FAILED test_guarded_chop_cli_accepts_object_scene
  twin-sim: error: unrecognized arguments: --scene object
2 failed, 8 passed
```

The handler test was also extended to invoke `--scene object` and assert that
the resulting `GuardedChopConfig.scene_mode` is `object`. After the minimal CLI
implementation:

```text
.venv/bin/pytest tests/simulation/test_cli.py -q
10 passed in 1.87s
```

## Verification

```text
.venv/bin/pytest tests/simulation/test_guarded_chop_*.py \
  tests/simulation/test_cli.py -q
51 passed in 93.19s

.venv/bin/pytest -q
165 passed, 1 warning in 132.36s
```

The only warning was the pre-existing chop force-monitor warning:

```text
RuntimeWarning: contact force 39.611 N exceeds warning threshold 30.000 N
```

Default plane headless acceptance:

```text
.venv/bin/twin-sim guarded-chop --headless --final-hold 0
success=True cuts=5 shifts=4 total_shift_m=0.080 min_distance_m=0.044 reason=-
```

Additional checks:

```text
sha256sum -c docs/simulation/protected-files.sha256
all protected files: OK; exit 0

git diff --check
no output; exit 0

.venv/bin/twin-sim guarded-chop --help
shows --scene {plane,object}; exit 0
```

## Viewer acceptance facts

`DISPLAY=:0` was available, so
`.venv/bin/twin-sim guarded-chop --final-hold 10` was launched. The Viewer
process started without a launch error and continued running without output
for approximately 120 seconds, but it did not automatically finish within the
available acceptance window. It was stopped with Ctrl-C and exited 130 from
`guarded_chop_visualization.py` while updating the overlay.

No desktop pixels were visible through this agent interface. Therefore this
run does **not** claim visual acceptance of the right-to-left cuts, hand
cycles, trails, or cut marks, and it does not confirm automatic success exit.
Those visual items remain for a human-visible Viewer check.

## Commit

The functional commit contains only:

- `src/twin_sim/cli.py`
- `tests/simulation/test_cli.py`
- `docs/simulation/usage.md`
- `docs/simulation/guarded_chopping_development_log.md`

Commit: `3d79089` (`docs: expose plane guarded chopping demo`). This report
and the existing uncommitted Task 4 report are intentionally excluded from
that commit.

## Concerns at the initial Task 5 handoff

At the initial handoff, automated, headless, hash, and diff verification had
passed, while Viewer performance and human-visible inspection were still open.
The following review-repair section supersedes the automatic-exit concern:
the repaired Viewer completed successfully in 87.11 seconds, and a later
parent-run demonstration completed in approximately 62 seconds. The user then
requested branch integration after the visible demonstration.

## Review repair: bounded Viewer work and automatic exit

The first Viewer acceptance result above was investigated as a performance
failure rather than accepted with a larger timeout. A temporary wrapper timed
the real Viewer methods without changing production code. After 85.852 seconds
the task had reached only 2,410 samples:

```text
draw_calls=243 overlay_calls=242 max_segments_per_draw=9 final_ngeom=433
draw ngeom 0-249:   mean 1.475 ms, max 5.859 ms
draw ngeom 250-499: mean 1.421 ms, max 1.979 ms
overlay ngeom 0-249:   mean 100.825 ms, max 156.836 ms
overlay ngeom 250-499: mean 166.050 ms, max 228.296 ms
```

The initial plan was the only nine-segment draw; each runtime draw contained
at most two new segments. Capsule construction was therefore already
incremental and constant-time. The expensive operation was repeated
`set_texts`, whose Viewer-lock cost increased as the accumulated user scene
grew. A real Viewer run with a no-op trace completed successfully in 86.238
seconds, excluding a state-machine deadlock and showing that the unbounded
trace consumed the remaining runtime budget.

### TDD repair evidence

The first RED regression appended 100 samples with `max_points=4` and expected
the five cut marks, four planned-path segments, and at most six actual trail
segments. Before the fix the fake scene filled all 64 slots. It also expected
one `set_texts` call for 100 identical status strings; the old implementation
submitted all 100.

The fix uses a fixed-size ring of Viewer geometry slots, reusing old actual
knife/guard trail capsules instead of increasing `scene.ngeom`. The default
trace stores 16 points, so the nine permanent plan/cut-mark geometries plus 30
rolling actual segments cap the scene at 39 geometries. Identical formatted
overlay strings are not resubmitted.

The second RED regression verified that a Viewer physics step could defer
`sync()` without changing its pacing. `RightArmRobot.step()` now has an
optional `sync_viewer` keyword whose default remains `True`. Guarded chopping
uses it to retain 100 Hz physics and safety observation while limiting Viewer
state synchronization to the display rate. A pure regression verifies strides
of 3, 2, and 1 for control periods of 0.01, 0.02, and 0.05 seconds.

```text
tests/simulation/test_guarded_chop_visualization.py
7 passed

tests/simulation/test_robot.py + visualization + state-machine
34 passed

tests/simulation/test_guarded_chop_integration.py
4 passed in 57.72s
```

### Fresh final verification

```text
.venv/bin/pytest -q
169 passed, 1 pre-existing contact-force warning in 131.45s

.venv/bin/twin-sim guarded-chop --headless --final-hold 0
success=True cuts=5 shifts=4 total_shift_m=0.080 min_distance_m=0.044 reason=-

sha256sum -c docs/simulation/protected-files.sha256
all protected files: OK; exit 0

git diff --check
no output; exit 0
```

The repaired Viewer command completed automatically below the 90-second
acceptance limit:

```text
.venv/bin/twin-sim guarded-chop --final-hold 10
success=True cuts=5 shifts=4 total_shift_m=0.080 min_distance_m=0.044 reason=-
viewer_wall_s=87.11 viewer_exit=0
```

This agent could not see desktop pixels, so the run verifies automatic success
exit and absence of Viewer errors only. Visual appearance of the right-to-left
cuts, four hand cycles, trails, and cut marks still requires a human-visible
Viewer check.

The parent agent subsequently launched the same Viewer command in the user's
visible session. It completed automatically in approximately 62 seconds with
the same `success=True`, five-cut, four-shift result. The user proceeded to
request local branch integration after that demonstration; no further visual
change request was made.

Review-repair commit: `70d2a4d` (`fix: bound guarded chop viewer work`). This
report and the existing Task 4 report remain excluded from the functional
commit.
