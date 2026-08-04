# Wuji Table-Retreat Looping and Viewer Lifecycle Design

## Goal

Make `tianji-robot sim wuji-table-retreat` replay the recorded tabletop retreat
at its recorded speed for a configurable number of repetitions, without an
abrupt cycle boundary or a MuJoCo Viewer process that remains stuck after its
window closes.

## User-visible behavior

- Add `--loops N` to `wuji-table-retreat`; `N` is a positive integer and
  defaults to `3` in both Viewer and Headless modes.
- Each forward gesture follows the corrected trajectory's relative MCAP
  timestamps instead of treating every recorded frame as one MuJoCo step.
- Between forward gestures, interpolate from the final corrected hand and palm
  pose back to the initial corrected pose. The reset uses bounded joint steps,
  continuously interpolated palm position, and the existing table projection,
  so it cannot teleport or cross the table.
- A loop means one forward recorded gesture plus its reset, except the final
  loop, which ends on the recording's final pose without an unnecessary reset.
- Headless mode exits immediately after all loops.
- Viewer mode keeps the final pose visible after all loops. Closing the Viewer
  ends the hold and allows the command to exit normally.

## Architecture

Planning and playback remain separate. `build_looped_table_retreat` receives
the corrected trajectory, planner backend, and loop count, then returns a fully
preflighted playback sequence containing forward gestures and inter-loop reset
segments. `replay_table_retreat` consumes that sequence only. Its timestamp
scheduler advances MuJoCo by cumulative time and sends each sample once. The
reset generator delegates table-safety projection to the existing workflow
utilities.

Viewer lifetime remains owned by the CLI/backend boundary. The backend exposes
whether a Viewer exists and is running, plus a final-pose wait operation. Its
`close()` method must not call MuJoCo's close routine again after the user has
already closed the window. Headless backends treat the final wait as a no-op.

## Timing and continuity

- Forward duration is the last relative timestamp minus the first.
- Simulation steps are scheduled against cumulative time to prevent rounding
  drift, following the established `simulation.replay` implementation.
- Reset duration is deterministic and derived from the largest joint/palm
  displacement under the existing per-step limit; it must contain at least one
  step.
- Consecutive joint commands, including the final-to-reset and reset-to-first
  boundaries, remain at or below `0.12 rad`.
- Every reset pose is checked/projected against the existing `0.5 mm` maximum
  table-penetration tolerance and `10 mm` thumb-clearance requirement.

## Reporting and CLI validation

The correction report records the requested/executed loop count and scheduled
playback duration. `--loops 0`, negative values, and non-integers are rejected
by argument parsing. Existing invocation without the flag therefore performs
three loops.

## Error handling

- If the Viewer is manually closed during playback, replay stops cleanly
  instead of continuing invisibly.
- If it is closed during the final hold, the wait returns and the CLI exits.
- A table-safety or continuity violation in reset generation fails preflight
  before Viewer playback begins.
- Keyboard interruption still reaches cleanup, which performs an idempotent,
  state-aware Viewer close.

## Verification

Automated tests cover default/custom loop parsing, timestamp-derived MuJoCo
step counts, three forward executions, two continuous resets, invalid loop
counts, early Viewer closure, final Viewer hold, idempotent close, and Headless
completion. Real-MCAP acceptance checks the preferred 499-frame recording with
the default three loops and confirms that closing the Viewer removes both the
window and Python process.
