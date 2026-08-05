# Recorded-Hand Guarded Chop Design

## Goal

Add a new hardware-free MuJoCo task in which the Wuji left hand remains mounted
on the Tianji left arm, performs one continuous recorded palm-down tabletop
retreat across five cuts, and the Tianji right arm cuts in parallel while
maintaining a fixed robot-lateral offset. The stable `guarded-chop` task and
its launch scripts remain unchanged.

## Public interface

The new command is:

```bash
.venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop \
  --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

Use the existing `.venv-wuji-teleop` environment because it contains both the
arm simulator and offline MCAP dependencies. The preferred 499-frame
`/joint_states` recording is the default when
`--hand-mcap` is omitted. The option also accepts a raw
`/right_glove/hand_skeleton` MCAP through the existing official offline
retargeter. `--headless`, `--final-hold`, and an optional recording/report path
follow the current simulation conventions. This command never imports or
accesses physical Tianji or Wuji runtimes.

## Scene and ownership

Use one combined MuJoCo model, one `MjData`, and one simulation clock. The
existing Tianji left-arm wrist body owns the Wuji palm body; there is no second
free or mocap-driven hand. The right arm retains its knife and a compact
five-cut trajectory synchronized with the recorded guard motion. Both sides
interact with the same raised work surface.

Create the new task beside `twin_sim.tasks.guarded_chop`. Reuse its right-arm
cut planning, knife/hand distance calculation, state-machine vocabulary,
visualization, and recording formats where their contracts remain valid. Do
not add recorded-hand behavior to the old task via conditionals.

## Recorded hand mapping

Load the canonical corrected hand trajectory through `tianji_robotics` data
and workflow boundaries. Preserve all 499 recorded joint samples and relative
timestamps. For each cut, transform the corrected palm-relative 30 mm retreat
into a left-arm end-effector path while commanding the recorded 20 hand joints
on the already-mounted Wuji hand.

The palm-down calibration from the standalone tabletop workflow becomes a
world-frame placement target for the left wrist. The left-arm IK solves every
sample (or a deterministic timestamp-preserving resampling at the combined
control rate) with the preceding solution as its seed. The continuous motion
between cuts respects the existing 0.12 rad hand-joint step limit and arm
velocity limits; there is no inter-cut reset.

### Chopping-frame alignment correction

The recorded retreat must be expressed in the chopping task's lateral frame,
not the camera front-back frame. In the combined scene, the ordered cut points
progress along world `+Y`, while the first implementation maps the recorded
30 mm retreat onto world `-X`. Apply one world-frame `-90 degree` rotation
about the work-surface normal to the complete recorded palm anchor. This
rotates the wrist orientation, finger direction, and every relative palm
sample together; do not rewrite translation components independently.

After correction, the horizontal start-to-end palm displacement must be
parallel to the ordered cut-point direction and point along world `+Y`, within
one degree of angular tolerance. Its magnitude remains 30 mm within the
existing recording tolerance. The five anchor positions still follow their
corresponding cut locations, and the shared-surface search must be rerun rather
than retaining safety measurements from the unrotated plan.

### Robot-relative left side and table-only cutting

Robot-relative left is world `+Y`, as established by the left-arm base at
`Y=+0.04 m` and the right-arm base at `Y=-0.04 m`. The ordered knife cuts
continue from robot-right toward robot-left along `+Y`. Replace the recording's
full 30 mm per-cycle palm translation with one compensated 32 mm carrier over
all five cuts. The knife follows the knife-side nearest long-finger pad in
world `+Y`; preflight selects their fixed lateral spacing through the tiered
clearance search below. In world `X`, move the knife path to the recorded hand
depth so blade and pad contact positions differ by at most `0.010 m`.

This task cuts the shared chopping-board surface directly. It must not raise
the knife contact target to the guarded cube top. In this task's private
MuJoCo model instance, hide and disable collision for the guarded cube, both
pick-place pedestals, the pick cube, and pick target markers. Do not edit the
base MJCF or change `guarded-chop`, `pick-place`, or other task behavior.
Board contact targets and the recorded hand contact plane still move together
under the selected raised-surface offset.

### Continuous anatomical hand-over-hand motion

Treat the five cuts as one continuous guard trajectory rather than five
independent recording loops. The left wrist must never execute a visible
robot-right reset between cuts. It moves monotonically toward robot-left
(`+Y`) by a small total distance of `0.025-0.040 m` across all five cycles.
Per-cycle wrist displacement is therefore approximately `0.005-0.008 m`, not
the recording's full 30 mm palm translation.

For long fingers 2-5, reduce MCP flexion (`joint1`) toward extension and move
the dominant flexion contribution into PIP (`joint3`). Preserve abduction
(`joint2`) and use DIP (`joint4`) only as a smaller natural follower. The index
finger may move independently; middle and ring remain synchronized within the
recording's relative timing; little finger follows; thumb keeps its recorded
non-contact behavior. Every adjusted value remains inside the MuJoCo actuator
range, and the adjusted trajectory must not exceed the existing `0.12 rad`
per-sample hand-joint step limit. During active retreat, each long-finger MCP
target is at most `0.30 rad`; its PIP target is at least `0.25 rad` more flexed
than that MCP target. These thresholds make "mostly straight MCP, PIP-led
curl" testable rather than visual-only.

The reshaped recording plays once and is partitioned into five contiguous time
segments, so there is no finger or wrist reset at a cut boundary. Across all
recorded samples, the world `Y` coordinate of every long-finger
pad must not move toward the knife by more than `0.5 mm` between consecutive
samples. The first-to-last long-finger pad displacement must be toward
robot-left. There is no arm RESET phase after cuts 1-4; the continuous guard
motion retains the table, thumb, and knife-clearance safety checks.

### Distal palmar-pad contact and visible gesture amplitude

For long fingers 2-5, define the distal-phalanx direction as each
`left_fingerN_link4` body's local `+Z` axis, which points from the DIP body
toward its finger pad in the Wuji MJCF. During `RETREAT` and `HOLD`, retain a
`25-50 degree` angle from world `-Z` so the fingertip surface biased toward the
palmar pad, rather than the distal tip alone, meets the table. The corresponding
pad remains at the work surface with zero planned penetration. Thumb behavior
remains non-contacting.

Do not satisfy the orientation constraint with a single static pose. Preserve
the corrected recording's temporal shape and redistribute its visible motion
into PIP/DIP while retaining the `0.30 rad` MCP limit. Scale current PIP
peak-to-peak motion to approximately 80 percent and bound every long-finger DIP
peak-to-peak motion to `0.25-0.50 rad`. Index remains independently timed,
middle/ring remain positively correlated without becoming identical, and
little finger follows at reduced amplitude. PREPARE may open the fingers;
RETREAT progressively establishes palmar-pad contact; HOLD maintains pressure
with small recorded variations rather than freezing.

Solve PIP/DIP targets against MuJoCo link orientation and pad height for every
sample, then rerun the continuous wrist compensation because altered finger
geometry changes pad motion and knife clearance. Reject the plan if contact
orientation, amplitude, actuator range, joint-step, table, thumb, or selected
clearance-tier constraints cannot all be met.

### Parallel knife/guard carrier and searched lateral spacing

Replace the serial `HAND_MOTION -> CUT_DOWN` execution with one synchronized
control timeline. Every control sample commands right-arm knife joints,
left-arm wrist joints, and Wuji hand joints together. Knife descent/retract is
the vertical component; the knife's robot-left (`+Y`) target follows the
knife-side nearest long-finger pad so their lateral offset remains fixed.
Neither side waits for the other to complete a phase.

Replace the old independent 20 mm cut spacing with pad-following compact cut
locations. The wrist retains its existing 32 mm total retreat, while knife
travel follows the nearest pad's smaller net retreat because finger flexion
changes pad position relative to the palm. Partition the continuous hand
recording into five segments as before, but time-stretch each segment onto its
corresponding knife down/up interval.

Define relative spacing in robot coordinates, not as full 3-D Euclidean
distance: preflight searches the `+Y` separation between the knife-side nearest
long-finger pad and blade reference, then holds the selected separation within
`+/- 0.003 m` throughout synchronized cutting. Vertical knife travel
necessarily changes Euclidean distance. Preflight first requires `0.020 m`
full-geometry clearance and may use the explicit `0.010 m` fallback tier only
when no 20 mm candidate exists.

The former event-order interlock (hand segment completely safe before knife
descent) is replaced by a per-sample joint safety gate: a synchronized sample
may execute only when lateral spacing, 3-D clearance, table penetration,
thumb clearance, joint limits, and finite-state checks all pass. Any violation
aborts before advancing to the next sample or cut.

### Robot-depth alignment and recording-scale contact posture

The knife and guarding fingertips must operate at the same robot depth. Define
depth as world `X`; during synchronized cutting, the blade reference and the
mean long-finger pad contact position differ by at most `0.010 m` in `X`.
Achieve this by moving the right-arm knife path toward the robot, not by moving
the left hand away from its reachable recorded placement. Remove the previous
`-0.180 m` hand-only longitudinal separation.

Do not deform the hand gesture to manufacture knife clearance. Preserve the
recording-derived finger timing and reduce the current exaggerated shaping:
PIP peak-to-peak amplitude is approximately 80 percent of the current shaped
trajectory, while every long-finger DIP remains visibly active with
peak-to-peak amplitude between `0.25 rad` and `0.50 rad`. Use an offset plus
bounded temporal variation, rather than forcing the distal phalanx almost
vertical. The contact target is the fingertip surface biased toward the palmar
pad, with no hand/table penetration; the thumb remains non-contacting.

Clearance search follows a strict priority order:

1. preserve the recording-scale gesture and palmar-pad contact posture;
2. preserve blade/pad world-`X` alignment within `0.010 m`;
3. search for the smallest fixed robot-lateral spacing that provides at least
   `0.020 m` complete-geometry clearance;
4. if and only if no candidate satisfies `0.020 m`, repeat the search with a
   `0.010 m` clearance floor;
5. reject every candidate with knife/hand intersection or table penetration,
   even when using the fallback floor.

The selected clearance tier, fixed lateral spacing, maximum depth mismatch,
PIP/DIP amplitudes, and pad-contact measurements are reported by preflight and
covered by regression tests. Clearance fallback may change knife/hand spacing;
it must never trigger additional finger shaping, wrist rotation, or depth
misalignment.

## Five-cut synchronization

After `GUARD_READY` places the palm-down hand at initial table contact, each of
the five cycles runs one synchronized down/up interval. The hand joints follow
the corresponding contiguous MCAP segment while the knife follows the nearest
long-finger pad in `+Y`. The right arm superimposes its vertical cut/retract
waveform on that pad-following target. Both arms therefore advance
continuously from robot-right to robot-left without an inter-cut RESET, shift,
or wait state. The final sample holds both arms at their final safe poses.

## Shared raised work surface

The preflight evaluates deterministic work-surface height candidates from
`0.04 m` through `0.12 m` above the current height. It selects the lowest candidate for which all five
right-arm cut trajectories and every left-arm recorded/reset pose are IK
reachable and collision-safe. The chopping board, guarded object, cut contact
targets, and hand contact plane move together by the selected offset; the two
arms therefore interact with one coherent surface rather than separate planes.

The selected offset is reported. If no candidate is feasible, preflight fails
before opening a Viewer and reports the first limiting constraint (left IK,
right IK, hand/table contact, or knife/hand clearance).

## Safety invariants

- Knife-to-Wuji-hand distance remains at least the selected `0.020 m` preferred
  or `0.010 m` fallback tier whenever the knife is below its safe height.
- Whole-hand work-surface penetration remains at most `0.0005 m`.
- Thumb clearance remains at least `0.010 m`.
- Every synchronized control sample satisfies the lateral-spacing and complete
  geometry-clearance invariants before either arm advances.
- IK, joint range, velocity, collision, and timing checks cover the complete
  five-cycle plan before the first actuator command.
- Any runtime divergence changes the task to `ABORTED`, retracts the right
  knife when safe to do so, and never advances to the next cut.

## Outputs and visualization

The result reports completed cuts, completed recorded-hand cycles, selected
surface offset, minimum knife/hand distance, maximum hand penetration, minimum
thumb clearance, and the abort reason. Viewer overlays distinguish recorded
hand phases, right-cut phase, cycle/cut number, and live safety values. A new
single-purpose shell script launches the task; existing guarded-chop scripts
are untouched.

## Verification

Automated tests cover MCAP loading, five-cycle mapping, lowest-feasible-height
selection, left/right IK preflight, timestamp alignment, no-reset continuity,
synchronized per-sample commands, lateral-spacing invariance, complete-geometry
clearance, hand/table safety, CLI routing, script syntax, and repository
boundaries. Real acceptance uses the preferred 499-frame recording in Headless
and Viewer modes, verifies five completed cuts and five hand cycles, and then
runs the complete regression suite with no new warning category.
