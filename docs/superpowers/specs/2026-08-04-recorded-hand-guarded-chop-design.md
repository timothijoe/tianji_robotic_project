# Recorded-Hand Guarded Chop Design

## Goal

Add a new hardware-free MuJoCo task in which the Wuji left hand remains mounted
on the Tianji left arm, performs the recorded palm-down tabletop retreat once
per cut, and the Tianji right arm completes the existing five guarded cuts.
The stable `guarded-chop` task and its launch scripts remain unchanged.

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
free or mocap-driven hand. The right arm retains its knife and the existing
five-cut trajectory. Both sides interact with the same raised work surface.

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
control rate) with the preceding solution as its seed. The reset between cuts
lasts at least 0.5 seconds and respects the existing 0.12 rad hand-joint step
limit and arm velocity limits.

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
continue from robot-right toward robot-left along `+Y`. For every cut, place
the recorded hand anchor `0.160 m` farther along `+Y` than that cut point; the
recorded 30 mm retreat then moves still farther left. The palm reference must
remain on the robot-left side of the active blade throughout each cycle, while
the existing full-geometry knife/hand clearance remains at least `0.020 m`.
The palm anchor is also `0.080 m` closer to the robot along world `-X` because
the rotated wrist pose is not stably reachable at the knife's full depth.

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

Between cuts, fingers may extend toward the knife to recover joint travel while
the wrist continues moving left far enough to compensate. Across recorded
motion and transition samples, the world `Y` coordinate of every long-finger
pad must not move toward the knife by more than `0.5 mm` between consecutive
samples. The first-to-last long-finger pad displacement must be toward
robot-left. There is no arm RESET phase after cuts 1-4; transitions are part of
the continuous guard motion and retain the table, thumb, and knife-clearance
safety checks.

## Five-cut synchronization

Each of the five cycles is:

1. `GUARD_READY`: left arm places the palm-down hand at initial table contact;
2. recorded `PREPARE/RETREAT/HOLD`: hand joints follow MCAP and the left wrist
   follows the corrected palm path;
3. the right knife may descend only after the left guard has reached the
   cycle's planned safe retreat state;
4. right knife retracts and shifts to the next cut point;
5. `RESET`: left arm and hand return smoothly to the next initial pose.

The final cycle omits RESET and holds both arms at their final safe poses.
Right-arm cutting continues to use the existing five cut locations and
right-to-left order.

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

- Knife-to-Wuji-hand distance remains at least `0.020 m` whenever the knife is
  below its safe height.
- Whole-hand work-surface penetration remains at most `0.0005 m`.
- Thumb clearance remains at least `0.010 m`.
- The right knife cannot enter `CUT_DOWN` until the recorded hand has reached
  the planned retreat guard state.
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
selection, left/right IK preflight, timestamp alignment, reset continuity,
knife interlock, hand/table safety, CLI routing, script syntax, and repository
boundaries. Real acceptance uses the preferred 499-frame recording in Headless
and Viewer modes, verifies five completed cuts and five hand cycles, and then
runs the complete regression suite with no new warning category.
