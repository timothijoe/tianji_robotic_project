# Wuji Table-Contact Retreat Gesture Design

Date: 2026-08-03

## Goal

Derive a deterministic tabletop retreat gesture from a Wuji glove MCAP. The
system automatically selects a feasible source pose, places four long
fingertips on a table, then curls those fingers while translating the palm
backward. The fingertips may lift gradually during retreat, but every hand
collision geometry remains above the table. The thumb remains clear of the
table.

The original MCAP and original retargeted trajectory are immutable inputs. Raw
playback remains available through `tianji-robot sim wuji-replay`.

## Public interface

Add a separate command so corrected and raw behavior cannot be confused:

```bash
tianji-robot sim wuji-table-retreat SOURCE [OPTIONS]
```

Defaults:

- retreat distance: `0.03 m`;
- placement duration: `1.0 s`;
- retreat duration: `2.0 s`;
- final hold: `2.0 s`;
- fingertip clearance target above table: `0.0015 m`;
- minimum thumb clearance: `0.010 m`;
- table height: `0.0 m`;
- Viewer enabled unless `--headless` is supplied.

Options may override source frame/time, retreat distance, table height and
phase durations. Optional output paths save the corrected trajectory and a
JSON correction report. There is no hardware domain command for this gesture.

## Scene architecture

Keep the vendored official hand-only MJCF unchanged. At runtime, construct a
self-owned derived MuJoCo scene from it:

- mark the official `palm_link` as a mocap body so the palm pose is controlled
  without adding generalized coordinates;
- retain exactly the official 20 finger joints and actuators;
- add one finite-looking collision plane at the configured table height;
- add named contact sites at the distal tips of fingers 2 through 5;
- add a thumb-clearance site at finger 1;
- add no Tianji/Marvin arm body, joint or actuator.

`TabletopWujiHand` is a new backend. The existing `MujocoWujiHand` raw replay
backend stays unchanged. Both share canonical hand joint ordering, but the
tabletop backend additionally commands and reads a palm position/quaternion,
reports fingertip positions and exposes contact/penetration diagnostics.

## Automatic source-frame selection

Evaluate the retargeted recording at a deterministic sampling stride, always
including the first and last frames. For each candidate:

1. Set the 20 joint positions kinematically and run `mj_forward`.
2. Fit a table-facing palm orientation and vertical translation.
3. Reject joint-limit violations, self-collision, table penetration beyond the
   numerical tolerance of `0.0005 m`, or thumb clearance below `0.010 m`.
4. Score the remaining candidate using:
   - variance of four long-fingertip heights;
   - distance to simultaneous table contact;
   - required palm rotation/translation;
   - required joint correction from the recorded frame;
   - margin from joint limits.
5. Select the lowest score, breaking ties by earliest frame index.

An explicitly supplied source frame bypasses automatic selection but still
passes all feasibility checks. If no candidate is feasible, fail before Viewer
launch and report the best rejected candidate with its violated constraints.

## Contact correction and trajectory generation

Use a NumPy/MuJoCo-Jacobian projected damped-least-squares solver with fixed
iteration limits and tolerances; add no optimization runtime dependency. Do
not rely on uncontrolled contact dynamics to create the trajectory.

### PLACE

Interpolate for `1.0 s` from the selected recorded pose to a corrected pose.
Build the palm frame from the palm root, the long-finger extension direction,
and the finger-root lateral direction. Rotate this anatomical frame so the
palm plane is parallel to the table. Optimize palm height and small joint
deltas so fingers 2–5 are no more than `3 mm` above the plane with low height
variance. Keep the thumb at least `10 mm` above the plane and verify clearance
using every collision geom, not only named sites. Penalize deviation from the
selected recording and joint-limit margins.

### RETREAT

For `2.0 s`, interpolate the palm backward by `30 mm` along the table and curl
the four long fingers. Release the PLACE fingertip-position constraint: the
fingertips may slide or lift gradually as the palm withdraws. At each sample,
preserve joint continuity and query real MuJoCo hand/table collision distances.
If any hand geom would cross the plane, raise the palm along world `+Z` by the
minimum correction and recompute that frame. The resulting retreat is coupled
to finger curl without forcing an infeasible planted-fingertip posture.

### HOLD

Repeat the final command and palm pose for `2.0 s`. HOLD exists for Viewer
inspection and is included in saved outputs.

All phases use smooth endpoint interpolation and the MuJoCo timestep. The
complete corrected trajectory is preflighted before the first dynamic step.

## Coordinate and direction contract

The table normal is world `+Z`. The palm anatomical plane remains approximately
parallel to the table. Retreat is the negative direction of the palm's
table-projected forward axis at the PLACE pose, not a hard-coded world axis.
The report stores both the palm-local definition and resolved world retreat
vector.

## Outputs

The corrected NPZ stores:

- timestamps;
- canonical 20-joint positions in radians;
- palm positions in metres;
- palm unit quaternions in MuJoCo `wxyz` order;
- phase per frame (`PLACE`, `RETREAT`, `HOLD`);
- original selected frame index and timestamp;
- table and correction configuration.

The JSON report stores candidate scores, selected frame, optimizer residuals,
minimum thumb clearance, per-finger maximum vertical error and slip, maximum
penetration, actual palm retreat distance and joint-limit margins. Both outputs
are local generated data under ignored `recordings/` paths by default.

## Safety and failure behavior

Before playback, reject nonfinite values, invalid durations/distances, missing
model landmarks, nonconverged optimization, joint steps above `0.12 rad`,
joint-limit violations, thumb/table clearance below `10 mm`, any hand-geometry
penetration beyond the `0.5 mm` numerical tolerance, initial long-fingertip
height above `3 mm`, or palm retreat outside `±2 mm`. Retreat fingertip lift is
reported but is not rejected when it is continuous and the hand remains above
the plane. No partial trajectory is played after a failed preflight. The
workflow imports no physical SDK runtime, discovers no device, and publishes
no ROS command.

## Verification

Unit and integration tests must prove:

- deterministic candidate scoring and earliest-index tie breaking;
- explicit frame override still validates feasibility;
- four long-finger targets and thumb clearance are correctly identified;
- PLACE brings all four long fingertips within `3 mm` of the table;
- RETREAT moves the palm `0.030 ± 0.002 m` in the resolved direction;
- RETREAT allows continuous fingertip lift instead of requiring planted tips;
- thumb clearance remains at least `0.010 m`;
- every real hand/table collision remains within the `0.5 mm` numerical
  penetration tolerance;
- joint ranges and frame-to-frame steps remain valid;
- output timestamps, phases, palm poses and reports round-trip without pickle;
- the derived scene contains one hand and a table but no arm;
- real-recording Headless execution succeeds;
- existing raw hand-only replay and all twin-arm simulations remain unchanged.

Manual Viewer acceptance uses both oblique and side views. It checks that the
four long fingers initially touch the table, the palm plane is approximately
parallel to it, the thumb stays raised, the fingers curl and may lift gradually
while the palm moves backward, no mesh crosses the table, and HOLD makes the
final posture easy to inspect.

## Deferred work

This gesture is a simulation and correction tool. Physical hand/arm execution,
force-controlled contact, table calibration from sensors, ROS 2 publishing and
closed-loop slip control remain future hardware plans behind the existing
safety interfaces.
