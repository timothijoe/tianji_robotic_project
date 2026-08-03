# Wuji Table-Contact Retreat Gesture Design

Date: 2026-08-03

## Goal

Derive a deterministic tabletop retreat gesture from a Wuji glove MCAP. The
system automatically selects a feasible source pose, places four long
fingertips on a table, curls those fingers while translating the palm backward,
and holds the final pose for inspection. The thumb remains clear of the table.

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
3. Reject joint-limit violations, self-collision, table penetration beyond
   `0.002 m`, or thumb clearance below `0.010 m`.
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
Optimize palm pose and small joint deltas so fingers 2–5 reach `1.5 mm` above
the plane with low height variance. Keep the thumb at least `10 mm` above the
plane. Penalize deviation from the selected recording and joint-limit margins.

### RETREAT

For `2.0 s`, interpolate the palm backward by `30 mm` along the table. At each
sample, solve the four long-finger joints so their distal contact sites remain
near the PLACE world positions. Penalize contact slip, vertical error,
joint-step size and deviation from the previous solution. The resulting
finger curl is therefore coupled to the palm retreat rather than scripted as
an unrelated offset.

### HOLD

Repeat the final command and palm pose for `2.0 s`. HOLD exists for Viewer
inspection and is included in saved outputs.

All phases use smooth endpoint interpolation and the MuJoCo timestep. The
complete corrected trajectory is preflighted before the first dynamic step.

## Coordinate and direction contract

The table normal is world `+Z`. The palm is fitted palm-down. Retreat is the
negative direction of the palm's table-projected forward axis at the PLACE
pose, not a hard-coded world axis. The report stores both the palm-local
definition and resolved world retreat vector.

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
joint-limit violations, thumb/table clearance below `10 mm`, long-finger
penetration beyond `2 mm`, contact-site slip above `3 mm`, contact-site height
error above `2 mm`, or palm retreat outside `±2 mm`. No partial trajectory is played after a failed
preflight. The workflow imports no physical SDK runtime, discovers no device,
and publishes no ROS command.

## Verification

Unit and integration tests must prove:

- deterministic candidate scoring and earliest-index tie breaking;
- explicit frame override still validates feasibility;
- four long-finger targets and thumb clearance are correctly identified;
- PLACE reaches all four target heights;
- RETREAT moves the palm `0.030 ± 0.002 m` in the resolved direction;
- maximum long-finger contact-site slip is at most `0.003 m` and reported;
- thumb clearance remains at least `0.010 m`;
- joint ranges and frame-to-frame steps remain valid;
- output timestamps, phases, palm poses and reports round-trip without pickle;
- the derived scene contains one hand and a table but no arm;
- real-recording Headless execution succeeds;
- existing raw hand-only replay and all twin-arm simulations remain unchanged.

Manual Viewer acceptance checks that the four long fingers visibly touch the
table, the thumb stays raised, the fingers curl while the palm moves backward,
there is no obvious mesh penetration, and HOLD makes the final posture easy to
inspect.

## Deferred work

This gesture is a simulation and correction tool. Physical hand/arm execution,
force-controlled contact, table calibration from sensors, ROS 2 publishing and
closed-loop slip control remain future hardware plans behind the existing
safety interfaces.
