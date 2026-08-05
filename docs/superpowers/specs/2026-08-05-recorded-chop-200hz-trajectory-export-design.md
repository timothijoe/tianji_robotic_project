# Recorded Chop 200 Hz Trajectory Export Design

## Goal

Export the verified offline `recorded-hand-guarded-chop` motion as a deterministic
200 Hz reference trajectory: timestamps, both Tianji arms (7 joints each), and the
left Wuji hand (20 joints). This is an offline artifact for later inspection and
adapter development; it must not import, connect to, or command ROS 2 or hardware.

## Data Contract

The canonical output is a versioned NPZ containing float64 arrays:

- `time_s`: `(N,)`, beginning at zero, spaced exactly `0.005` s;
- `right_arm_target_rad`: `(N, 7)`;
- `left_arm_target_rad`: `(N, 7)`;
- `left_hand_target_rad`: `(N, 20)`;
- scalar metadata for format version, source control period, source MCAP path,
  selected safety tier, planned minimum distance and maximum table penetration.

A companion CSV contains the same 35 numeric columns with explicit joint names and
radians in its header. NPZ is authoritative; CSV is only an inspection/import aid.

## Generation and Validation

The existing task remains controlled at 10 ms with a 2 ms MuJoCo physics step.
Export uses piecewise-linear interpolation between validated 10 ms target samples to
produce 5 ms targets; it does not alter the simulation controller or physics timestep.
Every exported sample must be finite, inside the corresponding actuator range, and
checked in the existing scene for table penetration and blade-to-hand clearance. Arm
steps between adjacent 5 ms samples may not exceed `0.12 rad`; the selected clearance
tier remains `0.020 m` unless the established `0.010 m` fallback is required.

## Interface and Scope

Add a `--export-trajectory` path option to `recorded-hand-guarded-chop`; on a
successful preflight it writes `<path>.npz` and `<path>.csv` before optional Viewer
execution. A dedicated offline loader validates the format and provides the recorded
arrays for tests. Existing commands, simulation behavior, ROS 2 files and hardware
directories remain unchanged.

## Acceptance

- Export from the real local MCAP creates 200 Hz timestamps, both 7-axis arrays and
  the 20-axis hand array with equal lengths.
- The NPZ and CSV represent identical numeric samples.
- Rechecking every exported sample reports five cuts/five hand cycles, a valid
  clearance tier, no planned table penetration, and no out-of-range target.
- Existing recorded-hand task tests continue to pass.
