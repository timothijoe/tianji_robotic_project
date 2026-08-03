# Wuji Recorded Palm-Down Table Retreat Design

Date: 2026-08-04

## Goal

Reproduce the user's recorded tabletop retreat gesture on the official Wuji
left-hand MuJoCo model. The palm faces down, the recorded finger coordination
is preserved, the thumb stays clear of the table, and no hand collision geom
crosses the table.

The primary reference is the already-retargeted left-hand recording:

```text
recordings/wuji/august_02/
session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

The original right-glove recording remains a supported regeneration source:

```text
recordings/wuji/august_02/session_20260802_174440_936.mcap
```

Both files remain ignored recording data and are never copied into tracked
source directories.

## Confirmed recording characteristics

The left-hand recording contains 499 frames over 4.150116 seconds. Measured
finger-displacement correlations support the user's description:

- the thumb has the least motion and is not coupled to the long fingers;
- the index finger contains an independent motion component;
- middle and ring finger displacement correlation is approximately `0.924`;
- ring and little finger displacement correlation is approximately `0.828`;
- the strongest retreat motion is concentrated around frames `320–457`.

These are validation evidence, not hard-coded animation rules. The recorded
20-joint samples remain the source of finger motion.

## Input selection

The table-retreat command accepts either input representation:

1. A `/joint_states` MCAP with the canonical 20 left-hand joints is loaded
   directly. This is the preferred deterministic path.
2. A Wuji Studio right-glove skeleton MCAP is mirrored and retargeted with the
   adjacent official Wuji retargeter, preserving the existing offline path.

Input type is selected from MCAP topics, not from filename conventions. A
joint-state file with missing, duplicate, reordered without names, nonfinite,
or noncanonical joints is rejected before simulation.

## Palm-down calibration

The official left-hand asset receives one explicit, versioned palm-side
calibration. Palm-facing direction is not inferred solely from a cross product,
because an anatomical plane has a sign ambiguity and the prior implementation
selected the palm-up solution.

The calibration establishes:

- finger extension projected along table-world `-X`;
- finger-root lateral span along table-world `Y`;
- the known palmar side facing table-world `-Z`;
- the dorsal side facing table-world `+Z`.

The derived scene exposes named palmar and dorsal reference sites for tests and
diagnostics. A preflight assertion requires the palmar reference to be below
the dorsal reference at PLACE. This makes a future accidental 180-degree flip
an automated failure rather than a visual surprise.

## Recorded trajectory processing

The workflow retains all 499 recorded joint samples and their relative timing.
It does not zero long-finger joints and does not synthesize a uniform curl
target.

Processing has three spatial phases:

### PREPARE

Use the opening recorded samples while the palm remains stationary. Fit palm
height so the hand approaches the table without penetration. The recorded
finger changes remain intact.

### RETREAT

Detect the dominant motion interval from smoothed per-frame joint-motion
energy. Resolve deterministic start and end boundaries with minimum-duration
and hysteresis thresholds; the reference recording should resolve near frames
`320–457` without hard-coding those indices.

During this interval, translate the palm `30 mm` backward along the
table-projected negative finger-extension direction using smooth endpoint
interpolation. Joint samples continue to come from the recording.

### HOLD

Keep the palm at the final translated position for the remaining recorded
samples. Preserve the recorded joint tail instead of replacing it with a
constant synthetic pose.

## Minimal correction policy

Corrections are ordered from least to most invasive:

1. apply the fixed palm-down calibration;
2. add a global palm-height offset;
3. project individual frames upward only enough to keep all real MuJoCo hand
   collision geoms above the table;
4. clamp only genuine joint-range overflow;
5. smooth only frame-to-frame jumps above the existing `0.12 rad` limit.

The workflow must not clear long-finger joints, impose common per-finger
offsets, or solve planted fingertip positions throughout retreat. Any joint
correction is reported per finger against the loaded recording.

## Finger behavior and safety contracts

- Index motion remains independently recorded.
- Middle/ring coordination must retain a displacement correlation of at least
  `0.85` when the source correlation is at least `0.85`.
- Little-finger following must retain a ring/little correlation of at least
  `0.75` when the source correlation is at least `0.75`.
- The thumb uses recorded joints and remains at least `10 mm` above the table.
- Maximum real hand/table penetration is `0.5 mm`, including numerical contact
  tolerance.
- Palm retreat is `30 ± 2 mm` by default.
- Maximum joint correction and RMS joint correction are included in the JSON
  report, both overall and per finger.

If these constraints cannot be met without materially changing the recording,
the workflow fails preflight and reports the violated constraint instead of
playing a fabricated gesture.

## Outputs and CLI

Keep `tianji-robot sim wuji-table-retreat SOURCE` as the public command. The
report adds:

- detected motion start/end frame and timestamp;
- input representation (`joint_states` or `right_glove_skeleton`);
- calibrated palm-side check;
- maximum and RMS joint correction overall and per finger;
- source and corrected inter-finger correlations;
- thumb clearance, maximum penetration, retreat distance, and palm-height
  projection.

Corrected NPZ output retains all recorded samples, canonical joint ordering,
timestamps, palm poses, and phase labels. Raw replay remains a separate command.

## Verification

Automated tests cover:

- direct loading of the provided `/joint_states` MCAP;
- raw right-glove fallback through the official retargeter;
- palm reference ordering proving palm-down orientation;
- deterministic motion-interval detection near the measured active interval;
- preservation of all samples and relative timestamps;
- absence of the previous joint-zeroing and common curl target;
- middle/ring and ring/little correlation preservation;
- thumb clearance, joint ranges, joint steps, and real collision penetration;
- report and NPZ round trips;
- raw hand replay and twin-arm regression safety.

Manual Viewer acceptance captures three stages from two useful angles:

1. PREPARE/PLACE: front-oblique view proves the palm, not the back of the hand,
   faces the table;
2. mid-RETREAT: oblique view shows index independence, middle/ring coupling,
   little-finger following, and raised thumb;
3. HOLD: side view proves the whole hand remains above the table.

The new reference recording must be used for these images. The older
`session_20260802_162909_764.mcap` is no longer the visual acceptance baseline
for this gesture.

## Deferred hardware work

This remains an offline kinematic simulation. Physical Wuji hand execution,
force-controlled table contact, external palm tracking, ROS 2 publishing, and
Tianji arm coordination remain behind the existing hardware safety interfaces.
