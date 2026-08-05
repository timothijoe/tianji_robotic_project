# Official Left/Right Hand Viewer Design

## Goal

Visually compare the existing original left-hand trajectory and the generated
numeric-mirror right-hand trajectory using their respective official Wuji MJCF
models. The workflow is MuJoCo-only and must not import, connect to, or command
ROS 2, SDK devices, or hardware.

## Playback

The launcher prepares two temporary 20-joint NPZ inputs on the shared 5 ms time
axis: `left_joint_positions_rad` from the source trajectory and
`right_joint_positions_rad` from the mirrored trajectory. It opens the official
left model Viewer first, then the official right model Viewer after the first
window closes. Each animation plays once at 200 Hz and holds its final frame for
60 seconds, unless the user closes its Viewer sooner.

## Validation and Scope

Before opening either Viewer, validate source/mirror sample counts, identical
time vectors, finite `(N,20)` arrays, 5 ms spacing, and the corresponding
official MJCF actuator ranges. The Viewer result is visual evidence about the
MJCF convention only; it is not evidence that SDK indexing, offsets, command
signs, firmware behavior, or a physical hand are safe to use.
