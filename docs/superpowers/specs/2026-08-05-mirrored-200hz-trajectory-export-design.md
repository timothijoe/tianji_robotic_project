# Mirrored 200 Hz Trajectory Export Design

## Goal

Create a new offline 200 Hz trajectory package from
`recorded_hand_guarded_chop_200hz_latest` by exchanging the two Tianji arm
trajectories and converting the left Wuji hand values to the official right Wuji
hand coordinate convention. The source NPZ and CSV remain unchanged. No ROS 2,
hardware SDK, controller, or physical command is in scope.

## Numeric Transform

For each 5 ms sample, let `R` and `L` be the source right/left arm 7-vectors.
The mirror sign vector is `(-1, +1, -1, +1, -1, +1, -1)`.

- output left arm = sign vector multiplied elementwise by `R`;
- output right arm = sign vector multiplied elementwise by `L`;
- output right hand has the same five-finger, four-joint order as the source left
  hand; all values are unchanged except `right_finger1_joint2`, which is the
  negation of `left_finger1_joint2`.

The hand rule is derived from the official local MJCF files:
`wuji-description/hand/body/mjcf/left.xml` declares the left thumb second-joint
axis `(0,-1,0)`, whereas `right.xml` declares `(0,1,0)`; all other corresponding
joint axes and numeric ranges match.

## Output and Validation

Write a new versioned NPZ and CSV using the existing 200 Hz time vector and
metadata. CSV columns name both arms and `right_finger*_joint*_rad`. Validate
the source format, exact 5 ms timestamps, finite values, unchanged array shapes,
the stated transform identities, and right-hand MJCF control ranges. Mark output
metadata as an offline mirror conversion, not a real-machine command or safety
certificate.
