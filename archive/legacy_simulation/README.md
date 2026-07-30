# Legacy Simulation Archive

This directory is a historical snapshot of the superseded torque- and
impedance-control MuJoCo simulation. It is intentionally outside the active
source tree, package configuration, and pytest test paths.

- Design baseline: commit `a8cde84`.
- Source snapshot before this archive: commit `1d7e044ea9eb9f38e2d2e2b90cd2f4d61e2cb55f`.

The active simulator surface is intentionally empty until the replacement
`twin_sim` implementation is added. The vendor SDK, real-robot debugging
files, test fixtures, and model assets remain in their protected locations.
