# Legacy Simulation Archive

**Archive date:** 2026-07-30
**Design baseline:** commit `a8cde84`
**Historical source snapshot:** commit
`1d7e044ea9eb9f38e2d2e2b90cd2f4d61e2cb55f`

This is the historical snapshot of the superseded torque- and
impedance-control MuJoCo simulation. It is intentionally outside the active
source tree, package configuration, and pytest test paths. The active
simulator surface is deliberately empty until the replacement `twin_sim`
implementation arrives in subsequent tasks.

## Contents and former entrypoints

- `src/` preserves the former `twin_control`, `twin_core`, `twin_mujoco`, and
  `twin_description` packages.
- `examples/` preserves the former demonstration scripts.
- `tests/` preserves the legacy simulation test suite; it is not collected by
  the active pytest configuration.
- `docs/` preserves the former architecture document, repository README, and
  superseded simulation plans and specifications.

The retired console entrypoints were `twin-chop`, `twin-trajectory`,
`twin-chop-sdk`, and `twin-ik`. They describe the archived implementation and
must not be treated as active simulator interfaces.

## Historical dependencies and retirement reason

The archived stack used Python, MuJoCo, NumPy, and pytest; its SDK-compatibility
and real-backend paths also referenced the vendor files under `SDK_PYTHON/`.
It was retired because its torque, impedance, force-control, and
SDK-compatibility APIs do not match the approved 2026-07-30 MuJoCo-native
position-control rebuild. Compatibility is intentionally not preserved.

## Inspect and restore

Inspect this snapshot in place without installing it, for example by reading
the archived `src/`, `examples/`, and `docs/` trees. To inspect the exact
pre-archive version of a path, use the source snapshot directly:

```bash
git show 1d7e044ea9eb9f38e2d2e2b90cd2f4d61e2cb55f:src/twin_control/robot.py
```

If restoration is required, work on a dedicated branch and restore only the
needed paths from that source snapshot. This overwrites any replacement paths
that later tasks may have created:

```bash
git restore --source=1d7e044ea9eb9f38e2d2e2b90cd2f4d61e2cb55f -- \
  src/twin_control src/twin_core src/twin_mujoco src/twin_description examples tests
```

The vendor SDK, real-robot debugging files, test fixtures, and MarvinCCS model
assets remain at their protected root locations and are not part of this
archive.
