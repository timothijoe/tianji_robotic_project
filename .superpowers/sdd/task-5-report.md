# Task 5 Report: SI-Only Kinematics and Continuous IK

## Delivered

- Added `src/twin_sim/kinematics.py` with:
  - `IkResult`;
  - TCP-site `Kinematics.fk()` and 6-by-7 `jacobian()`;
  - damped least-squares `ik()` using translation and rotation-vector error;
  - MJCF joint-limit clipping on the initial seed and every update;
  - continuous `solve_path()` preflight seeded from each preceding solution;
  - indexed `PathIkError` failures for unreachable samples and joint-step
    violations.
- Added `tests/simulation/test_kinematics.py` covering reachable and
  unreachable IK, continuous path solving, joint-step rejection, rigid-pose
  validation, parameter validation, and shared MuJoCo state preservation.
- All values and APIs are SI-only: metres, radians, seconds, and native MuJoCo
  arrays. No SDK-unit conversion or impedance behavior was introduced.

## State-safety design

Every FK and Jacobian query first copies the complete shared `MjData`, performs
the temporary right-arm configuration and MuJoCo query, and restores the full
copy in a `finally` block. IK is composed only from those guarded queries.

This preserves `qpos`, `qvel`, `ctrl`, `time`, and derived state exactly,
including state immediately after `mj_step` and failures raised during
`mj_forward`. Regression tests compare site transforms, accelerations, bias
and constraint forces, and sensor data before and after FK, Jacobian, and IK.

## TDD evidence

The initial focused test run failed during collection as required:

```text
ModuleNotFoundError: No module named 'twin_sim.kinematics'
```

The first implementation run exposed a frame mismatch on nontrivial path
samples. A numerical derivative showed that `mju_subQuat` produced a
current-site-local rotation vector while `mj_jacSite` produced world-frame
angular rows. Converting the error through the current TCP rotation made both
path regressions pass.

Self-review then added three further RED cases before their fixes:

- post-step derived state changed when restoration only re-ran `mj_forward`;
- malformed homogeneous transforms and reflections were silently accepted;
- fractional `max_iterations` leaked an incidental `TypeError`.

The fixes respectively use full `MjData` copy/restore, validate proper rigid
4-by-4 transforms, and require a non-negative integral iteration count.

## Review

Independent code review found no Critical or Important issues and assessed the
implementation ready. Its rigid-transform validation finding and minor
iteration-validation finding were both addressed test-first. The remaining
test-coverage suggestion is already exercised by the multi-sample path test,
which requires nontrivial translational and rotational DLS updates; an
additional finite-difference probe measured a maximum Jacobian error of
`2.11e-08`.

## Verification

```text
.venv/bin/python -m pytest tests/simulation/test_kinematics.py -v
14 passed

.venv/bin/python -m pytest -v
26 passed

.venv/bin/python -m compileall -q src tests
exit 0

git diff --check
exit 0
```

## Concerns

No remaining functional concerns. Full `MjData` copies intentionally trade a
small amount of preflight computation and allocation for exact preservation of
the shared runtime state; kinematics probes remained fast for the current
14-DOF model.
