# Task 4 Report: Right-Arm Position Runtime

## Delivered

- Added `src/twin_sim/robot.py` with `RightArmRobot`, `RIGHT_HOME_RAD`, and
  `NumericalSafetyError`.
- The runtime resets and commands only the seven right-arm native MuJoCo
  position actuators; left-arm actuator targets are held at their reset
  positions.
- `step()` validates that its duration is a finite, positive integer multiple
  of the MJCF timestep, advances the required substeps, synchronizes an
  optional passive viewer, and rejects non-finite `qpos`, `qvel`, or `ctrl`.
- `tcp_pose()` returns `[x, y, z, qw, qx, qy, qz]` from
  `right_tool_tip_site`; `close()` is idempotent in headless use.
- Right-arm body-level MuJoCo gravity compensation is configured during
  initialization. The scene's required force-limited position actuators could
  not otherwise hold the specified home target against gravity; this remains
  native position-actuator simulation and adds no torque, impedance, or
  admittance-control API.

## TDD evidence

Initial runtime tests were written before `robot.py` existed and failed during
collection with:

```text
ModuleNotFoundError: No module named 'twin_sim.robot'
```

After implementation, the tracking test initially failed with a 0.193 rad
error. Diagnostic runs isolated the cause to gravity acting through the scene's
fixed actuator force limits; enabling only MuJoCo body gravity compensation on
the right arm reduced the same error to 0.0084 rad.

During self-review, `NaN` control periods were found to expose a Python
conversion error rather than the public `ValueError` contract. A regression
test was added first, observed failing with `cannot convert float NaN to
integer`, then passed after finite-duration validation was moved before
substep rounding.

## Verification

```text
.venv/bin/python -m pytest tests/simulation/test_robot.py -v
6 passed in 0.67s

.venv/bin/python -m pytest -v
11 passed in 0.86s

.venv/bin/python -m compileall -q src/twin_sim tests/simulation/test_robot.py
exit 0

sha256sum --check --quiet docs/simulation/protected-files.sha256
exit 0

git diff --check
exit 0
```

## Concerns

No remaining functional concerns. Viewer creation is opt-in and was not
exercised in headless tests; its lifecycle is guarded so `close()` safely does
nothing when no viewer was launched.
