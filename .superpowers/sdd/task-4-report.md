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
- The runtime does not mutate model dynamics: it does not configure gravity
  compensation, call `mj_setConst`, or apply any force/torque feedforward.
- The active MJCF retains native force-limited position actuators and uses
  symmetric left/right gains `kp=(1280, 1280, 960, 800, 480, 320, 240)` and
  `kv=(72, 72, 56, 48, 32, 24, 20)`.

## TDD evidence

Initial runtime tests were written before `robot.py` existed and failed during
collection with:

```text
ModuleNotFoundError: No module named 'twin_sim.robot'
```

After implementation, the tracking test initially failed with a 0.193 rad
error. The runtime gravity-compensation experiment was removed following
review. With unmodified runtime dynamics, the same test measured a 0.192771
rad error, so only the active MJCF native position-actuator `kp`/`kv` constants
were tuned symmetrically for both arms; force limits and actuator semantics
were retained.

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

## Review-fix verification

- Added a focused exact-period test. Before the fix,
  `step(0.002000000001)` did not raise; it now raises the documented
  `ValueError` rather than accepting a near multiple.
- Confirmed `src/twin_sim/robot.py` contains no `body_gravcomp`, `mj_setConst`,
  `qfrc_applied`, or `qfrc_bias` usage.
- Native position-actuator measurement after the MJCF-only tuning:
  - tracking error at 250 ticks: `0.021570` rad;
  - tracking error at 1000 ticks: `0.021757` rad;
  - maximum right-arm velocity over 1000 ticks: `0.176541` rad/s;
  - all `qpos`, `qvel`, and `ctrl` values remained finite.
- `.venv/bin/python -m pytest tests/simulation/test_model.py
  tests/simulation/test_robot.py -v`: `10 passed in 0.90s`.
