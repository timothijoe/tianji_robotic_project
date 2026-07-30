# Legacy test baseline

Recorded before the simulation rebuild with the repository's global CPython
3.12 environment, which did not have MuJoCo installed.

```bash
python3.12 -m pytest -q
```

Result: exit status 2; 0 passed, 10 collection errors in 0.26s; no tests ran.
Every error was `ModuleNotFoundError: No module named 'mujoco'`.

Failing collection node IDs:

- `tests/test_chopping.py`
- `tests/test_control.py`
- `tests/test_description_scene.py`
- `tests/test_ik_cli.py`
- `tests/test_kinematics.py`
- `tests/test_robot.py`
- `tests/test_runtime_arm_view.py`
- `tests/test_sdk_compat.py`
- `tests/test_source_model.py`
- `tests/test_twin_control_chopping.py`

The protected-file hash manifest contained 91 files across `SDK_PYTHON`,
`test`, and `real_robot_debug` before this task.

With the pinned clean environment installed, `pytest --collect-only -q`
successfully collects 210 tests in 0.45s. This is the legacy collection count
to preserve during the rebuild.
