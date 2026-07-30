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

The committed [protected-file manifest](protected-files.sha256) contains 91
SHA-256 hashes across `SDK_PYTHON`, `test`, and `real_robot_debug`. Before a
simulation rebuild task, verify these files from the repository root with:

```bash
sha256sum --check docs/simulation/protected-files.sha256
```

Every entry must report `OK`. Do not regenerate the manifest unless an
authorized change deliberately modifies one of the protected files.

With the pinned clean environment installed, `pytest --collect-only -q`
successfully collects 210 tests. This is the legacy collection count to
preserve during the rebuild.
