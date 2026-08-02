# Final fix report: non-editable Wuji SDK installation

## Scope

Fixed the final-review finding only in the project worktree. No upstream checkout
was edited.

## Read-only upstream evidence

The following read-only commands both completed with exit code 0 and produced no
output, which means their short working-tree statuses are clean:

```text
git -C /home/zhoutong/catkin_robotic_ws/august_ws/wujihandpy status --short
git -C /home/zhoutong/catkin_robotic_ws/august_ws/wujihandros2 status --short
```

## Changes

- `scripts/setup_ubuntu24_wuji_env.sh`: installs the sibling `wujihandpy` source
  into `.venv-wujihand` with the non-editable local command
  `.venv-wujihand/bin/python -m pip install "${WUJI_HAND_PY}"`.
- `tests/simulation/test_guarded_chop_launcher.py`: requires that exact command
  and rejects the editable `pip install -e "${WUJI_HAND_PY}"` variant.
- `docs/simulation/ros2_wuji_hand_bridge.md`: corrects the heading to say three
  isolated environments.

## TDD evidence

After changing the source-level test but before changing the setup script:

```text
python3 -m pytest tests/simulation/test_guarded_chop_launcher.py \\
  -k ubuntu24_wuji_setup_is_hardware_free_and_validates_siblings
1 failed, 10 deselected
```

The failure was the expected missing non-editable command; the script still
contained `pip install -e "${WUJI_HAND_PY}"`.

After the implementation:

```text
python3 -m pytest tests/simulation/test_guarded_chop_launcher.py
11 passed in 0.06s

bash -n scripts/setup_ubuntu24_wuji_env.sh
# exit 0, no output

git diff --check
# exit 0, no output
```

## Commit

Functional fix commit: `3316b6a fix: install Wuji SDK non-editably`.
