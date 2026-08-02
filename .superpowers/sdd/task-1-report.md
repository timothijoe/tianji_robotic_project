# Task 1 report: package boundaries

## Evidence inherited from the interrupted agent

The prior agent's RED run cannot be confirmed from a persistent report or command log. The working tree contained the expected uncommitted implementation and tests when this agent took over.

## Verification run by this agent

- Required command from the brief: `.venv/bin/python -m pytest tests/architecture/test_package_boundaries.py tests/simulation/test_repository_boundaries.py -q`
  - Result: could not run because `.venv/bin/python` does not exist in this worktree.
- Equivalent focused command run: `python3 -m pytest tests/architecture/test_package_boundaries.py tests/simulation/test_repository_boundaries.py -q`
  - Exact result: `7 passed in 0.10s` (Python 3.12.7; pytest 7.4.4).
- `git diff --check`
  - Exact result: exit 0; no whitespace errors.

## Files reviewed and included

- `.gitignore`
- `pyproject.toml`
- `src/tianji_robotics/__init__.py`
- `src/tianji_robotics/{wuji_hand,simulation,hardware,data,workflows,wuji_sdk}/__init__.py`
- `tests/architecture/test_package_boundaries.py`

## Self-review

- Package discovery now covers both `twin_sim*` and `tianji_robotics*` from `src`.
- The new architecture test parses every Python source file in the simulation package and rejects direct hardware/SDK/ROS imports.
- The `tianji-robot` entry point is intentionally a declaration only in this task: its `cli.py` implementation is assigned to Task 5 in the plan.
- Removed test-generated `__pycache__` directories; none are staged.

## Commit

`af264a8` — `refactor: establish tianji robotics package boundaries`

## Concerns

- The requested `.venv` is absent, so verification used the available Python 3.12 interpreter with pytest 7.4.4 rather than the project's pinned pytest 9.1.1.
- The existing broad `data/` ignore rule initially excluded the required source package marker. It was narrowed to `/data/` so repository-root generated data remains ignored while `src/tianji_robotics/data/` remains maintainable.

---

## Task 1 review fix: complete import paths and default architecture collection

### Fix

- Changed the architecture import collector to retain complete AST module paths and added prefix-aware forbidden-package matching. This rejects `wuji_sdk`, `tianji_robotics.wuji_sdk`, and every submodule while retaining the existing SDK_PYTHON, fx_robot, fx_kine, wujihandpy, and rclpy protections.
- Added `tests/architecture` to pytest's default `testpaths` alongside the existing `tests/simulation` path.

### RED evidence

1. Command: `python3 -m pytest tests/architecture/test_package_boundaries.py::test_imported_roots_preserves_full_sdk_module_paths -q`
   - Exact result: `1 failed in 0.03s`.
   - Expected failure: the old helper returned `{'tianji_robotics', 'wuji_sdk'}` instead of `{'tianji_robotics.wuji_sdk', 'wuji_sdk.client'}`.
2. Command: `python3 -m pytest tests/simulation/test_repository_boundaries.py::test_pytest_collects_simulation_and_architecture_tests_by_default -q`
   - Exact result: `1 failed in 0.02s`.
   - Expected failure: configured `testpaths` was `['tests/simulation']`, missing `tests/architecture`.

### GREEN evidence

- Requested virtualenv command: `.venv/bin/python -m pytest tests/architecture/test_package_boundaries.py tests/simulation/test_repository_boundaries.py -q`
  - Exact result: `/bin/bash: line 2: .venv/bin/python: No such file or directory` (exit 127); the worktree does not contain `.venv`.
- Equivalent focused command: `python3 -m pytest tests/architecture/test_package_boundaries.py tests/simulation/test_repository_boundaries.py -q`
  - Exact result: `9 passed in 0.10s`.
- `git diff --check`
  - Exact result: exit 0; no whitespace errors.
- A default `python3 -m pytest --collect-only -q` listed all four repository-boundary tests plus all five architecture tests, confirming default collection includes `tests/architecture`. It then reported 32 unrelated collection errors because the available interpreter lacks `mujoco`; the requested `.venv` is absent, so no full or slow suite was run.

### Self-review

- The temporary-file regression test exercises both `import wuji_sdk.client` and `from tianji_robotics.wuji_sdk import transport`, and asserts that prefix matching rejects both full paths.
- Matching uses package-boundary prefixes (`package` or `package.`), avoiding false positives such as similarly prefixed module names.
- Existing simulation default collection is preserved, with architecture added rather than substituted.

### Fix commit

`ed554d3` — `test: enforce full SDK import boundaries`
