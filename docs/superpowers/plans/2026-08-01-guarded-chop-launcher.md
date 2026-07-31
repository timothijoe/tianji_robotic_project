# Guarded Chop Launcher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add one executable Shell script that reliably launches the default plane guarded-chop Viewer from any working directory.

**Architecture:** The script resolves the repository root relative to its own path, validates the existing virtual-environment executable, and replaces itself with the `twin-sim guarded-chop --scene plane` process. A pytest test executes a copied script against a fake repository and fake `twin-sim`, proving path resolution and error handling without opening MuJoCo.

**Tech Stack:** Bash, Python 3.12, pytest, existing `.venv/bin/twin-sim` CLI.

## Global Constraints

- Create only `scripts/run_guarded_chop.sh`; additional launcher scripts and mode arguments are out of scope.
- The script always launches `guarded-chop --scene plane` with Viewer enabled.
- It must work when invoked outside the repository root.
- It must not invoke ROS 2, physical robots, Wuji hardware, or hardware-send interfaces.
- Missing `.venv/bin/twin-sim` must produce a clear nonzero failure with installation commands.

---

### Task 1: Add and verify the plane Viewer launcher

**Files:**
- Create: `scripts/run_guarded_chop.sh`
- Create: `tests/simulation/test_guarded_chop_launcher.py`
- Modify: `docs/simulation/usage.md`

**Interfaces:**
- Produces: executable `scripts/run_guarded_chop.sh` with no arguments.
- Consumes: repository-local `.venv/bin/twin-sim` and the existing `guarded-chop --scene plane` CLI.

- [ ] **Step 1: Write failing launcher tests**

```python
import os
import subprocess
from pathlib import Path


SCRIPT = Path(__file__).parents[2] / "scripts" / "run_guarded_chop.sh"


def test_launcher_resolves_repo_and_forwards_plane_command(tmp_path):
    fake_repo = tmp_path / "repo"
    script = fake_repo / "scripts" / SCRIPT.name
    executable = fake_repo / ".venv" / "bin" / "twin-sim"
    script.parent.mkdir(parents=True)
    executable.parent.mkdir(parents=True)
    script.write_bytes(SCRIPT.read_bytes())
    script.chmod(0o755)
    executable.write_text("#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n")
    executable.chmod(0o755)

    result = subprocess.run(
        [str(script)], cwd=tmp_path, text=True, capture_output=True
    )

    assert result.returncode == 0
    assert result.stdout.splitlines() == ["guarded-chop", "--scene", "plane"]


def test_launcher_reports_missing_virtual_environment(tmp_path):
    fake_repo = tmp_path / "repo"
    script = fake_repo / "scripts" / SCRIPT.name
    script.parent.mkdir(parents=True)
    script.write_bytes(SCRIPT.read_bytes())
    script.chmod(0o755)

    result = subprocess.run(
        [str(script)], cwd=tmp_path, text=True, capture_output=True
    )

    assert result.returncode != 0
    assert ".venv/bin/twin-sim" in result.stderr
    assert "python3.12 -m venv .venv" in result.stderr
```

- [ ] **Step 2: Run tests and verify the missing script fails**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_launcher.py -q`  
Expected: FAIL because `scripts/run_guarded_chop.sh` does not exist.

- [ ] **Step 3: Create the minimal executable script**

```bash
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
TWIN_SIM="${REPO_ROOT}/.venv/bin/twin-sim"

if [[ ! -x "${TWIN_SIM}" ]]; then
  printf '%s\n' \
    "Missing executable: ${TWIN_SIM}" \
    "Install the simulation environment from ${REPO_ROOT}:" \
    "  python3.12 -m venv .venv" \
    "  .venv/bin/python -m pip install -r requirements-sim.lock" \
    "  .venv/bin/python -m pip install -e ." >&2
  exit 1
fi

cd -- "${REPO_ROOT}"
exec "${TWIN_SIM}" guarded-chop --scene plane
```

Create the file with `apply_patch`, then run `chmod +x scripts/run_guarded_chop.sh`.

- [ ] **Step 4: Document the launcher**

Add this exact command to the guarded-chop section of `docs/simulation/usage.md`:

```bash
./scripts/run_guarded_chop.sh
```

State that it opens the default plane Viewer, resolves the repository path
automatically, requires the repository-local `.venv`, and never sends hardware
commands.

- [ ] **Step 5: Run focused verification**

Run: `.venv/bin/pytest tests/simulation/test_guarded_chop_launcher.py -q`  
Expected: `2 passed`.

Run: `bash -n scripts/run_guarded_chop.sh`  
Expected: exit 0, no output.

Run: `git diff --check`  
Expected: exit 0, no output.

- [ ] **Step 6: Commit**

```bash
git add scripts/run_guarded_chop.sh tests/simulation/test_guarded_chop_launcher.py docs/simulation/usage.md
git commit -m "feat: add guarded chop launcher script"
```

- [ ] **Step 7: Launch the real Viewer for user inspection**

Run: `./scripts/run_guarded_chop.sh`  
Expected: MuJoCo Viewer opens, completes five right-to-left cuts and four
open-shift-close handovers, prints `success=True`, and exits automatically.

