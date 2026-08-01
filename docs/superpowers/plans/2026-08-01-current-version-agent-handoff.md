# Current Version Agent Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide a repository-relative, cross-machine runbook that lets a new user or agent install, verify, record, replay, troubleshoot, and safely continue the current MuJoCo guarded-chop version.

**Architecture:** Add one canonical handoff guide and link to it from the README. Keep environment mechanics in `setup.md` and command details in `usage.md`, while a lightweight documentation contract test prevents script names, default paths, safety boundaries, and machine-specific paths from drifting.

**Tech Stack:** Markdown, Bash command examples, Python 3.12, pytest

## Global Constraints

- Use repository-relative paths; the canonical handoff must not contain `/home/linux` or another developer-machine path.
- Require CPython `3.12.x`, MuJoCo `3.10.0`, NumPy `2.5.1`, and the repository `.venv`.
- Describe both headless and interactive Viewer workflows.
- Document all four guarded-chop scripts and default `recordings/guarded_chop_latest.npz` behavior.
- State that arms and Wuji Hand use MuJoCo joint position actuators, not Cartesian impedance control.
- Preserve the 0.02 m knife-hand limit, zero blade-hand contact, and simulation-only hardware boundary.
- Link ROS 2 and hardware topics without adding them to the default reproduction path.

---

### Task 1: Add a documentation contract

**Files:**
- Create: `tests/simulation/test_current_version_handoff.py`
- Create: `docs/simulation/current_version_handoff.md`

**Interfaces:**
- Consumes: repository documentation and scripts.
- Produces: automated assertions for the canonical guide's paths, commands, version pins, safety statements, and absence of developer-specific absolute paths.

- [ ] **Step 1: Write the failing documentation test**

Create `tests/simulation/test_current_version_handoff.py`:

```python
from pathlib import Path

ROOT = Path(__file__).parents[2]
HANDOFF = ROOT / "docs" / "simulation" / "current_version_handoff.md"


def test_current_version_handoff_is_cross_machine_and_complete():
    text = HANDOFF.read_text()
    required = (
        "python3.12 -m venv .venv",
        "mujoco.__version__",
        "sha256sum --check docs/simulation/protected-files.sha256",
        ".venv/bin/python -m pytest -q",
        "./scripts/run_guarded_chop.sh",
        "./scripts/run_guarded_chop_record.sh",
        "./scripts/replay_guarded_chop_2x.sh",
        "./scripts/run_guarded_chop_record_replay.sh",
        "recordings/guarded_chop_latest.npz",
        "0.02 m",
        "position",
        "ROS 2",
    )
    assert all(value in text for value in required)
    assert "/home/linux" not in text
    assert all((ROOT / path).exists() for path in (
        "scripts/run_guarded_chop.sh",
        "scripts/run_guarded_chop_record.sh",
        "scripts/replay_guarded_chop_2x.sh",
        "scripts/run_guarded_chop_record_replay.sh",
    ))
```

- [ ] **Step 2: Run the contract and verify RED**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_current_version_handoff.py -v
```

Expected: FAIL because `docs/simulation/current_version_handoff.md` does not exist.

- [ ] **Step 3: Write the canonical handoff guide**

Create `docs/simulation/current_version_handoff.md` with these exact top-level sections:

```markdown
# 当前仿真版本复现与智能体交接
## 1. 当前版本摘要
## 2. 新机器环境安装
## 3. 分层验证
## 4. 切菜、录制与回放入口
## 5. 录制文件跨机器使用
## 6. Viewer 与 OpenGL 排障
## 7. 控制架构和安全不变量
## 8. 核心代码地图
## 9. 新智能体接手清单
## 10. ROS 2 与实体机边界
```

Include the exact setup and smoke commands from the design, the four-script comparison table, NPZ schema/model compatibility notes, `DISPLAY`/SSH/X11/EGL troubleshooting, module paths, and before/after modification checklists. Use only relative repository paths.

- [ ] **Step 4: Run the documentation contract and verify GREEN**

Run:

```bash
.venv/bin/python -m pytest tests/simulation/test_current_version_handoff.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add docs/simulation/current_version_handoff.md tests/simulation/test_current_version_handoff.py
git commit -m "docs: add current simulation agent handoff"
```

### Task 2: Connect and verify the canonical guide

**Files:**
- Modify: `README.md`
- Modify: `docs/simulation/setup.md`
- Modify: `docs/simulation/usage.md`
- Modify: `docs/superpowers/plans/2026-08-01-current-version-agent-handoff.md`

**Interfaces:**
- Consumes: `docs/simulation/current_version_handoff.md` from Task 1.
- Produces: discoverable README link, consistent setup/usage commands, and fresh cross-machine verification evidence.

- [ ] **Step 1: Add discoverability and current commands**

Update `README.md` to add current guarded-chop capability bullets and place this link first in “详细说明”:

```markdown
- [当前版本复现与智能体交接](docs/simulation/current_version_handoff.md)
```

Update `docs/simulation/setup.md` to distinguish headless from Viewer prerequisites and link to the handoff. Update `docs/simulation/usage.md` with:

```bash
./scripts/run_guarded_chop_record.sh [recording.npz]
./scripts/replay_guarded_chop_2x.sh [recording.npz]
```

and a four-script behavior table identical to the canonical guide.

- [ ] **Step 2: Verify documentation consistency**

Run:

```bash
.venv/bin/python -m pytest \
  tests/simulation/test_current_version_handoff.py \
  tests/simulation/test_guarded_chop_launcher.py -q
bash -n scripts/run_guarded_chop.sh \
  scripts/run_guarded_chop_record.sh \
  scripts/replay_guarded_chop_2x.sh \
  scripts/run_guarded_chop_record_replay.sh
rg -n "/home/linux" docs/simulation/current_version_handoff.md
```

Expected: pytest PASS, `bash -n` exit zero, and `rg` exit one with no matches.

- [ ] **Step 3: Run fresh headless smoke and complete regression**

Run:

```bash
.venv/bin/twin-sim guarded-chop --headless --final-hold 0
.venv/bin/python -m pytest -q
```

Expected: guarded-chop reports 5 cuts, 4 shifts, total 0.080 m, minimum distance at least 0.02 m; all tests PASS with only the existing 39.611 N observation warning.

- [ ] **Step 4: Record exact verification and complete the plan**

Append a short “本次验证基线” section to the handoff with the current test count, warning, task metrics, branch name, and the rule that counts may grow while zero failures remains mandatory. Mark all evidenced plan checkboxes complete.

- [ ] **Step 5: Commit**

```bash
git add README.md docs/simulation/setup.md docs/simulation/usage.md \
  docs/simulation/current_version_handoff.md \
  docs/superpowers/plans/2026-08-01-current-version-agent-handoff.md
git commit -m "docs: connect cross-machine simulation runbook"
```

- [ ] **Step 6: Report branch state**

Verify `git status --short` is empty and report that all commits are already on `develop_9_kinematic_branch`. If the user names `develop_8_kinematic_branch` or `main` as a separate target, run the finishing-development-branch merge workflow; otherwise do not create a meaningless self-merge.
