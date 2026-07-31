# Wuji Hand Control Guide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a verified Chinese guide that explains the simulated left Wuji Hand's 20-joint layout and safe position-command workflow.

**Architecture:** Keep detailed material in one focused guide and add lightweight links from the repository README and simulation usage page. Treat the compiled MuJoCo model and current Python API as the sources of truth, and clearly segregate future real-hardware concepts.

**Tech Stack:** Markdown, Python 3.12, NumPy 2.5.1, MuJoCo 3.10.0, pytest 9.1.1

## Global Constraints

- Document current simulation behavior as executable and real-hand transport as unimplemented.
- Use exact finger-major joint/actuator order and compiled control ranges.
- Use radians and shape `(20,)` in every command example.
- Do not invent anatomical finger labels.
- Do not modify protected real-robot files.

---

### Task 1: Write and Verify the Control Guide

**Files:**
- Create: `docs/simulation/wuji_hand_control.md`
- Create: `tests/simulation/test_hand_documentation.py`

**Interfaces:**
- Consumes: `HAND_JOINTS`, `HAND_ACTUATORS`, `DEFAULT_OPEN_RAD`,
  `RELAXED_CLOSE_RAD`, `RightArmRobot.hand`
- Produces: exact joint table, CLI/API instructions, smooth-send example,
  validation behavior, and future-hardware boundary

- [ ] **Step 1: Write a failing documentation contract test**

Create a test that requires the guide to exist, contain every joint and actuator
name, contain every compiled control range formatted to four decimal places,
and include the headings `仿真控制`, `平滑发送`, `读取状态`, and
`未来实体手接入`.

- [ ] **Step 2: Run the test to verify red**

Run:
`.venv/bin/pytest tests/simulation/test_hand_documentation.py -v`

Expected: FAIL because `docs/simulation/wuji_hand_control.md` does not exist.

- [ ] **Step 3: Write the complete guide**

Document the model, ordered 20-joint table, both built-in target vectors, CLI,
minimal Python send/read example, smooth interpolation, API semantics, errors,
and simulation-versus-hardware table. Explain that `command()` stores,
`apply()` writes `data.ctrl`, and `step()` applies then advances physics.

- [ ] **Step 4: Run the documentation contract**

Run:
`.venv/bin/pytest tests/simulation/test_hand_documentation.py -v`

Expected: PASS.

- [ ] **Step 5: Execute both documented Python workflows**

Run the minimal command/read example and the interpolated open-to-relaxed-close
example headlessly using `.venv/bin/python`.

Expected: both exit zero, print 20 finite joint positions, and close resources.

- [ ] **Step 6: Commit**

```bash
git add docs/simulation/wuji_hand_control.md tests/simulation/test_hand_documentation.py
git commit -m "docs: add Wuji hand control guide"
```

### Task 2: Link the Guide and Run Final Verification

**Files:**
- Modify: `README.md`
- Modify: `docs/simulation/usage.md`

**Interfaces:**
- Produces: discoverable guide links from both documentation entry points

- [ ] **Step 1: Write failing link assertions**

Extend `test_hand_documentation.py` to require
`docs/simulation/wuji_hand_control.md` from `README.md` and
`wuji_hand_control.md` from `docs/simulation/usage.md`.

- [ ] **Step 2: Run the test to verify red**

Run:
`.venv/bin/pytest tests/simulation/test_hand_documentation.py -v`

Expected: FAIL because neither entry point links the new guide.

- [ ] **Step 3: Add concise links**

Add “Wuji Hand 控制手册” to the README's detailed-document list and a short
“灵巧手控制” section to the simulation usage page.

- [ ] **Step 4: Verify repository state**

Run:

```bash
.venv/bin/pytest -q
sha256sum --check docs/simulation/protected-files.sha256
git diff --check
rg -n "TBD|TODO|待补充|直接连接实体" docs/simulation/wuji_hand_control.md
```

Expected: all tests pass, all protected files report `OK`, no whitespace
errors, and the scan finds no placeholders or unsupported hardware claim.

- [ ] **Step 5: Commit**

```bash
git add README.md docs/simulation/usage.md tests/simulation/test_hand_documentation.py
git commit -m "docs: link Wuji hand control guide"
```
