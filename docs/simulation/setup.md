# Simulation setup

The MuJoCo simulation uses CPython 3.12 and the exact dependencies in
`requirements-sim.lock`. Create a clean local environment from the repository
root:

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-sim.lock
.venv/bin/python -m pip install -e .
```

Confirm the runtime is using the pinned simulation libraries:

```bash
.venv/bin/python -c "import mujoco, numpy; print(mujoco.__version__, numpy.__version__)"
```

Expected output is:

```text
3.10.0 2.5.1
```

To verify test discovery after installation, run:

```bash
.venv/bin/python -m pytest --collect-only -q
```

The baseline environment collects 210 tests.

Before simulation work, also verify the protected real-robot and SDK files:

```bash
sha256sum --check docs/simulation/protected-files.sha256
```

Run this command from the repository root; every manifest entry must report
`OK`.
