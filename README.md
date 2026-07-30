# tianji_robotic_project

The legacy MuJoCo torque/impedance simulator has been archived while the
approved position-control rebuild is in progress. The active `twin_sim`
implementation arrives in subsequent tasks; this repository does not yet
provide an active simulator CLI.

The approved rebuild documents are:

- [Design](docs/superpowers/specs/2026-07-30-mujoco-position-simulation-rebuild-design.md)
- [Implementation plan](docs/superpowers/plans/2026-07-30-mujoco-position-simulation-rebuild.md)

Historical code, examples, tests, and documentation are available in
[archive/legacy_simulation](archive/legacy_simulation/README.md).

The following real-robot and vendor paths are protected during the rebuild:
`SDK_PYTHON/`, `test/`, `real_robot_debug/`, `MarvinCCS/`, and
`MarvinCCS_mujoco.zip`.
