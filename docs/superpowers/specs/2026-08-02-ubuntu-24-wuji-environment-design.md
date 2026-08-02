# Ubuntu 24 ROS 2 Jazzy and Wuji environment design

## Objective

Configure this repository for its MuJoCo simulation, ROS 2 simulation bridge,
and official Wuji Hand SDK on Ubuntu 24.04.  The result must remain
hardware-free: it may import the official SDK but must not discover, enable, or
command a physical hand.

## Constraints

- The host is Ubuntu 24.04 with ROS 2 Jazzy and CPython 3.12.
- Existing upstream checkouts are siblings of the primary repository checkout:
  `../wujihandros2` and `../wujihandpy`.  Setup entry points must locate that
  primary checkout through Git's common directory so they also work from a
  linked worktree.
- Upstream documents Humble and Kilted, not Jazzy.  The integration therefore
  builds only its interface package, `wujihand_msgs`, rather than the upstream
  USB driver.
- The simulation's CPython 3.12 environment and the ROS environment remain
  isolated from the SDK environment.
- No container, virtual machine, USB rule, system ROS reinstallation, or
  hardware-driver launch is in scope.

## Design

Three repository-local environments are created:

| Environment | Interpreter | Purpose |
| --- | --- | --- |
| `.venv` | CPython 3.12 | Pinned MuJoCo simulation and pytest suite |
| `.venv-ros2` | CPython 3.12 with system packages | ROS 2 Jazzy bridge and MuJoCo runtime |
| `.venv-wujihand` | CPython 3.12 | Official `wujihandpy` SDK import-only verification |

The ROS overlay uses `ros2_ws/build`, `ros2_ws/install`, and `ros2_ws/log`.
Its source inputs are `../wujihandros2/wujihand_msgs` and
`ros2_ws/src/twin_wuji_sim`.  It does not include `wujihand_driver` or
`wujihand_bringup`, so no USB implementation becomes build or runtime state.

To make the setup repeatable, the repository will provide a setup script that
uses these fixed sibling paths, checks that their package metadata exists,
creates the environments, and installs the documented dependencies.  It will
also provide a Jazzy-specific ROS build script that sources `/opt/ros/jazzy`,
sets the ROS virtual-environment interpreter ahead of other Python locations,
and builds only the two allowed packages.

## Validation

Validation proceeds without hardware:

1. Check the real-robot and vendor-file checksum manifest.
2. Verify pinned MuJoCo and NumPy versions and run the simulation regression
   suite.
3. Verify `wujihandpy` can be imported and report its installed version without
   constructing `Hand`.
4. Build the restricted ROS overlay, source it, and run the existing headless
   bridge smoke test.

If Jazzy cannot compile the upstream message package, stop at that concrete
error and make the smallest source-level Jazzy compatibility adjustment in this
repository's setup integration.  Upstream checkouts remain unmodified.

## Error handling and safety

The setup scripts fail early for a missing Python version, ROS distribution,
or sibling checkout.  They do not run `sudo`, edit udev rules, probe USB, or
instantiate the official hand SDK.  Dependency installation is the only
networked action and uses the official upstream package/repository locations.
