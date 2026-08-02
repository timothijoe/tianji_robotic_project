# Simulation setup

The supported Ubuntu 24.04 simulation environment uses CPython 3.12, ROS 2
Jazzy, and the exact simulation dependencies in `requirements-sim.lock`.
Run the setup script from this repository root; do not copy virtual
environments from another machine:

```bash
./scripts/setup_ubuntu24_wuji_env.sh
```

The script expects sibling checkouts of this project, `wujihandros2`, and
`wujihandpy` under the same parent directory. It creates separate local
environments for the simulation, ROS bridge, and Wuji SDK.

Confirm the runtime is using the pinned simulation libraries:

```bash
.venv/bin/python -c "import mujoco, numpy, twin_sim; print(mujoco.__version__, numpy.__version__)"
```

Expected output is:

```text
3.10.0 2.5.1
```

运行当前仿真测试。若环境预设了 ROS 的 `PYTHONPATH`，请取消该变量并禁用 pytest
的第三方插件自动加载；否则 ROS 插件可能从核心仿真测试中被意外加载：

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
```

旧系统归档前的 210 项收集结果记录在 `baseline.md`；当前命令只运行
`tests/simulation/` 中的新仿真测试。

没有桌面环境时可以运行 pytest、headless 任务和录制。交互 Viewer 还要求有效的
`DISPLAY`/XWayland 会话和 OpenGL；`MUJOCO_GL=egl` 只适合离屏诊断，不能代替显示
服务器。先运行以下无窗口 smoke：

```bash
.venv/bin/twin-sim guarded-chop --headless --final-hold 0
```

Before simulation work, also verify the protected real-robot and SDK files:

```bash
sha256sum --check docs/simulation/protected-files.sha256
```

Run this command from the repository root; every manifest entry must report
`OK`.

## ROS 2 Jazzy bridge build and smoke test

The bridge build is deliberately message-only: it builds `wujihand_msgs` and
`twin_wuji_sim`, but does not build or launch a USB hardware driver. The smoke
test is import-only/headless and must not be used to discover, enable, or
write to a physical hand.

```bash
./scripts/build_ros2_jazzy_wuji_sim.sh
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src:${PYTHONPATH:-}"
.venv-ros2/bin/python tests/ros2/ros2_bridge_smoke.py
```

The smoke test should print `ROS2_WUJI_SIM_SMOKE_OK`.

新机器完整复现步骤、Viewer/SSH 排障、录制跨机器兼容条件和智能体接手清单见
[当前仿真版本复现与智能体交接](current_version_handoff.md)。
