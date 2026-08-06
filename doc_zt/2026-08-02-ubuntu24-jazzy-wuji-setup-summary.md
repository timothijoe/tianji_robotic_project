# Ubuntu 24 / ROS 2 Jazzy / Wuji 仿真环境配置记录

本次工作在 `develop_10_kinematic_branch` 完成了 MuJoCo 仿真、ROS 2 Jazzy
桥接和官方 Wuji Hand SDK 的本地配置。配置严格限定在仿真范围：官方 SDK 只做
导入与版本检查，ROS overlay 只构建消息包和本项目仿真节点，不启动 USB 驱动或
发送实体硬件命令。最后已生成一次右臂切菜、左手回撤/倒手的正常速度录制与 2×
回放。

## 最新进度（2026-08-02）

- 已从 `develop_9_kinematic_branch` 创建并切换到
  `develop_10_kinematic_branch`，环境配置相关提交已合并到该分支。
- 三套本地环境、Jazzy ROS overlay 和 headless ROS bridge smoke 均已在项目根目录
  实际运行；当前可以直接使用下文的 setup、build 和 replay 命令。
- 已生成 `recordings/guarded_chop_latest.npz`，可用于观察右臂切菜与左手回撤/倒手
  的 2× 状态回放。
- 本文档及 `doc_zt/` 是本次新增的工作记录，当前尚未提交到 Git；其余项目工作区在
  记录时保持干净。

## 当前结论

- 当前分支：`develop_10_kinematic_branch`。
- 当前系统：Ubuntu 24.04、CPython 3.12、ROS 2 Jazzy（`/opt/ros/jazzy`）。
- 项目根目录已创建并 Git 忽略三个环境：`.venv`、`.venv-ros2`、
  `.venv-wujihand`。
- 官方上游源码位于项目同级目录：`../wujihandros2` 与 `../wujihandpy`；二者
  均未被修改。
- SDK 使用非 editable 的本地安装，避免在上游 `wujihandpy` checkout 中生成
  构建元数据。

## 已加入的配置入口

### 三环境安装

```bash
./scripts/setup_ubuntu24_wuji_env.sh
```

该脚本会检查 Python 3.12、ROS 2 Jazzy 和两个同级上游仓库，然后创建：

- `.venv`：固定 `mujoco==3.10.0`、`numpy==2.5.1`、`pytest==9.1.1`，并以
  editable 方式安装本项目；
- `.venv-ros2`：使用系统 ROS Jazzy Python 包，并安装 MuJoCo/NumPy；
- `.venv-wujihand`：从同级 `wujihandpy` 源码构建并非 editable 安装 SDK。

### Jazzy ROS overlay 构建

```bash
./scripts/build_ros2_jazzy_wuji_sim.sh
```

构建脚本会安全 source `/opt/ros/jazzy/setup.bash`，并且只选择：

```text
wujihand_msgs
twin_wuji_sim
```

它不会构建或启动 `wujihand_driver`、`wujihand_bringup`，因此不涉及 USB 或实体手。

## 验证与证据

本次在项目根目录完成了以下验证：

```bash
sha256sum --check docs/simulation/protected-files.sha256
.venv/bin/python -c "import mujoco, numpy, twin_sim; print(mujoco.__version__, numpy.__version__)"
.venv-wujihand/bin/python -c "import importlib.metadata as m; import wujihandpy; print(m.version('wujihandpy'))"
./scripts/build_ros2_jazzy_wuji_sim.sh
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src:${PYTHONPATH:-}"
.venv-ros2/bin/python tests/ros2/ros2_bridge_smoke.py
```

结果：

- 保护文件清单全部报告 `OK`；
- MuJoCo/NumPy 版本为 `3.10.0 2.5.1`；
- SDK 可导入，报告版本 `1.8.1.dev12+gfdba4e446`；
- colcon 成功构建两个允许的 package；
- ROS smoke 输出 `ROS2_WUJI_SIM_SMOKE_OK`；该测试只控制 MuJoCo 中的模拟左手。

完整仿真回归应在没有 ROS `PYTHONPATH` 污染的 shell 中运行：

```bash
env -u PYTHONPATH bash --noprofile --norc -c '.venv/bin/python -m pytest --cache-clear -q'
```

本次无缓存回归完成后未留下 pytest 失败条目。验证期间发现并移除了遗留的
`src/twin_control/__pycache__`；该目录是被 Git 忽略的旧 Python 缓存，存在时会
违反旧模块已归档的边界测试。

## 切菜录制与回放

已运行以下命令打开 MuJoCo Viewer：

```bash
./scripts/run_guarded_chop_record_replay.sh --record
```

它先显示右臂切菜和左手回撤/倒手的在线仿真，随后在同一 Viewer 中进行 2× 状态
回放。生成的本地录制文件为 `recordings/guarded_chop_latest.npz`，该目录已被
Git 忽略。之后可只回放已有录制：

```bash
./scripts/replay_guarded_chop_2x.sh
```

## 相关文档

- `docs/simulation/setup.md`：核心环境安装、测试和保护文件检查；
- `docs/simulation/ros2_wuji_hand_bridge.md`：Jazzy bridge、topic/service 与 smoke；
- `docs/simulation/usage.md`：仿真任务、录制和回放；
- `docs/simulation/wuji_hand_control.md`：Wuji Hand 的 20 关节接口和安全约束；
- `docs/simulation/current_version_handoff.md`：当前版本复现与交接说明。

## 未完成项与风险

- 上游 `wujihandros2` 没有明确声明 Jazzy 支持；目前仅验证了其中的
  `wujihand_msgs` 与本项目桥接包的构建和 headless smoke。
- `.venv-ros2` 使用 `--system-site-packages`，安装 MuJoCo 时会显示系统 SciPy
  对 NumPy 版本的依赖提示；项目的仿真桥接不依赖 SciPy，但若未来添加 SciPy 工作流，
  应单独解决版本约束。
- 当前没有启动、发现、使能或控制实体 Wuji Hand。连接硬件前必须遵循
  `docs/simulation/ros2_wuji_hand_bridge.md` 的硬件安全门控。

## 下一步

1. 需要继续观察动作时，运行 `./scripts/replay_guarded_chop_2x.sh`。
2. 需要启动 ROS 仿真桥接时，source Jazzy 与 `ros2_ws/install/setup.bash` 后运行：

   ```bash
   ros2 launch twin_wuji_sim sim_hand.launch.py viewer:=true
   ```

3. 若准备升级上游 ROS 驱动或接入实体手，先在独立任务中验证 Jazzy 兼容性和硬件
   安全流程，不能把当前仿真成功视为实体机安全证明。
