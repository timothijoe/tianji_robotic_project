# Tianji + Wuji + MuJoCo 跨机器迁移与交接手册

本文说明如何把当前开发机上的 Tianji 双臂、Wuji 左手、MuJoCo 仿真、真实 MCAP
录制和验证流程迁移到另一台 Linux 电脑。目标是复现同一套设备和软件功能，不是
更换机器人厂商或重新设计模型。

本文以 2026-08-05 的以下版本为基线：

- 自研仓库分支：`develop_12_kinematic_branch`
- 文档编写前远端 HEAD：`32d4a8df31305bca54da0b6f2b185cc00fa98f62`
- 深度对齐与指腹接触实现：`6b99835`
- Python：3.12.7；项目要求 `>=3.12,<3.13`
- MuJoCo：3.10.0
- NumPy：2.5.1
- pytest：9.1.1
- MCAP Python：1.4.0
- Wuji SDK Python 包：2026.7.21

迁移完成的定义是：新机器能在不连接 Tianji 或 Wuji 真机的情况下运行完整
Headless 和 Viewer 案例，读到同一份真实 MCAP，得到相同数量级的几何安全指标，
并通过自动化测试。真机 SDK 和 ROS 2 接口保留，但不属于本次仿真验收。

## 1. 先理解哪些内容在哪里

### 1.1 必须随 Git 获取的内容

以下内容都在 `tianji_robotic_project` Git 仓库内，新机器正常 clone 后即可得到：

- `src/twin_sim/`：Tianji 双臂 MuJoCo 控制、IK、轨迹和任务；
- `src/tianji_robotics/`：Wuji MCAP、离线重定向适配、hand-only 仿真和硬件接口边界；
- `robot_assets/mujoco/`：当前运行所需的 Tianji/Wuji/刀具/桌面 MuJoCo 资产；
- `MarvinCCS/`：Tianji/Marvin 相关模型资产；
- `ros2_ws/`：ROS 2 仿真桥接源码；
- `scripts/`：环境安装、验证和 Viewer 启动脚本；
- `tests/`：仿真、架构、Wuji 数据和 CLI 回归；
- `docs/`：设计、开发状态、运行说明和本迁移手册。

当前资产大致大小：

| 路径 | 大小 | Git 状态 | 新机器处理 |
|---|---:|---|---|
| `robot_assets/` | 约 6.2 MB | 已跟踪 | 随 clone 获取 |
| `MarvinCCS/` | 约 46 MB | 已跟踪 | 随 clone 获取 |
| `ros2_ws/` | 约 6.4 MB | 源码跟踪，构建目录忽略 | 随 clone 获取源码 |
| `recordings/` | 当前约 90 MB | 完全忽略 | 必须单独复制 |
| `.venv-wuji-teleop/` | 当前约 256 MB | 完全忽略 | 禁止复制，必须重建 |

本项目的联合仿真从仓库内加载：

```text
robot_assets/mujoco/right_chopping_scene.xml
```

hand-only Wuji 回放从仓库内加载：

```text
robot_assets/mujoco/wuji_hand_standalone/mjcf/left.xml
```

因此当前运行路径不会在启动时读取 `../wuji-technology/wuji-description` 或
`../wuji-technology/mujoco-sim`。这些相邻仓库用于官方来源追溯、SDK/ROS 2 开发
和未来同步，不应成为隐藏的绝对路径依赖。

### 1.2 不随 Git 迁移的内容

`.gitignore` 明确排除了：

```text
.venv/
.venv-wuji-teleop/
.venv-wujihand/
.venv-ros2/
recordings/
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
logs/
*.csv
```

迁移时最容易遗漏的是 `recordings/`。虚拟环境和 ROS 2 构建结果则不应复制；它们
包含本机解释器、绝对路径或平台相关二进制，必须在目标机重新生成。

## 2. 标准目录拓扑与相对路径契约

新机器统一采用下面的布局。顶层目录名称可以改变，但两个子目录的相对关系不能
改变：`wuji-technology` 必须与 `tianji_robotic_project` 相邻。

```text
<workspace>/
├── tianji_robotic_project/          # 我们控制的 Git 仓库
│   ├── src/
│   ├── robot_assets/
│   ├── recordings/                  # 本地复制，Git 忽略
│   ├── scripts/
│   ├── tests/
│   ├── docs/
│   └── .venv-wuji-teleop/           # 本地创建，Git 忽略
└── wuji-technology/                 # Wuji 官方源码集合，独立 Git 仓库
    ├── isaaclab-sim/
    ├── mujoco-sim/
    ├── robotics-rigid-body-mechanics/
    ├── wuji-description/
    ├── wuji-mjlab/
    ├── wuji-retargeting/
    ├── wuji-sdk/
    ├── wuji-studio/
    ├── wujihandpy/
    └── wujihandros2/
```

从项目根目录看，官方仓库的标准路径必须是：

```text
../wuji-technology/<official-repository>
```

当前开发机还存在下列历史平级副本：

```text
../wuji-description
../wuji-mjlab
../wuji-studio
../wujihandpy
../wujihandros2
```

这些不是新机器的标准布局。不要复制它们，不要建立同名软链接，也不要让新代码
依赖它们。所有官方源码统一放到 `../wuji-technology/`。

可以从项目根目录检查布局：

```bash
test -d ../wuji-technology/wuji-sdk
test -d ../wuji-technology/wuji-description
test -d ../wuji-technology/mujoco-sim
```

## 3. GitHub 仓库和外部源码依赖

### 3.1 我们的仓库

| 所有者 | GitHub | 新机器标准位置 | 必需性 |
|---|---|---|---|
| Tianji/Wuji 自研集成 | `https://github.com/timothijoe/tianji_robotic_project.git` | `<workspace>/tianji_robotic_project` | 必需 |

当前机器使用 SSH remote：

```text
git@github.com:timothijoe/tianji_robotic_project.git
```

没有配置 SSH key 时使用 HTTPS clone 即可。

### 3.2 Wuji 官方 GitHub

下表记录了当前开发机 `../wuji-technology/` 中的真实 remote 和 HEAD。为了严格
复现可以 checkout 表中提交；为了跟随官方更新可以保留 `main`，但更新后必须重新
执行验证，不能默认兼容。

| 相对项目路径 | GitHub | 当前 HEAD | 当前案例作用 | 首选 Headless 是否直接需要 |
|---|---|---|---|---|
| `../wuji-technology/isaaclab-sim` | `https://github.com/wuji-technology/isaaclab-sim.git` | `67c8a36743ef12e341b9c814126aafbbd1167ac8` | Isaac Lab 官方示例参考 | 否 |
| `../wuji-technology/mujoco-sim` | `https://github.com/wuji-technology/mujoco-sim.git` | `394d0017c415557c855e555ed279a8cc8ee2e3c1` | 官方 MuJoCo 示例和资产来源参考 | 否；运行资产已在本仓库 |
| `../wuji-technology/robotics-rigid-body-mechanics` | `https://github.com/wuji-technology/robotics-rigid-body-mechanics.git` | `ea52216e94a4d954d1a351cbc0f695d5f2e2e165` | 刚体力学工具参考 | 否 |
| `../wuji-technology/wuji-description` | `https://github.com/wuji-technology/wuji-description.git` | `1407beed7f478f6ba472d1c42bbdee6f4eec8f7f` | Wuji URDF/MJCF 官方模型来源 | 否；当前使用仓库内资产 |
| `../wuji-technology/wuji-mjlab` | `https://github.com/wuji-technology/wuji-mjlab.git` | `8b491528ff36ec72412d0df41e6904f62e71e472` | MJLab 集成参考 | 否 |
| `../wuji-technology/wuji-retargeting` | `https://github.com/wuji-technology/wuji-retargeting.git` | `6eafdb22085f0e29c1d58c62f88f77ae1e971d8c` | 官方重定向算法来源参考 | 否；Python 包路径见下文 |
| `../wuji-technology/wuji-sdk` | `https://github.com/wuji-technology/wuji-sdk.git` | `03750c7e3b334f4dea8ac6ae2c570389bc584d14` | SDK、离线 RetargetSession 来源 | 原始骨架 MCAP 开发时需要参考 |
| `../wuji-technology/wuji-studio` | `https://github.com/wuji-technology/wuji-studio.git` | `049e637ff7ab22763473f231bf21d3129f4756e6` | Studio/录制格式来源 | 否 |
| `../wuji-technology/wujihandpy` | `https://github.com/wuji-technology/wujihandpy.git` | `fdba4e446d85b721cd20dba21d163a62037e8fa6` | Wuji 真机 Python SDK | 仿真否；真机接口才需要 |
| `../wuji-technology/wujihandros2` | `https://github.com/wuji-technology/wujihandros2.git` | `03b1e792e127377b2ad620e8cd78435c441e3eff` | Wuji ROS 2 真机节点 | 仿真否；ROS 2 真机才需要 |

重要边界：相邻源码仓库和 Python 安装包不是一回事。当前离线环境通过 PyPI 安装：

```text
wuji-sdk==2026.7.21
```

代码只在读取原始 `/right_glove/hand_skeleton` MCAP 时惰性导入 `wuji_sdk`，并使用
`HandModel`、`Handedness` 和 `RetargetSession`。首选的已校正
`/joint_states` MCAP 已包含 20 关节结果，不需要重新运行官方重定向，但统一安装
脚本仍会安装 `wuji-sdk`，以保证两种输入都可用。

### 3.3 克隆官方源码的标准命令

从 `<workspace>` 执行：

```bash
mkdir -p wuji-technology
git clone https://github.com/wuji-technology/isaaclab-sim.git wuji-technology/isaaclab-sim
git clone https://github.com/wuji-technology/mujoco-sim.git wuji-technology/mujoco-sim
git clone https://github.com/wuji-technology/robotics-rigid-body-mechanics.git wuji-technology/robotics-rigid-body-mechanics
git clone https://github.com/wuji-technology/wuji-description.git wuji-technology/wuji-description
git clone https://github.com/wuji-technology/wuji-mjlab.git wuji-technology/wuji-mjlab
git clone https://github.com/wuji-technology/wuji-retargeting.git wuji-technology/wuji-retargeting
git clone https://github.com/wuji-technology/wuji-sdk.git wuji-technology/wuji-sdk
git clone https://github.com/wuji-technology/wuji-studio.git wuji-technology/wuji-studio
git clone https://github.com/wuji-technology/wujihandpy.git wuji-technology/wujihandpy
git clone https://github.com/wuji-technology/wujihandros2.git wuji-technology/wujihandros2
```

若只需要尽快复现当前 Headless/Viewer，可先跳过这 10 个 clone；当前案例运行资产
和 PyPI 依赖都由自研仓库与安装脚本提供。完整交接环境则应全部 clone，以保留来源
追溯和后续 SDK/ROS 2 开发能力。

## 4. 新机器从零部署

### 4.1 系统前提

- Linux x86_64；当前验证机为 Linux；
- CPython 3.12.x，不能用 3.10、3.11 或 3.13 创建本项目环境；
- 可访问 GitHub 和 Python 包索引；
- Headless 不需要显示器；
- Viewer 需要有效桌面会话和 OpenGL。SSH 纯终端若没有 DISPLAY/Wayland 转发，
  应先完成 Headless，不要把窗口错误误判成轨迹错误；
- 至少预留约 2 GB，用于 Git 仓库、Python wheel、虚拟环境、测试缓存和录制。

### 4.2 克隆自研仓库

从 `<workspace>` 执行：

```bash
git clone https://github.com/timothijoe/tianji_robotic_project.git
cd tianji_robotic_project
git checkout develop_12_kinematic_branch
git pull --ff-only
```

确认分支和提交：

```bash
git status --short --branch
git log -3 --oneline
```

若要严格复现本文基线而非分支后续版本：

```bash
git checkout 32d4a8df31305bca54da0b6f2b185cc00fa98f62
```

这会进入 detached HEAD，只适合复现。继续开发应回到分支。

### 4.3 创建离线仿真环境

从项目根目录执行：

```bash
python3.12 --version
./scripts/setup_wuji_teleop_env.sh
```

脚本执行：

```text
python3.12 -m venv .venv-wuji-teleop
.venv-wuji-teleop/bin/python -m pip install --upgrade pip setuptools wheel
.venv-wuji-teleop/bin/python -m pip install -e ".[wuji-offline,test]"
```

不要复制旧机器的 `.venv-wuji-teleop`。即使 Python 小版本一致，虚拟环境中的 shebang、
解释器路径和二进制 wheel 仍可能绑定旧路径或旧系统。

确认版本：

```bash
.venv-wuji-teleop/bin/python -m pip show mujoco numpy pytest mcap wuji-sdk
```

基线应包含：

```text
mujoco 3.10.0
numpy 2.5.1
pytest 9.1.1
mcap 1.4.0（项目允许 >=1.3,<2）
wuji-sdk 2026.7.21（项目允许 >=2026.7.21）
```

MuJoCo、NumPy 和 pytest 被精确固定；MCAP 与 Wuji SDK 允许兼容范围。若未来安装到
更高版本后出现差异，应先用上述基线版本复现，再判断是否为依赖升级问题。

### 4.4 复制真实录制数据

当前首选文件：

```text
recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

它是已校正的 `/joint_states` 轨迹：

```text
大小：107726 bytes
SHA-256：8b3afb35cddffe54cc2b804221cf328dd28b6a93f877e773296e79cf5896a6af
```

对应原始右手手套录制：

```text
recordings/wuji/august_02/session_20260802_174440_936.mcap
大小：3736159 bytes
SHA-256：0d8ef1225e8f01854436b421660e071033c816ea715caeb38076fe6436b915a4
```

推荐复制整个被忽略目录，保留其他回放与回归数据。示例从旧机器执行：

```bash
rsync -av --progress \
  /OLD_WORKSPACE/tianji_robotic_project/recordings/ \
  USER@NEW_HOST:/NEW_WORKSPACE/tianji_robotic_project/recordings/
```

也可以只复制上述两个文件，但必须在新机器建立完全相同的相对路径。复制后执行：

```bash
sha256sum \
  recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap \
  recordings/wuji/august_02/session_20260802_174440_936.mcap
```

若 hash 不同，先重新复制，不要调 IK、手势或安全阈值来掩盖数据差异。

## 5. 当前功能究竟做了什么

### 5.1 数据流

首选联合切菜路径：

```text
corrected /joint_states MCAP
  -> load_recorded_guard_cycle
  -> 10 ms 重采样（416 联合样本）
  -> MCP/PIP/DIP 录制尺度整形
  -> 指腹高度与左腕 Z 校准
  -> 左腕连续 +Y 后退 32 mm
  -> 5 个连续手势分段
  -> 右刀 X/Y 跟踪 + 垂直切削波形
  -> MuJoCo 逐样本安全预检
  -> 同时命令右臂、左臂和 20 路 Wuji 手关节
```

原始骨架 MCAP 路径多一层：

```text
/right_glove/hand_skeleton MCAP
  -> OfficialWujiRetargeter
  -> wuji_sdk.RetargetSession
  -> 第一代左手 20 关节
  -> 后续流程同上
```

### 5.2 当前动作和安全语义

- Tianji 右臂持刀，左臂挂载 Wuji 左手；
- 双臂、手和桌面位于同一个 MuJoCo 模型、`MjData` 和时钟；
- 一次录制连续分成五段，不在每刀之间把左臂重置回右侧；
- 刀与手在每个 10 ms tick 同时更新，不再串行执行“手先退、刀再切”；
- 右刀世界 `X` 跟踪四指接触区，最大误差要求不超过 10 mm，当前小于 0.5 mm；
- 侧向间距从 30 mm 开始按 5 mm 增长，先寻找全几何距离至少 20 mm 的候选；
- 若整个首选层无解，才允许全几何距离至少 10 mm；当前数据选择 45 mm 侧向间距，
  最小全几何距离 22 mm，因此没有使用 10 mm 回退；
- PIP 动态幅度为上一版的约 80%；四指 DIP 峰峰值均为 0.35 rad；
- 四个长指最低离桌约 0–5 mm，拇指约 2.98 mm，计划桌面穿透为 0；
- 场景隐藏与当前任务无关的 cube 和 pedestal，但不修改公共 MJCF 和其他任务。

### 5.3 关键实现位置

| 路径 | 责任 |
|---|---|
| `src/twin_sim/tasks/recorded_hand_guarded_chop.py` | 联合任务、手势整形、双臂 IK、分级安全搜索、执行与结果 |
| `src/twin_sim/recorded_hand_guard.py` | MCAP 载入、时间重采样、PIP-led 初级整形 |
| `src/twin_sim/model.py` | MuJoCo 模型载入和关节/执行器契约 |
| `src/twin_sim/paths.py` | 联合场景的仓库内路径 |
| `src/tianji_robotics/simulation/paths.py` | hand-only Wuji MJCF 的仓库内路径 |
| `src/tianji_robotics/wuji_sdk/retargeter.py` | 官方 `wuji_sdk` 惰性适配边界 |
| `robot_assets/mujoco/right_chopping_scene.xml` | 当前双臂联合场景 |
| `robot_assets/mujoco/wuji_hand_standalone/mjcf/left.xml` | hand-only 官方左手模型副本 |
| `scripts/run_recorded_hand_guarded_chop.sh` | 默认 MCAP Viewer 启动入口 |
| `scripts/setup_wuji_teleop_env.sh` | 可重复创建离线环境 |
| `tests/simulation/test_recorded_hand_guard*.py` | 真实 MCAP、幅度、接触、碰撞和同步执行证据 |

## 6. 新机器分阶段验收

所有命令都从 `tianji_robotic_project` 根目录执行。

### 6.1 第零级：文件与解释器

```bash
test -x .venv-wuji-teleop/bin/python
test -f robot_assets/mujoco/right_chopping_scene.xml
test -f robot_assets/mujoco/wuji_hand_standalone/mjcf/left.xml
test -f recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
.venv-wuji-teleop/bin/python -c "import mujoco, numpy, mcap; print(mujoco.__version__, numpy.__version__)"
```

预期最后一行：

```text
3.10.0 2.5.1
```

### 6.2 第一级：快速 CLI 与架构测试

```bash
.venv-wuji-teleop/bin/pytest \
  tests/simulation/test_cli.py \
  tests/architecture \
  -q
```

它验证命令路由、项目包边界和硬件依赖没有泄漏到仿真层。

### 6.3 第二级：真实 MCAP 聚焦回归

```bash
.venv-wuji-teleop/bin/pytest \
  tests/simulation/test_recorded_hand_guarded_chop.py \
  tests/simulation/test_recorded_hand_guarded_chop_preflight.py \
  tests/simulation/test_recorded_hand_guard.py \
  tests/simulation/test_cli.py \
  -q
```

基线结果：

```text
32 passed in 96.55s
```

不同 CPU 的耗时会变化，测试数量和结果不应变化。若真实 MCAP 未复制，部分测试会
显示 skip；这不算完整迁移成功。

### 6.4 第三级：真实 Headless 端到端

```bash
.venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop \
  --headless --final-hold 0 \
  --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

基线输出：

```text
success=True cuts=5 hand_cycles=5 surface_offset_m=0.040 \
min_distance_m=0.022 lateral_spacing_m=0.045..0.045 \
clearance_tier_m=0.020 selected_spacing_m=0.045 depth_mismatch_m=0.000 \
pad_clearance_m=-0.000..0.005 max_penetration_m=0.000000 \
thumb_clearance_m=0.003 reason=-
```

`pad_clearance_m=-0.000` 是小于 1 微米的浮点接触噪声，不是可见穿透；自动化阈值
仍拒绝超过 1 微米的负间隙。

### 6.5 第四级：完整仿真回归

```bash
.venv-wuji-teleop/bin/pytest tests/simulation -q
```

基线结果：

```text
261 passed, 1 warning in 468.28s
```

唯一已知 warning 来自旧 `chop` 测试：39.611 N 超过 30 N 的观察阈值。它不是
Wuji MCAP、刀手碰撞或迁移失败。新增 warning 或失败必须调查。

### 6.6 第五级：Viewer 人工验收

```bash
.venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop \
  --final-hold 60 \
  --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

观察：

- 刀与指垫相对机器人前后距离基本一致；不要用相机画面左右判断机器人坐标；
- 刀与手同时向机器人左侧移动，刀同时上下切削；
- 手指动作克制，PIP 不应大幅蜷曲，DIP 有小幅可见变化；
- 指尖偏手指肚的一侧贴近桌面，整手不能穿过桌面；
- 刀始终位于左手的机器人右侧，且两者间距基本固定。

当前 Viewer 只播放一次约 4.15 秒动作，随后保持最终姿势。用户调整相机时容易
错过动作；可配置循环和相机准备停留已经记录为下一开发项，但本文基线尚未实现。

## 7. 可选功能的迁移

### 7.1 Hand-only MCAP 回放

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-replay \
  recordings/wuji/august_02/session_20260802_162909_764.mcap \
  --headless
```

该命令只加载 Wuji 左手，不加载 Tianji 机械臂。它用于验证手套录制、官方离线
重定向和独立 MuJoCo 后端。

### 7.2 桌面后退手势

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-table-retreat \
  recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap \
  --loops 3
```

这是联合切菜手势的数据来源和独立视觉基线。

### 7.3 真机接口边界

当前仅提供只读/预检接口：

```bash
.venv-wuji-teleop/bin/tianji-robot hardware wuji-sdk preflight TRAJECTORY.npz
```

它不连接设备、不 enable、不发送关节命令。不要因为跨机迁移完成就推断真机已验证。
真机需要单独安装厂商运行时、USB/网络权限和设备配置。

### 7.4 ROS 2

ROS 2 源码位于同仓库 `ros2_ws/src/`，但构建目录被忽略。新机器必须按目标 ROS 2
发行版重新执行 `colcon build`。当前仿真 Python 环境要求 3.12，而已有 ROS 2 环境
可能使用不同 Python；不要把 `.venv-wuji-teleop` 与 ROS 2 overlay 混成一个环境。
细节见 `docs/simulation/ros2_wuji_hand_bridge.md`。

## 8. 常见迁移失败与诊断顺序

### 8.1 `default MCAP does not exist` 或测试被 skip

原因：`recordings/` 被 Git 忽略。

处理：复制录制文件，检查相对路径和 SHA-256。不要把大录制直接提交 Git 来临时解决。

### 8.2 `No module named mcap` 或 `No module named wuji_sdk`

原因：使用了系统 Python、`.venv`，或环境安装不完整。

处理：

```bash
./scripts/setup_wuji_teleop_env.sh
.venv-wuji-teleop/bin/python -m pip show mcap wuji-sdk
```

运行命令时必须显式使用 `.venv-wuji-teleop/bin/...`。

### 8.3 找不到 MuJoCo XML 或 mesh

原因：不在项目根目录运行、clone 不完整、Git LFS/资产缺失，或代码被改成绝对路径。

处理：检查 `robot_assets/` 和 `MarvinCCS/` 是否存在，运行 `git status`，确认
`src/twin_sim/paths.py` 仍从项目根目录构造路径。不要用旧机器的
`/home/linux/august_folder/...` 绝对路径修补。

### 8.4 Viewer 不出现但 Headless 成功

原因：目标机没有桌面显示会话、DISPLAY/Wayland 未导出或 OpenGL 不可用。

处理：先保留 Headless 成功证据；在本地桌面终端运行 Viewer，或正确配置远程图形
转发。不要修改轨迹和 IK。

### 8.5 Headless 指标与基线差异很大

按顺序检查：

1. Git 分支和提交；
2. MCAP SHA-256；
3. Python 和 MuJoCo/NumPy/Wuji SDK 版本；
4. 是否使用了原始 MCAP 而不是已校正 MCAP；
5. 是否修改了 `robot_assets/`；
6. 是否从正确项目根目录运行；
7. 最后才调查算法或平台浮点差异。

不要首先放宽 `minimum_distance_m`、桌面穿透、拇指或指垫阈值。迁移问题应通过还原
输入和环境解决，而不是降低安全检查。

### 8.6 相邻官方仓库存在但代码仍找不到 SDK

原因：clone 源码不会自动安装 Python 包，且当前代码不会把
`../wuji-technology/wuji-sdk` 隐式加入 `PYTHONPATH`。

处理：正常运行 `setup_wuji_teleop_env.sh` 安装 PyPI 包。只有明确开发官方源码时，
才按照官方仓库自己的说明做 editable install；不要在迁移手册中把源码目录注入
全局 `PYTHONPATH`。

### 8.7 新机器出现 `../wuji-description` 等平级目录要求

这是旧布局或新引入的路径耦合。当前标准只允许：

```text
../wuji-technology/wuji-description
```

若功能依赖旧平级路径，应修复代码或文档，而不是在每台机器复制一套重复仓库。

## 9. 迁移验收清单

交接人和接收人应共同确认：

- [ ] 自研仓库位于 `<workspace>/tianji_robotic_project`；
- [ ] checkout 到 `develop_12_kinematic_branch` 或明确记录的提交；
- [ ] 需要完整开发环境时，10 个官方仓库位于 `../wuji-technology/`；
- [ ] 没有依赖 `../wuji-description` 等旧平级副本；
- [ ] `.venv-wuji-teleop` 在新机器本地重建；
- [ ] Python 为 3.12.x；
- [ ] MuJoCo 3.10.0、NumPy 2.5.1；
- [ ] 首选 MCAP 存在且 SHA-256 匹配；
- [ ] 聚焦测试没有 skip，并全部通过；
- [ ] 真实 Headless 返回 `success=True cuts=5 hand_cycles=5`；
- [ ] 使用 20 mm 首选安全层，或明确记录为什么在未来数据上使用 10 mm 回退；
- [ ] 完整 `tests/simulation` 回归通过，只有已知旧力阈值 warning；
- [ ] Viewer 中刀手世界 `X` 对齐、动作同步、指腹贴桌且无穿透；
- [ ] 没有连接或命令真机；若需要真机，另立验收记录。

## 10. 交接时必须保存的证据

建议把下列输出粘贴到新机器的交接记录中：

```bash
git status --short --branch
git rev-parse HEAD
python3.12 --version
.venv-wuji-teleop/bin/python -m pip show mujoco numpy mcap wuji-sdk pytest
sha256sum recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
.venv-wuji-teleop/bin/pytest tests/simulation/test_recorded_hand_guarded_chop_preflight.py -q
.venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop --headless --final-hold 0 \
  --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

不要只写“运行成功”。至少保留 commit、MCAP hash、依赖版本、测试计数和 Headless
指标，后续才能区分代码变化、数据变化和机器环境变化。
