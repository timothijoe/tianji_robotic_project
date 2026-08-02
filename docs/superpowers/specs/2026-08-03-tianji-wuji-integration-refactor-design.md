# Tianji 与 Wuji 一体化工程重构设计

日期：2026-08-03

## 1. 目标

将当前分散在 `tianji_robotic_project`、`wuji-technology/wuji-glove-recorder`、
`wuji-technology/doc_zt`、`wuji-record-data` 和 `wuji-teleop-venv` 中的自研实现，
重构为由 `tianji_robotic_project` 单一 Git 仓库维护的机器人软件工程。

首个必须完整复现的功能是离线 Wuji 手套链路：读取 Wuji Studio 录制的右手
MCAP，转换到左手腕坐标系，使用官方 `wuji_sdk.RetargetSession` 生成第一代左手
20 关节轨迹，严格校验后在 Tianji 项目的 MuJoCo 整机模型中回放。

工程同时保留三类执行方式的清晰边界：MuJoCo 仿真、Wuji SDK 实体手控制、ROS 2
实体手控制。首轮只实际运行仿真；真机代码保留和重构接口，但不会连接或驱动设备。

## 2. 仓库所有权与外部依赖

### 2.1 Tianji 仓库管理的内容

以下内容全部归入 `tianji_robotic_project`：

- 天机机械臂的通用模型、运动学、轨迹、仿真和实体 SDK 适配；
- Wuji 灵巧手的通用模型、重定向、轨迹、仿真、SDK 适配和 ROS 2 适配；
- 原 `wuji-glove-recorder` 的自研源码、测试和操作文档；
- 原 `real_robot_debug` 中仍有效的自研逻辑；
- 我们开发的 ROS 2 bridge、node、launch 和参数；
- 可重建的环境声明、安装脚本和验证命令。

### 2.2 官方 Wuji 仓库

`wuji-technology` 只保留具有独立 `.git` 且 remote 指向 Wuji Technology 官方
GitHub 组织的仓库，包括 `wuji-sdk`、`wujihandpy`、`wujihandros2`、
`wuji-description`、`wuji-retargeting`、`wuji-mjlab`、`mujoco-sim`、
`isaaclab-sim` 和 `wuji-studio`。这些仓库保持现有目录、历史和 remote，不复制到
Tianji 仓库，也不在本轮修改。

默认相邻布局为：

```text
workspace/
├── tianji_robotic_project/
└── wuji-technology/
    ├── wuji-sdk/
    ├── wujihandpy/
    ├── wujihandros2/
    └── ...
```

正式运行优先导入已安装的官方 Python 包。开发环境可以通过显式配置定位相邻官方
源码，但业务模块不得在导入时隐式修改 `sys.path`。缺少依赖时，错误信息必须列出
缺失包、用途和官方来源。

### 2.3 数据与环境

原 `wuji-record-data` 移入 `tianji_robotic_project/recordings/wuji/`。整个
`recordings/` 由 Git 忽略，不提交大体积或现场录制数据。自动化测试只使用
`testdata/wuji/` 中小型、脱敏、可提交的固定样例，或由测试即时生成的 MCAP。

原 `wuji-teleop-venv` 不直接搬运，因为其 `pyvenv.cfg` 含旧机器绝对路径且当前
环境不可用。项目内重新创建 `.venv-wuji-teleop/`，由 Git 忽略；依赖版本和创建
方法由仓库内声明与脚本管理。

## 3. 顶层结构

```text
tianji_robotic_project/
├── src/
│   └── tianji_robotics/
├── tests/
├── scripts/
├── docs/
├── recordings/wuji/
├── testdata/wuji/
├── robot_assets/
├── ros2_ws/src/
├── examples/
├── archive/
├── .venv/
├── .venv-wuji-teleop/
├── pyproject.toml
└── README.md
```

`src/` 是安装型源码布局，`tianji_robotics` 是唯一主 Python 包。`recordings/`、
环境、文档、脚本、ROS 工作区和资产不会误入 Python 包。

## 4. Python 包结构

```text
src/tianji_robotics/
├── tianji_arm/
│   ├── interface.py
│   ├── state.py
│   ├── limits.py
│   ├── kinematics.py
│   └── trajectory.py
├── wuji_hand/
│   ├── interface.py
│   ├── state.py
│   ├── names.py
│   ├── trajectory.py
│   ├── transforms.py
│   ├── retargeting.py
│   └── validation.py
├── simulation/
│   ├── world.py
│   ├── tianji_arm.py
│   ├── wuji_hand.py
│   ├── marvin_robot.py
│   └── tasks/
├── hardware/
│   ├── safety.py
│   ├── tianji_arm/
│   └── wuji_hand/
├── data/
│   ├── mcap.py
│   └── npz.py
├── workflows/
│   ├── wuji_glove_replay.py
│   ├── hand_teleop.py
│   └── chop.py
└── cli.py
```

### 4.1 设备领域

`tianji_arm` 和 `wuji_hand` 定义设备能力、状态、顺序、单位、限位和纯算法。它们
不导入 MuJoCo、实体 SDK 或 ROS 2。Wuji Hand 的唯一权威顺序是
`left_finger1_joint1` 至 `left_finger5_joint4`，按手指优先排列，共 20 个弧度值。

### 4.2 仿真

`simulation` 只依赖通用领域模块、NumPy 和 MuJoCo。它实现机械臂、灵巧手和
Marvin 整机装配，并承载只适用于仿真的任务循环。该目录禁止导入 Tianji 实体
SDK、`wujihandpy`、`wuji_sdk` 的设备连接接口或 `rclpy`。

官方 `wuji_sdk.RetargetSession` 是无硬件重定向算法，放在硬件连接边界之外，
通过 `wuji_hand.retargeting.Retargeter` 协议延迟加载。仅创建 retarget session
不得发现或连接设备。

### 4.3 真机

`hardware/tianji_arm` 承接原 `real_robot_debug` 中的连接、反馈、控制模式和发送
逻辑。`hardware/wuji_hand` 保留 Wuji SDK 的手套与实体手适配。模块导入和对象
构造均不得连接设备；连接、使能和动作必须是分开的显式调用。

原调试脚本中的纯轨迹与 IK 进入领域模块，任务编排进入 `workflows`，人工入口进入
统一 CLI。重复实现和 `bak.py` 进入 `archive/real_robot_debug`，不再作为活跃代码。

### 4.4 ROS 2

ROS 2 仍属于同一个 `tianji_robotic_project` Git 仓库，但使用标准工作区：

```text
ros2_ws/src/
├── tianji_arm_bridge/
├── wujihand_bridge/
└── twin_wuji_sim/
```

ROS 2 包负责 node、message、topic、parameter 和 launch。可测试的转换、名称映射、
轨迹校验和安全规则复用 `tianji_robotics`，不复制算法。官方 `wujihandros2` 保持
在外部官方仓库，通过安装或明确的工作区配置提供依赖。

## 5. 通用接口

领域层至少定义以下能力契约：

```python
class SkeletonSource(Protocol):
    def frames(self) -> Iterator[SkeletonFrame]: ...

class Retargeter(Protocol):
    def step(self, keypoints_m: np.ndarray) -> np.ndarray: ...

class WujiHandBackend(Protocol):
    def read_position_rad(self) -> np.ndarray: ...
    def command_position_rad(self, target: np.ndarray) -> None: ...
    def close(self) -> None: ...
```

实体后端另外实现显式的 `connect()`、`arm()` 和 `disarm()` 生命周期；仿真后端
不伪装这些危险操作。工作流依赖上述能力，不直接调用 USB、ROS Topic 或 MuJoCo
数组。

`SkeletonFrame` 包含时间戳、frame id、手侧和 `(21, 3)` 米制关键点。
`HandTrajectory` 包含严格递增的纳秒时间戳、`(N, 20)` 弧度目标、关节名称和来源
元数据。数组在边界处复制，校验通过后才可交给后端。

## 6. 离线 Wuji 仿真数据流

```text
recordings/wuji/*.mcap
  → StudioMcapSkeletonSource
  → mirror_right_to_left（Y *= -1）
  → OfficialWujiRetargeter（WujiHand + Handedness.Left）
  → HandTrajectory
  → 时间戳、有限性、关节范围、帧间变化预检
  → MujocoWujiHand
  → Headless 验证或 Viewer 回放
```

原始 MCAP topic 为 `/right_glove/hand_skeleton`，每帧必须含 21 个有限 XYZ 点。
右转左必须在 retargeting 前镜像腕坐标 Y 轴。重定向结果必须恰好为 20 个有限弧度
值。空文件、topic 缺失、时间戳不递增、越界或变化过大均拒绝执行，并报告具体帧与
关节。

首轮继续支持将 `HandTrajectory` 写为 NPZ 和标准 `/joint_states` MCAP，但文件
格式转换不连接设备。

## 7. 命令行

统一入口使用显式运行域，避免仿真误切真机：

```bash
tianji-robot sim wuji-replay recordings/wuji/session.mcap --headless
tianji-robot sim wuji-replay recordings/wuji/session.mcap
tianji-robot sim guarded-chop

tianji-robot hardware wuji-sdk replay trajectory.mcap
tianji-robot hardware wuji-sdk replay trajectory.mcap --arm

tianji-robot hardware tianji-arm chop
tianji-robot hardware tianji-arm chop --execute

ros2 launch wujihand_bridge ...
ros2 launch twin_wuji_sim ...
```

`sim` 命令没有真机参数，也不能探测设备后切换后端。`hardware` 默认只进行文件、
配置和控制权预检。Wuji 使用 `--arm`，天机机械臂使用 `--execute`，保持与既有
安全语义兼容。

迁移期保留 `twin_sim` 包和 `twin-sim` 命令作为薄兼容层；内部转发到
`tianji_robotics`。所有调用和测试迁移后再移除兼容层。

## 8. 真机安全规则

- 模块导入、CLI `--help`、配置解析和默认预检不得连接设备；
- SDK 连接、设备使能和轨迹执行是三个显式阶段；
- Wuji SDK 与 ROS 2 对同一实体手的控制权互斥；
- 轨迹必须先通过与仿真相同的名称、有限性、限位和时序校验；
- 发送层另外执行速度、单步变化、状态新鲜度和设备侧别检查；
- 退出、异常和 `Ctrl+C` 的 disable/disconnect 语义必须在后端中明确测试；
- 仿真命令永远没有隐式真机路径；
- 首轮实现与验证不得连接任何实体设备。

## 9. 测试策略

### 9.1 纯单元测试

- 右手到左手 Y 轴镜像不修改输入；
- MCAP JSON 解析、topic 过滤和错误帧定位；
- 21 点输入与 20 关节输出契约；
- 时间戳、NaN/Inf、关节名称、范围和帧间变化校验；
- NPZ 与 JointState MCAP 往返；
- CLI 的仿真与真机参数不能混用。

### 9.2 适配器契约测试

- fake retargeter 验证完整离线管线，不要求安装官方 SDK；
- 官方 retargeter 可用时运行可选集成测试，确认输出形状和有限性；
- fake Tianji SDK、fake Wuji SDK 和 fake ROS 验证未显式授权时零连接、零命令；
- 仿真后端和实体后端共享同一关节顺序与轨迹校验契约。

### 9.3 本地集成测试

- 使用精简 MCAP 完成原始骨架到 `HandTrajectory`；
- 使用真实样例完成官方 retargeting；
- 在 Headless MuJoCo 中执行完整轨迹，确认仿真时间推进、目标有限且最终状态有效；
- Viewer 作为人工验收，不作为 CI 必需条件。

### 9.4 架构边界测试

静态扫描确保 `simulation` 不导入实体 SDK 或 ROS，核心领域不依赖后端，脚本不
承载控制算法，真机模块导入不产生连接副作用。

## 10. 迁移顺序

1. 建立 `tianji_robotics` 包、兼容层、领域类型与架构边界测试；
2. 迁移 Wuji MCAP、镜像、重定向、轨迹校验和文件格式；
3. 接入现有 `SimWujiHand` 和 Marvin MuJoCo 模型，完成 Headless 与 Viewer 回放；
4. 迁移 Wuji SDK 与 ROS 2 真机接口，保持默认无动作；
5. 拆分 `real_robot_debug`，迁移 Tianji SDK 后端与任务；
6. 迁移文档、录制目录和可重建环境；
7. 完整测试后，将 `wuji-technology` 中自研目录移出，使其只保留官方仓库；
8. 所有使用方迁移后清理 `twin_sim` 兼容层和废弃入口。

对外部数据和旧实现使用移动或归档而非直接删除；任何不可恢复的清理都在独立验证
后进行。

## 11. 首轮验收标准

- `tianji_robotic_project` 可独立找到相邻官方依赖并给出可操作的缺失提示；
- 新建 Wuji teleop 环境可由仓库声明重复创建；
- 一条命令读取现有原始右手 MCAP，经镜像和官方重定向后在 Tianji MuJoCo 中完成
  Headless 回放；
- 同一命令可打开 Viewer 供人工观察；
- 输出轨迹可保存为 NPZ 和 JointState MCAP；
- 自动化测试覆盖转换、验证、仿真和零硬件副作用；
- SDK 与 ROS 2 真机路径有独立目录、接口、预检入口和安全门；
- `recordings/`、`.venv/` 与 `.venv-wuji-teleop/` 均被 Git 忽略；
- 官方 Wuji Git 仓库的内容、remote 和历史不被修改；
- 现有 Tianji 仿真测试与主要 CLI 行为保持通过。
