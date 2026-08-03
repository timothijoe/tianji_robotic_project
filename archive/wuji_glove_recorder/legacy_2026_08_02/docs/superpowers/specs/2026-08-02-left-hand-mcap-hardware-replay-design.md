# 左手 MCAP 实体回放设计

日期：2026-08-02

## 目标

新增一个小型、一次性离线轨迹回放工具：读取已完成右手到左手镜像与 retargeting 的 MCAP，经过严格预检后，以受控倍率向已运行的左手 Wuji Hand ROS 2 驱动发布关节位置。

首个验收输入为：

```text
/tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

已只读验证该文件包含 499 条 JSON 编码的 `/joint_states` 消息、每条有 20 个具名左手关节、录制时长约 4.15 秒。该工具只接受此类**转换后的左手关节 MCAP**，拒绝原始右手手套骨架 MCAP。

## 非目标

- 不连接 Wuji Glove，不采集实时手套数据。
- 不直接调用 Wuji Hand SDK，不与 ROS 驱动争夺硬件控制会话。
- 不启动、停止、使能、回零或张开实体手驱动。
- 不自动恢复到开始姿态、零位或任何预设姿态。
- 不把 MuJoCo 回放器改成硬件控制器。

## 已确认的接口与风险

实体手当前通过 ROS 2 驱动控制：

```text
发布: /hand_0/joint_commands  (sensor_msgs/msg/JointState)
订阅: /hand_0/joint_states    (sensor_msgs/msg/JointState)
```

驱动支持 20 元的位置数组，也支持带名称的关节命令。回放程序将始终发布 20 个明确的左手关节名称，避免依赖位置数组的隐式顺序。

驱动源码中的 `joint_lower_limits` / `joint_upper_limits` 明确标注为占位值，且命令回调不会裁剪输入。因此这些参数不能当作硬件安全保护。样例 MCAP 中有多个第 2 关节负值（最低约 `-0.370 rad`），与该占位限位矛盾；这符合左手 MuJoCo 模型的相应范围，但仍要求回放器独立预检，不能把驱动限位当作权威来源。

## 推荐架构

```text
转换后的 *_right_to_left_wuji_hand.mcap
        │
        ▼
纯 Python 文件读取和预检（不初始化 ROS、不发布）
        │  - Topic/schema/names/20 维/有限值/单调时间戳
        │  - 左手 MuJoCo 关节范围验证
        │  - 帧间最大差异验证
        ▼
ROS 2 回放节点（仅 --arm 时创建发布器）
        │  - 等待 /hand_0/joint_states
        │  - 当前姿态 → 第一录制帧的缓入
        │  - 按原始时间戳 × speed 定时发布
        ▼
/hand_0/joint_commands → 既有 Wuji Hand ROS 2 驱动 → 实体左手
```

程序源文件放在 `wuji-glove-recorder/`，以便和 MCAP 转换器、MuJoCo 回放器共享文件格式和测试。提供一个很薄的 POSIX `sh` 启动脚本，显式使用 ROS 系统 Python，并将已有 `wuji-teleop-venv` 的 `mcap` 包加入 `PYTHONPATH`。

这样不需要新增 sudo 安装：系统 Python 已可导入 `rclpy` 和 `numpy`，而 `mcap` 在 `wuji-teleop-venv` 已存在。启动脚本会保留已有 ROS `PYTHONPATH`，而非覆盖它。

## 命令行接口

建议程序名：`replay_left_hand_mcap.py`；建议启动脚本名：`replay_left_hand_mcap.sh`。

```bash
# 默认仅预检，不会初始化 ROS 或向手发布命令
sh replay_left_hand_mcap.sh /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap

# 唯一允许实体动作的显式方式
sh replay_left_hand_mcap.sh --arm \
  /tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap

# 更慢或更快；默认 0.1
sh replay_left_hand_mcap.sh --arm --speed 0.05 <mcap>
sh replay_left_hand_mcap.sh --arm --speed 0.2 <mcap>
```

参数：

- 位置参数 `mcap`：必需；必须存在，后缀为 `.mcap`。
- `--arm`：缺省为 false。缺失时只执行完全离线的预检并退出。
- `--speed FLOAT`：默认 `0.1`；必须在 `(0, 1]`。不允许大于原始速度。
- `--ramp-seconds FLOAT`：默认 `3.0`；必须大于零。启动后从实际当前姿态插值到录制第一帧。
- `--hand-name NAME`：默认 `hand_0`，用于构造 `/<name>/joint_states`、`/<name>/joint_commands`。首版仅文档化此参数，不自动探测其他手。
- `--state-timeout FLOAT`：默认 `5.0`；等待实际状态的最大秒数。
- `--max-step-rad FLOAT`：默认 `0.08`；任意相邻录制帧的单关节差异大于该值则预检失败。该值也用于缓入时的最大单次命令变化，必要时插入更多中间命令。

不提供“跳过预检”“忽略限位”“强制运行”参数。

## 数据校验

预检必须读取整个 MCAP，并拒绝以下任一情形：

1. 没有 `/joint_states` Topic 或没有消息；
2. 消息 JSON 无法解析；
3. `name`、`position` 不存在，长度不是 20，或位置不是有限数；
4. 关节名集合与下列集合不完全一致，或名称重复：

   ```text
   left_finger{1..5}_joint{1..4}
   ```

5. 每帧按名称重排后，任一位置超出左手 MJCF 模型 `mujoco-sim/wuji_hand_description/mjcf/left.xml` 的 joint range；
6. MCAP 日志时间戳不严格递增；
7. 相邻帧任一关节差异超过 `--max-step-rad`；
8. 文件名不含 `_right_to_left_wuji_hand`。这是降低误把原始右手手套 MCAP 交给实体手的二次保护，不能替代内容校验。

预检结果应报告帧数、录制时长、每关节最小/最大值、最大帧间变化、第一帧目标值和输出 Topic。未携带 `--arm` 时，到此退出，退出码为 0。

## 实体执行状态机

仅在预检成功且显式给出 `--arm` 后进入以下状态：

1. **等待状态**：初始化 ROS，订阅 `/<hand>/joint_states`，等待 20 个有限位置。超时后报错退出；不发布命令。
2. **缓入**：以收到的实际状态为起点，线性插值至录制第一帧，持续 `--ramp-seconds`。发布频率为 50 Hz，且分段确保单关节增量不超过 `--max-step-rad`。
3. **回放**：按录制相邻时间戳除以 `--speed` 的实际间隔发布。使用已经按名称规范化的 20 个关节值；每帧发布前再次检查范围和相邻增量，异常则立即停止发布、以非零退出。
4. **结束/中断**：正常到最后一帧、收到 `Ctrl+C`、ROS 状态丢失或出现数据异常时，取消 timer 并销毁节点。程序**不额外发布任何姿态**；实体手保持最后一个已命令姿态。

“停止发布”不是硬件急停。操作人员必须在手旁监看，并保留可用的断电/硬件急停手段。

## 测试与验收

测试不使用真实 ROS 图或硬件。把可测试逻辑分离为纯函数，至少覆盖：

- 具名 20 关节的规范重排；
- 缺关节、重复关节、非有限数据、非单调时间戳和错误文件名的拒绝；
- MJCF 范围内/范围外判定；
- 帧间最大变化判定；
- 缓入插值的首尾、步幅限制；
- `--arm` 缺失时不创建 ROS 节点/发布器的行为；
- 速度与参数范围验证；
- POSIX `sh` 启动脚本的干跑命令构造。

在真实实体手验收时按顺序执行：

1. 启动既有左手 ROS 驱动，确保周围无接触物并有人监看；
2. 运行无 `--arm` 的预检，人工检查范围和首帧差异；
3. 打开 MuJoCo 回放同一轨迹复查；
4. 使用 `--arm --speed 0.05 --ramp-seconds 5` 进行首次低速动作；
5. 只有在观察到所有手指方向、关节顺序和幅度正常后，再考虑提升到默认 0.1。

## 明确决策

- 使用 ROS 2 发布，不使用 SDK 直接控制实体手。
- 默认速度 10%，速度可选但不得超过 1.0。
- 默认只读预检，`--arm` 为必要而非充分条件。
- 结束和 `Ctrl+C` 后停止发布、保持最后姿态，不自动回零。
- 使用 MJCF 关节范围进行独立的第一层软件校验；不将 ROS 驱动的占位限位当作安全保证。
- 不开发任何绕过校验的“强制执行”选项。
