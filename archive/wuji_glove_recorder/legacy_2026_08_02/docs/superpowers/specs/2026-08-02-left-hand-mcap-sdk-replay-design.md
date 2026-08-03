# 左手 MCAP SDK 实体回放设计

日期：2026-08-02

## 目标

新增独立的官方 SDK 直控回放工具。它读取已经完成右手到左手转换的 MCAP，将其中 20 个左手关节值以受控速度发送给第一代左手 Wuji Hand，不使用 ROS Topic 作为控制通道。

首个验收输入：

```text
/tmp/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

该文件已验证包含 499 帧、约 4.150 秒、20 个具名左手 `/joint_states`，并通过现有 MJCF 范围和帧间变化预检。

## 非目标

- 不删除或修改现有 ROS 回放器；ROS 版与 SDK 版分别保留。
- 不连接 Wuji Glove，不采集实时手套数据。
- 不从原始 `/right_glove/hand_skeleton` 直接控制实体手。
- 不启动、停止或配置 ROS 驱动；SDK 模式只检测其是否仍在运行。
- 不提供跳过预检、忽略限位或强制连接的参数。

## 控制权规则

第一代 Wuji Hand 不能由 ROS 驱动和独立 SDK 控制器同时占用。

SDK 回放器在 `--arm` 前必须检查：

1. `/hand_0/wujihand_driver` 不在 ROS 图中；
2. 没有名为 `wujihand_driver_node` 的进程；
3. 没有其他已知 SDK 回放器进程。

任一检查失败即拒绝启动并打印“先停止 ROS 驱动后再运行 SDK 回放”。检查本身不停止任何进程，避免脚本擅自终止用户的控制会话。

## 命令接口

新文件：

```text
wuji-glove-recorder/replay_left_hand_mcap_sdk.py
wuji-glove-recorder/replay_left_hand_mcap_sdk.sh
```

```bash
# 只读预检；不会扫描、连接、使能或移动实体手
sh replay_left_hand_mcap_sdk.sh <mcap>

# 首次 SDK 实体回放
sh replay_left_hand_mcap_sdk.sh --arm --speed 0.05 --ramp-seconds 5 <mcap>
```

参数与 ROS 回放器保持一致：

- `mcap`：必须存在、后缀 `.mcap`，且名称包含 `_right_to_left_wuji_hand`；
- `--arm`：缺省 false；没有该参数时只读预检后退出；
- `--speed`：默认 `0.1`，范围 `(0, 1]`；
- `--ramp-seconds`：默认 `3.0`，必须大于零；
- `--max-step-rad`：默认 `0.08`，必须大于零；
- `--effort-limit`：默认 `1.5 A`，范围 `(0, 1.5]`；
- `--cutoff-hz`：默认 `5.0 Hz`，范围 `(0, 20]`。

不提供 `--hand-name`、`--state-timeout` 等 ROS 专用参数。

## SDK 执行流程

1. 调用现有纯 Python `load_validated_trajectory()`，完成全部文件、名称、MJCF 限位和帧间变化检查；未带 `--arm` 时立刻报告并退出。
2. 带 `--arm` 时执行控制权检查；失败则退出，不扫描 USB。
3. `SdkManager.instance().scan()` 找出 `DeviceType.WujiHand`；要求恰好一只候选手，多个或零个均拒绝。
4. 使用候选设备 SN 调用 `manager.connect()`，确认 `hand.handedness_name() == "Left"`；否则断开并退出。
5. 读取 `hand.read_joint_state()`，要求有 20 个有限实际位置；设置 `hand.set_all_effort_limit(effort_limit)` 并 `hand.enable()`。
6. 在 `hand.realtime_controller(LowPass(cutoff_hz=cutoff_hz))` 中创建 `hand.joint_command().publish()`。
7. 以 100 Hz 将 SDK 实际位置缓入第一帧；缓入步数同时满足 `ramp_seconds` 与 `max_step_rad`。
8. 按 MCAP 时间戳/`speed` 调度轨迹。每个间隔用 100 Hz 线性插值填充，确保 SDK command stream 连续、任意命令增量不超过 `max_step_rad`。
9. 每个发送帧调用：

   ```python
   publisher.send([JointCommand(position, 0.0, 0.0) for position in target])
   ```

10. 正常结束、`Ctrl+C` 或异常时依序关闭 publisher、退出 realtime controller、`hand.disable()`、`manager.disconnect_all()`。不发送零位/回零命令。

## 错误处理和安全语义

- ROS/SDK 控制权冲突：拒绝，零硬件动作。
- 没有设备、多个设备、设备不是 left：拒绝，断开所有已连接设备。
- SDK 状态/关节读数异常：拒绝或停止，禁用并断开。
- 实时发送中任何 NaN、越界、增量越限或 SDK 异常：立即停止发送，清理并禁用。
- 用户 `Ctrl+C`：停止发送，清理并禁用；它仍不是物理急停。
- SDK 版退出会禁用实体手，这是与 ROS 版“保持最后目标”不同的刻意设计；操作员需要理解退出后手不再由控制器保持。

## 测试策略

不在自动化测试中扫描、连接、使能或控制硬件。

将 SDK 生命周期封装成可注入的 adapter，使用 fake manager/hand/controller/publisher 测试：

- `--arm` 缺失时绝不调用 SDK manager；
- 检测到 ROS 驱动时拒绝且不扫描；
- 右手/多设备/无设备拒绝并清理；
- 发送的每个命令恰为 20 个 `JointCommand`；
- 缓入和插值满足最大变化量；
- 成功、KeyboardInterrupt、SDK 异常均关闭 publisher、禁用、断开；
- 启动脚本可由 POSIX `sh` 运行，并复用 `wuji-teleop-venv`。

## 实体验收顺序

1. 停止 ROS driver，并以 `pgrep`/`ros2 node list` 确认没有 driver；
2. 空闲环境运行 SDK 版无 `--arm` 预检；
3. 清空手周围，保持人工监看和急停；
4. 首次执行 `--arm --speed 0.05 --ramp-seconds 5`；
5. 观察到正确方向和幅度后，逐步提升至 `0.1`、`0.2`；
6. 每次退出后确认 SDK 已禁用手，后续若要重用 ROS，重新启动 ROS driver。
