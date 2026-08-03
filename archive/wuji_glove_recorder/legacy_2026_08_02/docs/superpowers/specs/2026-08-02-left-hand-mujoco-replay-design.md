# 左手 MuJoCo 回放器设计

## 目标

以交互式 MuJoCo 窗口回放右手手套录制后得到的左 Wuji Hand 20 关节角序列，直观检查动作与关节方向。

## 输入与模型

- 输入为离线 NPZ，读取 `timestamps_ns` 与 `left_joint_positions_rad`。
- 关节数据必须为有限 `float` 数组，形状 `(N, 20)`，时间戳形状为 `(N,)`。
- 使用仓库已有模型 `mujoco-sim/wuji_hand_description/mjcf/left.xml`。
- 依照 MuJoCo 模型中执行器的关节顺序，将每帧 20 个目标角写入仿真；不使用真实时间动力学控制器。

## 窗口交互

- 空格：播放或暂停。
- 左/右方向键：暂停时向前或向后单帧。
- `R`：回到第一帧并暂停。
- `+` / `-`：播放倍率在 0.25×、0.5×、1×、2×、4× 间调整。
- `Esc`：关闭窗口。
- 窗口标题/控制台显示当前帧、总帧数、录制时间与倍率。

## 安全边界

- 工具只导入 `mujoco`、`numpy` 与标准库。
- 不导入 ROS 或 Wuji SDK，不扫描、连接、使能或控制任何手套或机械手。
- 工具只改变 MuJoCo `MjData.qpos` 并调用 `mj_forward`；不会调用物理步进或写入输入数据。

## 结构

- `wuji-glove-recorder/mujoco_left_replay.py`：命令行入口、NPZ 校验、执行器顺序解析、键盘控制与回放循环。
- `wuji-glove-recorder/tests/test_mujoco_left_replay.py`：无 GUI 单元测试，覆盖输入校验与帧索引计算。
- `wuji-glove-recorder/requirements.txt`：加入 MuJoCo 运行时依赖。
- `wuji-glove-recorder/README.md`：启动命令和快捷键。

## 验证

- 测试合法 `(N, 20)` 输入、错误形状、非有限数值和循环帧索引。
- 使用 `session_20260802_162909_764_left_wuji_hand.npz` 启动窗口，确认左手模型加载并能播放、暂停、单帧和调速。
- 静态扫描确认程序不含 ROS、Wuji SDK 或实体手控制调用。
