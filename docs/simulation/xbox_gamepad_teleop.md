# Xbox 游戏手柄 MuJoCo 遥操作

## 当前实现

这是一个仅限 MuJoCo 的连续笛卡尔速度遥操作入口；它不会导入、连接或控制真机 SDK。

- 手柄：系统已识别的 Microsoft Xbox 360 Controller。
- 稳定设备路径：`/dev/input/by-id/usb-Microsoft_Xbox360_For_Windows-joystick`。
- 实现：`src/twin_sim/gamepad_teleop.py`。
- CLI 注册：`src/twin_sim/cli.py` 的 `gamepad-teleop` 子命令。
- 离线输入测试：`tests/simulation/test_gamepad_teleop.py`。

启动命令：

```bash
PYTHONPATH=src .venv-wuji-teleop/bin/python -m twin_sim.cli gamepad-teleop \
  --speed-mm-s 100 \
  --workspace-radius-mm 350
```

默认速度为 `100 mm/s`；可在仿真中提高到最多 `300 mm/s`。

## 当前控制映射

| 手柄输入 | 仿真行为 |
| --- | --- |
| 按住 RB（右上肩键） | deadman：允许移动；松开即停止 |
| 左摇杆左右 | 基座坐标系 X 连续速度 |
| 左摇杆上下 | 基座坐标系 Y 连续速度 |
| 右摇杆上下 | 基座坐标系 Z 连续速度 |
| Start | 退出遥操作 |

输入采用 0.15 死区。每次启动会读取一次仿真 TCP，并将后续目标限制在该点每个坐标轴 `+/-350 mm` 的固定盒内。越界目标会被夹紧；IK 无解时保持上一有效目标。

## 重要边界

- 当前只控制 MuJoCo 的右臂 TCP 平移，保持初始工具姿态。
- 当前不控制腕部姿态，也不控制 Wuji 手指。
- 此处的 TCP 边界只是仿真目标边界，不是实体机器人碰撞安全证明。
- 真机遥操作必须作为独立工作：重做控制频率、碰撞/力限制、急停和硬件使能审查，不能直接复用本入口。

## 后续手柄扩展建议

### 手腕/工具姿态

保持左摇杆和右摇杆上下的平移映射不变，用右摇杆左右控制工具 yaw；用 D-pad 上下控制 pitch，D-pad 左右控制 roll。姿态速度应独立限幅，例如 `30 deg/s`，并继续要求按住 RB。

实现时应将平移和小角度旋转积分为目标 `4x4` TCP pose，交给同一个 MuJoCo IK 求解器；若 IK 失败则保持上一次有效 pose。这样是连续姿态遥操作，而不是突然跳变关节角。

### Wuji 手指

建议先用“协同动作”而不是 20 个关节逐个映射：

| 手柄输入 | 建议手部协同 |
| --- | --- |
| LT | 渐进张开 |
| RT | 渐进闭合/抓握 |
| A | 切换为预抓取姿态 |
| B | 切换为张开安全姿态 |
| X | 切换为猫爪护手姿态 |
| Y | 切换为精细捏取姿态 |

扳机应产生连续 `0..1` 的握力/闭合协同系数，并在 MuJoCo 中插值到预先校验过的 20 关节姿态；这比直接把手柄轴映射为各手指关节更稳定，也更适合人类干预 RL 的高层动作接口。

在仿真中确认后，可以增加一个手柄模式切换键：默认是臂平移模式，按住 LB 时右摇杆与 D-pad 改为腕部姿态；手指协同始终由 LT/RT 和 A/B/X/Y 控制。
