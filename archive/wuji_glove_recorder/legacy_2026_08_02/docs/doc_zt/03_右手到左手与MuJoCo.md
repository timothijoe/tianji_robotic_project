# 03. 右手 Wuji Glove 到左手 Wuji Hand：转换与 MuJoCo 验证

## 1. 总体数据流

```text
Wuji Studio 右手录制 MCAP
  /right_glove/hand_skeleton（21 个点）
        ↓
Y 轴镜像：右手 wrist basis → 左手 wrist basis
        ↓
官方 RetargetSession(HandModel.WujiHand, Handedness.Left)
        ↓
20 维左手机械手关节（NPZ）
        ├─ MuJoCo 左手机械手回放
        └─ /joint_states MCAP（供记录/通用工具使用）
```

所有关节位置单位均为弧度，顺序为五个手指、每指四关节。

## 2. 为什么右手不能直接映射到左手

最初将右手 `r_wrist` 的 21 个点直接交给目标为左手的 retargeter。文件能生成，但 MuJoCo 中食指到小指靠近指尖的关节严重过弯，第三、四关节大量接近限位。

根因是左右 wrist 坐标系的 Y 轴含义相反：

- 右手 Y 正方向指向掌面；
- 左手 Y 正方向指向手背；
- X（径向）和 Z（近端）保持一致。

所以右手转换左手前必须镜像 Y：

```python
mirrored = keypoints.copy()
mirrored[:, 1] *= -1.0
```

这个修复已位于 `wuji-glove-recorder/mcap_to_left_qpos.py` 的 `mirror_right_to_left()`。正确生成文件名含：

```text
_right_to_left_wuji_hand
```

旧的 `*_left_wuji_hand.*` 可能来自未镜像版本，仅用于问题对比，不应用于实体回放。

## 3. 一键转换并打开 MuJoCo

编辑下面脚本顶部唯一常用配置项：

```text
wuji-glove-recorder/visualize_right_glove_to_left.sh
```

```sh
DEFAULT_MCAP_PATH="/绝对路径/Studio录制文件.mcap"
```

然后运行：

```bash
cd ~/catkin_robotic_ws/august_ws/wuji-technology/wuji-glove-recorder
sh visualize_right_glove_to_left.sh
```

该脚本自动完成：

1. 读原始右手 Studio MCAP；
2. 镜像 Y 并调用官方 retargeter；
3. 生成同目录的 `*_right_to_left_wuji_hand.npz`；
4. 生成同目录的 `*_right_to_left_wuji_hand.mcap`；
5. 打开 MuJoCo。

先检查路径但不写文件/不开窗口：

```bash
sh visualize_right_glove_to_left.sh --dry-run
```

脚本内部已固定 SDK 和 MuJoCo Python 环境，无需手动激活。

## 4. 单独运行各环节

### 原始 Studio MCAP → 左手 NPZ

```bash
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python \
  mcap_to_left_qpos.py 原始右手.mcap 输出_right_to_left_wuji_hand.npz
```

### 左手 NPZ → 左手关节 MCAP

```bash
/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python \
  left_qpos_to_mcap.py 输入.npz 输出_right_to_left_wuji_hand.mcap
```

输出 MCAP Topic 为 `/joint_states`，每帧有 20 个 `left_finger*_joint*` 名称。

### NPZ → MuJoCo

```bash
/home/zhoutong/catkin_robotic_ws/august_ws/tianji_robotic_project/.venv/bin/python \
  mujoco_left_replay.py 输入_right_to_left_wuji_hand.npz
```

## 5. MuJoCo 操作与判读

| 按键 | 功能 |
| --- | --- |
| Space | 播放/暂停 |
| ← / → | 暂停状态下逐帧 |
| R | 回到第一帧并暂停 |
| + / - | 调整播放倍速 |
| Esc | 关闭 |

MuJoCo 回放仅写入仿真 `qpos` 并调用 `mj_forward`，不连接 ROS、SDK、手套或实体手。

Studio 手套图和 MuJoCo 机械手图不会外观完全一致：一个是人手/骨架，另一个是机械连杆。验证时看屈伸方向、手指时序和是否异常贴近关节限位；不要以外形逐像素一致作为标准。

## 6. 典型问题

| 现象 | 检查与处理 |
| --- | --- |
| 指尖关节异常过弯 | 确认输出文件名含 `_right_to_left_wuji_hand`，即已镜像 Y |
| Studio 能看手套，不能看左手机械手 | 使用 MuJoCo；`/joint_states` 不是 Studio Hand Skeleton 输入 |
| `sh` 提示 `Illegal option -o pipefail` | 使用当前版本脚本，已改为 POSIX `sh` 兼容 |
| MuJoCo 没有弹窗 | 检查 Tianji `.venv` 与模型 `mujoco-sim/.../left.xml` 是否存在 |

## 7. 文件命名、环境与可重复性

### 必须保留的三类文件

| 类型 | 推荐后缀 | 用途 | 是否可重新生成 |
| --- | --- | --- | --- |
| Studio 原始录制 | `session_*.mcap` | 右手 21 点骨架的原始依据 | 否，必须保留 |
| 左手关节轨迹 | `*_right_to_left_wuji_hand.npz` | MuJoCo/数值分析 | 可以从原始 MCAP 重新生成 |
| 左手关节 MCAP | `*_right_to_left_wuji_hand.mcap` | Studio 通用关节数据/实体回放 | 可以从 NPZ 重新生成 |

### 两个 Python 环境

| 环节 | 解释器 | 原因 |
| --- | --- | --- |
| Studio MCAP 读取、官方 SDK retargeting、MCAP 导出 | `/home/zhoutong/catkin_robotic_ws/august_ws/wuji-teleop-venv/bin/python` | 有 `wuji-sdk`、`mcap` |
| MuJoCo 回放 | `/home/zhoutong/catkin_robotic_ws/august_ws/tianji_robotic_project/.venv/bin/python` | 有 `mujoco 3.10.0` |

一键脚本已固定这些路径。若目录迁移或虚拟环境被删除，先修正脚本中的解释器路径，再运行；不要尝试用任意系统 Python 混跑。

## 8. 转换成功的验收标准

转换完成不等于可上实体手。至少检查：

1. 输出名称含 `_right_to_left_wuji_hand`；
2. MuJoCo 能加载该 NPZ；
3. 手指整体屈伸方向与手套动作相符；
4. 没有某个末端关节长时间贴上限；
5. 预检显示相邻帧变化平滑；
6. 只有上述都满足，才生成/使用实体回放 MCAP。

本次用于实体回放的样例 `session_20260802_174440_936_right_to_left_wuji_hand.mcap` 已通过预检：499 帧、4.150 秒、最大相邻变化 `0.02534 rad`。

## 9. 不要混淆的三个“左手”概念

- **左手骨架坐标系**：右手输入做 Y 镜像后表达出的 left wrist basis；
- **左手 retargeter**：官方 `Handedness.Left` 目标模型；
- **左手实体手**：ROS 驱动报告 `handedness: left` 的物理设备。

三者都必须是左手才可安全衔接。仅修改文件名、仅修改 Studio 模型或仅把 `side=Left` 传给 retargeter，都不能替代右转左的 Y 轴镜像。
