# 当前仿真版本复现与智能体交接

本文是当前 MuJoCo 仿真版本的首要交接入口。所有命令均从克隆后的仓库根目录运行，不依赖
原开发机目录。历史设计和排障过程见
[guarded chopping development log](guarded_chopping_development_log.md)。

## 0. 2026-08-05 当前首要交接：录制手势联动切菜

本节覆盖下方保留的旧 `guarded-chop` 摘要。当前用户验收的是独立命令
`recorded-hand-guarded-chop`；不要把它与旧猫爪任务混为一谈，也不要为了修改
新案例而改变旧任务。

### 动机与动作语义

目标不是逐字重放手套数据，而是借用真实录制的时序构造可信的切菜护手动作：右臂
从机器人自身右侧向左逐刀移动；Wuji 左手安装在天机左臂末端，始终位于刀的机器人
左侧；刀和手只与同一砧板平面交互；左腕连续小幅后退，不能每刀后返回；长指 MCP
尽量伸直、PIP 主屈曲、远节近似垂直向下，同时保留食指独立、中指/无名指相关、
小指跟随和拇指离桌的明显动作。

机器人自身左侧由双臂基座确认是世界 `+Y`，不是 Viewer 画面左侧。所有方向判断
必须使用机器人坐标，不能根据相机方位推断。

### 当前数据流与实现

- 默认 MCAP 是
  `recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap`；
- 原始 499 帧、4.150116 秒录制重采样为 416 个 10 ms 联合样本；一次录制按时间
  连续分成 5 段，每段与一刀的下降/回升波形在同一时钟并行执行，不循环五遍，
  也没有左臂向右 RESET；
- 左腕沿 `+Y` 移动 32 mm，分配量由四个指垫真实运动自动补偿；右刀逐样本跟踪
  最靠刀长指 pad 的 `+Y` 位置，使 pad 与 `right_blade_edge_bot` 保持 30 mm
  侧向间距。掌部沿 `X` 使用 `-180 mm` 纵向偏置以满足整手 20 mm 安全包络；
- 长指 MCP 不超过 0.30 rad，PIP 至少多屈曲 0.25 rad；RETREAT/HOLD 逐帧搜索
  DIP，使 link4 局部 `+Z` 接近世界 `-Z`，PREPARE 最后 0.3 秒平滑进入压持；
- 新任务只在自己的 MuJoCo 实例中隐藏并禁用 guarded cube、两个 pick pedestal、
  pick cube 和目标 marker；公共 MJCF、旧切菜与 pick-place 均不受影响；
- 右刀接触抬高后的砧板顶面，不再使用 cube 顶面。

### 最新实测证据

真实 MCAP 的 Headless 与 Viewer 均完成 `5/5` 刀和 `5/5` 连续分段：

| 指标 | 当前值 | 约束 |
|---|---:|---:|
| 桌面抬高 | 40 mm | 最低可行候选 |
| 左腕总后退 | 32 mm | 25–40 mm |
| 指垫—刀刃侧向距离 | 30.0–30.0 mm | 27–33 mm |
| 最小刀手距离 | 32 mm | ≥20 mm |
| 最大手部桌面穿透 | 0 mm | ≤0.5 mm |
| 拇指最小离桌 | 31.7 mm | ≥10 mm |
| 指垫最差单步向刀侧波动 | 0.481 mm | ≤0.5 mm |
| 四指净后退 | 15.1–19.4 mm | 均向机器人左侧 |

食指至小指最大远节偏角为 `7.63° / 1.02° / 2.86° / 5.64°`。PIP 峰峰值为
`0.385 / 0.261 / 0.310 / 0.358 rad`，DIP 为
`1.141 / 0.831 / 0.903 / 1.323 rad`。同步实现的聚焦回归为
`31 passed in 67.55s`，真实 Headless 命令返回 0 并完成 `5/5` 同步周期。

### 运行与代码入口

```bash
# Viewer
./scripts/run_recorded_hand_guarded_chop.sh

# Headless
.venv-wuji-teleop/bin/twin-sim recorded-hand-guarded-chop \
  --headless --final-hold 0 \
  --hand-mcap recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap
```

| 路径 | 职责 |
|---|---|
| `src/twin_sim/recorded_hand_guard.py` | MCAP 适配、相对掌轨迹、MCP→PIP 初级整形 |
| `src/twin_sim/tasks/recorded_hand_guarded_chop.py` | 纯桌面场景、同步刀/腕/手规划、DIP 求解、腕部补偿、逐样本距离预检 |
| `src/twin_sim/raised_work_surface.py` | 砧板及关联对象的可恢复高度偏移 |
| `tests/simulation/test_recorded_hand_guard*.py` | 真实录制、解剖约束、指垫、碰撞和执行证据 |
| `docs/simulation/recorded_hand_guarded_chop.md` | 操作者命令和安全说明 |
| `docs/superpowers/specs/2026-08-04-recorded-hand-guarded-chop-design.md` | 完整设计约束 |

### 已知问题与后续建议

- DIP 峰峰值最高 1.323 rad，动作明显但可能显得弹动；若需收敛，应增加时间连续性
  代价，同时保持 15° 远节约束，不能直接删除方向测试；
- DIP 使用 61 点逐帧网格搜索，可靠但增加 Viewer 启动预检时间；可改解析一维旋转或
  缓存，但必须保持数值证据一致；
- 当前“压持”对象是砧板平面，没有可移动食材，也不是力控；0 穿透不能证明真实压力；
- 结果主要报告计划几何指标。进入真机前必须补运行期力/位置偏差监控；
- 30 mm 侧向不等于 3-D 欧氏距离；刀具上下运动时完整几何距离会变化，因此两项
  指标必须继续分别验证，不能只保留侧向测试；
- MCAP 被 Git 忽略，新机器缺少默认文件时无法做真实验收；
- 从未连接 Tianji/Wuji 真机、官方物理 SDK 或 ROS 2 硬件节点，结论仅限仿真；
- 下方“5 刀、4 次猫爪倒手”只描述旧 `guarded-chop`，不是当前录制联动任务。

## 1. 当前版本摘要（旧 guarded-chop 基线）

当前默认演示是 Marvin 双臂配合 Wuji Hand 的平面切菜动作：右臂持刀完成 5 刀，左臂
和 20 自由度左手完成 4 次猫爪倒手。第 1、3 刀后左臂不动、手指回缩；第 2、4 刀后
左臂回退、手指沿同一条轨迹反向舒张。回缩与舒张各持续 2.5 秒。

当前版本还支持：

- 无窗口运行和完整 pytest 回归；
- 正常 1× Viewer；
- 正常速度在线运行并录制完整 MuJoCo 状态；
- 不重新运行控制器的独立 2× 状态回放；
- 右刀规划/实际轨迹及中央指垫实际轨迹 marker；
- 保留 object 回归场景、左手独立演示、抓取放置和 ROS 2 仿真桥接专项入口。

默认 guarded-chop 脚本只运行 MuJoCo，不连接实体机械臂、实体 Wuji Hand 或硬件 ROS
驱动。

## 2. 新机器环境安装

### 前提

- Linux；已验证环境使用 CPython 3.12.x；
- 仓库完整 clone，并 checkout 到包含本文件的提交或后续提交；
- 至少为依赖和虚拟环境预留约 2 GB 空间；
- 只有打开交互 Viewer 时才需要桌面显示会话和 OpenGL。

Python 3.12 可以来自系统包、pyenv 或 conda，但下面命令中的 `python3.12` 必须实际指向
3.12.x。不要使用 3.10、3.11 或 3.13 创建项目 `.venv`。

```bash
python3.12 --version
python3.12 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-sim.lock
.venv/bin/python -m pip install -e .
```

确认固定运行库：

```bash
.venv/bin/python -c "import mujoco, numpy; print(mujoco.__version__, numpy.__version__)"
```

当前预期输出为：

```text
3.10.0 2.5.1
```

不要复制另一台机器的 `.venv`。虚拟环境包含绝对解释器路径，应在每台机器本地重建。

## 3. 分层验证

### 第 1 层：仓库保护文件

```bash
sha256sum --check docs/simulation/protected-files.sha256
```

清单中的实体机器人和厂商 SDK 文件都应报告 `OK`。如果失败，先确认差异来源，不要用新
哈希掩盖未知改动。

### 第 2 层：快速导入和无窗口 smoke

```bash
.venv/bin/python -c "import mujoco, numpy, twin_sim; print('imports ok')"
.venv/bin/twin-sim hand-demo --headless
.venv/bin/twin-sim guarded-chop --headless --final-hold 0
```

guarded-chop 应报告 `success=True`、`cuts=5`、`shifts=4`、
`total_shift_m=0.080`，且 `min_distance_m` 不低于 0.02 m。

### 第 3 层：完整自动回归

```bash
.venv/bin/python -m pytest -q
```

要求零失败。测试数量会随后续开发增长，不应把某个固定数量作为永久通过条件。当前版本
存在一条已知的 39.611 N 接触力观测 warning；它来自普通 chop 测试，不是 guarded-chop
刀手碰撞。出现新的 warning 时仍需调查。

### 第 4 层：生成无窗口录制

没有桌面环境时仍可生成 latest：

```bash
.venv/bin/twin-sim guarded-chop \
  --headless \
  --final-hold 0 \
  --record
```

## 4. 切菜、录制与回放入口

| 入口 | 在线仿真 | 保存 NPZ | 状态回放 |
|---|---:|---:|---:|
| `./scripts/run_guarded_chop.sh` | 1× | 否 | 否 |
| `./scripts/run_guarded_chop_record.sh` | 1× | 是 | 否 |
| `./scripts/replay_guarded_chop_2x.sh` | 否 | 否 | 2× |
| `./scripts/run_guarded_chop_record_replay.sh` | 1× | 可选 | 2× |

### 只看一次正常速度仿真

```bash
./scripts/run_guarded_chop.sh
```

### 正常速度运行并录制

```bash
# 默认原子覆盖 recordings/guarded_chop_latest.npz
./scripts/run_guarded_chop_record.sh

# 显式保存到其他文件
./scripts/run_guarded_chop_record.sh recordings/demo_01.npz
```

### 只回放已有录制

```bash
# 默认读取 latest，不重新运行仿真
./scripts/replay_guarded_chop_2x.sh

# 回放显式文件
./scripts/replay_guarded_chop_2x.sh recordings/demo_01.npz
```

正式 CLI 等价命令为：

```bash
.venv/bin/twin-sim guarded-chop-replay \
  --recording recordings/guarded_chop_latest.npz \
  --rate 2.0
```

### 在线运行后立即回放

```bash
./scripts/run_guarded_chop_record_replay.sh

# 同时落盘
./scripts/run_guarded_chop_record_replay.sh --record
```

该组合脚本先显示 1× 在线仿真，再在同一 Viewer 中显示 2× 状态回放。独立二倍速脚本
则跳过在线执行，适合快速重复观察已有结果。

## 5. 录制文件跨机器使用

默认文件为 `recordings/guarded_chop_latest.npz`。`recordings/` 已被 Git 忽略，因此新的
clone 不包含录制文件；应先运行录制命令，或从其他机器显式复制 NPZ。

NPZ 包含 schema version、模型 `nq/nv/nu`、仿真时间、`qpos/qvel/ctrl`、phase、刀次、
刀刃点、护手点和安全状态。加载器会拒绝不支持的 schema 或模型维度不匹配的记录。

跨机器复制时建议：

1. 两台机器 checkout 同一 Git 提交；
2. 用 `sha256sum recording.npz` 校验传输；
3. 不修改 NPZ 内部数组；
4. 如果模型、geom 名称或 marker 语义发生变化，重新录制；
5. 不把大体积录制提交到 Git，除非项目以后明确建立测试夹具目录。

同一路径再次录制会原子覆盖旧文件。独立回放只读，不应改变文件内容、mtime 或大小。

## 6. Viewer 与 OpenGL 排障

### 窗口没有出现

先检查：

```bash
printf '%s\n' "${DISPLAY:-DISPLAY is unset}"
```

本地桌面通常会设置 `DISPLAY`。通过 SSH 使用时，需要以下任一方式：

- `ssh -X` 或 `ssh -Y` 并正确配置 X11 转发；
- 远程桌面进入该机器的图形会话；
- 在 CI 中使用 Xvfb，并只做自动 Viewer smoke；
- 只运行 headless 命令，把 NPZ 复制到有桌面的机器回放。

Wayland 桌面通常通过 XWayland 运行 MuJoCo Viewer。窗口可能出现在其他窗口后方或不同
workspace；先查看任务栏再判断命令失败。

### OpenGL 或离屏渲染失败

交互 Viewer 需要显示服务器和工作正常的 OpenGL 驱动。`MUJOCO_GL=egl` 适合无窗口离屏
诊断，但不会让纯终端会话出现交互窗口。软件渲染可以用于诊断性能问题，但不应作为实时
演示的默认配置。

### 回放启动看起来较慢

独立回放会先加载模型、运行安全预检并重建规划 marker，然后才按 2× 播放状态。预检
耗时不属于回放时间，因此命令总墙钟时间会大于录制仿真时长的一半。

## 7. 控制架构和安全不变量

左右机械臂与 Wuji Hand 当前都由 MuJoCo joint `position` actuator 控制。系统发送关节
位置目标；这不是 Cartesian 阻抗控制，也没有对外阻抗控制 API。未来导纳控制应作为上层
模块，根据测量力小幅修正位置目标，而不是修改当前接口含义。

默认 plane guarded-chop 的关键安全不变量：

- 刀片到全部手部碰撞代理的距离不低于 0.02 m；
- 刀片与手部真实接触数必须为零；
- 右刀下切时左臂和手指必须稳定；
- 右刀低位时才能倒手，且倒手期间右刀必须稳定；
- 左手稳定后右刀才允许抬起并移动到下一刀；
- 默认结果为 5 刀、4 次逻辑倒手和约 0.08 m 总推进；
- NaN、Inf、安全距离或接触互锁失败必须中止，不能为通过演示而放宽阈值。

位置执行器的 `kp/kv` 和力限制来自 MJCF。接触力 warning 是观测，不等价于力控。

## 8. 核心代码地图

| 路径 | 职责 |
|---|---|
| `src/twin_sim/tasks/guarded_chop.py` | 预检、双臂状态机、耦合时序和运行期采样 |
| `src/twin_sim/guarded_chop_safety.py` | 固定猫爪姿态和安全协调器数据结构 |
| `src/twin_sim/guarded_chop_recording.py` | 不可变帧、NPZ 原子保存/加载和纯状态回放 |
| `src/twin_sim/guarded_chop_playback.py` | 从已有 NPZ 组装独立 Viewer 回放 |
| `src/twin_sim/guarded_chop_visualization.py` | 规划/实际 marker 和 overlay |
| `src/twin_sim/cli.py` | `guarded-chop` 与 `guarded-chop-replay` 命令入口 |
| `robot_assets/mujoco/` | 双臂、刀具、砧板和 Wuji Hand 模型资产 |
| `scripts/` | 用户可执行的稳定 shell 入口 |
| `tests/simulation/test_guarded_chop_*.py` | 姿态、预检、安全、集成、录制和 launcher 回归 |

完整架构说明见 [architecture.md](architecture.md)，开发历程见
[guarded_chopping_development_log.md](guarded_chopping_development_log.md)。

## 9. 新智能体接手清单

开始修改前：

1. 阅读本文件、`README.md`、相关设计 spec 和最新 development log；
2. 运行 `git status --short`，保留用户已有修改；
3. 确认当前分支和目标分支，不猜测要合入 `main`；
4. 检查保护文件哈希；
5. 运行相关 focused tests，较大修改前运行完整 pytest；
6. 对新行为先写失败测试，再修改运行代码；
7. 不用近景相机或 marker 代替用户要求的真实手指动作。

完成修改后：

1. 运行 focused tests 和完整 pytest；
2. 对动作变化生成新的 latest，并检查 5 刀/4 次倒手及最小安全距离；
3. 若修改回放，证明回放不调用 `mj_step` 且不修改 NPZ；
4. 用正常全景 Viewer 做人工验收；
5. 更新设计、实施计划和 development log，记录实际数值及未解决问题；
6. `git diff --check` 后提交；
7. 合并或推送前再次确认目标分支。

## 10. ROS 2 与实体机边界

默认 `.venv` 是 Python 3.12 MuJoCo 环境。ROS 2 Humble 通常使用 Ubuntu 22.04 的系统
Python 3.10 和独立 `.venv-ros2`，不要把两套 ABI 环境混在一起。ROS 2 仿真桥接步骤见
[ros2_wuji_hand_bridge.md](ros2_wuji_hand_bridge.md)，Wuji Hand 关节顺序和 Python 接口见
[wuji_hand_control.md](wuji_hand_control.md)。

`SDK_PYTHON/`、`test/` 和 `real_robot_debug/` 属于实体机器人或厂商代码。当前默认仿真
入口不会调用它们。接入真实硬件前必须单独确认设备、急停、通信接口、控制模式和动作
限位；MuJoCo 中成功不能作为实体机安全证明。

## 本次验证基线

2026-08-01 在 `develop_9_kinematic_branch` 对本交接版本进行了 fresh 验证：

- MuJoCo `3.10.0`，NumPy `2.5.1`，CPython `3.12.7`；
- headless guarded-chop：`success=True`、5 刀、4 次倒手、0.080 m 总推进、
  0.057 m 最小刀手距离；
- 文档与 launcher 集中验证：`8 passed`；
- 完整回归：`221 passed`；
- 仅有一条既有 39.611 N 接触力观测 warning；
- 四个 guarded-chop shell 均通过 `bash -n`；
- 本文件不含开发机绝对路径。

测试数量会增长，后续智能体应以 fresh 运行的零失败结果为准，而不是要求数量永远等于
221。目标分支仍应在 merge 或 push 前由用户明确确认。
