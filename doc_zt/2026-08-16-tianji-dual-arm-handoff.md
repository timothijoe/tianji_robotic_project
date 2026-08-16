# Tianji 双臂轨迹与真机调试交接（2026-08-16）

## 目标与当前结论

本次工作为双臂录制轨迹建立了可靠的低层批量回放路径，并对右臂录制数据做了离线 FK → TCP 平移 → IK 重求解。

当前最适合下一个 agent 继续的方向是：先恢复/确认 Tianji 控制器的 control-system 服务与 SDK 版本握手，再做只读反馈验证；**不要在该验证成功前执行任何真机运动**。

## 当前真机状态（最后一次观察）

- 当前没有运行中的 `dual_arm_batch_stream.py`、`arm_command_diagnostic.py` 或旧回放进程。
- 当前没有由 agent 保持的机器人连接。
- 最后两次只读诊断均失败：网络层显示“Robot connected”，但 SDK 报告 `control system 0` 对比本地 SDK `100343001` 的版本不匹配；连续反馈帧均为 `0`，然后 SDK 返回 `robot connection failed`。
- 该故障发生在读取有效反馈之前，未发送轨迹目标；不是新的 FK/IK 轨迹数据引起。
- 用户此前一次执行新回放程序时，双臂配置调用也出现相同版本告警，入场第 0 帧后 A 臂反馈为 `state=0`，程序立即退出；没有完成入场或播放。
- 过去已验证成功的回放会得到有效反馈帧和 `state=3`。因此当前现象更像控制柜 control-system 未就绪、重启中或服务/固件与 SDK 兼容性问题。IP 地址在本文中省略。

在控制柜/示教器侧确认服务恢复后，先运行只读检查（不运动）：

```bash
PYTHONPATH=. python3 real_robot_debug/arm_command_diagnostic.py --monitor-s 2 --poll-hz 5
```

只有其成功输出非零、递增的反馈帧及有效状态，才可进行后续真机回放。

## 新增/关键程序

### 为什么不继续使用 `a_arm_impedance_two_stage.py`

`a_arm_impedance_two_stage.py` 的原始回放循环使用 `Concise_Marvin_Robot.set_joint_position_cmd()`，即对 A、B 两臂分别从 Python 直接下发一帧目标。开发 `dual_arm_batch_stream.py` 的直接原因是：这条 Concise SDK 路径在真机上实测**每次调用约阻塞 0.5 秒**，而录制轨迹采样周期为 5 ms（200 Hz）。实测证据为：计划发送第 0、20、40 帧应分别在 0、0.1、0.2 s，但实际累计耗时约为 0.5、10.6、20.7 s。

这不意味着机器人控制器不能以 200 Hz 工作；SDK 文档也说明底层命令可进入控制器缓冲区。问题仅在于该 Python Concise API 的逐帧调用路径不适合 200 Hz 串流。同时，旧脚本中 15 秒的五次入场段起步位移非常小，用户曾在它到达真正播放阶段前中断，视觉上容易误判为卡住。

`dual_arm_batch_stream.py` 改用低层 `Marvin_Robot` API：每帧执行 `clear_set()` → 暂存 A 目标与 B 目标 → 单次 `send_cmd()`。这样两臂被一并提交，并且已经通过 200 帧/约 1 秒的 200 Hz 当前姿态保持实测（首尾反馈帧递增，最大保持误差约 0.357°）。

### `real_robot_debug/dual_arm_batch_stream.py`

推荐的双臂回放程序，取代旧的 `a_arm_impedance_two_stage.py` 的直接 Concise SDK 流式路径。

- 默认干跑；仅 `--execute` 才连接并控制真机。
- 每帧用底层 `Marvin_Robot` 批量暂存 A/B 关节目标，再一次 `send_cmd()`；此前实机观察到这一路径可在 200 Hz 当前位姿保持探针中运行。
- 阻抗状态/类型/速度加速度先提交一次，K/D 参数再单独提交一次（控制器要求的顺序）。
- 入场使用 200 Hz 五次多项式关节轨迹；播放周期为 `0.005 / speed_scale`。
- 每 200 帧与最后一帧校验反馈帧是否递增、状态是否为 3、误差是否不大于 5°。
- 清理时先释放底层连接，再用新的 Concise SDK 连接直接禁用两臂；若控制系统握手失败，此清理连接也会失败并警告。

常用命令（先干跑）：

```bash
PYTHONPATH=. python3 real_robot_debug/dual_arm_batch_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_100mm.npz
```

真机执行（仅在前述只读验证通过、现场净空确认后）：

```bash
PYTHONPATH=. python3 real_robot_debug/dual_arm_batch_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_100mm.npz \
  --execute --entry-duration-s 15 --speed-scale 0.05
```

### `real_robot_debug/arm_command_diagnostic.py`

默认只读的 A 臂诊断工具。其 `--execute` 路径存在，但当前阶段不要用；仅用于控制系统恢复后的状态/帧号确认。

### `real_robot_debug/shift_right_arm_trajectory.py`

完全离线，不连接机器人。读取 NPZ，右臂每帧：SDK FK（角度输入、毫米位置）→ 在右臂基座坐标系平移 TCP → SDK IK（以前一帧解作参考）→ 用 FK 回验位置误差，并检查 1° 关节限位裕度。左臂、右手和时间轴保留不变，原文件不覆盖。

目前坐标约定由用户确认：

- 右臂向前：基座 `+X`
- 右臂向右：基座 `+Z`

命令行参数：`--shift-x-mm`、`--shift-y-mm`、`--shift-z-mm`。默认值目前为 `+30, 0, +100 mm`。

本轮的平移语义和实现：

1. 用户最初要求“右移 10 cm”，首次按基座 `+X 100 mm` 生成；用户实机观察确认这在该右臂坐标系中表现为“向前”。
2. 用户随后明确坐标：右臂基座 `+X` 是向前、`+Z` 是向右。因此转换器升级为完整三维偏移，保持 TCP 姿态不变。
3. 随后依次生成“前 3 cm + 右 10 cm”以及当前推荐的“前 8 cm + 右 10 cm”。当前 8 cm 文件是从**原始轨迹**重新 FK/IK 计算，而不是在 3 cm 文件上再次平移。

## 生成的数据文件

原始源（未修改）：

`recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate.npz`

已生成的派生文件：

1. `recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_right_x_plus_100mm.npz`
   - 早期试验：基座 `+X 100 mm`；用户现场观察为“向前”，不建议当前使用。
2. `recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_30mm_right_z_plus_100mm.npz`
   - 前 `+30 mm`、右 `+100 mm`。
3. `recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_100mm.npz`
   - **当前推荐数据：前 `+80 mm`、右 `+100 mm`**。从原始源重新计算，而非在 30 mm 文件上再次叠加。
   - 831 帧全部有 IK 解；独立 FK 复核相对原始轨迹的偏移误差最大 `0.000066 mm`；最大逐帧关节变化 `0.2086°`；右臂关节范围满足 1° 余量。

若需重新生成当前文件：

```bash
PYTHONPATH=. python3 real_robot_debug/shift_right_arm_trajectory.py \
  --output-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_100mm.npz \
  --shift-x-mm 80 --shift-z-mm 100
```

生成后，先做不连接机器人的回放数据检查：

```bash
PYTHONPATH=. python3 real_robot_debug/dual_arm_batch_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_100mm.npz
```

控制系统恢复、只读诊断成功且现场净空确认后，执行真机回放：

```bash
PYTHONPATH=. python3 real_robot_debug/dual_arm_batch_stream.py \
  --source-npz recordings/recorded_hand_guarded_chop_200hz_official_right_retarget_candidate_forward_x_plus_80mm_right_z_plus_100mm.npz \
  --execute --entry-duration-s 15 --speed-scale 0.05
```

## 关键历史诊断

- 旧 `a_arm_impedance_two_stage.py` 先配置 position/impedance，真实运动是 Phase 1 入场与 Phase 2 播放。用户曾在 15 秒慢入场期间中断，误以为“卡住”；五次入场起步很慢，加之目标和当前状态约 61° 相差很大。
- 通过小幅 A 臂单关节测试已确认硬件此前能实际运动。
- 直接 Concise SDK 的逐帧 `set_joint_position_cmd` 路径实测约每调用阻塞 0.5 秒，无法用于 200 Hz 串流；这不表示控制器本身不能 200 Hz。
- 底层 `clear_set()` + A/B `set_joint_cmd_pose()` + 一次 `send_cmd()` 路径实测 200 帧（约 1 秒）当前位姿保持，首尾反馈帧递增，最大误差约 0.357°。
- 曾经用该批量路径完整执行过原始双臂轨迹：入场期间最大跟踪误差约 1.27°，事后关节反馈与末帧目标约 0.25° 内吻合，且两臂已禁用。
- 本轮失败日志中的 `entry_max_jump_deg=129.920` 是新右臂轨迹起点与当时反馈姿态的最大关节差，不是已执行的运动量；因 state=0 在第 0 帧被拒绝。

## 验证与测试

最近一次离线测试：

```bash
PYTHONPATH=. python3 -m pytest -q \
  tests/hardware/test_shift_right_arm_trajectory.py \
  tests/hardware/test_dual_arm_batch_stream.py
```

结果：`14 passed`。

相关测试：

- `tests/hardware/test_shift_right_arm_trajectory.py`
- `tests/hardware/test_dual_arm_batch_stream.py`
- `tests/hardware/test_arm_command_diagnostic.py`

注意：完整测试套件未作为本轮验收运行；已有的 `tests/hardware/test_a_arm_impedance_two_stage.py` 与旧脚本当前 API 不一致，导入时会失败，此问题未修复。

## 工作区注意事项

- 工作区在开始前就已是 dirty；不要删除或覆盖未追踪的用户文件。
- 本轮新增的程序和测试目前为未追踪文件，未 git add、未提交。
- 原有交接与 SDK 背景可参考：
  - `doc_zt/2026-08-05-agent-handoff.md`
  - `doc_zt/2026-08-05-sdk-control-summary.md`
  - `doc_zt/wuji_sdk_playback_guide.md`

## 建议 skills

- `superpowers:systematic-debugging`：控制系统握手/反馈异常的根因诊断。
- `superpowers:test-driven-development`：修改任一回放或数据转换逻辑前。
- `superpowers:verification-before-completion`：声称真机恢复、轨迹可运行或修复完成前。
