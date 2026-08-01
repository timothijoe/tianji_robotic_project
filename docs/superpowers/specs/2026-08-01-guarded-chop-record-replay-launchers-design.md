# 切菜录制与独立倍速回放脚本设计

## 目标

为默认平面切菜演示提供两个职责单一的 shell 入口：一个以真实仿真速度运行并录制，另
一个不重新求解仿真、只读取已有状态记录并以二倍速播放。用户无需复制 Python 临时代码，
也不必先观看一次实时仿真才能查看回放。

## 命令入口

新增：

```bash
./scripts/run_guarded_chop_record.sh
./scripts/replay_guarded_chop_2x.sh
```

`run_guarded_chop_record.sh` 启动正常 1× Viewer 仿真，并将状态原子覆盖保存到
`recordings/guarded_chop_latest.npz`。第一个可选位置参数可指定其他 NPZ 路径；未指定
时只维护 latest 文件，防止录制冗余。

`replay_guarded_chop_2x.sh` 直接读取同一默认 latest 文件，以 2× 恢复状态和 marker；
第一个可选位置参数可指定其他 NPZ。它不调用 `mj_step`、不执行控制器和安全协调器，也
不覆盖录制文件。文件不存在、格式不兼容或模型维度不匹配时返回非零并显示明确错误。

现有入口保持兼容：

- `run_guarded_chop.sh`：只运行一次 1×，不录制；
- `run_guarded_chop_record_replay.sh`：运行一次 1× 后立即在同一 Viewer 中 2× 回放。

## 正式 CLI

新增命令：

```bash
.venv/bin/twin-sim guarded-chop-replay \
  --recording recordings/guarded_chop_latest.npz \
  --rate 2.0
```

CLI 创建与当前任务一致的 MuJoCo 模型和 plane 场景，执行预检仅用于恢复规划 marker，
随后加载 NPZ、恢复首帧、打开 Viewer，并调用既有 `replay_recording(..., rate=...)`。
该命令不提供 headless 模式，因为状态回放的用途就是可视化；`rate` 必须为有限正数。

高层回放编排放入独立模块 `src/twin_sim/guarded_chop_playback.py`，避免继续扩大任务状态机
文件。模块公开：

```python
def play_guarded_chop_recording(path: Path, *, rate: float = 2.0) -> None:
    ...
```

底层 NPZ 校验、状态恢复、时间间隔和 marker 更新继续复用
`twin_sim.guarded_chop_recording`，不复制实现。

## Shell 行为

两个脚本均从自身路径解析仓库根目录，因此可从任意当前目录启动；均检查
`.venv/bin/twin-sim` 是否可执行，并复用现有环境安装提示。路径参数之后不接受模糊的
任意透传选项，避免把录制路径误解析为 CLI flag。

具体调用为：

```bash
twin-sim guarded-chop --scene plane --record "$recording_path"
twin-sim guarded-chop-replay --recording "$recording_path" --rate 2.0
```

路径默认值由 shell 显式设置为 `recordings/guarded_chop_latest.npz`，帮助输出和测试均能
清楚显示实际目标。

## 测试与验收

自动测试覆盖：

- CLI 默认录制路径、显式路径、默认 2× rate 和非法 rate；
- 高层回放只加载已有记录，恢复首帧并调用状态回放，不运行新的切菜任务；
- 两个 shell 可从仓库外目录启动，向假 `twin-sim` 传递准确参数；
- 录制脚本第二次运行覆盖同一路径；回放脚本不修改文件；
- 现有三个 guarded-chop shell 入口继续通过回归；
- 完整测试无新增 warning。

人工验收依次运行录制脚本和独立二倍速脚本，确认前者显示原速度动作并生成 latest，后者
只显示二倍速回放。所有入口仍只控制 MuJoCo，不连接实体机械臂、ROS 2 或手部 SDK。

## 合并策略

当前可见手指优化和新脚本都直接开发在 `develop_9_kinematic_branch`。脚本、测试和文档
完成并提交后，再根据用户指定的目标分支执行一次合并并在合并结果上重跑测试；在目标
分支未明确前不猜测为 `main` 或 `develop_8_kinematic_branch`。
