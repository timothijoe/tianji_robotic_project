# 当前仿真版本跨机器复现与智能体交接设计

## 目标

为没有本次对话上下文的新用户或智能体提供一条从全新 Linux 机器到可验证运行的完整
路径。文档描述仓库当前已经实现并验收的 MuJoCo 版本，不把未来设想、尚未验证的实体机
行为或特定开发机绝对路径写成通用步骤。

读者完成文档后应能：

1. 创建隔离的 Python 3.12 仿真环境并安装固定依赖；
2. 运行无窗口测试，确认模型和代码完整；
3. 打开默认双臂猫爪切菜 Viewer；
4. 以原速度运行并录制，随后只读取记录进行二倍速回放；
5. 理解当前控制方式、安全边界、代码入口和后续开发检查顺序；
6. 明确哪些命令只控制 MuJoCo，哪些 ROS 2/实体机内容不属于默认复现路径。

## 文档结构

新增 `docs/simulation/current_version_handoff.md` 作为单一交接入口。它不复制全部历史，
而是按实际操作顺序组织：

- 当前版本摘要和能力边界；
- 操作系统、Python、GUI 和磁盘文件前提；
- 获取代码后的环境创建与固定依赖安装；
- 分层验证：版本、受保护文件、集中 smoke、完整 pytest；
- 四个 guarded-chop shell 入口的行为对照；
- 默认录制、显式录制路径和跨机器 NPZ 兼容条件；
- Viewer/OpenGL/SSH 排障；
- 控制架构、安全不变量和核心模块地图；
- 新智能体开始修改前及完成修改后的检查清单；
- ROS 2、Wuji Hand 和实体机专项文档链接。

同时更新：

- `README.md`：在详细说明列表首部增加“当前版本复现与智能体交接”；补充当前 guarded
  chopping、录制和回放能力；
- `docs/simulation/setup.md`：补充跨发行版 Python 3.12 要求、GUI/无 GUI 区分、当前
  版本验收命令和交接指南链接；
- `docs/simulation/usage.md`：补充原速度录制与独立二倍速回放脚本、可选路径和四入口
  对照。

## 跨机器环境约定

通用命令只使用仓库相对路径。文档不要求新机器复用 `/home/linux/...`，而要求先进入
克隆后的仓库根目录。支持任何提供 CPython `3.12.x` 的方式，包括系统包、pyenv 或
conda；虚拟环境固定命名为 `.venv`，依赖来源固定为 `requirements-sim.lock` 和 editable
project install。

最低验证为：

```bash
python3.12 --version
python3.12 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-sim.lock
.venv/bin/python -m pip install -e .
.venv/bin/python -c "import mujoco, numpy; print(mujoco.__version__, numpy.__version__)"
sha256sum --check docs/simulation/protected-files.sha256
.venv/bin/python -m pytest -q
```

当前参考结果为 MuJoCo `3.10.0`、NumPy `2.5.1`、`220 passed` 和一条既有
39.611 N 接触力观测 warning。测试数会随后续开发增长，因此文档要求零失败，不把
220 永久作为硬编码通过条件。

## GUI 与无窗口边界

无窗口 pytest、headless 任务和 NPZ 生成可在没有桌面的机器上运行。交互 Viewer 需要
有效的 `DISPLAY` 或 Wayland/XWayland 会话和可用 OpenGL；SSH 机器需要 X11 转发、远程
桌面或虚拟显示。`MUJOCO_GL=egl` 可用于离屏诊断，但不能让没有显示服务器的终端凭空
出现交互窗口。

文档先提供不依赖 Viewer 的 smoke：

```bash
.venv/bin/twin-sim guarded-chop --headless --final-hold 0 --record
```

再提供四个有窗口入口：

| 脚本 | 在线仿真 | 保存 NPZ | 状态回放 |
|---|---:|---:|---:|
| `run_guarded_chop.sh` | 1× | 否 | 否 |
| `run_guarded_chop_record.sh` | 1× | 是 | 否 |
| `replay_guarded_chop_2x.sh` | 否 | 否 | 2× |
| `run_guarded_chop_record_replay.sh` | 1× | 可选 | 2× |

## 录制可移植性

`recordings/` 被 Git 忽略，clone 不会包含 latest。新机器必须先运行录制脚本，或显式
复制一个 NPZ。NPZ 保存模型维度和 schema；回放加载时拒绝 schema 不支持或
`nq/nv/nu` 不匹配的文件。即使维度相同，跨提交复用也应优先使用生成录制的同一 Git
提交，避免 marker、geom 名称或语义变化。

默认录制路径为 `recordings/guarded_chop_latest.npz`，重复录制原子覆盖。独立回放读取
文件但不修改内容、mtime 或大小。显式路径通过脚本的唯一可选位置参数传入。

## 架构与安全交接

当前左右机械臂和 Wuji Hand 均使用 MuJoCo joint `position` actuator；不存在对外的
Cartesian 阻抗控制 API。轨迹和 IK 预计算后发送关节位置目标，运行期安全协调器检查刀手
距离、接触、速度和数值状态。后续导纳控制只能作为修改位置目标的独立上层模块，不能
把当前位置伺服误写成阻抗控制。

交接文档必须突出以下不可放宽项：

- 刀片到全部手部碰撞代理至少 0.02 m；
- 刀片与手部真实接触数为零；
- 下切时左臂和手指稳定；低刀倒手时右刀稳定；
- 默认 plane 场景为 5 刀、4 次逻辑倒手；
- ROS 2、实体机械臂和实体 Wuji Hand 不得由默认仿真脚本触发；
- 修改实体/SDK 目录前必须取得单独范围授权并重新验证保护文件。

模块地图只列接手当前功能所需文件：任务状态机、固定手势与安全数据、状态录制、独立
回放、Viewer trace、CLI、shell 和对应测试。历史设计细节继续留在 development log 和
spec/plan 文件，不复制到日常操作步骤。

## 验收

文档验收通过以下方式完成：

- 所有链接和仓库相对路径存在；
- 所有 shell 通过 `bash -n`；
- README、setup、usage 与交接指南中的脚本名称和默认路径一致；
- 对文档中可自动执行的核心命令进行 fresh 验证；
- 完整 pytest 零失败；
- `rg` 检查交接指南不包含当前开发机绝对路径；
- Git diff 只包含文档变更，不改控制、模型、ROS 2 或实体机代码。

目标分支合并继续单独等待用户指定；编写交接文档不隐含向 `main` 或 `develop_8` 合并。
