# Wuji 灵巧手本地角度条仿真

这个入口在 MuJoCo 中打开一个本地角度条面板，用鼠标实时控制一只 20 自由度
Wuji 灵巧手。它只改变仿真模型，不连接 ROS、USB 或真实手 SDK。

## 启动

在仓库根目录运行：

```bash
.venv/bin/tianji-robot sim wuji-angle-bar
```

默认载入左手。启动时直接选择右手：

```bash
.venv/bin/tianji-robot sim wuji-angle-bar --hand right
```

面板顶部也可以在左手和右手之间切换。切换会关闭当前 MuJoCo Viewer，载入对应
手模型，并用该模型经过校验的张开姿态重新初始化角度条。

每根滑条对应一个关节，五指各有四个关节。显示值和滑条范围均为弧度（rad）；
范围取自当前模型的 position actuator，输入不会超出模型允许的控制范围。点击
“Reset open hand”可将当前手恢复为安全的张开姿态。

## 运行条件

- 已按[仿真环境设置](setup.md)创建并安装依赖的 `.venv`。
- 需要有效的图形会话（`DISPLAY`/XWayland）和可用的 OpenGL，才能显示 Tk 面板
  和 MuJoCo Viewer。
- 无图形会话时可以继续运行自动化测试，但不能使用交互式角度条窗口。

## 边界与安全

此命令是 MuJoCo-only 的离线仿真工具。它不会启动 ROS，不发布或订阅 ROS 话题，
不会打开 USB，也不会导入或调用真实 Wuji Hand SDK。关闭窗口会关闭仿真后端；只有
明确设计后续硬件适配时，才应另行实现真机输出端。

## 生命周期与步进

MuJoCo 后端的每次 `step()` 只推进一个模型 timestep，不自行等待；Tk 面板按该
timestep 重新调度下一次步进，因此它是唯一的实时节拍拥有者。关闭 MuJoCo Viewer
或关闭面板都会执行同一套幂等清理：关闭后端并退出、销毁 Tk 主循环。可运行下列
无窗口回归测试验证左右手切换、窗口关闭和步进调度契约：

```bash
python3 -m pytest tests/simulation/test_local_angle_bar.py -q
```
