# Guarded Chop 启动脚本设计

日期：2026-08-01

## 目标

新增一个最小、可直接执行的 Shell 入口，用于从任意当前目录启动默认平面版
MuJoCo 猫爪倒手切菜 Viewer。更多模式和更多脚本留到后续需求中实现。

## 文件与行为

脚本路径：`scripts/run_guarded_chop.sh`

脚本必须：

- 使用脚本自身位置解析仓库根目录，不依赖调用者当前目录；
- 检查 `<repo>/.venv/bin/twin-sim` 是否可执行；
- 环境缺失时打印 README 中已有的虚拟环境安装命令并返回非零状态；
- 环境存在时执行 `<repo>/.venv/bin/twin-sim guarded-chop --scene plane`；
- 使用 `exec` 转交进程，使退出码和 Ctrl-C 直接传递给 MuJoCo 进程；
- 不调用 ROS 2、实体机械臂、实体 Wuji Hand 或硬件发送接口；
- 暂不接受模式参数，不加入 object/headless 分支。

## 验证

- Shell 语法检查通过；
- 从仓库根目录以外执行时仍能找到项目和虚拟环境；
- 缺少虚拟环境的路径检查具有清晰错误信息；
- 正常运行时弹出默认平面版 Viewer，并自动完成 5 刀、4 次倒手后成功退出。

