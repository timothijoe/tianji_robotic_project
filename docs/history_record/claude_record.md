# 维修记录 — Codex 版本切换

**日期**：2026-06-27

## 问题

机器上存在两份 Codex 安装：

| 路径 | 版本 |
|------|------|
| `~/.npm-global/bin/codex` | 0.142.0（旧） |
| `~/.nvm/versions/node/v24.17.0/bin/codex` | 0.142.3（新） |

Shell PATH 中 `~/.npm-global/bin` 优先于 nvm 路径，导致运行 `codex` 时实际使用的是旧版 0.142.0，Codex 会持续提示安装/更新。

## 操作步骤

1. **确认当前版本**：`command -v codex && codex --version` → 确认命中旧版 0.142.0
2. **卸载旧包**：`npm --prefix ~/.npm-global uninstall @openai/codex` → 卸载成功但 symlink 残留
3. **手动删除旧 symlink**：`rm ~/.npm-global/bin/codex`
4. **刷新 shell 缓存**：`hash -r`
5. **验证**：`command -v codex && codex --version` → 正确指向 nvm 路径的 0.142.3

## 结果

- Codex 现在指向 `/home/zhoutong/.nvm/versions/node/v24.17.0/bin/codex`，版本 0.142.3
- 用户配置、插件、认证信息均在 `~/.codex/` 下，未受影响
- Claude Code（`@anthropic-ai/claude-code`）与 Codex 相互独立，此操作不影响 Claude

## 备注

- 如果以后出现找不到 codex 的情况，临时修复：`export PATH="$HOME/.nvm/versions/node/v24.17.0/bin:$PATH"`
- 长期确保 nvm 初始化在 `~/.bashrc` 或 `~/.zshrc` 中正确加载
