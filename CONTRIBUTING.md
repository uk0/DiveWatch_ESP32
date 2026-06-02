# 贡献指南

欢迎贡献！本项目欢迎以下形式的参与：

## 报告 Bug

提 Issue 时请包含：
- 设备型号 / 固件版本 / App 版本
- 复现步骤
- 期望行为 vs 实际行为
- 串口日志（必要时）

## 提交 PR

### 固件 (arduino/DiveWatch_v4)

- C++ Arduino 风格，函数命名 camelCase
- 全局变量加 `g_` 前缀，static 加 `s_` 前缀
- 添加新页面：参考 PAGE_HUD..PAGE_STATS 的实现
- **不要扩字体**：sketch 已 93% 满，新功能优先复用现有 wqy12/16 + logisoso24/28/50
- BLE 协议改动需同步更新 App 端

### 3D 模型 (case_v5)

- 用 build123d，禁止手画 mesh
- 改尺寸前先量好 PCB / 元件实物（自己的 PCB 可能与设计稍异）
- 加新孔/凹槽前确认不切断主体结构
- bottom 已固化的版本：**永远不能改 BOT_THICK / 按钮位置 / 传感器位置**

### Android App (android_app)

- Capacitor 7 + Vanilla JS（不引入 React/Vue 减少体积）
- 配色锁定 `#0d0f12` + `#ff7a3a`（与设备 HUD 一致）
- 改 BLE 协议同步更新固件

## 提交规范

- Commit 标题：`<模块>: <动作简述>` (例 `firmware: 修 HUD 速度条边缘残影`)
- 中文 / 英文均可
- 一次 commit 一个完整改动，不要混合多个无关修改

## 行为准则

- 友善、专业
- 不要在 Issue/PR 里讨论无关话题
- **永远不要**鼓励他人在未充分测试的设备上进行真实潜水活动

## 法律

提交 PR 即表示同意你的贡献以 MIT 协议授权。
