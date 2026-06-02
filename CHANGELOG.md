# 变更日志

## [v1.0] 2026-05 (开源初版)

### 固件 (DiveWatch_v4)

**核心**
- ZHL-16C Bühlmann 16 组织室减压算法 + 海拔补偿 + 保守因子
- 安全停留 AOW 标准 (2.5-6m, 累积计时, 30s 抖动容差)
- TTS + Ceiling 计算
- 进出水自动识别 + 1.5s GO! 进入画面 + 5s 退出总结

**UI**
- 11 页 Garmin Descent X50i 风格 (HUD/PROFILE/TISSUE/N2/PLAN/AIR/TEMP/BAT/LASTDIVE/LOG/STATS)
- HUD 主区左右分屏：左大字深度 + 右动态状态 (NDL/安停/上升过快/下降过快/减压)
- 安停 / 安停完成 绿色，告警红色
- 分隔线 2px 加粗（水下镜面清晰）
- 大字深度阴影描边（模拟 anti-aliasing）
- SPI 27MHz (40 → 27 降速防细线毛刺)
- 屏保 → 唤醒/退出全屏清，无残影

**电源**
- Light Sleep 屏保 ~5-10mA
- Deep Sleep 关机 ~10μA (GPIO hold 锁背光防浮亮)
- 长按 MODE 5s 立即关机 / 屏保后 10 分钟自动关机
- MODE 长按 2s 唤醒
- 屏幕背光 GPIO 8 软控制

**OTA**
- 双 OTA 分区 (min_spiffs 1.94MB × 2 + 128KB SPIFFS)
- BLE 配 WiFi → WiFi OTA 双通道：HTTP /update + ArduinoOTA :3232
- 5s 内未 markValid 自动回滚
- 4 种触发方式 (菜单/长按 UP/DOWN/串口 OTA)

### 3D 外壳 (case_v5)

- 总厚 28mm (bottom 21 + top 7), 58×86.5mm 主体
- top 7mm: 屏幕窗贯穿 + PCB 沉槽 1.5mm + FPC 排线槽 4mm + 4 凸起卡 PCB
- bottom 21mm: 4 层斜坡 ledge 支撑密封 lip + 内腔顶部缩口防悬空打印失败
- 按钮 z 居中 / 间距 12.6mm / 与 PCB SMT 对齐
- 压力传感器孔右上角对齐 PCB
- 表带耳 + 弹簧棒孔
- 无线充凹槽内腔顶 (45mm × 1.3mm 深)
- TPU bumper 63×91.5×33mm 软保护套 (开屏幕窗/充电窗/按钮孔/传感器孔/表带通道)

### Android App (1.0.0)

- Capacitor 7 + Vanilla JS, APK 4.5MB
- 3 tab 扁平化深色 UI (`#0d0f12` + `#ff7a3a`)
- 主页: 电池 hero + 深度/温度/WiFi/IP 四宫格 + 实时深度曲线 (Chart.js)
- 日志: 历次潜水柱状图 + 列表 (IndexedDB, 60s 静止自动落库)
- 工具: BLE 扫描 + WiFi 配置 + WiFi OTA + 实时日志
- gpt-image-2 生成 logo 5 档 mipmap launcher icon
- Capacitor BLE 原生插件 (Android 12+ 自动请求权限)

---

## 历史里程碑

- **2026-05** Android App 上线, TPU bumper 设计
- **2026-05** HUD 左右分屏 + 屏幕 lip + 凸起卡 PCB
- **2026-05** 安停绿色 + 自动 deep sleep + 背光 GPIO 控制
- **2026-05** 案 v5 完整 (按钮间距 / 传感器位置 / ledge 斜坡支撑)
- **2026-05** OTA 双 slot + BLE 配 WiFi 架构
- **2026-05** ZHL-16C 减压算法 + 安停 + 进出水识别
- **2026-05** Garmin 风格 UI 11 页面
- **2026-05** v4 首版固件
