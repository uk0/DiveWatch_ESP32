<p align="center">
  <img src="assets/logo-256.png" alt="DiveWatch" width="180"/>
</p>

<h1 align="center">DiveWatch</h1>

<p align="center">
  <b>DIY 开源潜水电脑</b> · ESP32-S3 + ST7789 + MS5837<br/>
  完整工程：<b>固件 + 3D 外壳 + 安卓 App</b>
</p>

<p align="center">
  <a href="README.md">中文</a> ·
  <a href="README.en.md">English</a> ·
  <a href="LICENSE">MIT License</a>
</p>

---

> ⚠ **本项目仅供学习娱乐，未经任何潜水认证机构认证。下水必须携带认证潜水电脑或潜水表作为主设备。**

## 特性

- **Garmin Descent X50i 风格 HUD**：11 个数据页面 + 全屏告警 + 进出水画面
- **ZHL-16C Bühlmann 减压算法**：16 组织室 + 海拔补偿 + 保守因子
- **安全停留 AOW 标准**：2.5-6m 窗口，累积计时，30s 抖动容差
- **OTA 刷不死**：BLE 配 WiFi → WiFi OTA 双 slot 自动回滚
- **超低功耗待机**：Light Sleep ~5mA / Deep Sleep ~10μA
- **完整 3D 打印外壳**：树脂底壳 + 树脂顶盖 + TPU 软套
- **Android App**：扁平化 BLE 配 WiFi + WiFi OTA + 潜水记录本地存储

## 仓库结构

```
watcher/
├── arduino/DiveWatch_v4/    # 固件 (~3400 行 .ino + BLE OTA 模块)
├── case_v5/                 # 3D 外壳源码 (build123d Python)
├── case/                    # 3D 外壳产物 (.step / .stl / .3mf)
├── android_app/             # 安卓 App (Capacitor 7 + Vanilla JS)
├── assets/                  # 图片资源
├── PINOUT.md                # ESP32-S3 引脚映射
├── HARDWARE.md              # BOM + 接线图
└── README.md / README.en.md
```

## 硬件

| 件 | 型号 | 接线 |
|---|---|---|
| 主控 | ESP32-S3-Nano (4 MB Flash / 8 MB PSRAM) | — |
| 显示屏 | ST7789 2.4" SPI 240×320 | CS=21 / DC=18 / RST=17 / MOSI=38 / SCK=48 / BL=8 |
| 压力传感器 | MS5837-30BA | I²C SDA=11 / SCL=12 (地址 0x76) |
| 按键 | 3 × 6×6 SMT | MODE=6 / UP=7 / DOWN=10 |
| 蜂鸣器 | 9042 + S8050 NPN | GPIO 5 经 1kΩ → 三极管基极 |
| 电池监测 | 2:1 分压 (推荐 10K+10K) | ADC A0 = GPIO 1 |
| 无线充电 | Qi 5V/1A 接收 + TP4056 | 详见 [HARDWARE.md](HARDWARE.md) |

完整引脚清单 → [PINOUT.md](PINOUT.md) · BOM/接线图 → [HARDWARE.md](HARDWARE.md)

## 快速开始

### 1. 固件烧录

```bash
cd arduino
arduino-cli compile --upload \
  -p /dev/cu.usbmodem2101 \
  --fqbn esp32:esp32:esp32s3:PartitionScheme=min_spiffs,CDCOnBoot=cdc \
  DiveWatch_v4
```

第一次烧入需要 BOOT+RST 进下载模式。后续可通过 OTA 升级（无需 USB）。

### 2. 3D 打印外壳

```bash
cd case_v5
pip install build123d
python3 case_v5.py
# 输出: ../case/{top_cover, bottom_case, tpu_bumper}.{step, stl, 3mf}
```

| 件 | 工艺 | 推荐材料 |
|---|---|---|
| `bottom_case` | 树脂 (SLA/DLP) | 高韧性灰树脂, 层高 0.05mm |
| `top_cover`   | 树脂 (SLA/DLP) | 同上，屏幕窗精度要求高 |
| `tpu_bumper`  | FDM | TPU 95A，0.2mm 层高，30mm/s |

### 3. Android App 编译

```bash
cd android_app
npm install
npm run build           # esbuild bundle
npx cap sync android
JAVA_HOME=/Library/Java/JavaVirtualMachines/temurin-25.jdk/Contents/Home \
  ./android/gradlew -p ./android assembleDebug
# APK 输出: android/app/build/outputs/apk/debug/app-debug.apk
```

## OTA 升级流程

**第一次配 WiFi (BLE)**

1. 设备进 OTA 模式：设置菜单 → 第 14 项「进入 OTA」
2. 手机 App → 工具页 → 扫描连接 `DiveWatch-OTA`
3. WiFi 配置卡填 SSID + 密码 → 点连接
4. 设备屏幕显示分配的 IP

**后续升级 (WiFi)**

1. 设备进 OTA 模式（自动连之前保存的 WiFi）
2. App 工具页 → 固件 OTA → 选 .bin 文件 → 推送
3. 进度条 100% 设备自动重启
4. **5 秒内未启动成功，自动回滚到上一版本**

完整 OTA 说明 → [arduino/DiveWatch_v4/OTA_GUIDE.md](arduino/DiveWatch_v4/OTA_GUIDE.md)

## 屏幕布局

```
┌─顶栏: 时间│DIVE/水面│AIR│电量│海/淡────┐  黑底白字
├─[2px 黑]──────────────────────────────┤
│ 深度        │  NDL / 安停 / 上升过快  │  橙底
│ XX.X 米     │  XX (大字)              │
│ ▼最深 X.Xm │  状态/动作               │
├─[2px 黑]──────────────────────────────┤
│ +X.X ◀━━━●━━━▶ m/min                  │  上升速度
├─[2px 黑]──────────────────────────────┤
│最深│温度│N2│用时                       │  数据条
├─[2px 黑]──────────────────────────────┤
│ ⓂM翻页│ ▼最深0│ ▲计时                 │  黑底白字按键提示
└─────────────────────────────────────────┘
```

## 11 个页面

| 页 | 内容 |
|---|---|
| HUD       | 主页面 (深度/NDL/安停/速度/温度/用时/N2/最深) |
| PROFILE   | 潜水深度曲线 |
| TISSUE    | 16 组织室饱和度条 |
| N2        | 当前 N2 负载详情 |
| PLAN      | 下潜计划 (NDL 表) |
| AIR       | 气瓶 SPG + SAC + Air Time |
| TEMP      | 温度趋势 |
| BAT       | 电池电压/百分比/曲线 |
| LASTDIVE  | 上次潜水摘要 |
| LOG       | 潜水日志列表 |
| STATS     | 累计统计 |

## 文档

- [PINOUT.md](PINOUT.md) — ESP32-S3 引脚完整映射
- [HARDWARE.md](HARDWARE.md) — BOM + 接线图 + 装配顺序
- [arduino/DiveWatch_v4/OTA_GUIDE.md](arduino/DiveWatch_v4/OTA_GUIDE.md) — OTA 防砖架构
- [android_app/README.md](android_app/README.md) — App 开发说明
- [CHANGELOG.md](CHANGELOG.md) — 版本历史
- [CONTRIBUTING.md](CONTRIBUTING.md) — 贡献指南

## 许可证

MIT — 见 [LICENSE](LICENSE)。

## 致谢

- [build123d](https://github.com/gumyr/build123d) — Python CAD
- [u8g2](https://github.com/olikraus/u8g2) — 字体引擎
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) — 轻量 BLE 栈
- [Adafruit GFX / ST7789](https://github.com/adafruit/Adafruit-GFX-Library) — 显示驱动
- [Capacitor](https://capacitorjs.com/) — Web → 原生 App 框架
- [Chart.js](https://www.chartjs.org/) — 图表
- Bühlmann ZHL-16C 减压算法参考 [Shearwater](https://www.shearwater.com/) 公开文档
