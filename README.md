# DiveWatch

ESP32-S3 + MS5837-30BA + OLED SH1106 + 9042 蜂鸣器 的 DIY 潜水表固件。

## 硬件接线

| ESP32-S3 | 外设 |
|---|---|
| 3V3 | MS5837 VCC、OLED VCC、蜂鸣器(+) |
| GND | MS5837 GND、OLED GND、S8050 发射极(E) |
| GPIO 8 (SDA) | MS5837 SDA、OLED SDA |
| GPIO 9 (SCL) | MS5837 SCL、OLED SCL |
| GPIO 5 | → 1kΩ → S8050 基极(B) |
| - | 蜂鸣器(-) → S8050 集电极(C) |

I²C 通常需要 4.7kΩ 上拉到 3V3，但绝大多数 OLED / MS5837 模块板上已自带，可省。

## 烧录步骤（推荐 Arduino IDE，最快）

### 1. 安装 ESP32 板支持
- Arduino IDE → 文件 → 首选项 → 附加开发板管理器网址：
  ```
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
  ```
- 工具 → 开发板管理器 → 搜索 `esp32` → 安装 `esp32 by Espressif Systems`（≥ 2.0.14）

### 2. 安装库
- 工具 → 库管理器：
  - 搜索 **U8g2**（作者 oliver） → 安装
- 安装 MS5837：
  - 下载 zip：<https://github.com/bluerobotics/BlueRobotics_MS5837_Library/archive/refs/heads/master.zip>
  - 项目 → 包含库 → 添加 .ZIP 库 → 选择刚下载的 zip

### 3. 板卡设置
工具菜单选择如下：
- **开发板**：`ESP32S3 Dev Module`
- **USB CDC On Boot**：`Enabled`（让串口通过 USB 工作）
- **CPU Frequency**：`240MHz`
- **Flash Size**：`8MB`
- **Partition Scheme**：`8M with spiffs (3MB APP/1.5MB SPIFFS)` 或默认即可
- **PSRAM**：`OPI PSRAM`
- **Upload Speed**：`921600`
- **端口**：插上 USB 线后选择对应的 `/dev/cu.usbmodem*`（macOS）或 `/dev/ttyUSB*`（Linux）

### 4. 第一步：先烧 I²C 扫描器（强烈建议）
打开 `arduino/I2CScanner/I2CScanner.ino`，烧录，开 **串口监视器（115200）**。
应该看到：
```
Found device at 0x3C  <- OLED
Found device at 0x76  <- MS5837
```
如果一个都没有：检查 SDA/SCL 是否接反、3V3 是否给电、共地。

### 5. 烧主程序
打开 `arduino/DiveWatch/DiveWatch.ino`，烧录。
开机会有"嘀嘀"两声，OLED 显示开机画面与水面气压基准。

## 行为
- 开机时自动校准水面气压（**开机务必把传感器放在空气中**）。
- 主界面：大字深度 / 潜水时长 / 最大深度 / 温度 / 上升速率。
- 报警：
  - 深度 > 30 m → 三声短鸣
  - 上升速率 > 9 m/min 且深度 > 3 m → 一声长鸣

## 调参（在 `DiveWatch.ino` 顶部）
```cpp
ALARM_DEPTH        = 30.0f;   // 深度警告
ASCENT_LIMIT_MPM   = 9.0f;    // 上升速率限制
FLUID_DENSITY      = 1029.0f; // 海水 1029, 淡水 997
```

## PlatformIO 用户
项目根目录已带 `platformio.ini`：
```bash
pio run -t upload
pio device monitor
```

## 常见问题

| 现象 | 原因 |
|---|---|
| 串口看不到 boot log | `USB CDC On Boot` 没启用 |
| OLED 全黑/全亮 | I²C 没连通；先跑 I2CScanner |
| 深度永远 0 或乱跳 | `setModel` 不对（确保是 `MS5837_30BA`），或 SDA/SCL 接反 |
| 蜂鸣器不响 | 三极管 EBC 接反；或基极电阻太大；先 `digitalWrite(5, HIGH)` 直测 |
| 深度负值 | 水面气压基准是开机时取的，气压变了。重启即可 |
