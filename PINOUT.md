# DiveWatch v4 引脚总表 (ESP32-S3-Nano)

## 一、开发板信息

| 项目 | 值 |
|---|---|
| 主控芯片 | ESP32-S3 (双核 240MHz, 4MB Flash, 8MB PSRAM) |
| 开发板名称 | ESP32-S3-Nano (Arduino Nano 形态) |
| 板载接口 | USB-C (兼容 OTG + CDC) |
| 工作电压 | 3.3V |
| 板载稳压 | 5V → 3.3V LDO |
| 引脚数 | 30 (左右各 15) |
| 通讯接口 | UART × 1, I²C × 1, SPI × 1, USB-CDC × 1 |
| ADC | 12-bit, 多通道 (GPIO 1-10 可作 ADC) |
| Arduino-ESP32 核心 | 3.3.8 |
| FQBN | `esp32:esp32:esp32s3:PartitionScheme=min_spiffs,CDCOnBoot=cdc` |
| 分区方案 | min_spiffs (1.94MB × 2 OTA slot + 128KB SPIFFS) |
| Sketch 大小 | ~1.84 MB (占 93% 单 slot) |
| RAM 使用 | ~80 KB |
| 烧录波特率 | 921600 baud |

### CPU 频率档位 (代码 130-131 行)

| 状态 | 频率 | 估计电流 |
|---|---|---|
| 活动工作 (CPU_FREQ_ACTIVE) | 240 MHz | ~80 mA |
| 屏保 (CPU_FREQ_IDLE) | 80 MHz | ~40 mA |
| Light Sleep | ~ | ~5-10 mA |
| Deep Sleep | ~ | ~10 μA |

---

## 二、引脚分配总表

### 显示器 ST7789 2.4" SPI (HSPI 总线, 40 MHz)

| 信号 | GPIO | 板上丝印 | 用途 |
|---|---|---|---|
| **TFT_CS** | **GPIO 21** | D10 | SPI 片选 |
| **TFT_DC** | **GPIO 18** | D9  | Data/Command 切换 |
| **TFT_RST** | **GPIO 17** | D8  | 硬复位 |
| **TFT_MOSI** | **GPIO 38** | D11 | SPI 主出从入 (HSPI MOSI) |
| **TFT_SCK** | **GPIO 48** | D13 | SPI 时钟 (HSPI SCK) |
| (TFT_MISO) | — | — | 未连接 (显示器单向只写) |
| **BLK 背光** | 3V3 | — | 接 3V3 永亮 (软件无控制) |
| VCC | 3V3 | — | 显示器电源 |
| GND | GND | — | 共地 |

### MS5837-30BA 压力传感器 (I²C, 400 kHz)

| 信号 | GPIO | 板上丝印 | 用途 |
|---|---|---|---|
| **I2C_SDA** | **GPIO 11** | D2 | I²C 数据 |
| **I2C_SCL** | **GPIO 12** | D3 | I²C 时钟 |
| VIN | 3V3 | — | 传感器电源 |
| GND | GND | — | 共地 |
| 100nF 电容 | VIN↔GND | — | 退耦 (必须) |

**I²C 地址**: `0x76` (MS5837-30BA 固定地址)

### 按钮 (内部上拉, 按下短接 GND)

| 信号 | GPIO | 板上丝印 | 用途 |
|---|---|---|---|
| **BTN_MODE_PIN** | **GPIO 6** | D4 | 模式键 (翻页 / 长按关机) |
| **BTN_UP_PIN** | **GPIO 7** | D5 | 上键 (最深0 / 设置增) |
| **BTN_DOWN_PIN** | **GPIO 10** | D7 | 下键 (计时重置 / 设置减) |

**按键时序**: 30ms 防抖 + 1200ms 长按阈值 + 5s 关机阈值。

### 蜂鸣器 (有源, 经 S8050 NPN 驱动)

| 信号 | GPIO | 板上丝印 | 用途 |
|---|---|---|---|
| **BUZZER_PIN** | **GPIO 5** | D1 | NPN 基极 (1kΩ 限流) |

**驱动电路**: GPIO 5 → 1kΩ → S8050 Base; Emitter→GND; Collector→蜂鸣器−; 蜂鸣器+ → 3V3。

### 电池电压检测 (ADC 分压)

| 信号 | GPIO | 板上丝印 | 用途 |
|---|---|---|---|
| **BAT_ADC_PIN** | **GPIO 1** | A0 | 电池电压 ADC (2:1 分压) |

**分压电路**: 电池 + → 4.7kΩ → ADC 引脚 → 4.7kΩ → GND。  
**软件校准系数** `g_batDivider = 2.10` (适配 ±5% 电阻容差, 可在设置菜单微调 1.80-2.30)。

### USB-C (板载, 自动复用)

| 信号 | 用途 |
|---|---|
| USB-C 5V | 充电 + 通讯 (经 TP4056 充电保护板) |
| USB D+/D− | USB CDC 串口 (CDCOnBoot=cdc 启用) |

---

## 三、未使用引脚 (可扩展)

下列引脚目前空闲，可用于未来扩展（磁力计/加速度计/外置 Flash 等）：

| GPIO | 板上丝印 | 推荐用途 |
|---|---|---|
| GPIO 2 | A1 | 备用 ADC |
| GPIO 3 | A2 | 备用 ADC / 触摸 |
| GPIO 4 | A3 | 备用 ADC / 触摸 |
| GPIO 8 | A4 | 备用 (磁力计 INT) |
| GPIO 9 | A5 | 备用 |
| GPIO 13 | A6 | 备用 |
| GPIO 14 | D6 | 备用 |
| GPIO 15 | — | 备用 |
| GPIO 16 | — | 备用 |
| GPIO 33-37 | — | PSRAM 用 (不能动) |
| GPIO 39-42 | — | 备用 |
| GPIO 45-47 | — | 备用 |

⚠️ **不能使用**：GPIO 19/20 (USB D+/D−), GPIO 0 (Boot 控制), GPIO 22-32 (内部 Flash/PSRAM)

---

## 四、外围模块 BOM

| 模块 | 型号 | 数量 | 接线引脚 |
|---|---|---|---|
| 主控板 | ESP32-S3-Nano (R8, 8MB Flash + 8MB PSRAM) | 1 | — |
| 显示模块 | ST7789 2.4" SPI 240×320 | 1 | GPIO 17/18/21/38/48 + 3V3/GND |
| 压力传感器 | MS5837-30BA | 1 | GPIO 11/12 + 3V3/GND |
| 蜂鸣器 | 9042 有源蜂鸣器 (3-5V) | 1 | GPIO 5 (经 NPN) |
| NPN 三极管 | S8050 | 1 | 驱动蜂鸣器 |
| 限流电阻 | 1 kΩ (棕黑红金) | 1 | S8050 Base |
| 分压电阻 | 4.7 kΩ ×2 | 2 | ADC 分压 |
| 充电板 | TP4056 + DW01 + FS8205 (带保护) | 1 | USB-C + 电池 |
| 锂电池 | 3.7V 850 mAh (50×30×5 mm) | 1 | TP4056 B+ / B− |
| 退耦电容 | 100 nF (104) | 1 | MS5837 VIN-GND |
| 按钮 | 4 脚扁平微动按钮 | 3 | GPIO 6/7/10 |

---

## 五、走线建议 (飞线方案)

```
ESP32-S3-Nano        外设
─────────────────────────────────
D2 (GPIO11) ──I2C── SDA  MS5837
D3 (GPIO12) ──I2C── SCL  MS5837
3V3        ──────── VIN  MS5837 (+ 100nF 退耦)
GND        ──────── GND  MS5837

D8 (GPIO17) ─SPI─── RST  ST7789
D9 (GPIO18) ─SPI─── DC   ST7789
D10(GPIO21) ─SPI─── CS   ST7789
D11(GPIO38) ─SPI─── SDA  ST7789 (MOSI)
D13(GPIO48) ─SPI─── SCL  ST7789 (SCK)
3V3        ──────── VCC + BLK
GND        ──────── GND

D1 (GPIO5)  ──1kΩ── Base S8050
GND        ──────── Emitter
                    Collector ── 蜂鸣器(−)
3V3        ──────── 蜂鸣器(+)

D4 (GPIO6)  ──────── 按钮 MODE ─── GND
D5 (GPIO7)  ──────── 按钮 UP   ─── GND
D7 (GPIO10) ──────── 按钮 DOWN ─── GND

A0 (GPIO1)  ─4.7k── 电池 BAT+
                   ─4.7k── GND  (分压中点)

TP4056:
  IN+/IN−  ──── USB-C
  B+/B−    ──── 电池
  OUT+     ──── ESP32-S3 5V (经板载稳压 → 3V3)
  OUT−     ──── GND
```

---

## 六、烧录命令

```bash
# 编译
arduino-cli compile \
  --fqbn esp32:esp32:esp32s3:PartitionScheme=huge_app,CDCOnBoot=cdc \
  arduino/DiveWatch_v4

# 烧录
arduino-cli upload \
  -p /dev/cu.usbmodem2101 \
  --fqbn esp32:esp32:esp32s3:PartitionScheme=huge_app,CDCOnBoot=cdc \
  arduino/DiveWatch_v4
```

⚠️ 首次烧录前需手动进入下载模式：**按住 BOOT + 短按 RST → 松开 RST → 松开 BOOT**。  
`CDCOnBoot=cdc` 启用后第二次起 esptool 自动 reset 进入下载模式。

---

## 七、功耗估算

| 场景 | 电流 | 850mAh 续航 |
|---|---|---|
| HUD 主屏运行 (240MHz, 屏幕亮) | ~120 mA | ~7 h |
| 屏保 + Light Sleep (80MHz, 屏幕仍亮) | ~50 mA | ~17 h |
| 屏保 + Light Sleep + 屏幕关 (理论) | ~10 mA | ~85 h |
| Deep Sleep (RTC 唤醒) | ~10 μA | 数年 |

⚠️ 当前 BLK 接 3V3 永亮，背光占主要功耗（~25 mA）。如需省电，可把 BLK 改接 GPIO 做 PWM 调光（**PINOUT 升级方案**）。

---

## 八、引脚定义索引 (代码位置)

| 定义 | 文件 | 行号 |
|---|---|---|
| TFT_CS / DC / RST / MOSI / SCK | DiveWatch_v4.ino | 38-42 |
| I2C_SDA / I2C_SCL | DiveWatch_v4.ino | 106-107 |
| BUZZER_PIN | DiveWatch_v4.ino | 108 |
| BTN_MODE_PIN / BTN_UP_PIN / BTN_DOWN_PIN | DiveWatch_v4.ino | 109-111 |
| BAT_ADC_PIN | DiveWatch_v4.ino | 112 |
| WIFI_SSID / NTP 服务器 | DiveWatch_v4.ino | 73-77 |
| CPU 频率档位 | DiveWatch_v4.ino | 129-130 |

如改动接线，只需改这些 `#define`，无需动其他代码。
