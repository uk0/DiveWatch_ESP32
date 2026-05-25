# 刷不死方案: BLE 配 WiFi + WiFi OTA

## 架构

```
   ┌──────────┐      BLE (慢, 仅控制)        ┌────────────────┐
   │ 用户手机/ │ ─── SSID/PSK + CMD ────→    │  DiveWatch     │
   │ 浏览器    │                              │  ESP32-S3      │
   └──────────┘                              └────────────────┘
        ↓                                            ↓
        │                                       WiFi STA 模式
        │                                            ↓
        │                                     ArduinoOTA :3232
        │                                            ↑
        ↓ 通过家里 WiFi 路由器                       │
   ┌──────────┐    WiFi (快 ~500 KB/s)       ┌─────┴────────┐
   │ PC / 笔记本│ ────── 推 .bin 固件 ──→     │  ESP32-S3    │
   └──────────┘                              └──────────────┘
```

**核心思路**：BLE 只做"配网+控制"（数据量小），WiFi 做"高速传输"（固件 1.9MB / WiFi 4 秒，BLE 200 秒）。

## 三种触发方式

| 方式 | 操作 | 适用 |
|---|---|---|
| **A** | 长按 MODE+UP 3 秒 | 主固件正常, 想升级 |
| **B** | 开机时按 UP+DOWN (救援) | 主固件 brick |
| **C** | 串口/代码 `otaScheduleNextBoot()` | 远程/脚本触发 |

## 一次性首烧 (后面不再需要 USB)

```bash
# 编译 (使用自定义双 OTA 分区)
cd arduino
arduino-cli compile \
  --fqbn esp32:esp32:esp32s3:PartitionScheme=custom,CDCOnBoot=cdc \
  --build-property "build.custom_partitions=partitions" \
  DiveWatch_v4

# 按 BOOT+RST 进下载模式, 然后:
arduino-cli upload \
  -p /dev/cu.usbmodem2101 \
  --fqbn esp32:esp32:esp32s3:PartitionScheme=custom,CDCOnBoot=cdc \
  --build-property "build.custom_partitions=partitions" \
  DiveWatch_v4
```

## 日常 OTA 升级流程

### 第一次配 WiFi (BLE)

1. 设备进 OTA 模式 (任一触发方式)
2. 屏幕显示 "OTA 模式 / BLE 配 WiFi 中..."
3. 手机/电脑用 Chrome/Edge 打开 `ota_web_client.html`
4. 点 "连接 DiveWatch-OTA"
5. 填 WiFi 名 + 密码 → 点 "连接 WiFi"
6. 设备屏幕显示 "OTA 已就绪 / IP: 192.168.x.x"
7. 凭据自动保存到 NVS

### 后续 OTA (WiFi 自动连)

1. 设备进 OTA 模式 (任一方式)
2. **自动用保存的 WiFi 凭据连接** (无需 BLE)
3. 屏幕显示 "OTA 已就绪 / IP: ..."
4. 用 PC 推固件:

```bash
# 用 arduino-cli (推荐, 自动找 .bin)
arduino-cli upload -p 192.168.1.50 \
  --fqbn esp32:esp32:esp32s3:PartitionScheme=custom,CDCOnBoot=cdc \
  --build-property "build.custom_partitions=partitions" \
  DiveWatch_v4

# 或 espota.py 直接推 .bin
python3 ~/Library/Arduino15/packages/esp32/hardware/esp32/3.3.8/tools/espota.py \
  -i 192.168.1.50 -p 3232 \
  -f ~/Library/Caches/arduino/sketches/*/DiveWatch_v4.ino.bin

# 或 Arduino IDE: 工具 → 端口 → 选 DiveWatch (192.168.1.50)
```

5. 进度条到 100% 设备自动重启，新固件运行
6. **5 秒内成功启动则标记 valid，否则下次重启自动回滚**

## BLE GATT 协议

| Char | UUID | 操作 | 内容 |
|---|---|---|---|
| Service | `1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0` | - | OTA 服务 |
| SSID | `f7bf3564-fb6d-4e53-88a4-5e37e0326063` | Write | UTF-8 字符串 |
| PSK | `984227f3-34fc-4045-a5d0-2c581f81a153` | Write | UTF-8 字符串 |
| CMD | `0c533cef-d6b0-4ba2-9f08-7d6cea1b3e7e` | Write | 1 字节: `C` 连 WiFi / `F` 忘记 / `R` 重启 |
| STAT | `3b1f9c5e-8a72-4d11-9b3a-7c4e2f1a5d80` | Notify | UTF-8 状态: `WIFI_OK 192.168.x.x` / `OTA_PROGRESS 45` 等 |

## 自动回滚机制

1. 新固件第一次启动后，调用 `otaArmValidTimer()` 注册 5s 计时
2. 5s 后主程序在 loop 中调 `otaMarkValidIfReady()` → 标记当前 slot valid
3. 如果新固件在 5s 内崩溃 / 看门狗复位 / 卡死
4. 下次重启 ESP32 检测到 slot 没标 valid → 自动切回上一个 slot

**实测保护**：即使推了一个完全跑不起来的固件，5s 后断电重启，会自动恢复到上一版本。

## 安全性

- BLE 无加密 (信任内网环境)
- WiFi PSK 用 NVS 存储 (明文, 设备物理接触可读)
- ArduinoOTA 默认无密码 (内网信任), 可改 `ArduinoOTA.setPassword("...")` 加密码

如果要严格安全：
1. ArduinoOTA 加密码
2. BLE 加 GAP 配对
3. NVS 启用 encryption (esp32-encrypted-flash)

## 文件清单

| 文件 | 作用 |
|---|---|
| `partitions.csv` | 双 OTA 分区表 (3MB + 3MB + 2MB SPIFFS) |
| `ota_ble.ino` | BLE 配 WiFi + ArduinoOTA + 救援模式 |
| `DiveWatch_v4.ino` | 主固件 (含 OTA hooks) |
| `ota_web_client.html` | 浏览器 BLE WiFi 配置工具 |
| `OTA_GUIDE.md` | 本文档 |

## 故障排查

| 问题 | 原因 | 解决 |
|---|---|---|
| 屏幕显示 "WIFI_FAIL" | SSID/PSK 错 | BLE 重新配 |
| WiFi 连上但 OTA 推不进 | 防火墙拦 3232 端口 | PC 同一 WiFi, 关防火墙 |
| PC 看不到 OTA 端口 | mDNS 没解析 | 用 IP 直接连 `-p 192.168.x.x` |
| OTA 进度卡住 | WiFi 信号弱 | 靠近路由器 / 用 2.4G 频段 |
| 新固件白屏 | 编译错误/缺库 | 等 5s 断电, 自动回滚 |
| 不知道设备 IP | BLE 失效 / 屏幕坏 | 路由器后台查 `DiveWatch` 主机名 |

## 速度对比

| 方式 | 速度 | 1.9MB 固件耗时 |
|---|---|---|
| USB 921600 baud | 1.2 MB/s | ~15 秒 |
| **WiFi OTA** | **~500 KB/s** | **~4 秒** |
| BLE 5.0 直传 | ~12 KB/s | ~160 秒 |
| BLE 4.2 直传 | ~5 KB/s | ~400 秒 |

WiFi OTA 速度接近 USB，且不需要接线。
