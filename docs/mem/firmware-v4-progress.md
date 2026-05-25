# DiveWatch v4 固件开发进度快照 (2026-05-26)

## 当前固件版本
- 主目录: `arduino/DiveWatch_v4/`
- 文件:
  - `DiveWatch_v4.ino` (主固件, ~3400 行)
  - `ota_ble.ino` (BLE + WiFi OTA 模块)
  - `partitions.csv` (已删, 改用预设 `min_spiffs`)
  - `OTA_GUIDE.md` (OTA 使用文档)
  - `ota_web_client.html` (浏览器 BLE 配 WiFi 工具)

## 编译/烧录命令
```bash
# 编译 + USB 烧录
arduino-cli compile --upload \
  -p /dev/cu.usbmodem2101 \
  --fqbn esp32:esp32:esp32s3:PartitionScheme=min_spiffs,CDCOnBoot=cdc \
  DiveWatch_v4

# WiFi OTA (浏览器访问 http://<IP>/ 上传 .bin)
# 在 OTA 模式下设备显示 IP, 浏览器打开即可
```

## 硬件
- ESP32-S3-Nano (4MB Flash, 8MB PSRAM)
- ST7789 2.4 寸 SPI 屏 240x320 (HSPI 40MHz)
- MS5837-30BA 压力传感器 (I2C 0x76)
- 3 按钮 (GPIO 6/7/10) + 蜂鸣器 (GPIO 5) + 电池 ADC (GPIO 1)
- 4MB Flash 限制下 sketch ~1.84MB, min_spiffs 双 OTA slot 1.96MB 各

## 已实现功能
1. **Garmin Descent X50i 风格 UI**: 11 个页面 (HUD/PROFILE/TISSUE/N2/PLAN/AIR/TEMP/BAT/LASTDIVE/LOG/STATS) + 设置 + 时间编辑 + 屏保 + 全屏告警
2. **ZHL-16C Bühlmann 减压算法** + 16 组织室 + 海拔补偿 + 保守因子
3. **安全停留 AOW 标准** (2.5-6m 窗口, 累积计时, 30s 抖动容差)
4. **TTS + Ceiling 计算** (上升到水面时间, 含减压停留)
5. **气瓶 SPG**: 初始 bar / 瓶容 / SAC / Air Time 估算
6. **进出水画面**: 进入 1.5s "GO!", 退出 5s 总结
7. **WiFi NTP 校时** (用户填 SSID/PSK)
8. **Light sleep + Deep sleep** 省电
9. **配色**: 橙底 + 黑文字 + 红警告 (Shearwater 水下哲学)
10. **OTA 防砖**: BLE 配 WiFi → WiFi OTA / HTTP Web 上传 / 双 slot 自动回滚

## OTA 防砖架构
- **双 OTA 分区** (min_spiffs: 1.94MB×2 + 128KB SPIFFS)
- **触发方式** (4 种):
  - 设置菜单 → 第 14 项「进入 OTA」→ UP/DOWN 确认
  - 长按 UP 5 秒 / DOWN 5 秒
  - 串口发 `OTA\n`
  - 开机时按 UP **或** DOWN (救援模式, 跳过所有初始化)
- **退出 OTA**: MODE 双击 (400ms 内 2 次) / 长按 1 秒
- **OTA 通道**:
  - HTTP Web (端口 80, 浏览器访问 `http://<IP>/` 上传)
  - ArduinoOTA (端口 3232, `espota.py` 命令行)
  - BLE 配 WiFi (NimBLE-Arduino 2.5.0)
- **自动回滚**: 新固件 5s 内未 `mark_valid` 下次重启回上一 slot

## 屏幕刷新优化
- HUD 渲染 100ms (10Hz)
- 缓存层: time/HH:MM, DIVE 状态, 用时秒, 电量, 海/淡, 大字深度, NDL, 安停, 速度条, 数据条 4 项, AIR (静态)
- 上升速度阈值 0.5 m/min (避免传感器噪声触发重画)

## 已知问题/待办
- 4MB Flash 限制下 sketch 91% 占用, 加新功能需先瘦身
- ArduinoOTA UDP invitation 在某些路由器下不稳, 推荐用 HTTP Web 上传
- 字体合并副作用: wqy13→wqy12 (字号小 1px), logisoso26→24

## 关键文件大小
- DiveWatch_v4.ino: ~3400 行 / ~120 KB
- ota_ble.ino: ~250 行
- 编译产物 .bin: ~1.84 MB

## Git 历史 (最近)
```
7aa8a60 HUD 屏闪深度优化: 用时缓存(1Hz)+AIR静态(0Hz)+上升阈值放宽(0.5m/min)
b840627 OTA + 屏幕刷新优化
24d6283 OTA: 加 HTTP Web 80 端口
ca6908e v4 OTA: BLE 广播显式 name+UUID, TX +9dBm; 长按 MODE 3s 退出 OTA
0b55918 v4 OTA: 加设置菜单第 14 项 '进入 OTA' 避免按键冲突
1d654d3 v4 OTA 双 slot 启用: NimBLE + 字体合并 (sketch 2.06→1.80MB)
```
