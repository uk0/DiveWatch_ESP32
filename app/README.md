# DiveWatch App

扁平化深色单页 PWA，蓝牙连接 DiveWatch ESP32-S3 设备。

## 功能

| 模块 | 协议 | 说明 |
|---|---|---|
| **扫描连接** | BLE (Web Bluetooth) | filter namePrefix=DiveWatch |
| **WiFi 配置** | BLE GATT 写 SSID/PSK/CMD | 设备连上 WiFi 后通过 STAT 通知反馈 IP |
| **OTA 升级** | WiFi HTTP POST /update | multipart 表单上传 .bin, 进度条 |
| **状态卡** | BLE STAT notify | 电池 / WiFi / IP 实时显示 |
| **快捷操作** | BLE CMD | C=连接 F=忘记 R=重启 |

## 运行

```bash
# 本机起静态 server (Web Bluetooth 要求 HTTPS 或 localhost)
cd app
python3 -m http.server 8080
# 浏览器开 http://localhost:8080
```

部署到 HTTPS 站点（GitHub Pages / Vercel）后可在手机上"添加到主屏幕"作为 PWA 使用。

## 浏览器兼容

| 平台 | 支持 |
|---|---|
| Android Chrome / Edge | ✅ |
| Windows / macOS Chrome / Edge | ✅ |
| iOS Safari | ❌ Apple 不支持 Web Bluetooth |
| iOS Bluefy 浏览器 | ✅ (App Store 免费) |

## BLE 协议

```
Service: 1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0
├ SSID (WRITE):    f7bf3564-fb6d-4e53-88a4-5e37e0326063
├ PSK  (WRITE):    984227f3-34fc-4045-a5d0-2c581f81a153
├ CMD  (WRITE):    0c533cef-d6b0-4ba2-9f08-7d6cea1b3e7e   1 byte: C/F/R
└ STAT (NOTIFY):   3b1f9c5e-8a72-4d11-9b3a-7c4e2f1a5d80   UTF-8 状态字符串
```

设备需在 OTA 模式下才广播 BLE（设置菜单第 14 项进入）。
