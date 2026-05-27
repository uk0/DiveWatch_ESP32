# DiveWatch Android App

Capacitor 7 + Vanilla JS + Chart.js + IndexedDB

## 功能

- **主页**: 大字电池 + 深度/温度/WiFi/IP 四宫格 + 深度实时曲线 + 累计统计
- **日志**: 历次潜水柱状图 + 列表（自动按 60s 静止判定潜水结束并落库）
- **工具**: BLE 扫描连接 / WiFi 配置 / WiFi OTA 推送 / 实时日志

## 存储

- 设置项 (WiFi SSID): `@capacitor/preferences`
- 潜水记录: IndexedDB (database `divewatch`, store `dives`)

## BLE

走 `@capacitor-community/bluetooth-le` 原生插件，Android 12+ 自动请求 BLUETOOTH_SCAN/CONNECT 权限。
浏览器开发时回退 Web Bluetooth。

## 构建

```bash
cd android_app
npm install
npm run build        # esbuild bundle www/app.js → www/bundle.js
npm run sync         # cap sync android (拷贝 www → android/app/src/main/assets/public)

# build APK (debug)
JAVA_HOME=/Library/Java/JavaVirtualMachines/temurin-25.jdk/Contents/Home \
  ./android/gradlew -p ./android assembleDebug

# 出 APK 路径
ls android/app/build/outputs/apk/debug/app-debug.apk
```

## Logo

由 gpt-image-2 生成 (`icon.png`)，已切成 5 档 mipmap 放入 `android/app/src/main/res/mipmap-*`.

## 安装到手机

1. 用 USB 把 `app-debug.apk` 拷到手机 (或 adb install)
2. 手机开 "未知来源安装" 权限
3. 双击 APK 安装
4. 打开 App → 工具页 → 扫描连接 (DiveWatch-OTA)
