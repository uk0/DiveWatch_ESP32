// =====================================================================
// 刷不死方案 v2: BLE 配 WiFi → WiFi OTA (高速 + 远程)
// =====================================================================
// 流程:
//   1) 设备进入 OTA 模式 (任一方式触发)
//   2) BLE 广播 "DiveWatch-OTA", 用户手机/浏览器连接
//   3) 通过 BLE 写 WiFi SSID + PSK + "CONNECT" 命令
//   4) 设备连 WiFi → 显示 IP → 启动 ArduinoOTA (端口 3232)
//   5) 用户用 espota.py 或 Arduino IDE 推送固件 (WiFi 速度 ~500KB/s)
//   6) 固件完成后自动重启, 新固件 5s 后 mark_valid (失败自动回滚)
//
// 触发方式 (任一):
//   1) 长按 MODE+UP 3 秒
//   2) 开机时按住 UP+DOWN (救援模式, 主固件 brick 也能用)
//   3) NVS magic flag
//
// BLE GATT 协议:
//   Service: 1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0
//   SSID:    f7bf3564-...  (Write, UTF-8 字符串)
//   PSK:     984227f3-...  (Write, UTF-8 字符串)
//   CMD:     0c533cef-...  (Write, 1 字节命令)
//              'C' = CONNECT (用 SSID+PSK 连 WiFi)
//              'F' = FORGET  (清除 NVS WiFi 凭据)
//              'R' = REBOOT  (重启回主固件)
//   STAT:    3b1f9c5e-...  (Notify, UTF-8 状态字符串)
//              "WIFI_CONNECTING"
//              "WIFI_OK 192.168.1.50"
//              "WIFI_FAIL"
//              "OTA_READY"  (ArduinoOTA 服务已启动)
//              "OTA_PROGRESS 45"
//              "OTA_END"
//
// WiFi OTA 命令 (用户在 PC 上):
//   arduino-cli upload -p IP_ADDR -i IP_ADDR ...
//   或:
//   espota.py -i 192.168.1.50 -p 3232 -f DiveWatch_v4.ino.bin
// =====================================================================

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <esp_ota_ops.h>

#define OTA_SVC_UUID  "1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0"
#define OTA_SSID_UUID "f7bf3564-fb6d-4e53-88a4-5e37e0326063"
#define OTA_PSK_UUID  "984227f3-34fc-4045-a5d0-2c581f81a153"
#define OTA_CMD_UUID  "0c533cef-d6b0-4ba2-9f08-7d6cea1b3e7e"
#define OTA_STAT_UUID "3b1f9c5e-8a72-4d11-9b3a-7c4e2f1a5d80"

bool     g_otaMode      = false;
bool     g_otaWifiOk    = false;
bool     g_otaRunning   = false;
String   g_otaSsid      = "";
String   g_otaPsk       = "";
String   g_otaIp        = "";
int      g_otaProgress  = 0;
BLECharacteristic *g_otaStatCh = nullptr;

// 由主 sketch 提供
void otaDrawScreen(const char *line1, const char *line2, int pct);

// ---------------- 工具: 状态通知 ----------------
static void otaNotify(const char *msg) {
  Serial.printf("[OTA] %s\n", msg);
  if (g_otaStatCh) {
    g_otaStatCh->setValue((uint8_t*)msg, strlen(msg));
    g_otaStatCh->notify();
  }
}

// ---------------- WiFi 凭据持久化 ----------------
static void otaSaveWifiCreds(const String &ssid, const String &psk) {
  Preferences p;
  if (p.begin("ota", false)) {
    p.putString("ssid", ssid);
    p.putString("psk", psk);
    p.end();
  }
}
static bool otaLoadWifiCreds(String &ssid, String &psk) {
  Preferences p;
  if (!p.begin("ota", true)) return false;
  ssid = p.getString("ssid", "");
  psk  = p.getString("psk", "");
  p.end();
  return ssid.length() > 0;
}
static void otaForgetWifiCreds() {
  Preferences p;
  if (p.begin("ota", false)) {
    p.remove("ssid"); p.remove("psk");
    p.end();
  }
}

// ---------------- WiFi 连接 + 启动 ArduinoOTA ----------------
static void otaStartWifiOTA() {
  otaNotify("WIFI_CONNECTING");
  otaDrawScreen("连接 WiFi", g_otaSsid.c_str(), 0);
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_otaSsid.c_str(), g_otaPsk.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(200);
  }
  if (WiFi.status() != WL_CONNECTED) {
    otaNotify("WIFI_FAIL");
    otaDrawScreen("WiFi 连接失败", "请重试", -1);
    g_otaWifiOk = false;
    return;
  }
  g_otaWifiOk = true;
  g_otaIp = WiFi.localIP().toString();
  String msg = "WIFI_OK " + g_otaIp;
  otaNotify(msg.c_str());

  // 保存凭据
  otaSaveWifiCreds(g_otaSsid, g_otaPsk);

  // 启动 ArduinoOTA
  ArduinoOTA.setHostname("DiveWatch");
  ArduinoOTA.setPort(3232);
  // 无密码, 局域网内任何人可推 (内网信任) - 加密码改为:
  // ArduinoOTA.setPassword("yourpass");

  ArduinoOTA.onStart([]() {
    g_otaRunning = true;
    g_otaProgress = 0;
    otaNotify("OTA_BEGIN");
    otaDrawScreen("接收固件", "", 0);
  });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int total) {
    int pct = (total > 0) ? (int)(p * 100 / total) : 0;
    if (pct != g_otaProgress) {
      g_otaProgress = pct;
      char buf[32]; snprintf(buf, sizeof(buf), "OTA_PROGRESS %d", pct);
      otaNotify(buf);
      char ln[32]; snprintf(ln, sizeof(ln), "%u / %u KB", p/1024, total/1024);
      otaDrawScreen("接收固件", ln, pct);
    }
  });
  ArduinoOTA.onEnd([]() {
    otaNotify("OTA_END");
    otaDrawScreen("刷写完成", "重启中...", 100);
    delay(500);
  });
  ArduinoOTA.onError([](ota_error_t e) {
    char buf[40]; snprintf(buf, sizeof(buf), "OTA_ERROR %d", (int)e);
    otaNotify(buf);
    otaDrawScreen("OTA 错误", buf, -1);
  });
  ArduinoOTA.begin();
  otaNotify("OTA_READY");

  char ipLine[40];
  snprintf(ipLine, sizeof(ipLine), "IP: %s", g_otaIp.c_str());
  otaDrawScreen("OTA 已就绪", ipLine, 0);
}

// ---------------- BLE 回调 ----------------
class SsidCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    g_otaSsid = c->getValue();
    Serial.printf("[OTA] SSID=%s\n", g_otaSsid.c_str());
  }
};
class PskCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    g_otaPsk = c->getValue();
    Serial.println("[OTA] PSK received");
  }
};
class CmdCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue();
    if (v.length() == 0) return;
    char cmd = v[0];
    if (cmd == 'C') {
      if (g_otaSsid.length() == 0) { otaNotify("ERR_NO_SSID"); return; }
      otaStartWifiOTA();
    } else if (cmd == 'F') {
      otaForgetWifiCreds();
      otaNotify("WIFI_FORGOTTEN");
    } else if (cmd == 'R') {
      otaNotify("REBOOT");
      delay(300);
      ESP.restart();
    }
  }
};
class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer *) override     { Serial.println("[OTA] BLE connected"); }
  void onDisconnect(BLEServer *s) override { Serial.println("[OTA] BLE disconnected");
                                             s->startAdvertising(); }
};

// ---------------- 公共 API ----------------
void otaStartBLE() {
  if (g_otaMode) return;
  g_otaMode = true;

  // 仅启动 BLE 服务等待用户命令; 自动 WiFi 连接由用户触发 (避免 setup 早期 block 20s)
  BLEDevice::init("DiveWatch-OTA");
  BLEDevice::setMTU(247);
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new SrvCb());
  BLEService *svc = srv->createService(OTA_SVC_UUID);

  auto ssidCh = svc->createCharacteristic(OTA_SSID_UUID, BLECharacteristic::PROPERTY_WRITE);
  ssidCh->setCallbacks(new SsidCb());
  auto pskCh = svc->createCharacteristic(OTA_PSK_UUID, BLECharacteristic::PROPERTY_WRITE);
  pskCh->setCallbacks(new PskCb());
  auto cmdCh = svc->createCharacteristic(OTA_CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmdCh->setCallbacks(new CmdCb());
  g_otaStatCh = svc->createCharacteristic(OTA_STAT_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  g_otaStatCh->addDescriptor(new BLE2902());

  svc->start();
  auto adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(OTA_SVC_UUID);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("[OTA] BLE advertising as 'DiveWatch-OTA'");
  otaDrawScreen("OTA 模式", "BLE 配 WiFi 中...", 0);

  // BLE 启动后尝试自动连上次保存的 WiFi (非阻塞: 5s timeout)
  String savedSsid, savedPsk;
  if (otaLoadWifiCreds(savedSsid, savedPsk)) {
    g_otaSsid = savedSsid;
    g_otaPsk = savedPsk;
    Serial.printf("[OTA] auto-connect saved WiFi: %s\n", savedSsid.c_str());
    otaStartWifiOTA();
  }
}

void otaLoopTick() {
  if (!g_otaMode) return;
  if (g_otaWifiOk) {
    ArduinoOTA.handle();
  }
}

// ---------------- 主程序 hooks ----------------
bool otaCheckMagicFlag() {
  Preferences p;
  if (!p.begin("ota", true)) return false;
  bool flag = p.getBool("next_boot", false);
  p.end();
  if (flag) {
    if (p.begin("ota", false)) { p.remove("next_boot"); p.end(); }
    return true;
  }
  return false;
}

void otaScheduleNextBoot() {
  Preferences p;
  if (p.begin("ota", false)) { p.putBool("next_boot", true); p.end(); }
  Serial.println("[OTA] next boot → OTA mode");
  delay(200);
  ESP.restart();
}

static uint32_t g_otaValidArm = 0;
void otaArmValidTimer() { g_otaValidArm = millis(); }
void otaMarkValidIfReady() {
  if (g_otaValidArm == 0 || g_otaMode) return;
  if (millis() - g_otaValidArm > 5000) {
    g_otaValidArm = 0;
    esp_ota_img_states_t state;
    const esp_partition_t *p = esp_ota_get_running_partition();
    if (p && esp_ota_get_state_partition(p, &state) == ESP_OK
        && state == ESP_OTA_IMG_PENDING_VERIFY) {
      esp_ota_mark_app_valid_cancel_rollback();
      Serial.println("[OTA] app marked valid");
    }
  }
}

// 开机按 UP+DOWN → 救援模式
bool otaCheckBootRecovery() {
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  delay(80);
  if (digitalRead(BTN_UP_PIN) == LOW && digitalRead(BTN_DOWN_PIN) == LOW) {
    Serial.println("\n[BOOT-RECOVERY] UP+DOWN held, entering BLE+WiFi OTA");
    return true;
  }
  return false;
}
