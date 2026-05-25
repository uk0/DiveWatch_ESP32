// =====================================================================
// 刷不死方案 v3: NimBLE 配 WiFi + WiFi OTA (节省 ~150KB Flash)
// =====================================================================
#include <NimBLEDevice.h>
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
NimBLECharacteristic *g_otaStatCh = nullptr;

// 由主 sketch 提供
void otaDrawScreen(const char *line1, const char *line2, int pct);

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

// ---------------- WiFi + ArduinoOTA ----------------
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

  otaSaveWifiCreds(g_otaSsid, g_otaPsk);

  ArduinoOTA.setHostname("DiveWatch");
  ArduinoOTA.setPort(3232);
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

// ---------------- NimBLE 回调 ----------------
class SsidCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
    g_otaSsid = String(c->getValue().c_str());
    Serial.printf("[OTA] SSID=%s\n", g_otaSsid.c_str());
  }
};
class PskCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
    g_otaPsk = String(c->getValue().c_str());
    Serial.println("[OTA] PSK received");
  }
};
class CmdCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
    std::string v = c->getValue();
    if (v.empty()) return;
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
class SrvCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &info) override {
    Serial.println("[OTA] BLE connected");
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &info, int reason) override {
    Serial.printf("[OTA] BLE disconnected, reason=%d\n", reason);
    NimBLEDevice::startAdvertising();
  }
};

// ---------------- 公共 API ----------------
void otaStartBLE() {
  if (g_otaMode) return;
  g_otaMode = true;

  NimBLEDevice::init("DiveWatch-OTA");
  NimBLEDevice::setMTU(247);
  NimBLEServer *srv = NimBLEDevice::createServer();
  srv->setCallbacks(new SrvCb());
  NimBLEService *svc = srv->createService(OTA_SVC_UUID);

  auto ssidCh = svc->createCharacteristic(OTA_SSID_UUID, NIMBLE_PROPERTY::WRITE);
  ssidCh->setCallbacks(new SsidCb());
  auto pskCh = svc->createCharacteristic(OTA_PSK_UUID, NIMBLE_PROPERTY::WRITE);
  pskCh->setCallbacks(new PskCb());
  auto cmdCh = svc->createCharacteristic(OTA_CMD_UUID, NIMBLE_PROPERTY::WRITE);
  cmdCh->setCallbacks(new CmdCb());
  g_otaStatCh = svc->createCharacteristic(OTA_STAT_UUID, NIMBLE_PROPERTY::NOTIFY);

  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  // 显式配置 advertising data + scan response, 确保扫描器能看到 name 和 service
  NimBLEAdvertisementData advData;
  advData.setName("DiveWatch-OTA");
  advData.setFlags(0x06);                                  // LE General Discoverable + BR/EDR Not Supported
  adv->setAdvertisementData(advData);
  NimBLEAdvertisementData scanData;
  scanData.setCompleteServices(NimBLEUUID(OTA_SVC_UUID));
  adv->setScanResponseData(scanData);
  adv->enableScanResponse(true);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);                  // 最大发射功率 +9 dBm
  adv->start();

  Serial.println("[OTA] BLE advertising as 'DiveWatch-OTA' (TX +9dBm)");
  otaDrawScreen("OTA 模式", "BLE 配 WiFi 中...", 0);

  // 自动尝试连上次保存的 WiFi
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

  // 长按 MODE 3 秒 → 退出 OTA, 重启回主固件
  static uint32_t modePressMs = 0;
  static bool     modePinReady = false;
  if (!modePinReady) { pinMode(BTN_MODE_PIN, INPUT_PULLUP); modePinReady = true; }
  if (digitalRead(BTN_MODE_PIN) == LOW) {
    if (modePressMs == 0) {
      modePressMs = millis();
      Serial.println("[OTA] MODE pressed (hold 3s to exit)");
    } else if (millis() - modePressMs > 3000) {
      Serial.println("[OTA] MODE 3s → exit OTA, restart");
      otaDrawScreen("退出 OTA", "回主固件...", -1);
      delay(500);
      ESP.restart();
    }
  } else {
    if (modePressMs != 0) Serial.println("[OTA] MODE released");
    modePressMs = 0;
  }

  if (g_otaWifiOk) ArduinoOTA.handle();
}

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
