// =====================================================================
// 刷不死方案 v3: NimBLE 配 WiFi + WiFi OTA (节省 ~150KB Flash)
// =====================================================================
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Update.h>
#include <esp_ota_ops.h>

WebServer g_otaWeb(80);

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
    const char *desc = "未知";
    switch (e) {
      case OTA_AUTH_ERROR:    desc = "认证失败"; break;
      case OTA_BEGIN_ERROR:   desc = "BEGIN失败"; break;
      case OTA_CONNECT_ERROR: desc = "连接失败"; break;
      case OTA_RECEIVE_ERROR: desc = "接收丢包"; break;
      case OTA_END_ERROR:     desc = "END校验失败"; break;
    }
    char buf[60]; snprintf(buf, sizeof(buf), "ERR %d %s", (int)e, desc);
    otaNotify(buf);
    otaDrawScreen("OTA 错误", buf, -1);
    Serial.printf("[OTA] error %d (%s), state can retry\n", (int)e, desc);
    // 不重启, 让 ArduinoOTA 内部可以接受下一次 invitation 重试
  });
  ArduinoOTA.begin();

  // ---- HTTP Web 服务 (端口 80) ----
  g_otaWeb.on("/", HTTP_GET, []() {
    String html = F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
                    "<title>DiveWatch OTA</title>"
                    "<style>body{font-family:-apple-system,sans-serif;max-width:480px;"
                    "margin:30px auto;padding:0 20px}h1{color:#FF6600}"
                    ".card{background:#f5f5f5;border-radius:8px;padding:16px;margin:12px 0}"
                    "button,input[type=submit]{background:#FF6600;color:#fff;border:0;"
                    "padding:12px 24px;border-radius:6px;font-size:15px;cursor:pointer;width:100%}"
                    "input[type=file]{width:100%;padding:10px;margin:8px 0}"
                    ".bar{background:#eee;border-radius:6px;height:24px;overflow:hidden;margin:12px 0}"
                    ".fill{background:#FF6600;height:100%;width:0;transition:width 0.2s}"
                    "</style></head><body>"
                    "<h1>🔄 DiveWatch OTA</h1>"
                    "<div class='card'>"
                    "<p>设备在线 ✓ &nbsp;|&nbsp; 内存 ");
    html += String(ESP.getFreeHeap()/1024) + "/" + String(ESP.getHeapSize()/1024) + " KB</p>"
            "<p>IP: " + g_otaIp + " &nbsp;|&nbsp; Sketch: " +
            String(ESP.getSketchSize()/1024) + " KB</p>"
            "<p>当前 slot: " + esp_ota_get_running_partition()->label +
            " &nbsp;|&nbsp; 下次写入: " + (esp_ota_get_next_update_partition(NULL) ?
              esp_ota_get_next_update_partition(NULL)->label : "?") + "</p>"
            "</div><div class='card'>"
            "<h3>上传固件</h3>"
            "<form id='f' method='POST' action='/update' enctype='multipart/form-data'>"
            "<input type='file' name='firmware' accept='.bin' required>"
            "<input type='submit' value='📤 开始刷写'>"
            "</form>"
            "<div class='bar'><div class='fill' id='b'></div></div>"
            "<div id='st'></div>"
            "</div><div class='card'>"
            "<button onclick=\"fetch('/restart').then(_=>alert('已重启'))\">↻ 重启设备</button>"
            "</div>"
            "<script>"
            "document.getElementById('f').onsubmit=function(e){e.preventDefault();"
            "var fd=new FormData(this);var x=new XMLHttpRequest();"
            "x.upload.onprogress=function(ev){if(ev.lengthComputable){"
            "var p=Math.round(ev.loaded*100/ev.total);"
            "document.getElementById('b').style.width=p+'%';"
            "document.getElementById('st').innerText='上传 '+p+'%'}};"
            "x.onload=function(){document.getElementById('st').innerText="
            "x.status==200?'✓ 完成! 设备重启中':'✗ 失败: '+x.responseText};"
            "x.open('POST','/update');x.send(fd);};"
            "</script></body></html>";
    g_otaWeb.send(200, "text/html; charset=utf-8", html);
  });

  g_otaWeb.on("/restart", HTTP_GET, []() {
    g_otaWeb.send(200, "text/plain", "restarting");
    delay(300); ESP.restart();
  });

  g_otaWeb.on("/update", HTTP_POST, []() {
    g_otaWeb.sendHeader("Connection", "close");
    g_otaWeb.send(Update.hasError() ? 500 : 200, "text/plain",
                  Update.hasError() ? Update.errorString() : "OK");
    delay(500); ESP.restart();
  }, []() {
    HTTPUpload &up = g_otaWeb.upload();
    if (up.status == UPLOAD_FILE_START) {
      Serial.printf("[OTA-HTTP] start: %s\n", up.filename.c_str());
      otaDrawScreen("HTTP 上传中", up.filename.c_str(), 0);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Serial.println(Update.errorString());
      }
    } else if (up.status == UPLOAD_FILE_WRITE) {
      if (Update.write(up.buf, up.currentSize) != up.currentSize) {
        Serial.println(Update.errorString());
      }
      static uint32_t lastShown = 0;
      if (millis() - lastShown > 200) {
        lastShown = millis();
        int pct = (int)(Update.progress() * 100 / (Update.size() ? Update.size() : 1));
        char ln[32]; snprintf(ln, sizeof(ln), "%u KB", up.totalSize/1024);
        otaDrawScreen("HTTP 上传", ln, pct);
      }
    } else if (up.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("[OTA-HTTP] done: %u bytes\n", up.totalSize);
        otaDrawScreen("刷写完成", "重启中...", 100);
      } else {
        Serial.printf("[OTA-HTTP] end fail: %s\n", Update.errorString());
        otaDrawScreen("END 失败", Update.errorString(), -1);
      }
    }
  });

  g_otaWeb.begin();
  Serial.printf("[OTA] HTTP server on http://%s/\n", g_otaIp.c_str());
  otaNotify("OTA_READY");

  char ipLine[40];
  snprintf(ipLine, sizeof(ipLine), "http://%s/", g_otaIp.c_str());
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

  // MODE 短按双击 (400ms 内 2 次) → 快速退出 OTA
  // 或长按 1 秒 → 也退出 (单手友好)
  static bool     modePinReady = false;
  static bool     modeLastState = true;     // pull-up 默认 HIGH=true
  static uint32_t modePressMs   = 0;
  static uint32_t lastClickMs   = 0;
  static uint8_t  clickCount    = 0;
  if (!modePinReady) { pinMode(BTN_MODE_PIN, INPUT_PULLUP); modePinReady = true; }
  bool modeNow = (digitalRead(BTN_MODE_PIN) == HIGH);    // true=松开, false=按下

  // 边沿检测
  if (modeLastState && !modeNow) {           // 按下沿
    modePressMs = millis();
  } else if (!modeLastState && modeNow) {    // 释放沿
    uint32_t pressDur = millis() - modePressMs;
    if (pressDur < 600) {                    // 短按
      if (millis() - lastClickMs < 400) {
        clickCount++;
        Serial.printf("[OTA] MODE double-click (count=%d)\n", clickCount);
        if (clickCount >= 2) {
          Serial.println("[OTA] MODE 双击 → 退出 OTA, restart");
          otaDrawScreen("退出 OTA", "回主固件...", -1);
          delay(400);
          ESP.restart();
        }
      } else {
        clickCount = 1;
      }
      lastClickMs = millis();
    }
  }
  // 长按 1 秒 (单击不松手) → 直接退
  if (!modeNow && modePressMs > 0 && millis() - modePressMs > 1000) {
    Serial.println("[OTA] MODE long press 1s → exit");
    otaDrawScreen("退出 OTA", "回主固件...", -1);
    delay(400);
    ESP.restart();
  }
  modeLastState = modeNow;

  if (g_otaWifiOk) {
    ArduinoOTA.handle();
    g_otaWeb.handleClient();
  }
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
  // 开机时按 UP 或 DOWN 任意一个 → 进 OTA 救援模式
  bool up = (digitalRead(BTN_UP_PIN) == LOW);
  bool dn = (digitalRead(BTN_DOWN_PIN) == LOW);
  if (up || dn) {
    Serial.printf("\n[BOOT-RECOVERY] %s held, entering OTA\n", up ? "UP" : "DOWN");
    return true;
  }
  return false;
}
