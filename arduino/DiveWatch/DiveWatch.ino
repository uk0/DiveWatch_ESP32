// =====================================================================
//  DiveWatch v2.0  -  ESP32-S3-Nano + MS5837-30BA + OLED SH1106 + Buzzer
//
//  Features:
//    - ZHL-16C 16-compartment Buhlmann NDL calculation (air only)
//    - 5m +/- 1m safety stop detection with 3 min timer
//    - Moving-average noise filter on depth/pressure
//    - 3-button UI (MODE/UP/DOWN), short + long press
//    - NVS persistent dive log (last 10 dives)
//    - 4 pages: HUD, Tissue/NDL, Last dive, Log list
//
//  Pin map (Arduino Nano ESP32 board labels):
//    A4 (GPIO 11) = SDA  [shared with OLED]
//    A5 (GPIO 12) = SCL  [shared with OLED]
//    D2 (GPIO  5) = Buzzer (via 1k -> S8050 base)
//    D3 (GPIO  6) = BTN_MODE  (active-low to GND, internal pullup)
//    D4 (GPIO  7) = BTN_UP    (active-low to GND, internal pullup)
//    D7 (GPIO 10) = BTN_DOWN  (active-low to GND, internal pullup)
//
//  Board: ESP32S3 Dev Module
//    USBMode=hwcdc, CDCOnBoot=cdc, FlashSize=8M, PSRAM=opi
// =====================================================================

#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <WiFi.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "MS5837.h"

// =====================================================================
// WiFi NTP 校时配置 (用户修改这两行后烧录即可启用)
//   留空字符串 "" 表示禁用 WiFi 校时
// =====================================================================
static const char* WIFI_SSID = "";
static const char* WIFI_PASS = "";
static const long  TZ_OFFSET_SEC = 8 * 3600;  // UTC+8 (中国)
static const char* NTP_SERVER1 = "ntp.aliyun.com";
static const char* NTP_SERVER2 = "ntp.tencent.com";
static const char* NTP_SERVER3 = "pool.ntp.org";

// ================== Forward type declarations =======================
// (Required so Arduino auto-generated function prototypes can reference them)
struct Button {
  uint8_t  pin;
  bool     stable;
  bool     last;
  uint32_t lastChangeMs;
  uint32_t pressedAtMs;
  bool     longFired;
  bool     evShort;
  bool     evLong;
};

struct DiveRecord {
  float    maxDepth;
  uint16_t durationSec;
  float    minTemp;
  uint8_t  saltwater;     // 1 = sea, 0 = fresh
  uint8_t  _pad;
  uint32_t endEpoch;      // Unix time when dive ended (0 = unknown)
};

// ================== Pin map ==========================================
#define I2C_SDA       11
#define I2C_SCL       12
#define BUZZER_PIN     5
#define BTN_MODE_PIN   6
#define BTN_UP_PIN     7
#define BTN_DOWN_PIN  10
#define BAT_ADC_PIN    1     // board pin "A0", reads battery via 2:1 divider

// Voltage divider: BAT+ -[100k]- ADC -[100k]- GND
static const float BAT_DIVIDER  = 2.0f;
static const float BAT_FULL_V   = 4.20f;
static const float BAT_EMPTY_V  = 3.30f;

// ================== 固定常量 (不可调) ================================
static const float    SURFACE_DEPTH       = 0.5f;
static const float    DIVE_START_DEPTH    = 1.2f;
static const float    SAFETY_STOP_DEPTH   = 5.0f;
static const float    SAFETY_STOP_BAND    = 1.5f;
static const uint32_t SAMPLE_MS           = 200;
static const uint32_t REARM_BEEP_MS       = 1500;
static const uint8_t  AVG_WINDOW          = 5;

// ================== 可调参数 (设置菜单可改, 持久化) ==================
float    g_alarmDepth      = 30.0f;     // 深度报警 m
float    g_ascentLimit     = 9.0f;      // 上升速率警告 m/min
uint32_t g_screenOffMs     = 5UL*60*1000;  // 屏保超时 ms
uint16_t g_safetyStopSec   = 180;       // 安全停留时长 s
bool     g_buzzerEnable    = true;      // 蜂鸣器开关
float    g_fluidDensity    = 1029.0f;   // 水密度 (1029 海, 997 淡)

// ================== Globals ==========================================
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
MS5837 sensor;
Preferences prefs;

float    g_surfacePressure = 1013.25f;
float    g_lastDepth       = 0.0f;
float    g_maxDepth        = 0.0f;
uint32_t g_lastSampleMs    = 0;
uint32_t g_lastBeepMs      = 0;
uint32_t g_diveStartMs     = 0;
bool     g_diving          = false;
bool     g_sensorOk        = false;
float    g_temp            = 0.0f;
float    g_pressureRaw     = 0.0f;
float    g_depthRaw        = 0.0f;
float    g_depthSmooth     = 0.0f;
float    g_ascentMpm       = 0.0f;
float    g_batVoltage      = 0.0f;
uint8_t  g_batPct          = 0;
bool     g_batPresent      = false;
uint32_t g_lastBatReadMs   = 0;

// Real-time clock (uses internal RTC, persisted to NVS every minute)
bool     g_editingTime     = false;
uint8_t  g_editField       = 0;       // 0 = HH, 1 = MM
uint8_t  g_editHH          = 12;
uint8_t  g_editMM          = 0;
uint32_t g_lastTimeSaveMs  = 0;

// Power saving: dim OLED after idle period (g_screenOffMs is configurable)
uint32_t g_lastInteractMs  = 0;
bool     g_screenOff       = false;

// Settings menu state
bool     g_inSettings        = false;
uint8_t  g_settingsItem      = 0;
static const uint8_t SETTINGS_COUNT = 6;

// Lifetime stats (separate from per-dive log, also in NVS)
uint32_t g_lifetimeUnderwaterSec = 0;

// Temperature history (1 sample/min, ring buffer of 60 = 1 hour)
static const uint8_t TEMP_HIST_LEN = 60;
float    g_tempHist[TEMP_HIST_LEN] = {0};
uint8_t  g_tempHistIdx   = 0;
uint8_t  g_tempHistCount = 0;
uint32_t g_lastTempSampleMs = 0;

// ================== Moving-average filter ============================
float    g_pressBuf[AVG_WINDOW] = {0};
uint8_t  g_pressBufIdx          = 0;
bool     g_pressBufFilled       = false;

float pushPressureAndAverage(float p) {
  g_pressBuf[g_pressBufIdx++] = p;
  if (g_pressBufIdx >= AVG_WINDOW) { g_pressBufIdx = 0; g_pressBufFilled = true; }
  uint8_t n = g_pressBufFilled ? AVG_WINDOW : g_pressBufIdx;
  float s = 0;
  for (uint8_t i = 0; i < n; i++) s += g_pressBuf[i];
  return s / n;
}

void resetPressureFilter() {
  g_pressBufIdx = 0;
  g_pressBufFilled = false;
}

// ================== ZHL-16C Buhlmann (air, N2 only) ==================
// 16 theoretical tissue compartments. Schreiner equation, instant constant depth.
// a/b parameters from ZHL-16C, atm units (1 bar = 0.987 atm; we work in bar).
static const float HALF_TIMES_N2[16] = {
   4.0f,   8.0f,  12.5f,  18.5f,  27.0f,  38.3f,  54.3f,  77.0f,
 109.0f, 146.0f, 187.0f, 239.0f, 305.0f, 390.0f, 498.0f, 635.0f
};
static const float A_N2[16] = {
  1.2599f, 1.0000f, 0.8618f, 0.7562f, 0.6667f, 0.5933f, 0.5282f, 0.4701f,
  0.4187f, 0.3798f, 0.3497f, 0.3223f, 0.2971f, 0.2737f, 0.2523f, 0.2327f
};
static const float B_N2[16] = {
  0.5050f, 0.6514f, 0.7222f, 0.7825f, 0.8126f, 0.8434f, 0.8693f, 0.8910f,
  0.9092f, 0.9222f, 0.9319f, 0.9403f, 0.9477f, 0.9544f, 0.9602f, 0.9653f
};

// Inert gas tension in each compartment, units = bar (N2 partial pressure)
float g_pN2[16];

// Air = 79% N2, surface ambient = 1.013 bar -> initial = 0.79 * 1.013
void initCompartmentsToSurface() {
  float pAlv0 = 0.79f * (g_surfacePressure / 1000.0f);
  for (int i = 0; i < 16; i++) g_pN2[i] = pAlv0;
}

// Update each compartment using Schreiner equation, dt in seconds, depth in m
void updateCompartments(float dt_sec, float depth_m) {
  float pAmb_bar = (g_surfacePressure / 1000.0f) + depth_m * g_fluidDensity * 9.80665f / 1e5f;
  float pAlv     = 0.79f * pAmb_bar;
  for (int i = 0; i < 16; i++) {
    float k = 0.6931472f / (HALF_TIMES_N2[i] * 60.0f);   // ln(2)/T1/2 (per second)
    g_pN2[i] += (pAlv - g_pN2[i]) * (1.0f - expf(-k * dt_sec));
  }
}

// NDL in minutes: time at current depth until any compartment reaches its M-value.
// Returns 99.0 (cap) if no compartment is loading or already saturated.
float computeNDL(float depth_m) {
  float pAmb_bar = (g_surfacePressure / 1000.0f) + depth_m * g_fluidDensity * 9.80665f / 1e5f;
  float pAlv     = 0.79f * pAmb_bar;
  float ndl_min  = 99.0f;
  for (int i = 0; i < 16; i++) {
    float Mv = pAmb_bar / B_N2[i] + A_N2[i];   // max allowed N2 tension
    if (g_pN2[i] >= Mv) return 0.0f;
    if (pAlv <= g_pN2[i]) continue;            // not loading
    float ratio = 1.0f - (Mv - g_pN2[i]) / (pAlv - g_pN2[i]);
    if (ratio <= 0.0f) continue;
    float k = 0.6931472f / (HALF_TIMES_N2[i] * 60.0f);
    float t_sec = -logf(ratio) / k;
    float t_min = t_sec / 60.0f;
    if (t_min < ndl_min) ndl_min = t_min;
  }
  return ndl_min;
}

// Highest tissue load percent vs M-value at current depth (0..100+)
float computeTissueLoadPct(float depth_m) {
  float pAmb_bar = (g_surfacePressure / 1000.0f) + depth_m * g_fluidDensity * 9.80665f / 1e5f;
  float maxPct = 0;
  for (int i = 0; i < 16; i++) {
    float Mv = pAmb_bar / B_N2[i] + A_N2[i];
    float pct = (g_pN2[i] / Mv) * 100.0f;
    if (pct > maxPct) maxPct = pct;
  }
  return maxPct;
}

// ================== Safety stop state ================================
enum SSState : uint8_t { SS_IDLE = 0, SS_ARMED, SS_RUNNING, SS_DONE };
SSState  g_ss          = SS_IDLE;
uint32_t g_ssEnterMs   = 0;
uint32_t g_ssAccumMs   = 0;

// Arm when the diver has been below 10 m at some point during this dive.
bool g_ssArmCondition = false;

void updateSafetyStop(float depth_m, uint32_t now) {
  if (!g_diving) { g_ss = SS_IDLE; g_ssAccumMs = 0; g_ssArmCondition = false; return; }
  if (depth_m > 10.0f) g_ssArmCondition = true;
  if (!g_ssArmCondition) return;

  bool inBand = fabsf(depth_m - SAFETY_STOP_DEPTH) <= SAFETY_STOP_BAND;
  switch (g_ss) {
    case SS_IDLE:
      if (depth_m < SAFETY_STOP_DEPTH + SAFETY_STOP_BAND) g_ss = SS_ARMED;
      break;
    case SS_ARMED:
      if (inBand) { g_ss = SS_RUNNING; g_ssEnterMs = now; }
      break;
    case SS_RUNNING:
      if (inBand) {
        g_ssAccumMs = now - g_ssEnterMs;
        if (g_ssAccumMs >= g_safetyStopSec * 1000UL) g_ss = SS_DONE;
      } else {
        g_ss = SS_ARMED;     // left band, will resume when re-enters
        g_ssEnterMs = now;
      }
      break;
    case SS_DONE: break;
  }
}

// ================== Buttons ==========================================
static const uint16_t DEBOUNCE_MS  = 30;
static const uint16_t LONG_PRESS_MS = 1200;

Button g_btnMode = {BTN_MODE_PIN, true, true, 0, 0, false, false, false};
Button g_btnUp   = {BTN_UP_PIN,   true, true, 0, 0, false, false, false};
Button g_btnDown = {BTN_DOWN_PIN, true, true, 0, 0, false, false, false};

void btnPoll(Button &b, uint32_t now) {
  bool raw = digitalRead(b.pin) == HIGH;       // HIGH = released (pullup)
  if (raw != b.last) { b.last = raw; b.lastChangeMs = now; }
  if (now - b.lastChangeMs >= DEBOUNCE_MS && b.stable != b.last) {
    bool wasReleased = b.stable;
    b.stable = b.last;
    if (wasReleased && !b.stable) {
      // release -> press
      b.pressedAtMs = now;
      b.longFired = false;
    } else if (!wasReleased && b.stable) {
      // press -> release: short press if not long-fired
      if (!b.longFired && (now - b.pressedAtMs) < LONG_PRESS_MS) b.evShort = true;
    }
  }
  // Long press fires while held
  if (!b.stable && !b.longFired && (now - b.pressedAtMs) >= LONG_PRESS_MS) {
    b.evLong = true;
    b.longFired = true;
  }
}

// ================== UI pages =========================================
enum Page : uint8_t { PAGE_HUD = 0, PAGE_TISSUE, PAGE_TEMP, PAGE_LASTDIVE, PAGE_LOG, PAGE_STATS, PAGE_COUNT };
uint8_t g_page = PAGE_HUD;
uint8_t g_logViewIdx = 0;

// ================== NVS Dive Log =====================================
static const uint8_t LOG_CAP = 10;
DiveRecord g_log[LOG_CAP];
uint8_t    g_logCount = 0;
uint16_t   g_diveTotal = 0;     // lifetime dive count
float      g_diveTotalMaxD = 0; // lifetime max depth
float      g_diveMinTemp = 999;

void saveSettingsToNVS() {
  prefs.begin("dive", false);
  prefs.putFloat("alarmD",   g_alarmDepth);
  prefs.putFloat("ascentL",  g_ascentLimit);
  prefs.putULong("scrOffMs", g_screenOffMs);
  prefs.putUShort("ssSec",   g_safetyStopSec);
  prefs.putBool("buzzer",    g_buzzerEnable);
  prefs.putFloat("density",  g_fluidDensity);
  prefs.end();
  Serial.println("[SET] Settings saved");
}

void loadSettingsFromNVS() {
  prefs.begin("dive", true);
  g_alarmDepth     = prefs.getFloat("alarmD",   30.0f);
  g_ascentLimit    = prefs.getFloat("ascentL",   9.0f);
  g_screenOffMs    = prefs.getULong("scrOffMs", 5UL * 60UL * 1000UL);
  g_safetyStopSec  = prefs.getUShort("ssSec",   180);
  g_buzzerEnable   = prefs.getBool("buzzer",    true);
  g_fluidDensity   = prefs.getFloat("density",  1029.0f);
  prefs.end();
}

void loadLogFromNVS() {
  prefs.begin("dive", true);
  g_logCount             = prefs.getUChar("count", 0);
  g_diveTotal            = prefs.getUShort("total", 0);
  g_diveTotalMaxD        = prefs.getFloat("totalMaxD", 0);
  g_lifetimeUnderwaterSec = prefs.getULong("uwSec", 0);
  if (g_logCount > LOG_CAP) g_logCount = LOG_CAP;
  for (uint8_t i = 0; i < g_logCount; i++) {
    char key[8];
    snprintf(key, sizeof(key), "r%u", i);
    size_t got = prefs.getBytes(key, &g_log[i], sizeof(DiveRecord));
    if (got != sizeof(DiveRecord)) g_log[i] = {0, 0, 0, 1};
  }
  prefs.end();
}

void saveDiveToNVS(const DiveRecord &rec) {
  // Shift right (newest at index 0)
  if (g_logCount < LOG_CAP) g_logCount++;
  for (int i = g_logCount - 1; i > 0; i--) g_log[i] = g_log[i - 1];
  g_log[0] = rec;
  g_diveTotal++;
  if (rec.maxDepth > g_diveTotalMaxD) g_diveTotalMaxD = rec.maxDepth;
  g_lifetimeUnderwaterSec += rec.durationSec;

  prefs.begin("dive", false);
  prefs.putUChar("count", g_logCount);
  prefs.putUShort("total", g_diveTotal);
  prefs.putFloat("totalMaxD", g_diveTotalMaxD);
  prefs.putULong("uwSec", g_lifetimeUnderwaterSec);
  for (uint8_t i = 0; i < g_logCount; i++) {
    char key[8];
    snprintf(key, sizeof(key), "r%u", i);
    prefs.putBytes(key, &g_log[i], sizeof(DiveRecord));
  }
  prefs.end();
}

void eraseLogNVS() {
  prefs.begin("dive", false);
  prefs.clear();
  prefs.end();
  g_logCount = 0;
  g_diveTotal = 0;
  g_diveTotalMaxD = 0;
  g_lifetimeUnderwaterSec = 0;
}

// ================== Real-time clock ==================================
void setSystemClock(uint8_t hh, uint8_t mm, uint8_t ss) {
  struct tm t = {};
  t.tm_year = 2026 - 1900;
  t.tm_mon  = 0;          // January
  t.tm_mday = 1;
  t.tm_hour = hh;
  t.tm_min  = mm;
  t.tm_sec  = ss;
  time_t epoch = mktime(&t);
  struct timeval tv = {epoch, 0};
  settimeofday(&tv, nullptr);
}

void getCurrentTime(uint8_t &hh, uint8_t &mm, uint8_t &ss) {
  time_t now;
  time(&now);
  struct tm *t = localtime(&now);
  hh = t->tm_hour;
  mm = t->tm_min;
  ss = t->tm_sec;
}

void saveClockToNVS() {
  time_t now;
  time(&now);
  prefs.begin("dive", false);
  prefs.putULong("clock", (unsigned long)now);
  prefs.end();
}

void loadClockFromNVS() {
  prefs.begin("dive", true);
  unsigned long saved = prefs.getULong("clock", 0);
  prefs.end();
  if (saved > 0) {
    struct timeval tv = {(time_t)saved, 0};
    settimeofday(&tv, nullptr);
  } else {
    // First boot ever -> default to 12:00:00
    setSystemClock(12, 0, 0);
  }
}

// 尝试通过 WiFi + NTP 同步时间, 完成后关闭 WiFi 省电
// 返回 true = 同步成功, false = 失败 (无 SSID / 连不上 / NTP 超时)
bool trySyncNTP() {
  if (strlen(WIFI_SSID) == 0) {
    Serial.println("[NTP] WIFI_SSID 未配置, 跳过");
    return false;
  }
  Serial.printf("[NTP] 连接 WiFi: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(120);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[NTP] WiFi 超时, 放弃");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }
  Serial.printf("[NTP] WiFi OK, IP=%s, 同步 NTP...\n", WiFi.localIP().toString().c_str());
  configTime(TZ_OFFSET_SEC, 0, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);

  time_t now = 0;
  start = millis();
  while (now < 1700000000 && millis() - start < 5000) {
    delay(200);
    time(&now);
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  if (now > 1700000000) {
    saveClockToNVS();
    Serial.printf("[NTP] 同步成功: epoch=%lu\n", (unsigned long)now);
    return true;
  }
  Serial.println("[NTP] NTP 超时");
  return false;
}

// ================== Battery ==========================================
float readBatteryVoltage() {
  uint32_t sum = 0;
  const int N = 16;
  for (int i = 0; i < N; i++) sum += analogRead(BAT_ADC_PIN);
  float v_adc = (sum / (float)N) * (3.3f / 4095.0f);
  return v_adc * BAT_DIVIDER;
}

uint8_t voltageToPct(float v) {
  if (v >= BAT_FULL_V) return 100;
  if (v <= BAT_EMPTY_V) return 0;
  // Piecewise linear curve approximating Li-ion discharge
  static const float pts[][2] = {
    {4.20f, 100}, {4.10f, 87}, {4.00f, 75}, {3.90f, 60},
    {3.80f, 45}, {3.70f, 30}, {3.60f, 17}, {3.50f, 8},
    {3.40f, 3},  {3.30f, 0}
  };
  for (int i = 0; i < 9; i++) {
    if (v <= pts[i][0] && v >= pts[i+1][0]) {
      float t = (v - pts[i+1][0]) / (pts[i][0] - pts[i+1][0]);
      return (uint8_t)(pts[i+1][1] + t * (pts[i][1] - pts[i+1][1]));
    }
  }
  return 0;
}

void updateBattery(uint32_t now) {
  if (now - g_lastBatReadMs < 5000) return;
  g_lastBatReadMs = now;
  g_batVoltage = readBatteryVoltage();
  // Only consider valid Li-ion range; floating ADC pin gives ~5V which
  // would otherwise be mis-read as a battery.
  g_batPresent = (g_batVoltage >= 3.0f && g_batVoltage <= 4.5f);
  g_batPct     = g_batPresent ? voltageToPct(g_batVoltage) : 0;
}

// Draws a small battery icon at (x, y), 18 wide x 7 tall (incl. tip)
void drawBatteryIcon(int x, int y) {
  if (!g_batPresent) return;
  int bodyW = 16, bodyH = 7;
  u8g2.drawFrame(x, y, bodyW, bodyH);
  u8g2.drawBox(x + bodyW, y + 2, 2, 3);  // tip
  int fill = (int)((bodyW - 2) * (g_batPct / 100.0f));
  if (fill > 0) u8g2.drawBox(x + 1, y + 1, fill, bodyH - 2);
}

// ================== Buzzer ===========================================
void beepBlocking(int times, int onMs, int offMs = 80) {
  if (!g_buzzerEnable) return;          // 用户在设置里关了蜂鸣器
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < times - 1) delay(offMs);
  }
}

// ================== Calibration / state actions ======================
void calibrateSurface() {
  if (!g_sensorOk) return;
  float sum = 0;
  const int N = 10;
  for (int i = 0; i < N; i++) { sensor.read(); sum += sensor.pressure(); delay(40); }
  g_surfacePressure = sum / N;
  resetPressureFilter();
  initCompartmentsToSurface();
  g_maxDepth = 0;
  Serial.printf("[CAL] Surface = %.2f mbar; tissues reset\n", g_surfacePressure);
  beepBlocking(2, 70);
}

float depthFromPressure(float pressureMbar) {
  float d = (pressureMbar - g_surfacePressure) * 100.0f / (g_fluidDensity * 9.80665f);
  return d < 0 ? 0 : d;
}

// ================== Drawing (中文 UI) ================================
// 字体说明:
//   u8g2_font_wqy12_t_chinese1  -> 12px 中文(约1000常用字), 含ASCII
//   u8g2_font_logisoso28_tn     -> 大字数字(深度)
//   u8g2_font_logisoso24_tn     -> 中字数字(时间编辑)
#define FONT_CN     u8g2_font_wqy12_t_chinese3
#define FONT_BIG    u8g2_font_logisoso28_tn
#define FONT_MID    u8g2_font_logisoso24_tn

void drawHud(float depth, float maxDepth, float temp, float ndl, float ascentMpm, uint32_t diveSec) {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);

  char buf[40];
  uint8_t hh, mm, ss;
  getCurrentTime(hh, mm, ss);

  // 顶部左: 实时时钟 HH:MM
  snprintf(buf, sizeof(buf), "%02u:%02u", hh, mm);
  u8g2.drawUTF8(0, 10, buf);

  // 顶部中: 潜水计时(仅潜水中显示)
  if (g_diving) {
    snprintf(buf, sizeof(buf), "[%02lu:%02lu]", (unsigned long)(diveSec / 60), (unsigned long)(diveSec % 60));
    int w = u8g2.getUTF8Width(buf);
    u8g2.drawUTF8((128 - w) / 2, 10, buf);
  }

  // 顶部右: 电池图标 + 百分比 + 海/淡水标识
  if (g_batPresent) {
    drawBatteryIcon(74, 2);
    snprintf(buf, sizeof(buf), "%d%%", g_batPct);
    u8g2.drawUTF8(94, 10, buf);
  }
  u8g2.drawUTF8(116, 10, g_fluidDensity > 1010 ? "海" : "淡");

  // 大字深度(居中)
  if (depth < 100.0f) snprintf(buf, sizeof(buf), "%4.1f", depth);
  else                snprintf(buf, sizeof(buf), "%4.0f", depth);
  u8g2.setFont(FONT_BIG);
  int dw = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - dw) / 2, 42, buf);

  // 中部: NDL / 安全停留 / 减压
  u8g2.setFont(FONT_CN);
  if (g_ss == SS_RUNNING || g_ss == SS_ARMED) {
    uint32_t remain = (g_ssAccumMs >= g_safetyStopSec * 1000UL) ? 0 : (g_safetyStopSec * 1000UL - g_ssAccumMs);
    snprintf(buf, sizeof(buf), "安全停%lu秒", (unsigned long)(remain / 1000));
    u8g2.drawUTF8(0, 54, buf);
  } else if (g_ss == SS_DONE) {
    u8g2.drawUTF8(0, 54, "安全停留完成");
  } else if (ndl >= 99.0f) {
    u8g2.drawUTF8(0, 54, "NDL>99分");
  } else if (ndl <= 0.0f) {
    u8g2.drawUTF8(0, 54, "需减压!");
  } else {
    snprintf(buf, sizeof(buf), "NDL %2.0f分", ndl);
    u8g2.drawUTF8(0, 54, buf);
  }

  // 中部右: 最深
  snprintf(buf, sizeof(buf), "最深%.1f", maxDepth);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(128 - w - 8, 54, buf);

  // 底部: 温度 + 上升速率
  snprintf(buf, sizeof(buf), "%.1f度", temp);
  u8g2.drawUTF8(0, 62, buf);

  snprintf(buf, sizeof(buf), "%+.1f米/分", ascentMpm);
  w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(128 - w - 8, 62, buf);

  // 警告标志(右边缘)
  if (depth > g_alarmDepth)          u8g2.drawUTF8(120, 54, "!");
  if (ascentMpm > g_ascentLimit) u8g2.drawUTF8(120, 62, "^");

  u8g2.sendBuffer();
}

void drawTissue(float depth) {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 10, "组织负荷");

  float pct = computeTissueLoadPct(depth);
  char buf[32];
  snprintf(buf, sizeof(buf), "最大 %3.0f%%", pct);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(128 - w, 10, buf);

  // 16 房间柱状图
  int x0 = 0;
  int barW = 7;
  int gap  = 1;
  int yTop = 14;
  int hMax = 32;
  for (int i = 0; i < 16; i++) {
    float pAmb_bar = (g_surfacePressure / 1000.0f) + depth * g_fluidDensity * 9.80665f / 1e5f;
    float Mv = pAmb_bar / B_N2[i] + A_N2[i];
    float p  = g_pN2[i] / Mv;
    if (p > 1.0f) p = 1.0f;
    if (p < 0.0f) p = 0.0f;
    int h = (int)(p * hMax);
    int x = x0 + i * (barW + gap);
    u8g2.drawFrame(x, yTop, barW, hMax);
    u8g2.drawBox(x, yTop + (hMax - h), barW, h);
  }

  // 底部: NDL
  float ndl = computeNDL(depth);
  if (ndl >= 99.0f)      snprintf(buf, sizeof(buf), "NDL >99 分");
  else if (ndl <= 0.0f)  snprintf(buf, sizeof(buf), "需要减压");
  else                   snprintf(buf, sizeof(buf), "NDL %.0f 分钟", ndl);
  u8g2.drawUTF8(0, 62, buf);
  u8g2.sendBuffer();
}

void drawLastDive() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 10, "上次潜水");
  char buf[32];
  snprintf(buf, sizeof(buf), "累计%u次", g_diveTotal);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(128 - w, 10, buf);

  if (g_logCount == 0) {
    u8g2.drawUTF8(0, 36, "无潜水记录");
  } else {
    DiveRecord &r = g_log[0];
    snprintf(buf, sizeof(buf), "最深  %.1f 米", r.maxDepth);
    u8g2.drawUTF8(0, 26, buf);
    snprintf(buf, sizeof(buf), "时长  %u:%02u",  r.durationSec/60, r.durationSec%60);
    u8g2.drawUTF8(0, 40, buf);
    snprintf(buf, sizeof(buf), "温度  %.1f 度",  r.minTemp);
    u8g2.drawUTF8(0, 52, buf);

    // 距上次潜水时长 (水面间隔)
    if (r.endEpoch > 0) {
      time_t nowEpoch;
      time(&nowEpoch);
      int32_t intervalSec = (int32_t)((uint32_t)nowEpoch - r.endEpoch);
      if (intervalSec >= 0 && intervalSec < 99 * 3600) {
        snprintf(buf, sizeof(buf), "%s  距今%ldh%02ldm",
                 r.saltwater ? "海水" : "淡水",
                 (long)(intervalSec / 3600), (long)((intervalSec / 60) % 60));
      } else {
        snprintf(buf, sizeof(buf), "%s", r.saltwater ? "海水" : "淡水");
      }
    } else {
      snprintf(buf, sizeof(buf), "%s", r.saltwater ? "海水" : "淡水");
    }
    u8g2.drawUTF8(0, 62, buf);
  }
  u8g2.sendBuffer();
}

void drawLogList() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 10, "潜水日志");
  char buf[32];
  snprintf(buf, sizeof(buf), "%u/%u", g_logViewIdx + 1, g_logCount);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(128 - w, 10, buf);

  if (g_logCount == 0) {
    u8g2.drawUTF8(0, 36, "日志为空");
  } else {
    if (g_logViewIdx >= g_logCount) g_logViewIdx = g_logCount - 1;
    DiveRecord &r = g_log[g_logViewIdx];
    snprintf(buf, sizeof(buf), "#%u  最深 %.1f米", g_diveTotal - g_logViewIdx, r.maxDepth);
    u8g2.drawUTF8(0, 26, buf);
    snprintf(buf, sizeof(buf), "时长 %u:%02u  %.1f度", r.durationSec/60, r.durationSec%60, r.minTemp);
    u8g2.drawUTF8(0, 40, buf);
    u8g2.drawUTF8(0, 52, r.saltwater ? "海水" : "淡水");
    u8g2.drawUTF8(0, 62, "上下键浏览");
  }
  u8g2.sendBuffer();
}

void drawTempChart() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);

  char buf[24];
  snprintf(buf, sizeof(buf), "温度 %.1f度", g_temp);
  u8g2.drawUTF8(0, 10, buf);

  // 右上: 当前采样数
  snprintf(buf, sizeof(buf), "%u/60", g_tempHistCount);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(128 - w, 10, buf);

  if (g_tempHistCount < 2) {
    u8g2.drawUTF8(0, 36, "数据收集中...");
    u8g2.drawUTF8(0, 50, "每分钟一个样本");
    u8g2.sendBuffer();
    return;
  }

  // 找 min/max
  float minT = 999, maxT = -999;
  for (int i = 0; i < g_tempHistCount; i++) {
    if (g_tempHist[i] < minT) minT = g_tempHist[i];
    if (g_tempHist[i] > maxT) maxT = g_tempHist[i];
  }
  if (maxT - minT < 0.5f) { float c = (maxT + minT) / 2; minT = c - 0.25f; maxT = c + 0.25f; }

  // Y 轴上下界标签
  snprintf(buf, sizeof(buf), "%4.1f", maxT);
  u8g2.drawUTF8(0, 22, buf);
  snprintf(buf, sizeof(buf), "%4.1f", minT);
  u8g2.drawUTF8(0, 60, buf);

  // 绘图区
  int x0 = 26, y0 = 14, gw = 100, gh = 44;
  u8g2.drawFrame(x0, y0, gw, gh);

  // 折线
  for (int i = 0; i < g_tempHistCount - 1; i++) {
    int idx1 = (g_tempHistIdx + TEMP_HIST_LEN - g_tempHistCount + i) % TEMP_HIST_LEN;
    int idx2 = (g_tempHistIdx + TEMP_HIST_LEN - g_tempHistCount + i + 1) % TEMP_HIST_LEN;
    int x1 = x0 + 1 + i * (gw - 2) / (g_tempHistCount - 1);
    int x2 = x0 + 1 + (i + 1) * (gw - 2) / (g_tempHistCount - 1);
    int y1 = y0 + gh - 2 - (int)((g_tempHist[idx1] - minT) * (gh - 4) / (maxT - minT));
    int y2 = y0 + gh - 2 - (int)((g_tempHist[idx2] - minT) * (gh - 4) / (maxT - minT));
    u8g2.drawLine(x1, y1, x2, y2);
  }

  u8g2.sendBuffer();
}

void recordTempSample(uint32_t now) {
  if (!g_sensorOk) return;
  if (g_lastTempSampleMs != 0 && now - g_lastTempSampleMs < 60000) return;
  g_lastTempSampleMs = now;
  g_tempHist[g_tempHistIdx] = g_temp;
  g_tempHistIdx = (g_tempHistIdx + 1) % TEMP_HIST_LEN;
  if (g_tempHistCount < TEMP_HIST_LEN) g_tempHistCount++;
}

void drawStats() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 10, "总览统计");

  char buf[40];
  snprintf(buf, sizeof(buf), "潜水次数  %u 次", g_diveTotal);
  u8g2.drawUTF8(0, 22, buf);

  snprintf(buf, sizeof(buf), "历史最深  %.1f m", g_diveTotalMaxD);
  u8g2.drawUTF8(0, 34, buf);

  uint32_t totMin = g_lifetimeUnderwaterSec / 60;
  uint32_t totH   = totMin / 60;
  snprintf(buf, sizeof(buf), "累计水下  %luh%02lum", (unsigned long)totH, (unsigned long)(totMin % 60));
  u8g2.drawUTF8(0, 46, buf);

  // Surface interval since last dive (if any)
  if (g_logCount > 0 && g_log[0].endEpoch > 0) {
    time_t nowEpoch;
    time(&nowEpoch);
    int32_t intervalSec = (int32_t)((uint32_t)nowEpoch - g_log[0].endEpoch);
    if (intervalSec >= 0 && intervalSec < 99 * 3600) {
      snprintf(buf, sizeof(buf), "上次距今 %ldh%02ldm",
               (long)(intervalSec / 3600), (long)((intervalSec / 60) % 60));
      u8g2.drawUTF8(0, 58, buf);
    }
  }

  if (g_batPresent) {
    snprintf(buf, sizeof(buf), "电 %.2fV %u%%", g_batVoltage, g_batPct);
  } else {
    snprintf(buf, sizeof(buf), "电池未接入");
  }
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(128 - w, 10, buf);

  u8g2.sendBuffer();
}

// 设置菜单的项名 / 值 / 调整
const char* settingsItemName(uint8_t i) {
  switch (i) {
    case 0: return "深度报警";
    case 1: return "上升警告";
    case 2: return "屏保超时";
    case 3: return "安全停留";
    case 4: return "蜂鸣器";
    case 5: return "水密度";
  }
  return "?";
}

void settingsItemValue(uint8_t i, char *buf, size_t sz) {
  switch (i) {
    case 0: snprintf(buf, sz, "%.0f米", g_alarmDepth); break;
    case 1: snprintf(buf, sz, "%.0f米/分", g_ascentLimit); break;
    case 2: snprintf(buf, sz, "%lu分", (unsigned long)(g_screenOffMs / 60000UL)); break;
    case 3: snprintf(buf, sz, "%u秒", g_safetyStopSec); break;
    case 4: snprintf(buf, sz, "%s", g_buzzerEnable ? "开" : "关"); break;
    case 5: snprintf(buf, sz, "%s", g_fluidDensity > 1010 ? "海水" : "淡水"); break;
  }
}

void settingsItemAdjust(uint8_t i, int delta) {
  switch (i) {
    case 0: g_alarmDepth     = constrain(g_alarmDepth + delta * 5.0f, 5.0f, 60.0f); break;
    case 1: g_ascentLimit    = constrain(g_ascentLimit + (float)delta, 3.0f, 18.0f); break;
    case 2: {
      int min = (int)(g_screenOffMs / 60000UL) + delta;
      if (min < 1) min = 1; if (min > 30) min = 30;
      g_screenOffMs = (uint32_t)min * 60000UL;
      break;
    }
    case 3: {
      int s = (int)g_safetyStopSec + delta * 30;
      if (s < 60) s = 60; if (s > 300) s = 300;
      g_safetyStopSec = (uint16_t)s;
      break;
    }
    case 4: g_buzzerEnable = !g_buzzerEnable; break;
    case 5: g_fluidDensity = (g_fluidDensity > 1010) ? 997.0f : 1029.0f;
            if (g_sensorOk) sensor.setFluidDensity(g_fluidDensity);
            break;
  }
}

void drawSettings() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);

  // 标题 + 当前项序号
  char buf[24];
  u8g2.drawUTF8(0, 10, "设置");
  snprintf(buf, sizeof(buf), "%u/%u", g_settingsItem + 1, SETTINGS_COUNT);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(128 - w, 10, buf);

  // 4 项可见, 按当前选中项滚动
  const int VISIBLE = 4;
  int first = 0;
  if (g_settingsItem >= VISIBLE) first = g_settingsItem - VISIBLE + 1;
  if (first > SETTINGS_COUNT - VISIBLE) first = SETTINGS_COUNT - VISIBLE;
  if (first < 0) first = 0;

  for (int i = 0; i < VISIBLE && first + i < SETTINGS_COUNT; i++) {
    uint8_t idx = first + i;
    int y = 24 + i * 11;
    if (idx == g_settingsItem) u8g2.drawUTF8(0, y, ">");
    u8g2.drawUTF8(8, y, settingsItemName(idx));
    char vbuf[20];
    settingsItemValue(idx, vbuf, sizeof(vbuf));
    int vw = u8g2.getUTF8Width(vbuf);
    u8g2.drawUTF8(128 - vw, y, vbuf);
  }
}

void drawSettingsAndSend() {
  drawSettings();
  u8g2.sendBuffer();
}

void drawTimeEdit() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 10, "设置时间");
  u8g2.drawUTF8(82, 10, "MODE确定");

  // 大字时间显示
  char buf[8];
  u8g2.setFont(FONT_BIG);
  snprintf(buf, sizeof(buf), "%02u:%02u", g_editHH, g_editMM);
  int w = u8g2.getStrWidth(buf);
  int x = (128 - w) / 2;
  u8g2.drawStr(x, 44, buf);

  // 下划线指示当前编辑字段
  int colonOffset = u8g2.getStrWidth("00");
  int afterColon  = u8g2.getStrWidth("00:");
  int hhX = x;
  int mmX = x + afterColon;
  if (g_editField == 0) u8g2.drawHLine(hhX, 48, colonOffset);
  else                  u8g2.drawHLine(mmX, 48, colonOffset);

  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 62, "上下增减   MODE切换");
  u8g2.sendBuffer();
}

void drawSplash() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_MID);
  u8g2.drawUTF8(0, 22, "潜水表");
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 36, "v2.1  ZHL-16C  中文");
  char buf[40];
  snprintf(buf, sizeof(buf), "水面: %.1f mbar", g_surfacePressure);
  u8g2.drawUTF8(0, 50, buf);
  snprintf(buf, sizeof(buf), "密度: %.0f kg/m3", g_fluidDensity);
  u8g2.drawUTF8(0, 62, buf);
  u8g2.sendBuffer();
}

void drawError(const char *l1, const char *l2 = nullptr) {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_MID);
  u8g2.drawUTF8(0, 22, "错误");
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 42, l1);
  if (l2) u8g2.drawUTF8(0, 58, l2);
  u8g2.sendBuffer();
}

// ================== Setup ============================================
void setup() {
  Serial.begin(115200);
  delay(120);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);
  pinMode(BTN_UP_PIN,   INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  u8g2.begin();
  drawSplash();

  uint8_t tries = 0;
  while (!sensor.init()) {
    Serial.println("[MS5837] init failed");
    drawError("MS5837 初始化失败", "检查接线/3V3");
    if (++tries >= 5) { g_sensorOk = false; break; }
    delay(800);
  }
  if (tries < 5) {
    sensor.setModel(MS5837::MS5837_30BA);
    sensor.setFluidDensity(g_fluidDensity);
    g_sensorOk = true;
  }

  // Calibrate surface (10 samples)
  if (g_sensorOk) {
    float sum = 0;
    for (int i = 0; i < 10; i++) { sensor.read(); sum += sensor.pressure(); delay(50); }
    g_surfacePressure = sum / 10.0f;
  }
  initCompartmentsToSurface();

  loadLogFromNVS();
  loadClockFromNVS();
  loadSettingsFromNVS();
  if (g_sensorOk) sensor.setFluidDensity(g_fluidDensity);

  // 显示开机画面 (含 WiFi 同步状态)
  drawSplash();
  if (strlen(WIFI_SSID) > 0) {
    u8g2.setFont(FONT_CN);
    u8g2.drawUTF8(0, 62, "WiFi 同步中...");
    u8g2.sendBuffer();
    bool ok = trySyncNTP();
    drawSplash();
    u8g2.setFont(FONT_CN);
    u8g2.drawUTF8(0, 62, ok ? "时间同步完成" : "WiFi 同步失败");
    u8g2.sendBuffer();
    delay(1200);
  }

  beepBlocking(2, 70);
  delay(400);
  g_lastInteractMs = millis();
}

// ================== Loop =============================================
void loop() {
  uint32_t now = millis();

  // ---- Buttons ----
  btnPoll(g_btnMode, now);
  btnPoll(g_btnUp,   now);
  btnPoll(g_btnDown, now);

  // ---- Power saving: wake / sleep OLED ----
  bool anyBtnEvent = g_btnMode.evShort || g_btnMode.evLong ||
                     g_btnUp.evShort   || g_btnUp.evLong   ||
                     g_btnDown.evShort || g_btnDown.evLong;
  if (anyBtnEvent || g_diving) {
    g_lastInteractMs = now;
    if (g_screenOff) {
      g_screenOff = false;
      u8g2.setPowerSave(0);
      // Consume the wake-up button event (don't trigger an action)
      if (anyBtnEvent) {
        g_btnMode.evShort = g_btnMode.evLong = false;
        g_btnUp.evShort   = g_btnUp.evLong   = false;
        g_btnDown.evShort = g_btnDown.evLong = false;
      }
    }
  }
  if (!g_screenOff && !g_editingTime && !g_diving &&
      (now - g_lastInteractMs > g_screenOffMs)) {
    g_screenOff = true;
    u8g2.setPowerSave(1);
    Serial.println("[POWER] OLED off (idle)");
  }

  // ---- Battery (every 5s) ----
  updateBattery(now);

  if (g_editingTime) {
    // ---- Time-edit mode: buttons remapped ----
    if (g_btnMode.evShort) {
      g_btnMode.evShort = false;
      g_editField = 1 - g_editField;     // toggle HH <-> MM
    }
    if (g_btnMode.evLong) {
      g_btnMode.evLong = false;
      // Save and exit
      setSystemClock(g_editHH, g_editMM, 0);
      saveClockToNVS();
      g_editingTime = false;
      Serial.printf("[ACTION] Clock set to %02u:%02u\n", g_editHH, g_editMM);
      beepBlocking(2, 80);
    }
    if (g_btnUp.evShort) {
      g_btnUp.evShort = false;
      if (g_editField == 0) g_editHH = (g_editHH + 1) % 24;
      else                  g_editMM = (g_editMM + 1) % 60;
    }
    if (g_btnDown.evShort) {
      g_btnDown.evShort = false;
      if (g_editField == 0) g_editHH = (g_editHH + 23) % 24;
      else                  g_editMM = (g_editMM + 59) % 60;
    }
    // Discard unused long-press events while editing
    g_btnUp.evLong = false;
    g_btnDown.evLong = false;
  } else if (g_inSettings) {
    // ---- Settings menu: buttons remapped ----
    if (g_btnMode.evShort) {
      g_btnMode.evShort = false;
      g_settingsItem = (g_settingsItem + 1) % SETTINGS_COUNT;
    }
    if (g_btnMode.evLong) {
      g_btnMode.evLong = false;
      saveSettingsToNVS();
      g_inSettings = false;
      beepBlocking(2, 80);
      Serial.println("[ACTION] Settings saved & exit");
    }
    if (g_btnUp.evShort) {
      g_btnUp.evShort = false;
      settingsItemAdjust(g_settingsItem, +1);
    }
    if (g_btnDown.evShort) {
      g_btnDown.evShort = false;
      settingsItemAdjust(g_settingsItem, -1);
    }
    g_btnUp.evLong = false;
    g_btnDown.evLong = false;
  } else {
    // ---- Normal mode ----
    if (g_btnMode.evShort) {
      g_btnMode.evShort = false;
      g_page = (g_page + 1) % PAGE_COUNT;
      if (g_page == PAGE_LOG) g_logViewIdx = 0;
    }
    if (g_btnMode.evLong) {
      g_btnMode.evLong = false;
      calibrateSurface();
    }
    if (g_btnUp.evShort) {
      g_btnUp.evShort = false;
      if (g_page == PAGE_LOG) {
        if (g_logCount > 0 && g_logViewIdx > 0) g_logViewIdx--;
      } else {
        g_maxDepth = 0;
        Serial.println("[ACTION] Max depth reset");
        beepBlocking(1, 60);
      }
    }
    if (g_btnUp.evLong) {
      g_btnUp.evLong = false;
      // 长按 UP -> 进入设置菜单 (原"切水"功能挪进菜单)
      g_inSettings = true;
      g_settingsItem = 0;
      Serial.println("[ACTION] Enter settings");
      beepBlocking(1, 100);
    }
    if (g_btnDown.evShort) {
      g_btnDown.evShort = false;
      if (g_page == PAGE_LOG) {
        if (g_logCount > 0 && g_logViewIdx < g_logCount - 1) g_logViewIdx++;
      } else if (g_diving) {
        g_diveStartMs = now;
        Serial.println("[ACTION] Dive timer reset");
        beepBlocking(1, 60);
      }
    }
    if (g_btnDown.evLong) {
      g_btnDown.evLong = false;
      if (g_page == PAGE_LOG) {
        eraseLogNVS();
        Serial.println("[ACTION] Log erased");
        beepBlocking(3, 70);
      } else {
        // Enter time-edit mode, prefill with current clock
        uint8_t hh, mm, ss;
        getCurrentTime(hh, mm, ss);
        g_editHH = hh;
        g_editMM = mm;
        g_editField = 0;
        g_editingTime = true;
        Serial.println("[ACTION] Enter clock edit");
        beepBlocking(1, 100);
      }
    }
  }

  // ---- Sample ----
  if (now - g_lastSampleMs < SAMPLE_MS) return;
  uint32_t dtMs = now - g_lastSampleMs;
  g_lastSampleMs = now;

  if (g_sensorOk) {
    sensor.read();
    g_pressureRaw = sensor.pressure();
    g_temp        = sensor.temperature();
    float pSmooth = pushPressureAndAverage(g_pressureRaw);
    g_depthRaw    = depthFromPressure(g_pressureRaw);
    g_depthSmooth = depthFromPressure(pSmooth);
    g_ascentMpm   = (g_lastDepth - g_depthSmooth) * 60000.0f / (float)dtMs;
    g_lastDepth   = g_depthSmooth;
    if (g_depthSmooth > g_maxDepth) g_maxDepth = g_depthSmooth;
    if (g_temp < g_diveMinTemp) g_diveMinTemp = g_temp;

    // Update tissues every sample
    updateCompartments(dtMs / 1000.0f, g_depthSmooth);

    // Record temperature sample (rate-limited to 1/min internally)
    recordTempSample(now);

    // Dive state machine
    if (!g_diving && g_depthSmooth > DIVE_START_DEPTH) {
      g_diving = true;
      g_diveStartMs = now;
      g_maxDepth = g_depthSmooth;
      g_diveMinTemp = g_temp;
      g_ss = SS_IDLE;
      g_ssAccumMs = 0;
      g_ssArmCondition = false;
      Serial.println("[DIVE] Started");
    } else if (g_diving && g_depthSmooth < SURFACE_DEPTH) {
      // End dive: persist
      uint32_t durSec = (now - g_diveStartMs) / 1000;
      if (durSec >= 30) {  // Ignore tiny accidental triggers
        time_t nowEpoch;
        time(&nowEpoch);
        DiveRecord rec = {
          g_maxDepth,
          (uint16_t)min((uint32_t)0xFFFF, durSec),
          g_diveMinTemp,
          (uint8_t)(g_fluidDensity > 1010 ? 1 : 0),
          0,
          (uint32_t)nowEpoch
        };
        saveDiveToNVS(rec);
        Serial.printf("[DIVE] Saved: max=%.1fm dur=%us minT=%.1fC end=%lu\n",
                      rec.maxDepth, rec.durationSec, rec.minTemp, (unsigned long)rec.endEpoch);
      } else {
        Serial.println("[DIVE] Ended (too short, not saved)");
      }
      g_diving = false;
    }

    updateSafetyStop(g_depthSmooth, now);
  }

  uint32_t diveSec = g_diving ? (now - g_diveStartMs) / 1000 : 0;
  float ndl = computeNDL(g_depthSmooth);

  // ---- Render ----
  if (g_screenOff) {
    // skip rendering while screen is off
  } else if (g_editingTime) {
    drawTimeEdit();
  } else if (g_inSettings) {
    drawSettingsAndSend();
  } else {
    switch (g_page) {
      case PAGE_HUD:      drawHud(g_depthSmooth, g_maxDepth, g_temp, ndl, g_ascentMpm, diveSec); break;
      case PAGE_TISSUE:   drawTissue(g_depthSmooth); break;
      case PAGE_TEMP:     drawTempChart(); break;
      case PAGE_LASTDIVE: drawLastDive(); break;
      case PAGE_LOG:      drawLogList();  break;
      case PAGE_STATS:    drawStats();    break;
    }
  }

  // ---- Periodic clock save (every 60s) ----
  if (now - g_lastTimeSaveMs > 60000) {
    g_lastTimeSaveMs = now;
    saveClockToNVS();
  }

  // ---- Alarms ----
  if (g_sensorOk && now - g_lastBeepMs > REARM_BEEP_MS) {
    if (g_depthSmooth > g_alarmDepth) {
      beepBlocking(3, 70); g_lastBeepMs = now;
    } else if (g_ascentMpm > g_ascentLimit && g_depthSmooth > 3.0f) {
      beepBlocking(1, 250); g_lastBeepMs = now;
    } else if (g_ss == SS_DONE && g_lastBeepMs == 0) {
      beepBlocking(2, 100); g_lastBeepMs = now;
    } else if (ndl <= 0.0f && g_diving) {
      beepBlocking(4, 80); g_lastBeepMs = now;
    }
  }

  // ---- Telemetry ----
  Serial.printf("D=%5.2fm Mx=%5.2fm T=%4.1fC P=%6.1f Up=%+5.1f NDL=%4.1f SS=%d Pg=%u Dive=%c Bat=%.2fV/%u%% BTN=%d%d%d\n",
                g_depthSmooth, g_maxDepth, g_temp, g_pressureRaw, g_ascentMpm,
                ndl, g_ss, g_page, g_diving ? 'Y' : 'N',
                g_batVoltage, g_batPct,
                digitalRead(BTN_MODE_PIN), digitalRead(BTN_UP_PIN), digitalRead(BTN_DOWN_PIN));
}
