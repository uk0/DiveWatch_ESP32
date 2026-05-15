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
#include <math.h>
#include "MS5837.h"

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
  uint8_t  saltwater;
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

// ================== Tunables =========================================
static const float    SURFACE_DEPTH       = 0.5f;   // m, below this = surface
static const float    DIVE_START_DEPTH    = 1.2f;   // m, above this triggers dive
static const float    ALARM_DEPTH         = 30.0f;  // m
static const float    ASCENT_LIMIT_MPM    = 9.0f;   // m/min
static const float    SAFETY_STOP_DEPTH   = 5.0f;   // m, target depth
static const float    SAFETY_STOP_BAND    = 1.5f;   // +/- m around 5m
static const uint32_t SAFETY_STOP_SEC     = 180;    // 3 minutes
static const uint32_t SAMPLE_MS           = 200;
static const uint32_t REARM_BEEP_MS       = 1500;
static const uint8_t  AVG_WINDOW          = 5;      // samples for moving avg

// Default fluid density (toggle with long-press UP)
float g_fluidDensity = 1029.0f;  // 1029 sea, 997 fresh

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
        if (g_ssAccumMs >= SAFETY_STOP_SEC * 1000UL) g_ss = SS_DONE;
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
enum Page : uint8_t { PAGE_HUD = 0, PAGE_TISSUE, PAGE_LASTDIVE, PAGE_LOG, PAGE_COUNT };
uint8_t g_page = PAGE_HUD;
uint8_t g_logViewIdx = 0;

// ================== NVS Dive Log =====================================
static const uint8_t LOG_CAP = 10;
DiveRecord g_log[LOG_CAP];
uint8_t    g_logCount = 0;
uint16_t   g_diveTotal = 0;     // lifetime dive count
float      g_diveTotalMaxD = 0; // lifetime max depth
float      g_diveMinTemp = 999;

void loadLogFromNVS() {
  prefs.begin("dive", true);
  g_logCount      = prefs.getUChar("count", 0);
  g_diveTotal     = prefs.getUShort("total", 0);
  g_diveTotalMaxD = prefs.getFloat("totalMaxD", 0);
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

  prefs.begin("dive", false);
  prefs.putUChar("count", g_logCount);
  prefs.putUShort("total", g_diveTotal);
  prefs.putFloat("totalMaxD", g_diveTotalMaxD);
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

// ================== Drawing ==========================================
void drawHud(float depth, float maxDepth, float temp, float ndl, float ascentMpm, uint32_t diveSec) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);

  // Top left: dive timer
  char buf[24];
  snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)(diveSec / 60), (unsigned long)(diveSec % 60));
  u8g2.drawStr(0, 8, buf);

  // Top right: battery icon + pct + salinity flag
  if (g_batPresent) {
    drawBatteryIcon(78, 1);
    snprintf(buf, sizeof(buf), "%d%%", g_batPct);
    u8g2.drawStr(98, 8, buf);
  }
  u8g2.drawStr(120, 8, g_fluidDensity > 1010 ? "S" : "F");

  // Big depth
  if (depth < 100.0f) snprintf(buf, sizeof(buf), "%4.1f", depth);
  else                snprintf(buf, sizeof(buf), "%4.0f", depth);
  u8g2.setFont(u8g2_font_logisoso28_tn);
  int dw = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - dw) / 2, 38, buf);

  // Mid row: NDL
  u8g2.setFont(u8g2_font_6x10_tr);
  if (g_ss == SS_RUNNING || g_ss == SS_ARMED) {
    uint32_t remain = (g_ssAccumMs >= SAFETY_STOP_SEC * 1000UL) ? 0 : (SAFETY_STOP_SEC * 1000UL - g_ssAccumMs);
    snprintf(buf, sizeof(buf), "SAFETY %lus", (unsigned long)(remain / 1000));
    u8g2.drawStr(0, 50, buf);
  } else if (g_ss == SS_DONE) {
    u8g2.drawStr(0, 50, "SS OK");
  } else if (ndl >= 99.0f) {
    u8g2.drawStr(0, 50, "NDL >99");
  } else if (ndl <= 0.0f) {
    u8g2.drawStr(0, 50, "DECO!");
  } else {
    snprintf(buf, sizeof(buf), "NDL %2.0fmin", ndl);
    u8g2.drawStr(0, 50, buf);
  }

  // Bottom row: Max + Temp
  snprintf(buf, sizeof(buf), "Mx %.1f %.1fC", maxDepth, temp);
  u8g2.drawStr(0, 62, buf);

  // Right edge: ascent + alarms
  snprintf(buf, sizeof(buf), "%+.1f", ascentMpm);
  int w = u8g2.getStrWidth(buf);
  u8g2.drawStr(128 - w - 6, 62, buf);
  if (depth > ALARM_DEPTH)          u8g2.drawStr(120, 50, "!");
  if (ascentMpm > ASCENT_LIMIT_MPM) u8g2.drawStr(120, 62, "^");

  u8g2.sendBuffer();
}

void drawTissue(float depth) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 8, "TISSUE LOAD");

  float pct = computeTissueLoadPct(depth);
  char buf[32];
  snprintf(buf, sizeof(buf), "Max %3.0f%%", pct);
  u8g2.drawStr(0, 22, buf);

  // Bar graph: 16 compartments, height proportional to current load
  int x0 = 0;
  int barW = 7;
  int gap  = 1;
  int yTop = 28;
  int hMax = 28;
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

  float ndl = computeNDL(depth);
  if (ndl >= 99.0f)      snprintf(buf, sizeof(buf), "NDL >99 min");
  else if (ndl <= 0.0f)  snprintf(buf, sizeof(buf), "DECO REQUIRED");
  else                   snprintf(buf, sizeof(buf), "NDL %.0f min", ndl);
  u8g2.drawStr(0, 62, buf);
  u8g2.sendBuffer();
}

void drawLastDive() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 8, "LAST DIVE");
  char buf[32];
  if (g_logCount == 0) {
    u8g2.drawStr(0, 32, "No dives recorded");
  } else {
    DiveRecord &r = g_log[0];
    snprintf(buf, sizeof(buf), "Max  %.1f m", r.maxDepth);   u8g2.drawStr(0, 22, buf);
    snprintf(buf, sizeof(buf), "Time %u:%02u", r.durationSec/60, r.durationSec%60); u8g2.drawStr(0, 34, buf);
    snprintf(buf, sizeof(buf), "Temp %.1f C", r.minTemp);    u8g2.drawStr(0, 46, buf);
    snprintf(buf, sizeof(buf), "%s", r.saltwater ? "Saltwater" : "Freshwater"); u8g2.drawStr(0, 58, buf);
  }
  snprintf(buf, sizeof(buf), "Total %u", g_diveTotal);
  int w = u8g2.getStrWidth(buf);
  u8g2.drawStr(128 - w, 8, buf);
  u8g2.sendBuffer();
}

void drawLogList() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 8, "LOG");
  char buf[32];
  snprintf(buf, sizeof(buf), "%u/%u", g_logViewIdx + 1, g_logCount);
  int w = u8g2.getStrWidth(buf);
  u8g2.drawStr(128 - w, 8, buf);

  if (g_logCount == 0) {
    u8g2.drawStr(0, 32, "Log is empty");
  } else {
    if (g_logViewIdx >= g_logCount) g_logViewIdx = g_logCount - 1;
    DiveRecord &r = g_log[g_logViewIdx];
    snprintf(buf, sizeof(buf), "#%u  %.1f m", g_diveTotal - g_logViewIdx, r.maxDepth);
    u8g2.drawStr(0, 22, buf);
    snprintf(buf, sizeof(buf), "%u:%02u  %.1fC", r.durationSec/60, r.durationSec%60, r.minTemp);
    u8g2.drawStr(0, 36, buf);
    u8g2.drawStr(0, 50, r.saltwater ? "Saltwater" : "Freshwater");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(0, 62, "UP/DOWN: browse");
  }
  u8g2.sendBuffer();
}

void drawSplash() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB12_tr);
  u8g2.drawStr(0, 18, "DiveWatch");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 34, "v2.0  ZHL-16C  3-btn");
  char buf[32];
  snprintf(buf, sizeof(buf), "Surface: %.1f mbar", g_surfacePressure);
  u8g2.drawStr(0, 48, buf);
  snprintf(buf, sizeof(buf), "Density: %.0f kg/m^3", g_fluidDensity);
  u8g2.drawStr(0, 62, buf);
  u8g2.sendBuffer();
}

void drawError(const char *l1, const char *l2 = nullptr) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(0, 16, "ERROR");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 36, l1);
  if (l2) u8g2.drawStr(0, 50, l2);
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
    drawError("MS5837 init fail", "Check wiring/3V3");
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

  drawSplash();
  beepBlocking(2, 70);
  delay(700);
}

// ================== Loop =============================================
void loop() {
  uint32_t now = millis();

  // ---- Buttons ----
  btnPoll(g_btnMode, now);
  btnPoll(g_btnUp,   now);
  btnPoll(g_btnDown, now);

  // ---- Battery (every 5s) ----
  updateBattery(now);

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
    g_fluidDensity = (g_fluidDensity > 1010) ? 997.0f : 1029.0f;
    if (g_sensorOk) sensor.setFluidDensity(g_fluidDensity);
    Serial.printf("[ACTION] Density -> %.0f\n", g_fluidDensity);
    beepBlocking(2, 60);
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
    eraseLogNVS();
    Serial.println("[ACTION] Log erased");
    beepBlocking(3, 70);
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
        DiveRecord rec = {g_maxDepth, (uint16_t)min((uint32_t)0xFFFF, durSec), g_diveMinTemp, (uint8_t)(g_fluidDensity > 1010 ? 1 : 0)};
        saveDiveToNVS(rec);
        Serial.printf("[DIVE] Saved: max=%.1fm dur=%us minT=%.1fC\n", rec.maxDepth, rec.durationSec, rec.minTemp);
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
  switch (g_page) {
    case PAGE_HUD:      drawHud(g_depthSmooth, g_maxDepth, g_temp, ndl, g_ascentMpm, diveSec); break;
    case PAGE_TISSUE:   drawTissue(g_depthSmooth); break;
    case PAGE_LASTDIVE: drawLastDive(); break;
    case PAGE_LOG:      drawLogList();  break;
  }

  // ---- Alarms ----
  if (g_sensorOk && now - g_lastBeepMs > REARM_BEEP_MS) {
    if (g_depthSmooth > ALARM_DEPTH) {
      beepBlocking(3, 70); g_lastBeepMs = now;
    } else if (g_ascentMpm > ASCENT_LIMIT_MPM && g_depthSmooth > 3.0f) {
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
