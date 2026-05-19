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
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "MS5837.h"

// ===== ST7789 SPI 引脚 =====
#define TFT_CS    21
#define TFT_DC    18
#define TFT_RST   17
#define TFT_MOSI  38
#define TFT_SCK   48

// ===== RGB565 颜色 (源代码用 RGB, 通过 C() swap 适配 BGR 屏) =====
#define RGB_ORANGE  0xFD20
#define RGB_BLACK   0x0000
#define RGB_WHITE   0xFFFF
#define RGB_RED     0xF800
#define RGB_GREEN   0x07E0
#define RGB_BLUE    0x001F
#define RGB_YELLOW  0xFFE0
#define RGB_CYAN    0x07FF
#define RGB_DARK    0x4208

// swap R/B 5-bit (G 6-bit 不变)
inline uint16_t swapRB(uint16_t c) {
  return ((c & 0x001F) << 11) | (c & 0x07E0) | ((c >> 11) & 0x001F);
}
#define C(x) swapRB(x)

// 前向声明 struct (避免 Arduino IDE 自动生成函数原型时找不到)
struct Button;
struct DiveRecord;
void btnPoll(Button &b, uint32_t now);
void saveDiveToNVS(const DiveRecord &rec);

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
  float    batVoltage;    // 潜水结束时的电池电压 (0 = 未记录)
  float    airUsedL;      // 本次潜水消耗气量 (估算)
  float    avgDepth;      // 本次潜水平均深度 m (0 = 未记录)
};

// ================== Pin map ==========================================
#define I2C_SDA       11
#define I2C_SCL       12
#define BUZZER_PIN     5
#define BTN_MODE_PIN   6
#define BTN_UP_PIN     7
#define BTN_DOWN_PIN  10
#define BAT_ADC_PIN    1     // board pin "A0", reads battery via 2:1 divider

// Voltage divider: BAT+ -[R1]- ADC -[R2]- GND
// 理想情况 R1=R2 时 BAT_DIVIDER = 2.0, 但 ±5% 精度下实际 1.9-2.1
// 该值现在可在设置菜单"电池校准"中微调 (1.80-2.30, 步 0.02)
float g_batDivider = 2.10f;            // 默认 2.10 (大部分场合够用)
static const float BAT_FULL_V   = 4.20f;
static const float BAT_EMPTY_V  = 3.30f;

// ================== 固定常量 (不可调) ================================
static const float    SURFACE_DEPTH       = 0.5f;
static const float    DIVE_START_DEPTH    = 1.2f;
static const float    SAFETY_STOP_DEPTH   = 5.0f;
static const float    SAFETY_STOP_BAND    = 1.5f;
static const uint32_t SAMPLE_MS           = 100;     // 200→100ms 提升 UI 顺滑度
static const uint32_t REARM_BEEP_MS       = 1500;
static const uint8_t  AVG_WINDOW          = 5;
static const uint32_t CPU_FREQ_ACTIVE     = 240;     // MHz, 正常工作
static const uint32_t CPU_FREQ_IDLE       = 80;      // MHz, 屏保时降频省电

// ================== 可调参数 (设置菜单可改, 持久化) ==================
float    g_alarmDepth      = 30.0f;     // 深度报警 m
float    g_ascentLimit     = 9.0f;      // 上升速率警告 m/min
float    g_descentLimit    = 18.0f;     // 下降速率警告 m/min (PADI 标准 18)
uint32_t g_screenOffMs     = 5UL*60*1000;  // 屏保超时 ms
uint16_t g_safetyStopSec   = 180;       // 安全停留时长 s
bool     g_buzzerEnable    = true;      // 蜂鸣器开关
float    g_fluidDensity    = 1029.0f;   // 水密度 (1029 海, 997 淡)
// OW 水肺潜水标准: AL80 铝瓶 = 11.1L * 210bar
// 改用气压表 (SPG) 标准方式: 输入 初始压力(bar) + 瓶容量(L), 内部算总气量
uint16_t g_initialBar      = 210;       // 初始压力 bar (AL80=210, 12L钢瓶=200/232)
uint8_t  g_tankVolL        = 11;        // 瓶容量 L (AL80=11, 12L钢瓶=12, 15L=15)
uint8_t  g_sacLmin         = 18;        // 水面气体消耗速率 L/min (新手休闲)
uint8_t  g_conservatism    = 100;       // ZHL-16C 保守度因子 % (80-120)
uint8_t  g_lowTempC        = 15;        // 低温报警阈值 ℃

// 派生量: 总气量 (L) = 初始压力 * 瓶容量
inline uint16_t totalAirL()  { return (uint16_t)g_initialBar * (uint16_t)g_tankVolL; }
inline float    barFromAirL(float airL) {
  return (g_tankVolL > 0) ? (airL / (float)g_tankVolL) : 0.0f;
}

// 气量监控运行时状态 (不持久化)
float    g_remainingAirL   = 0.0f;      // 当前剩余气量 L (内部用 L 算, 显示转 bar)
float    g_currentSacL     = 0.0f;      // 当前深度下的实际消耗速率 L/min
uint32_t g_lastAirCalcMs   = 0;

// ================== Globals ==========================================
SPIClass mySPI(HSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&mySPI, TFT_CS, TFT_DC, TFT_RST);
U8G2_FOR_ADAFRUIT_GFX u8g2;
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

// 平均深度统计 (本次潜水)
double   g_diveDepthSum  = 0.0;     // 累积 (depth * dt_ms)
uint32_t g_diveTimeMs    = 0;       // 累积时间 ms

// NDL 计划器
uint8_t  g_planDepthM    = 18;      // 用户在 PAGE_PLAN 设的计划深度

// Power saving: dim OLED after idle period (g_screenOffMs is configurable)
uint32_t g_lastInteractMs  = 0;
bool     g_screenOff       = false;
bool     g_pageDirty       = true;        // 切页或首次进入, 触发静态部分重绘

// Settings menu state
bool     g_inSettings        = false;
uint8_t  g_settingsItem      = 0;
static const uint8_t SETTINGS_COUNT = 13;

// Lifetime stats (separate from per-dive log, also in NVS)
uint32_t g_lifetimeUnderwaterSec = 0;

// Temperature history (1 sample/min, ring buffer of 60 = 1 hour)
static const uint8_t TEMP_HIST_LEN = 60;
float    g_tempHist[TEMP_HIST_LEN] = {0};
uint8_t  g_tempHistIdx   = 0;
uint8_t  g_tempHistCount = 0;
uint32_t g_lastTempSampleMs = 0;

// Battery voltage history (1 sample/hour, ring buffer of 24 = 1 day)
static const uint8_t BAT_HIST_LEN = 24;
float    g_batHist[BAT_HIST_LEN] = {0};
uint8_t  g_batHistIdx   = 0;
uint8_t  g_batHistCount = 0;
uint32_t g_lastBatHistMs = 0;

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
// 应用保守度因子: M-value 乘以 (g_conservatism / 100.0).
//   100% = 标准 ZHL-16C
//   80%  = 更激进 (NDL 增加)
//   120% = 更保守 (NDL 减少, 推荐 DIY)
float computeNDL(float depth_m) {
  float pAmb_bar = (g_surfacePressure / 1000.0f) + depth_m * g_fluidDensity * 9.80665f / 1e5f;
  float pAlv     = 0.79f * pAmb_bar;
  float consFactor = (float)g_conservatism / 100.0f;
  // 反向: 保守度大于 100 → M-value 阈值要小 → 提早达到 → NDL 短
  // M_effective = M / consFactor   (consFactor 大于 1 → M 小)
  float ndl_min  = 99.0f;
  for (int i = 0; i < 16; i++) {
    float Mv = (pAmb_bar / B_N2[i] + A_N2[i]) / consFactor;
    if (g_pN2[i] >= Mv) return 0.0f;
    if (pAlv <= g_pN2[i]) continue;
    float ratio = 1.0f - (Mv - g_pN2[i]) / (pAlv - g_pN2[i]);
    if (ratio <= 0.0f) continue;
    float k = 0.6931472f / (HALF_TIMES_N2[i] * 60.0f);
    float t_sec = -logf(ratio) / k;
    float t_min = t_sec / 60.0f;
    if (t_min < ndl_min) ndl_min = t_min;
  }
  return ndl_min;
}

// 估算海拔 (m), 基于水面气压 (mbar)
// 国际标准大气模型: alt = 44330 * (1 - (P/P0)^(1/5.255))
float estimateAltitudeM() {
  float ratio = g_surfacePressure / 1013.25f;
  return 44330.0f * (1.0f - powf(ratio, 1.0f / 5.255f));
}

// Highest tissue load percent vs M-value at current depth (0..100+)
float computeTissueLoadPct(float depth_m) {
  float pAmb_bar = (g_surfacePressure / 1000.0f) + depth_m * g_fluidDensity * 9.80665f / 1e5f;
  float consFactor = (float)g_conservatism / 100.0f;
  float maxPct = 0;
  for (int i = 0; i < 16; i++) {
    float Mv = (pAmb_bar / B_N2[i] + A_N2[i]) / consFactor;
    float pct = (g_pN2[i] / Mv) * 100.0f;
    if (pct > maxPct) maxPct = pct;
  }
  return maxPct;
}

// 单个组织室的"饱和度": 当前 P_N2 相对于该深度平衡值的百分比
// 0% = 水面平衡, 100% = 已在此深度完全饱和
float computeSaturationPct(int idx, float depth_m) {
  float pSurface = 0.79f * (g_surfacePressure / 1000.0f);
  float pCurrentDepth = 0.79f * ((g_surfacePressure / 1000.0f) + depth_m * g_fluidDensity * 9.80665f / 1e5f);
  if (pCurrentDepth <= pSurface) return 100.0f;  // 水面或正在脱饱和
  float pct = (g_pN2[idx] - pSurface) / (pCurrentDepth - pSurface) * 100.0f;
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

// 禁飞时间 (小时): 标准飞行环境压力 600 mbar (~0.6 bar)
// 公开算法: 每个组织室回到 < 飞行环境 M-value 所需最长时间
// 参考 DAN / PADI / DCIEM 标准: 单次潜水 12-18h, 多次潜水 18-24h
float computeNoFlyTimeHours() {
  const float pAmb_fly_bar = 0.600f;     // 飞机巡航舱压
  const float pAlv_fly = 0.79f * pAmb_fly_bar;
  float consFactor = (float)g_conservatism / 100.0f;
  float maxHours = 0;
  for (int i = 0; i < 16; i++) {
    float Mv = (pAmb_fly_bar / B_N2[i] + A_N2[i]) / consFactor;
    if (g_pN2[i] <= Mv) continue;          // 已在飞行 M-value 之下, 可飞
    if (g_pN2[i] <= pAlv_fly) continue;
    float ratio = (Mv - pAlv_fly) / (g_pN2[i] - pAlv_fly);
    if (ratio <= 0 || ratio >= 1) continue;
    float k = 0.6931472f / (HALF_TIMES_N2[i] * 60.0f);
    float hours = -logf(ratio) / k / 3600.0f;
    if (hours > maxHours) maxHours = hours;
  }
  return maxHours;
}

// 完全脱饱和时间 (小时): 所有组织室回到 ~ 水面平衡 (5% 容差)
// 经典假设: 5 个半衰期 (~97% 脱饱和)
float computeFullDesaturationHours() {
  float pSurface = 0.79f * (g_surfacePressure / 1000.0f);
  float maxHours = 0;
  for (int i = 0; i < 16; i++) {
    if (g_pN2[i] <= pSurface * 1.05f) continue;   // 已基本平衡
    float hours = 5.0f * HALF_TIMES_N2[i] / 60.0f;
    if (hours > maxHours) maxHours = hours;
  }
  return maxHours;
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
  // 任何超过 5m 的下潜都触发安全停留(更符合"每潜建议安全停留"标准)
  if (depth_m > 5.0f) g_ssArmCondition = true;
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
enum Page : uint8_t { PAGE_HUD = 0, PAGE_TISSUE, PAGE_N2, PAGE_PLAN, PAGE_AIR, PAGE_TEMP, PAGE_BAT, PAGE_LASTDIVE, PAGE_LOG, PAGE_STATS, PAGE_COUNT };
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
  prefs.putFloat("descentL", g_descentLimit);
  prefs.putULong("scrOffMs", g_screenOffMs);
  prefs.putUShort("ssSec",   g_safetyStopSec);
  prefs.putBool("buzzer",    g_buzzerEnable);
  prefs.putFloat("density",  g_fluidDensity);
  prefs.putUShort("barP",    g_initialBar);
  prefs.putUChar("vol",      g_tankVolL);
  prefs.putUChar("sac",      g_sacLmin);
  prefs.putUChar("cons",     g_conservatism);
  prefs.putUChar("lowT",     g_lowTempC);
  prefs.putFloat("batDiv",   g_batDivider);
  prefs.end();
  Serial.println("[SET] Settings saved");
}

void loadSettingsFromNVS() {
  prefs.begin("dive", true);
  g_alarmDepth     = prefs.getFloat("alarmD",   30.0f);
  g_ascentLimit    = prefs.getFloat("ascentL",   9.0f);
  g_descentLimit   = prefs.getFloat("descentL", 18.0f);
  g_screenOffMs    = prefs.getULong("scrOffMs", 5UL * 60UL * 1000UL);
  g_safetyStopSec  = prefs.getUShort("ssSec",   180);
  g_buzzerEnable   = prefs.getBool("buzzer",    true);
  g_fluidDensity   = prefs.getFloat("density",  1029.0f);
  g_initialBar     = prefs.getUShort("barP",    210);
  g_tankVolL       = prefs.getUChar("vol",      11);
  g_sacLmin        = prefs.getUChar("sac",      18);
  g_conservatism   = prefs.getUChar("cons",     100);
  g_lowTempC       = prefs.getUChar("lowT",     15);
  g_batDivider     = prefs.getFloat("batDiv",   2.10f);
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
    // 旧格式记录 size 较小, 用 zero-init + getBytes 兼容
    g_log[i] = {0, 0, 0, 1, 0, 0, 0.0f, 0.0f};
    prefs.getBytes(key, &g_log[i], sizeof(DiveRecord));
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
  // 使用 ESP32-S3 内置 ADC 校准 (eFuse 出厂校准曲线), 比 analogRead() 准很多
  uint32_t sum_mv = 0;
  const int N = 16;
  for (int i = 0; i < N; i++) sum_mv += analogReadMilliVolts(BAT_ADC_PIN);
  float v_mv = sum_mv / (float)N;
  return (v_mv / 1000.0f) * g_batDivider;
}

uint8_t voltageToPct(float v) {
  // 满电容忍: 充满后电池静置电压通常在 4.15-4.20V 之间, 视为 100%
  // (避免显示 98% 不到的心理落差; 实际 SoC 在 95-100% 范围)
  if (v >= 4.15f)   return 100;
  if (v <= BAT_EMPTY_V) return 0;
  // 精修过的锂电池 OCV → SoC 曲线 (开路电压, 无负载)
  // 数据源: 典型 18650 / Li-Po SoC vs OCV 表 (Battery University)
  static const float pts[][2] = {
    {4.15f, 100}, {4.10f, 95}, {4.05f, 88}, {4.00f, 80},
    {3.95f, 72},  {3.90f, 65}, {3.85f, 55}, {3.80f, 45},
    {3.75f, 35},  {3.70f, 25}, {3.65f, 15}, {3.55f, 8},
    {3.45f, 3},   {3.30f, 0}
  };
  const int N = sizeof(pts) / sizeof(pts[0]);
  for (int i = 0; i < N - 1; i++) {
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
  // 诊断: 输出 ADC 实测毫伏 (校准后, 分压后还原前)
  uint32_t mv_sum = 0;
  for (int i = 0; i < 8; i++) mv_sum += analogReadMilliVolts(BAT_ADC_PIN);
  float mv_avg = mv_sum / 8.0f;
  Serial.printf("[BAT] ADC=%.0fmV (分压后) -> 实际电池=%.3fV -> %d%%\n",
                mv_avg, g_batVoltage, g_batPct);
}

// Draws a small battery icon at (x, y), 18 wide x 7 tall (incl. tip)
void drawBatteryIcon(int x, int y) {
  if (!g_batPresent) return;
  int bodyW = 16, bodyH = 7;
  tft.drawRect(x, y, bodyW, bodyH, C(RGB_BLACK));
  tft.fillRect(x + bodyW, y + 2, 2, 3, C(RGB_BLACK));  // tip
  int fill = (int)((bodyW - 2) * (g_batPct / 100.0f));
  if (fill > 0) tft.fillRect(x + 1, y + 1, fill, bodyH - 2, C(RGB_BLACK));
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

// ================== Drawing (320x240 v4 UI) ==========================
// 通用 helpers - 黑底白字标题栏 / 底栏 / 中文文字
void v4DrawHeader(const char *title, const char *rightInfo = nullptr) {
  tft.fillRect(0, 0, 320, 32, C(RGB_BLACK));
  u8g2.setBackgroundColor(C(RGB_BLACK));
  u8g2.setForegroundColor(C(RGB_WHITE));
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.drawUTF8(10, 22, title);
  if (rightInfo) {
    int w = u8g2.getUTF8Width(rightInfo);
    u8g2.drawUTF8(310 - w, 22, rightInfo);
  }
  u8g2.setBackgroundColor(C(RGB_ORANGE));
  u8g2.setForegroundColor(C(RGB_BLACK));
}

void v4DrawFooter(const char *tip) {
  tft.fillRect(0, 210, 320, 30, C(RGB_BLACK));
  u8g2.setBackgroundColor(C(RGB_BLACK));
  u8g2.setForegroundColor(C(RGB_WHITE));
  u8g2.setFont(u8g2_font_wqy13_t_gb2312);
  u8g2.drawUTF8(8, 228, tip);
  u8g2.setBackgroundColor(C(RGB_ORANGE));
  u8g2.setForegroundColor(C(RGB_BLACK));
}

void v4Text(int x, int y, const char *s, uint16_t color, const uint8_t *font = u8g2_font_wqy16_t_gb2312) {
  u8g2.setFont(font);
  u8g2.setForegroundColor(color);
  u8g2.drawUTF8(x, y, s);
}

// ================== Drawing (legacy 128x64, 待迁移) ==================
// 字体说明:
//   u8g2_font_wqy12_t_chinese1  -> 12px 中文(约1000常用字), 含ASCII
//   u8g2_font_logisoso28_tn     -> 大字数字(深度)
//   u8g2_font_logisoso24_tn     -> 中字数字(时间编辑)
#define FONT_CN     u8g2_font_wqy12_t_gb2312
#define FONT_BIG    u8g2_font_logisoso28_tn
#define FONT_MID    u8g2_font_logisoso24_tn

// 128x64 布局规范:
//   行 baseline y: 10, 22, 34, 46, 58 (12 像素间距, 不重叠)
//   大字 baseline: 36 (logisoso24, 字符占 12-36)
// HUD 4 段: 顶 y=10 / 大字 y=36 / 中 y=48 / 底 y=60

// 通用页眉分隔线 (标题下方 y=12, 横跨整屏)
inline void drawDivider() {
  tft.drawFastHLine(0, 12, 128, C(RGB_BLACK));
}
// ============== v4 HUD - 320x240 横屏布局 ==============
// 用 static lastPage 跟踪页面切换, 切页时全屏清, 否则只局部清
// 避免每帧 fillScreen 闪烁
static uint8_t s_lastPage = 255;          // 强制首次重画
static char    s_lastTime[8] = "";        // 上次时钟字符串, 变了才重画
static float   s_lastDepth = -999;
static float   s_lastTemp = -999;
static float   s_lastNdl = -999;
static int     s_lastBat = -1;
static bool    s_lastDiving = false;

void drawHudStaticFrame() {
  // 一次性画静态背景 (页面切换时调用)
  tft.fillScreen(C(RGB_ORANGE));
  // 顶栏
  tft.fillRect(0, 0, 320, 36, C(RGB_BLACK));
  // 底栏
  tft.fillRect(0, 210, 320, 30, C(RGB_BLACK));
  // 标签
  u8g2.setBackgroundColor(C(RGB_ORANGE));
  u8g2.setFont(FONT_CN);
  u8g2.setForegroundColor(C(RGB_BLACK));
  u8g2.drawUTF8(10, 165, "NDL");
  u8g2.drawUTF8(180, 165, "N2");
  u8g2.drawUTF8(10, 190, "最深");
  u8g2.drawUTF8(180, 190, "温度");
  u8g2.drawUTF8(10, 205, "上升");
  // 底栏操作提示
  u8g2.setBackgroundColor(C(RGB_BLACK));
  u8g2.setForegroundColor(C(RGB_WHITE));
  u8g2.drawUTF8(8, 230, "MODE:翻页  UP:重置  DOWN:计时");
  u8g2.setBackgroundColor(C(RGB_ORANGE));
  // 分隔线
  tft.drawFastHLine(0, 140, 320, C(RGB_BLACK));
}

void drawHud(float depth, float maxDepth, float temp, float ndl, float ascentMpm, uint32_t diveSec) {
  // 切页或首次进入: 画完整背景
  if (s_lastPage != PAGE_HUD) {
    s_lastPage = PAGE_HUD;
    drawHudStaticFrame();
    s_lastTime[0] = 0; s_lastDepth = s_lastTemp = s_lastNdl = -999; s_lastBat = -1;
  }

  char buf[40];

  // ===== 顶栏 (黑底白字, y=0-36) =====
  u8g2.setBackgroundColor(C(RGB_BLACK));
  u8g2.setForegroundColor(C(RGB_WHITE));
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);

  uint8_t hh, mm, ss;
  getCurrentTime(hh, mm, ss);
  snprintf(buf, sizeof(buf), "%02u:%02u", hh, mm);
  if (strcmp(buf, s_lastTime) != 0) {
    strcpy(s_lastTime, buf);
    tft.fillRect(8, 8, 60, 24, C(RGB_BLACK));
    u8g2.drawUTF8(8, 26, buf);
  }

  // 潜水时长 (中部)
  tft.fillRect(80, 8, 160, 24, C(RGB_BLACK));
  if (g_diving) {
    snprintf(buf, sizeof(buf), "潜水中 %lu:%02lu", (unsigned long)(diveSec / 60), (unsigned long)(diveSec % 60));
    u8g2.setForegroundColor(C(RGB_GREEN));
    u8g2.drawUTF8(95, 26, buf);
  } else {
    u8g2.setForegroundColor(C(RGB_WHITE));
    u8g2.drawUTF8(130, 26, "水面");
  }

  // 电池 + 海淡
  tft.fillRect(240, 8, 80, 24, C(RGB_BLACK));
  if (g_batPresent) {
    snprintf(buf, sizeof(buf), "%d%% %s", g_batPct, g_fluidDensity > 1010 ? "海" : "淡");
    u8g2.setForegroundColor(C(RGB_WHITE));
    u8g2.drawUTF8(245, 26, buf);
  } else {
    u8g2.drawUTF8(245, 26, g_fluidDensity > 1010 ? "海" : "淡");
  }

  // ===== 大字深度 (橘黄底, y=40-130) =====
  u8g2.setBackgroundColor(C(RGB_ORANGE));
  bool depthAlarm = depth > g_alarmDepth;
  bool decoAlarm  = (ndl <= 0.0f) && g_diving;
  bool alarmRev   = depthAlarm || decoAlarm;

  if (depth != s_lastDepth) {
    s_lastDepth = depth;
    // 清大字区
    if (alarmRev) {
      tft.fillRect(0, 40, 320, 95, C(RGB_BLACK));
      u8g2.setBackgroundColor(C(RGB_BLACK));
      u8g2.setForegroundColor(C(RGB_WHITE));
    } else {
      tft.fillRect(0, 40, 320, 95, C(RGB_ORANGE));
      u8g2.setForegroundColor(C(RGB_BLACK));
    }
    // 大字 logisoso50 (50px)
    u8g2.setFont(u8g2_font_logisoso50_tn);
    if (depth < 100.0f) snprintf(buf, sizeof(buf), "%4.1f", depth);
    else                snprintf(buf, sizeof(buf), "%4.0f", depth);
    int dw = u8g2.getUTF8Width(buf);
    int dx = (320 - dw - 50) / 2;
    u8g2.drawUTF8(dx, 110, buf);
    // 米单位
    u8g2.setFont(u8g2_font_wqy16_t_gb2312);
    u8g2.drawUTF8(dx + dw + 10, 110, "米");
    if (alarmRev) {
      u8g2.setBackgroundColor(C(RGB_ORANGE));
      u8g2.setForegroundColor(C(RGB_BLACK));
    }
  }

  // ===== 数据区 (y=150-210) =====
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setBackgroundColor(C(RGB_ORANGE));

  // NDL 数值 (右于"NDL" 标签)
  if (ndl != s_lastNdl) {
    s_lastNdl = ndl;
    tft.fillRect(55, 150, 110, 20, C(RGB_ORANGE));
    bool ssActive = (g_ss == SS_RUNNING || g_ss == SS_ARMED);
    if (ssActive) {
      uint32_t remain = (g_ssAccumMs >= g_safetyStopSec * 1000UL) ? 0 : (g_safetyStopSec * 1000UL - g_ssAccumMs);
      snprintf(buf, sizeof(buf), "安停%lus", (unsigned long)(remain / 1000));
      u8g2.setForegroundColor(C(RGB_RED));
    } else if (g_ss == SS_DONE) {
      snprintf(buf, sizeof(buf), "已完成");
      u8g2.setForegroundColor(C(RGB_GREEN));
    } else if (decoAlarm) {
      snprintf(buf, sizeof(buf), "需减压!");
      u8g2.setForegroundColor(C(RGB_RED));
    } else if (ndl >= 99.0f) {
      snprintf(buf, sizeof(buf), ">99分");
      u8g2.setForegroundColor(C(RGB_GREEN));
    } else {
      snprintf(buf, sizeof(buf), "%.0f 分", ndl);
      u8g2.setForegroundColor(ndl < 5 ? C(RGB_RED) : C(RGB_BLACK));
    }
    u8g2.drawUTF8(55, 165, buf);
  }

  // N2 负荷
  tft.fillRect(220, 150, 90, 20, C(RGB_ORANGE));
  float n2Pct = computeTissueLoadPct(depth);
  snprintf(buf, sizeof(buf), "%.0f%%", n2Pct);
  u8g2.setForegroundColor(n2Pct > 80 ? C(RGB_RED) : (n2Pct > 60 ? C(RGB_BLUE) : C(RGB_BLACK)));
  u8g2.drawUTF8(220, 165, buf);

  // 最深
  tft.fillRect(60, 175, 110, 20, C(RGB_ORANGE));
  snprintf(buf, sizeof(buf), "%.1f米", maxDepth);
  u8g2.setForegroundColor(C(RGB_BLACK));
  u8g2.drawUTF8(60, 190, buf);

  // 温度
  if (temp != s_lastTemp) {
    s_lastTemp = temp;
    tft.fillRect(220, 175, 90, 20, C(RGB_ORANGE));
    snprintf(buf, sizeof(buf), "%.1f度", temp);
    u8g2.setForegroundColor(temp < g_lowTempC ? C(RGB_BLUE) : C(RGB_BLACK));
    u8g2.drawUTF8(220, 190, buf);
  }

  // 上升速率
  tft.fillRect(60, 195, 200, 18, C(RGB_ORANGE));
  snprintf(buf, sizeof(buf), "%+.1f 米/分", ascentMpm);
  u8g2.setForegroundColor((ascentMpm > g_ascentLimit || -ascentMpm > g_descentLimit) ? C(RGB_RED) : C(RGB_BLACK));
  u8g2.drawUTF8(60, 208, buf);
}

void drawTissue(float depth) {
  char buf[32];
  float pct = computeTissueLoadPct(depth);
  snprintf(buf, sizeof(buf), "最大 %.0f%%", pct);
  v4DrawHeader("组织负荷 16室", buf);

  // 局部清内容区
  tft.fillRect(0, 36, 320, 174, C(RGB_ORANGE));

  // 16 房间柱状图 (320 宽, 每柱 14px + 6 gap)
  int barW = 14, gap = 6, x0 = 8, yTop = 50, hMax = 120;
  for (int i = 0; i < 16; i++) {
    float pAmb_bar = (g_surfacePressure / 1000.0f) + depth * g_fluidDensity * 9.80665f / 1e5f;
    float Mv = pAmb_bar / B_N2[i] + A_N2[i];
    float p = g_pN2[i] / Mv;
    if (p > 1.0f) p = 1.0f;
    if (p < 0.0f) p = 0.0f;
    int h = (int)(p * hMax);
    int x = x0 + i * (barW + gap);
    uint16_t fill = (p > 0.8f) ? C(RGB_RED) : (p > 0.6f) ? C(RGB_BLUE) : C(RGB_BLACK);
    tft.drawRect(x, yTop, barW, hMax, C(RGB_BLACK));
    tft.fillRect(x, yTop + (hMax - h), barW, h, fill);
  }

  // 底部 NDL
  float ndl = computeNDL(depth);
  uint16_t color = C(RGB_BLACK);
  if (ndl >= 99.0f)      snprintf(buf, sizeof(buf), "NDL  >99 分"), color = C(RGB_GREEN);
  else if (ndl <= 0.0f)  snprintf(buf, sizeof(buf), "需要减压"), color = C(RGB_RED);
  else                   snprintf(buf, sizeof(buf), "NDL  %.0f 分钟", ndl), color = (ndl < 5) ? C(RGB_RED) : C(RGB_BLACK);
  v4Text(10, 195, buf, color);

  v4DrawFooter("16 个组织室惰性气体负荷可视化");
}

void drawLastDive() {
  char hdrR[24];
  snprintf(hdrR, sizeof(hdrR), "累计 %u 次", g_diveTotal);
  v4DrawHeader("上次潜水", hdrR);
  v4DrawFooter("MODE:翻页");

  tft.fillRect(0, 36, 320, 174, C(RGB_ORANGE));
  char buf[48];

  if (g_logCount == 0) {
    v4Text(20, 110, "无潜水记录", C(RGB_BLACK));
    return;
  }

  DiveRecord &r = g_log[0];
  // 大字最深深度
  u8g2.setFont(u8g2_font_logisoso26_tn);
  u8g2.setForegroundColor(C(RGB_BLACK));
  snprintf(buf, sizeof(buf), "%.1f", r.maxDepth);
  int dw = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(20, 90, buf);
  v4Text(30 + dw, 90, "米 最深", C(RGB_BLACK));

  // 4 行数据 (y=120/145/170/195)
  snprintf(buf, sizeof(buf), "时长  %u:%02u", r.durationSec/60, r.durationSec%60);
  v4Text(20, 122, buf, C(RGB_BLACK));
  if (r.avgDepth > 0.1f) {
    snprintf(buf, sizeof(buf), "均深  %.1f 米", r.avgDepth);
    v4Text(180, 122, buf, C(RGB_BLACK));
  }

  snprintf(buf, sizeof(buf), "温度  %.1f 度", r.minTemp);
  v4Text(20, 148, buf, C(RGB_BLACK));

  if (r.batVoltage > 0.1f) {
    snprintf(buf, sizeof(buf), "电池  %.2f V", r.batVoltage);
    v4Text(180, 148, buf, C(RGB_BLACK));
  }

  v4Text(20, 174, r.saltwater ? "海水" : "淡水", C(RGB_BLACK));
  if (r.airUsedL > 1.0f) {
    snprintf(buf, sizeof(buf), "耗气  %.0f L", r.airUsedL);
    v4Text(100, 174, buf, C(RGB_BLACK));
  }

  // 距今时间
  if (r.endEpoch > 0) {
    time_t nowEpoch;
    time(&nowEpoch);
    int32_t intervalSec = (int32_t)((uint32_t)nowEpoch - r.endEpoch);
    if (intervalSec >= 0 && intervalSec < 99 * 3600) {
      snprintf(buf, sizeof(buf), "距今 %ldh%02ldm",
               (long)(intervalSec / 3600), (long)((intervalSec / 60) % 60));
      v4Text(20, 200, buf, C(RGB_BLUE));
    }
  }
}

void drawLogList() {
  char hdrR[24];
  snprintf(hdrR, sizeof(hdrR), "%u/%u", g_logViewIdx + 1, g_logCount);
  v4DrawHeader("潜水日志", hdrR);
  v4DrawFooter("UP/DOWN 浏览  长按DOWN在LOG页清空");

  tft.fillRect(0, 36, 320, 174, C(RGB_ORANGE));

  char buf[48];

  if (g_logCount == 0) {
    v4Text(20, 110, "日志为空", C(RGB_BLACK));
    return;
  }

  if (g_logViewIdx >= g_logCount) g_logViewIdx = g_logCount - 1;
  DiveRecord &r = g_log[g_logViewIdx];

  // 大字: # 序号
  u8g2.setFont(u8g2_font_logisoso26_tn);
  u8g2.setForegroundColor(C(RGB_BLACK));
  snprintf(buf, sizeof(buf), "#%u", g_diveTotal - g_logViewIdx);
  u8g2.drawUTF8(20, 80, buf);

  // 最深  (大字右边)
  snprintf(buf, sizeof(buf), "%.1f", r.maxDepth);
  u8g2.drawUTF8(140, 80, buf);
  v4Text(230, 80, "米 最深", C(RGB_BLACK));

  // 4 行细节
  snprintf(buf, sizeof(buf), "时长  %u:%02u", r.durationSec/60, r.durationSec%60);
  v4Text(20, 115, buf, C(RGB_BLACK));
  v4Text(180, 115, r.saltwater ? "海水" : "淡水", C(RGB_BLACK));

  snprintf(buf, sizeof(buf), "温度  %.1f 度", r.minTemp);
  v4Text(20, 145, buf, C(RGB_BLACK));
  if (r.batVoltage > 0.1f) {
    snprintf(buf, sizeof(buf), "电池 %.2fV", r.batVoltage);
    v4Text(180, 145, buf, C(RGB_BLACK));
  }

  if (r.airUsedL > 1.0f) {
    snprintf(buf, sizeof(buf), "耗气  %.0f L", r.airUsedL);
    v4Text(20, 175, buf, C(RGB_BLACK));
  }
  if (r.avgDepth > 0.1f) {
    snprintf(buf, sizeof(buf), "均深  %.1f m", r.avgDepth);
    v4Text(180, 175, buf, C(RGB_BLACK));
  }
}

void drawAir() {
  char buf[48];
  uint16_t total_L = totalAirL();
  float remainingBar = barFromAirL(g_remainingAirL);
  uint8_t pct = (total_L > 0) ? (uint8_t)((g_remainingAirL / total_L) * 100.0f) : 0;

  // 顶栏右侧: 消耗速率
  if (g_diving && g_currentSacL > 0.1f) {
    float currentBarMin = (g_tankVolL > 0) ? (g_currentSacL / g_tankVolL) : 0;
    snprintf(buf, sizeof(buf), "%.1f bar/min", currentBarMin);
  } else {
    snprintf(buf, sizeof(buf), "水面");
  }
  v4DrawHeader("气量监控", buf);
  v4DrawFooter("瓶配置在设置中修改");

  tft.fillRect(0, 36, 320, 174, C(RGB_ORANGE));

  // 大字剩余压力
  u8g2.setFont(u8g2_font_logisoso26_tn);
  u8g2.setForegroundColor(pct < 25 ? C(RGB_RED) : (pct < 50 ? C(RGB_BLUE) : C(RGB_BLACK)));
  snprintf(buf, sizeof(buf), "%3d", (int)remainingBar);
  u8g2.drawUTF8(20, 90, buf);
  v4Text(110, 90, "bar", C(RGB_BLACK));
  snprintf(buf, sizeof(buf), "%d%%", pct);
  v4Text(220, 90, buf, C(RGB_BLACK));

  // Air Time
  if (g_diving && g_currentSacL > 0.1f) {
    int airMin = (int)(g_remainingAirL / g_currentSacL);
    if (airMin > 999) airMin = 999;
    snprintf(buf, sizeof(buf), "Air Time  %d 分钟", airMin);
  } else {
    snprintf(buf, sizeof(buf), "Air Time  --");
  }
  v4Text(20, 130, buf, C(RGB_BLACK));

  // 瓶配置
  snprintf(buf, sizeof(buf), "瓶  %d bar × %d L   SAC %d L/分",
           g_initialBar, g_tankVolL, g_sacLmin);
  v4Text(20, 160, buf, C(RGB_BLACK));

  // 大压力条 (y=180-200)
  int barY = 180, barH = 20;
  tft.drawRect(20, barY, 280, barH, C(RGB_BLACK));
  int fillW = (g_initialBar > 0) ? (int)(278.0f * remainingBar / g_initialBar) : 0;
  if (fillW > 278) fillW = 278;
  uint16_t barColor = (pct < 25) ? C(RGB_RED) : (pct < 50) ? C(RGB_BLUE) : C(RGB_GREEN);
  if (fillW > 0) tft.fillRect(21, barY + 1, fillW, barH - 2, barColor);
}

// 体内氮气安全页 - 基于 ZHL-16C 衍生指标
void drawN2() {
  v4DrawHeader("体内氮气 ZHL-16C");
  v4DrawFooter("基于 Bühlmann 模型");

  tft.fillRect(0, 36, 320, 174, C(RGB_ORANGE));
  char buf[40];

  // 1. 大字: 氮气负荷 % + 风险
  float loadPct = computeTissueLoadPct(g_depthSmooth);
  const char *risk;
  uint16_t riskColor;
  if (loadPct < 60.0f)      { risk = "低风险"; riskColor = C(RGB_GREEN); }
  else if (loadPct < 80.0f) { risk = "中风险"; riskColor = C(RGB_BLUE); }
  else                       { risk = "高风险"; riskColor = C(RGB_RED); }

  u8g2.setFont(u8g2_font_logisoso26_tn);
  u8g2.setForegroundColor(C(RGB_BLACK));
  snprintf(buf, sizeof(buf), "%.0f", loadPct);
  u8g2.drawUTF8(20, 80, buf);
  v4Text(85, 80, "%", C(RGB_BLACK));
  v4Text(20, 100, "氮气负荷", C(RGB_BLACK));

  v4Text(200, 80, risk, riskColor);

  // 2. 最快/最慢组织室
  float fastSat = computeSaturationPct(0,  g_depthSmooth);
  float slowSat = computeSaturationPct(15, g_depthSmooth);
  snprintf(buf, sizeof(buf), "最快 %.0f%%   最慢 %.0f%%", fastSat, slowSat);
  v4Text(20, 130, buf, C(RGB_BLACK));

  // 3. 禁飞时间
  float noFlyH = computeNoFlyTimeHours();
  if (noFlyH < 0.05f) {
    v4Text(20, 160, "禁飞:  可飞行", C(RGB_GREEN));
  } else if (noFlyH < 100.0f) {
    snprintf(buf, sizeof(buf), "禁飞:  %dh%02dm",
             (int)noFlyH, (int)((noFlyH - (int)noFlyH) * 60));
    v4Text(20, 160, buf, C(RGB_RED));
  } else {
    v4Text(20, 160, "禁飞:  >100h", C(RGB_RED));
  }

  // 4. 完全脱饱和
  float desatH = computeFullDesaturationHours();
  if (desatH < 0.05f) {
    v4Text(20, 190, "脱饱和:  已完成", C(RGB_GREEN));
  } else if (desatH < 100.0f) {
    snprintf(buf, sizeof(buf), "脱饱和:  %.0f 小时", desatH);
    v4Text(20, 190, buf, C(RGB_BLACK));
  } else {
    v4Text(20, 190, "脱饱和:  >100 小时", C(RGB_BLACK));
  }
}

void drawPlan() {
  char hdrR[24];
  snprintf(hdrR, sizeof(hdrR), "当前 %.1f m", g_depthSmooth);
  v4DrawHeader("潜水规划", hdrR);
  v4DrawFooter("UP/DOWN 改计划深度");

  tft.fillRect(0, 36, 320, 174, C(RGB_ORANGE));
  char buf[48];

  // 大字: 计划深度
  u8g2.setFont(u8g2_font_logisoso26_tn);
  u8g2.setForegroundColor(C(RGB_BLACK));
  snprintf(buf, sizeof(buf), "%u", g_planDepthM);
  u8g2.drawUTF8(20, 90, buf);
  v4Text(110, 90, "米 计划深度", C(RGB_BLACK));

  // NDL @ 该深度
  float ndl = computeNDL((float)g_planDepthM);
  uint16_t color = C(RGB_BLACK);
  if (ndl >= 99.0f)      snprintf(buf, sizeof(buf), "NDL  >99 分钟"), color = C(RGB_GREEN);
  else if (ndl <= 0.0f)  snprintf(buf, sizeof(buf), "NDL  需要减压!"), color = C(RGB_RED);
  else                   snprintf(buf, sizeof(buf), "NDL  %.0f 分钟", ndl), color = (ndl < 5) ? C(RGB_RED) : C(RGB_BLACK);
  v4Text(20, 130, buf, color);

  // Air Time @ 该深度
  uint16_t total_L = totalAirL();
  float sacAtDepth = g_sacLmin * (1.0f + g_planDepthM / 10.0f);
  if (sacAtDepth > 0) {
    int airMin = (int)(total_L / sacAtDepth);
    if (airMin > 999) airMin = 999;
    snprintf(buf, sizeof(buf), "满瓶气量  %d 分钟", airMin);
  } else {
    snprintf(buf, sizeof(buf), "满瓶气量  --");
  }
  v4Text(20, 160, buf, C(RGB_BLACK));

  // 提示
  v4Text(20, 195, "潜前规划工具, 估算该深度安全潜水时长", C(RGB_DARK), u8g2_font_wqy13_t_gb2312);
}

void drawTempChart() {
  char buf[40];
  snprintf(buf, sizeof(buf), "%.1f度 (%u/60)", g_temp, g_tempHistCount);
  v4DrawHeader("温度趋势 1h", buf);
  v4DrawFooter("每分钟一个采样, 60 个 = 1 小时");

  tft.fillRect(0, 36, 320, 174, C(RGB_ORANGE));

  if (g_tempHistCount < 2) {
    v4Text(20, 110, "数据收集中, 每分钟一个样本", C(RGB_BLACK));
    return;
  }

  // 找 min/max
  float minT = 999, maxT = -999;
  for (int i = 0; i < g_tempHistCount; i++) {
    if (g_tempHist[i] < minT) minT = g_tempHist[i];
    if (g_tempHist[i] > maxT) maxT = g_tempHist[i];
  }
  if (maxT - minT < 0.5f) { float c = (maxT + minT) / 2; minT = c - 0.25f; maxT = c + 0.25f; }

  // Y 轴标签
  snprintf(buf, sizeof(buf), "%.1f", maxT);
  v4Text(5, 60, buf, C(RGB_BLACK), u8g2_font_wqy13_t_gb2312);
  snprintf(buf, sizeof(buf), "%.1f", minT);
  v4Text(5, 195, buf, C(RGB_BLACK), u8g2_font_wqy13_t_gb2312);

  // 绘图区 x=50-310 y=50-200 (260x150)
  int x0 = 50, y0 = 50, gw = 260, gh = 150;
  tft.drawRect(x0, y0, gw, gh, C(RGB_BLACK));

  // 折线
  for (int i = 0; i < g_tempHistCount - 1; i++) {
    int idx1 = (g_tempHistIdx + TEMP_HIST_LEN - g_tempHistCount + i) % TEMP_HIST_LEN;
    int idx2 = (g_tempHistIdx + TEMP_HIST_LEN - g_tempHistCount + i + 1) % TEMP_HIST_LEN;
    int x1 = x0 + 2 + i * (gw - 4) / (g_tempHistCount - 1);
    int x2 = x0 + 2 + (i + 1) * (gw - 4) / (g_tempHistCount - 1);
    int y1 = y0 + gh - 4 - (int)((g_tempHist[idx1] - minT) * (gh - 8) / (maxT - minT));
    int y2 = y0 + gh - 4 - (int)((g_tempHist[idx2] - minT) * (gh - 8) / (maxT - minT));
    tft.drawLine(x1, y1, x2, y2, C(RGB_BLUE));
    tft.drawLine(x1, y1 + 1, x2, y2 + 1, C(RGB_BLUE));  // 加粗
  }
}

void recordTempSample(uint32_t now) {
  if (!g_sensorOk) return;
  if (g_lastTempSampleMs != 0 && now - g_lastTempSampleMs < 60000) return;
  g_lastTempSampleMs = now;
  g_tempHist[g_tempHistIdx] = g_temp;
  g_tempHistIdx = (g_tempHistIdx + 1) % TEMP_HIST_LEN;
  if (g_tempHistCount < TEMP_HIST_LEN) g_tempHistCount++;
}

// 每小时采样一次电池电压, 环形缓冲 24 小时
void recordBatHistSample(uint32_t now) {
  if (!g_batPresent) return;
  if (g_lastBatHistMs != 0 && now - g_lastBatHistMs < 3600000UL) return;
  g_lastBatHistMs = now;
  g_batHist[g_batHistIdx] = g_batVoltage;
  g_batHistIdx = (g_batHistIdx + 1) % BAT_HIST_LEN;
  if (g_batHistCount < BAT_HIST_LEN) g_batHistCount++;
}

void drawBatHist() {
  char hdrR[24];
  if (g_batPresent) snprintf(hdrR, sizeof(hdrR), "%.2fV %u%%", g_batVoltage, g_batPct);
  else              snprintf(hdrR, sizeof(hdrR), "未接入");
  v4DrawHeader("电池趋势 24h", hdrR);
  v4DrawFooter("每小时采样一次, 24 个 = 1 天");

  tft.fillRect(0, 36, 320, 174, C(RGB_ORANGE));

  char buf[40];
  if (g_batHistCount < 2) {
    snprintf(buf, sizeof(buf), "已采集 %u/24 小时", g_batHistCount);
    v4Text(20, 110, buf, C(RGB_BLACK));
    v4Text(20, 140, "等待数据中...", C(RGB_BLACK));
    return;
  }

  // 找 min/max 电压
  float minV = 5.0f, maxV = 0.0f;
  for (int i = 0; i < g_batHistCount; i++) {
    if (g_batHist[i] < minV) minV = g_batHist[i];
    if (g_batHist[i] > maxV) maxV = g_batHist[i];
  }
  if (maxV - minV < 0.05f) { float c = (maxV + minV) / 2; minV = c - 0.025f; maxV = c + 0.025f; }

  // Y 轴标签
  snprintf(buf, sizeof(buf), "%.2f", maxV);
  v4Text(5, 60, buf, C(RGB_BLACK), u8g2_font_wqy13_t_gb2312);
  snprintf(buf, sizeof(buf), "%.2f", minV);
  v4Text(5, 195, buf, C(RGB_BLACK), u8g2_font_wqy13_t_gb2312);

  // 绘图区 x=50-310 y=50-200
  int x0 = 50, y0 = 50, gw = 260, gh = 150;
  tft.drawRect(x0, y0, gw, gh, C(RGB_BLACK));

  // 折线
  for (int i = 0; i < g_batHistCount - 1; i++) {
    int idx1 = (g_batHistIdx + BAT_HIST_LEN - g_batHistCount + i) % BAT_HIST_LEN;
    int idx2 = (g_batHistIdx + BAT_HIST_LEN - g_batHistCount + i + 1) % BAT_HIST_LEN;
    int x1 = x0 + 2 + i * (gw - 4) / (g_batHistCount - 1);
    int x2 = x0 + 2 + (i + 1) * (gw - 4) / (g_batHistCount - 1);
    int y1 = y0 + gh - 4 - (int)((g_batHist[idx1] - minV) * (gh - 8) / (maxV - minV));
    int y2 = y0 + gh - 4 - (int)((g_batHist[idx2] - minV) * (gh - 8) / (maxV - minV));
    tft.drawLine(x1, y1, x2, y2, C(RGB_GREEN));
    tft.drawLine(x1, y1 + 1, x2, y2 + 1, C(RGB_GREEN));
  }

}

void drawStats() {
  char hdrR[24];
  float alt = estimateAltitudeM();
  if (alt > 50.0f) snprintf(hdrR, sizeof(hdrR), "海拔 %.0fm", alt);
  else if (g_batPresent) snprintf(hdrR, sizeof(hdrR), "%.2fV %u%%", g_batVoltage, g_batPct);
  else snprintf(hdrR, sizeof(hdrR), "未接电池");
  v4DrawHeader("总览统计", hdrR);
  v4DrawFooter("终身潜水数据汇总");

  tft.fillRect(0, 36, 320, 174, C(RGB_ORANGE));

  char buf[48];
  // 4 行数据 y=70/110/150/190
  snprintf(buf, sizeof(buf), "潜水次数      %u 次", g_diveTotal);
  v4Text(30, 70, buf, C(RGB_BLACK));

  snprintf(buf, sizeof(buf), "历史最深      %.1f 米", g_diveTotalMaxD);
  v4Text(30, 110, buf, C(RGB_BLACK));

  uint32_t totMin = g_lifetimeUnderwaterSec / 60;
  uint32_t totH   = totMin / 60;
  snprintf(buf, sizeof(buf), "累计水下      %luh%02lum",
           (unsigned long)totH, (unsigned long)(totMin % 60));
  v4Text(30, 150, buf, C(RGB_BLACK));

  // 距上次潜水
  if (g_logCount > 0 && g_log[0].endEpoch > 0) {
    time_t nowEpoch;
    time(&nowEpoch);
    int32_t intervalSec = (int32_t)((uint32_t)nowEpoch - g_log[0].endEpoch);
    if (intervalSec >= 0 && intervalSec < 99 * 3600) {
      snprintf(buf, sizeof(buf), "距上次潜水    %ldh%02ldm",
               (long)(intervalSec / 3600), (long)((intervalSec / 60) % 60));
      v4Text(30, 190, buf, C(RGB_BLUE));
    }
  }

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
    case 6: return "初始气压";
    case 7: return "瓶容量";
    case 8: return "SAC速率";
    case 9: return "保守度";
    case 10: return "低温报警";
    case 11: return "电池校准";
    case 12: return "下降警告";
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
    case 6: snprintf(buf, sz, "%u bar", g_initialBar); break;
    case 7: snprintf(buf, sz, "%u L", g_tankVolL); break;
    case 8: snprintf(buf, sz, "%u L/分", g_sacLmin); break;
    case 9: snprintf(buf, sz, "%u%%", g_conservatism); break;
    case 10: snprintf(buf, sz, "%u 度", g_lowTempC); break;
    case 11: snprintf(buf, sz, "%.2f", g_batDivider); break;
    case 12: snprintf(buf, sz, "%.0f米/分", g_descentLimit); break;
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
    case 6: {  // 初始气压 100-300 bar 步 10
      int v = (int)g_initialBar + delta * 10;
      if (v < 100) v = 100; if (v > 300) v = 300;
      g_initialBar = (uint16_t)v;
      break;
    }
    case 7: {  // 瓶容量 5-20 L 步 1
      int v = (int)g_tankVolL + delta;
      if (v < 5) v = 5; if (v > 20) v = 20;
      g_tankVolL = (uint8_t)v;
      break;
    }
    case 8: {  // SAC 8-30 L/min 步 1
      int v = (int)g_sacLmin + delta;
      if (v < 8) v = 8; if (v > 30) v = 30;
      g_sacLmin = (uint8_t)v;
      break;
    }
    case 9: {  // 保守度 80-120% 步 10
      int v = (int)g_conservatism + delta * 10;
      if (v < 80) v = 80; if (v > 120) v = 120;
      g_conservatism = (uint8_t)v;
      break;
    }
    case 10: {  // 低温 5-25 度 步 5
      int v = (int)g_lowTempC + delta * 5;
      if (v < 5) v = 5; if (v > 25) v = 25;
      g_lowTempC = (uint8_t)v;
      break;
    }
    case 11: {  // 电池分压系数 1.80-2.30 步 0.02
      g_batDivider += delta * 0.02f;
      if (g_batDivider < 1.80f) g_batDivider = 1.80f;
      if (g_batDivider > 2.30f) g_batDivider = 2.30f;
      // 立刻重新读一次, 让用户能看到变化
      g_batVoltage = readBatteryVoltage();
      g_batPct = (g_batVoltage >= 3.0f && g_batVoltage <= 4.5f) ? voltageToPct(g_batVoltage) : 0;
      break;
    }
    case 12: {  // 下降警告 10-30 m/min 步 2
      g_descentLimit = constrain(g_descentLimit + delta * 2.0f, 10.0f, 30.0f);
      break;
    }
  }
}

void drawSettings() {
  char buf[24];
  snprintf(buf, sizeof(buf), "%u/%u", g_settingsItem + 1, SETTINGS_COUNT);
  v4DrawHeader("设置", buf);
  v4DrawFooter("MODE切项 UP/DOWN改 长按MODE保存");

  // 6 项可见, 每项 25px 行高 (y=50,75,100,125,150,175)
  const int VISIBLE = 6;
  int first = 0;
  if (g_settingsItem >= VISIBLE) first = g_settingsItem - VISIBLE + 1;
  if (first > SETTINGS_COUNT - VISIBLE) first = SETTINGS_COUNT - VISIBLE;
  if (first < 0) first = 0;

  // 局部清内容区
  tft.fillRect(0, 36, 320, 170, C(RGB_ORANGE));

  for (int i = 0; i < VISIBLE && first + i < SETTINGS_COUNT; i++) {
    uint8_t idx = first + i;
    int y = 52 + i * 26;
    bool sel = (idx == g_settingsItem);
    if (sel) tft.fillRect(0, y - 18, 320, 22, C(RGB_DARK));
    uint16_t fg = sel ? C(RGB_WHITE) : C(RGB_BLACK);
    u8g2.setBackgroundColor(sel ? C(RGB_DARK) : C(RGB_ORANGE));
    v4Text(sel ? 8 : 16, y, sel ? ">" : "  ", fg);
    v4Text(30, y, settingsItemName(idx), fg);
    char vbuf[24];
    settingsItemValue(idx, vbuf, sizeof(vbuf));
    int vw = u8g2.getUTF8Width(vbuf);
    v4Text(310 - vw, y, vbuf, fg);
    u8g2.setBackgroundColor(C(RGB_ORANGE));
  }

  // 滚动箭头
  u8g2.setForegroundColor(C(RGB_BLACK));
  if (first > 0)                          u8g2.drawUTF8(305, 50,  "^");
  if (first + VISIBLE < SETTINGS_COUNT)   u8g2.drawUTF8(305, 200, "v");
}

void drawSettingsAndSend() {
  drawSettings();
}

void drawTimeEdit() {
  v4DrawHeader("设置时间", "MODE 保存");
  v4DrawFooter("UP/DN 增减   MODE 切换字段");

  tft.fillRect(0, 36, 320, 174, C(RGB_ORANGE));

  // 大字时间 居中 (logisoso50 50px)
  char buf[8];
  u8g2.setFont(u8g2_font_logisoso50_tn);
  u8g2.setForegroundColor(C(RGB_BLACK));
  u8g2.setBackgroundColor(C(RGB_ORANGE));
  snprintf(buf, sizeof(buf), "%02u:%02u", g_editHH, g_editMM);
  int w = u8g2.getUTF8Width(buf);
  int x = (320 - w) / 2;
  u8g2.drawUTF8(x, 130, buf);

  // 下划线指示当前编辑字段
  int colonOffset = u8g2.getUTF8Width("00");
  int afterColon  = u8g2.getUTF8Width("00:");
  int hhX = x;
  int mmX = x + afterColon;
  // 下划线指示当前编辑字段 (画 4px 粗)
  if (g_editField == 0) tft.fillRect(hhX, 135, colonOffset, 4, C(RGB_RED));
  else                  tft.fillRect(mmX, 135, colonOffset, 4, C(RGB_RED));

  v4Text((320 - u8g2.getUTF8Width("当前编辑: 时 / 分")) / 2, 170,
         g_editField == 0 ? "当前编辑: 时" : "当前编辑: 分", C(RGB_BLUE));
}

void drawSplash() {
  tft.fillScreen(C(RGB_ORANGE));

  // 大字标题
  u8g2.setFont(u8g2_font_logisoso26_tn);
  u8g2.setForegroundColor(C(RGB_BLACK));
  u8g2.setBackgroundColor(C(RGB_ORANGE));
  // 用中文字体显示
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);
  u8g2.setForegroundColor(C(RGB_BLACK));

  // 居中"潜水表"大标题
  u8g2.drawUTF8(80, 70, "潜水表 DiveWatch");

  // 版本/算法
  u8g2.drawUTF8(80, 105, "v4.0  ZHL-16C  ST7789");

  // 水面气压 + 密度
  char buf[48];
  snprintf(buf, sizeof(buf), "水面气压  %.1f mbar", g_surfacePressure);
  u8g2.drawUTF8(50, 140, buf);
  snprintf(buf, sizeof(buf), "水密度    %.0f kg/m3 (%s)",
           g_fluidDensity, g_fluidDensity > 1010 ? "海水" : "淡水");
  u8g2.drawUTF8(50, 165, buf);

  // 底部水平线
  tft.drawFastHLine(0, 200, 320, C(RGB_BLACK));
  u8g2.drawUTF8(80, 220, "开机中, 请稍候...");
}

void drawError(const char *l1, const char *l2 = nullptr) {
  tft.fillScreen(C(RGB_RED));   // 红色背景表示错误
  u8g2.setBackgroundColor(C(RGB_RED));
  u8g2.setForegroundColor(C(RGB_WHITE));
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);

  u8g2.drawUTF8(100, 60, "*** 错误 ***");
  if (l1) u8g2.drawUTF8(40, 110, l1);
  if (l2) u8g2.drawUTF8(40, 140, l2);
  u8g2.drawUTF8(40, 200, "检查接线后重启");

  // 恢复默认背景给后续绘制
  u8g2.setBackgroundColor(C(RGB_ORANGE));
  u8g2.setForegroundColor(C(RGB_BLACK));
}

// 进入深度睡眠 (软关机): ~10μA 功耗
// 唤醒源 = MODE 按键 (BTN_MODE_PIN = GPIO 6, RTC IO, LOW 触发)
void enterDeepSleep() {
  Serial.println("[POWER] 准备关机, 进入深度睡眠...");
  saveClockToNVS();
  saveSettingsToNVS();

  // 显示"关机中"画面
  // tft.enableDisplay (no-op v4)
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 16, "正在关机...");
  u8g2.drawUTF8(0, 36, "再次长按 MODE");
  u8g2.drawUTF8(0, 50, "2 秒以上即可开机");

  // 蜂鸣 3 长声做关机提示
  beepBlocking(3, 250, 150);
  delay(800);

  // 关 OLED + WiFi
  // tft.sleepMode (no-op v4)
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // 设置唤醒源: MODE 按键 (RTC GPIO 6) 拉低触发
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_MODE_PIN, 0);

  Serial.flush();
  delay(50);
  esp_deep_sleep_start();
  // 不会返回, 唤醒后从 setup() 重新执行
}

// 唤醒后验证: 按键必须持续按下 ≥ 2 秒才算"真正开机"
// 否则视为误触 / 短按, 立刻回深度睡眠
void verifyWakeOrSleepAgain() {
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  if (reason != ESP_SLEEP_WAKEUP_EXT0) return;  // 非按键唤醒(首次冷启动等), 不验证

  pinMode(BTN_MODE_PIN, INPUT_PULLUP);
  delay(20);  // 等 GPIO 稳定
  unsigned long pressStart = millis();
  while (digitalRead(BTN_MODE_PIN) == LOW && millis() - pressStart < 2200) {
    delay(50);
  }
  if (digitalRead(BTN_MODE_PIN) == HIGH) {
    // 按键已释放, 不足 2 秒 → 视为误触, 立刻回睡
    esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_MODE_PIN, 0);
    esp_deep_sleep_start();
  }
  // 按够 2 秒, 继续正常 setup
}

// 开机自检: I2C 设备 + 按钮 + 蜂鸣器 (320x240 版)
void runSelfTest() {
  tft.fillScreen(C(RGB_ORANGE));
  u8g2.setBackgroundColor(C(RGB_ORANGE));
  u8g2.setForegroundColor(C(RGB_BLACK));
  u8g2.setFont(u8g2_font_wqy16_t_gb2312);

  // 大字标题
  u8g2.drawUTF8(80, 50, "系统自检");
  u8g2.drawUTF8(80, 80, "正在检测...");

  // 检查 OLED I2C (能跑到这步说明 SPI 屏 OK, 但 MS5837 还要测 I2C)
  bool oledOK = true;

  // 检查 MS5837 0x76
  Wire.beginTransmission(0x76);
  bool ms5837OK = (Wire.endTransmission() == 0);

  // 蜂鸣器响 1 声
  digitalWrite(BUZZER_PIN, HIGH);
  delay(60);
  digitalWrite(BUZZER_PIN, LOW);
  bool buzzerTested = true;

  // 等待按键释放 (容忍开机长按 + 抖动)
  auto checkBtnStable = [](uint8_t pin, uint32_t timeoutMs) -> bool {
    unsigned long deadline = millis() + timeoutMs;
    unsigned long stableStart = millis();
    while (millis() < deadline) {
      if (digitalRead(pin) == LOW) stableStart = millis();
      else if (millis() - stableStart >= 500) return true;
      delay(20);
    }
    return false;
  };

  u8g2.drawUTF8(80, 130, "请松开按键");
  bool btnMOK = checkBtnStable(BTN_MODE_PIN, 4000);
  bool btnUOK = checkBtnStable(BTN_UP_PIN,   1000);
  bool btnDOK = checkBtnStable(BTN_DOWN_PIN, 1000);
  bool btnOK  = btnMOK && btnUOK && btnDOK;

  // 自检结果页 (重画)
  tft.fillScreen(C(RGB_ORANGE));
  u8g2.drawUTF8(60, 35, "系统自检结果");
  tft.drawFastHLine(40, 45, 240, C(RGB_BLACK));

  char buf[32];
  // 4 项, 每项 35px 行高
  snprintf(buf, sizeof(buf), "TFT 显示       %s", oledOK ? "正常" : "异常");
  u8g2.setForegroundColor(oledOK ? C(RGB_GREEN) : C(RGB_RED));
  u8g2.drawUTF8(40, 80, buf);

  snprintf(buf, sizeof(buf), "MS5837 传感器  %s", ms5837OK ? "正常" : "异常");
  u8g2.setForegroundColor(ms5837OK ? C(RGB_GREEN) : C(RGB_RED));
  u8g2.drawUTF8(40, 115, buf);

  snprintf(buf, sizeof(buf), "按键 M:%s U:%s D:%s",
           btnMOK ? "正常" : "异常",
           btnUOK ? "正常" : "异常",
           btnDOK ? "正常" : "异常");
  u8g2.setForegroundColor(btnOK ? C(RGB_GREEN) : C(RGB_RED));
  u8g2.drawUTF8(40, 150, buf);

  snprintf(buf, sizeof(buf), "蜂鸣器         %s", buzzerTested ? "已测试" : "异常");
  u8g2.setForegroundColor(C(RGB_GREEN));
  u8g2.drawUTF8(40, 185, buf);

  u8g2.setForegroundColor(C(RGB_BLACK));
  Serial.printf("[SELF-TEST] OLED=%d MS5837=%d BTN_M=%d BTN_U=%d BTN_D=%d\n",
                oledOK, ms5837OK, btnMOK, btnUOK, btnDOK);
  delay(2000);
}

// ================== Setup ============================================
void setup() {
  // 最先检测唤醒源: 如果是按键唤醒但按时不够 2s, 立刻回睡 (不开机)
  verifyWakeOrSleepAgain();

  Serial.begin(115200);
  delay(120);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);
  pinMode(BTN_UP_PIN,   INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // ---- ST7789 SPI 显示初始化 ----
  mySPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  tft.init(240, 320);
  tft.setSPISpeed(10000000);
  tft.setRotation(1);                    // 横屏 320x240
  tft.invertDisplay(true);
  tft.fillScreen(C(RGB_ORANGE));

  u8g2.begin(tft);
  u8g2.setFontMode(0);                   // solid 背景模式
  u8g2.setFontDirection(0);
  u8g2.setBackgroundColor(C(RGB_ORANGE));
  u8g2.setForegroundColor(C(RGB_BLACK));

  drawSplash();
  delay(400);
  runSelfTest();

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
    // 覆盖最后一行 (密度) 显示 WiFi 状态
    u8g2.setForegroundColor(C(RGB_WHITE));
    tft.fillRect(0, 46, 128, 18, C(RGB_BLACK));
    u8g2.setForegroundColor(C(RGB_BLACK));
    u8g2.setFont(FONT_CN);
    u8g2.drawUTF8(0, 58, "WiFi 同步中...");
    bool ok = trySyncNTP();
    u8g2.setForegroundColor(C(RGB_WHITE));
    tft.fillRect(0, 46, 128, 18, C(RGB_BLACK));
    u8g2.setForegroundColor(C(RGB_BLACK));
    u8g2.setFont(FONT_CN);
    u8g2.drawUTF8(0, 58, ok ? "时间同步完成" : "WiFi 同步失败");
    delay(1200);
  }

  beepBlocking(2, 70);
  delay(400);

  // 显式回到主 HUD 页, 重置任何残留状态 (防止开机进错页)
  g_page = PAGE_HUD;
  g_editingTime = false;
  g_inSettings  = false;
  g_screenOff   = false;
  g_btnMode.evShort = g_btnMode.evLong = false;
  g_btnUp.evShort   = g_btnUp.evLong   = false;
  g_btnDown.evShort = g_btnDown.evLong = false;

  // 重置潜水统计变量 (避免 NVS 残留乱值)
  g_maxDepth = 0;
  g_lastDepth = 0;
  g_diveDepthSum = 0;
  g_diveTimeMs = 0;
  g_ascentMpm = 0;
  g_diving = false;

  // Sanity check: NVS 加载的设置如果在异常范围, 用默认值
  if (g_conservatism < 50 || g_conservatism > 200)  g_conservatism = 100;
  if (g_fluidDensity < 500 || g_fluidDensity > 2000) g_fluidDensity = 1029;
  if (g_alarmDepth < 5 || g_alarmDepth > 100)       g_alarmDepth = 30;
  if (g_ascentLimit < 1 || g_ascentLimit > 50)      g_ascentLimit = 9;
  if (g_descentLimit < 1 || g_descentLimit > 100)   g_descentLimit = 18;
  Serial.printf("[SANITY] cons=%u density=%.0f alarmD=%.0f ascent=%.0f descent=%.0f\n",
                g_conservatism, g_fluidDensity, g_alarmDepth, g_ascentLimit, g_descentLimit);

  g_lastInteractMs = millis();
}

// ================== Loop =============================================
void loop() {
  uint32_t now = millis();

  // ---- Buttons ----
  btnPoll(g_btnMode, now);
  btnPoll(g_btnUp,   now);
  btnPoll(g_btnDown, now);

  // ---- 长按 MODE 5 秒 -> 关机 (深度睡眠) ----
  static bool s_shutdownArmed = false;
  if (!g_btnMode.stable && (now - g_btnMode.pressedAtMs) > 5000 && !s_shutdownArmed) {
    s_shutdownArmed = true;
    enterDeepSleep();
    // 不会返回
  }
  if (g_btnMode.stable) s_shutdownArmed = false;  // 松手重置

  // ---- Power saving: wake / sleep OLED ----
  bool anyBtnEvent = g_btnMode.evShort || g_btnMode.evLong ||
                     g_btnUp.evShort   || g_btnUp.evLong   ||
                     g_btnDown.evShort || g_btnDown.evLong;
  if (anyBtnEvent || g_diving) {
    g_lastInteractMs = now;
    if (g_screenOff) {
      g_screenOff = false;
      // tft.enableDisplay (no-op v4)
      setCpuFrequencyMhz(CPU_FREQ_ACTIVE);  // 唤醒回 240MHz
      Serial.println("[POWER] CPU 240MHz / OLED on");
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
    // tft.sleepMode (no-op v4)
    setCpuFrequencyMhz(CPU_FREQ_IDLE);     // 降 80MHz 省电 ~30%
    Serial.println("[POWER] CPU 80MHz / OLED off");
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
      } else if (g_page == PAGE_PLAN) {
        // 计划深度 + 1 (限 1-50m)
        if (g_planDepthM < 50) g_planDepthM++;
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
      } else if (g_page == PAGE_PLAN) {
        // 计划深度 - 1 (限 1-50m)
        if (g_planDepthM > 1) g_planDepthM--;
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

    // Record battery sample (rate-limited to 1/hour internally)
    recordBatHistSample(now);

    // Air consumption (only while actively diving)
    if (g_diving) {
      g_currentSacL = g_sacLmin * (1.0f + g_depthSmooth / 10.0f);
      float consumed = g_currentSacL * (dtMs / 60000.0f);
      g_remainingAirL -= consumed;
      if (g_remainingAirL < 0) g_remainingAirL = 0;
    }

    // 平均深度累计 (潜水时)
    if (g_diving) {
      g_diveDepthSum += (double)g_depthSmooth * (double)dtMs;
      g_diveTimeMs   += dtMs;
    }

    // Dive state machine
    if (!g_diving && g_depthSmooth > DIVE_START_DEPTH) {
      g_diving = true;
      g_diveStartMs = now;
      g_maxDepth = g_depthSmooth;
      g_diveMinTemp = g_temp;
      g_ss = SS_IDLE;
      g_ssAccumMs = 0;
      g_ssArmCondition = false;
      g_remainingAirL = (float)totalAirL();   // 满瓶气量
      g_currentSacL = 0;
      g_diveDepthSum = 0;       // 平均深度累计重置
      g_diveTimeMs = 0;
      Serial.println("[DIVE] Started");
    } else if (g_diving && g_depthSmooth < SURFACE_DEPTH) {
      // End dive: persist
      uint32_t durSec = (now - g_diveStartMs) / 1000;
      if (durSec >= 30) {  // Ignore tiny accidental triggers
        time_t nowEpoch;
        time(&nowEpoch);
        uint16_t total_L = totalAirL();
        float avgDep = (g_diveTimeMs > 0) ? (float)(g_diveDepthSum / (double)g_diveTimeMs) : 0.0f;
        DiveRecord rec = {
          g_maxDepth,
          (uint16_t)min((uint32_t)0xFFFF, durSec),
          g_diveMinTemp,
          (uint8_t)(g_fluidDensity > 1010 ? 1 : 0),
          0,
          (uint32_t)nowEpoch,
          g_batPresent ? g_batVoltage : 0.0f,
          (total_L > g_remainingAirL) ? (total_L - g_remainingAirL) : 0.0f,
          avgDep
        };
        saveDiveToNVS(rec);
        Serial.printf("[DIVE] Saved: max=%.1fm dur=%us minT=%.1fC end=%lu bat=%.2fV air=%.0fL\n",
                      rec.maxDepth, rec.durationSec, rec.minTemp,
                      (unsigned long)rec.endEpoch, rec.batVoltage, rec.airUsedL);
      } else {
        Serial.println("[DIVE] Ended (too short, not saved)");
      }
      g_diving = false;
    }

    updateSafetyStop(g_depthSmooth, now);
  }

  uint32_t diveSec = g_diving ? (now - g_diveStartMs) / 1000 : 0;
  float ndl = computeNDL(g_depthSmooth);

  // ---- Render (限频 200ms, 减少闪烁) ----
  static uint32_t lastRender = 0;
  static uint8_t  prevPage   = 255;
  static bool     prevEdit   = false;
  static bool     prevSettings = false;

  bool pageChanged = (g_page != prevPage) || (g_editingTime != prevEdit) || (g_inSettings != prevSettings);
  // HUD/编辑/设置 = 频繁更新; 其他静态页 = 仅切页 + 5s 慢刷
  uint32_t renderInterval = (g_page == PAGE_HUD || g_editingTime || g_inSettings) ? 300 : 5000;
  bool timeToRender = (now - lastRender > renderInterval);

  if (!g_screenOff && (pageChanged || timeToRender)) {
    lastRender = now;
    if (pageChanged) {
      tft.fillScreen(C(RGB_ORANGE));
      s_lastPage = 255;
      g_pageDirty = true;
      prevPage = g_page;
      prevEdit = g_editingTime;
      prevSettings = g_inSettings;
    }

    if (g_editingTime) {
      drawTimeEdit();
    } else if (g_inSettings) {
      drawSettingsAndSend();
    } else {
      switch (g_page) {
        case PAGE_HUD:      drawHud(g_depthSmooth, g_maxDepth, g_temp, ndl, g_ascentMpm, diveSec); break;
        case PAGE_TISSUE:   drawTissue(g_depthSmooth); break;
        case PAGE_N2:       drawN2(); break;
        case PAGE_PLAN:     drawPlan(); break;
        case PAGE_AIR:      drawAir(); break;
        case PAGE_TEMP:     drawTempChart(); break;
        case PAGE_BAT:      drawBatHist(); break;
        case PAGE_LASTDIVE: drawLastDive(); break;
        case PAGE_LOG:      drawLogList();  break;
        case PAGE_STATS:    drawStats();    break;
      }
    }
    g_pageDirty = false;
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
      // 上升过快 (1 长鸣)
      beepBlocking(1, 250); g_lastBeepMs = now;
    } else if (-g_ascentMpm > g_descentLimit && g_depthSmooth > 3.0f && g_diving) {
      // 下降过快 (2 短鸣 - 与上升的 1 长鸣区分)
      beepBlocking(2, 80); g_lastBeepMs = now;
    } else if (g_ss == SS_DONE && g_lastBeepMs == 0) {
      beepBlocking(2, 100); g_lastBeepMs = now;
    } else if (ndl <= 0.0f && g_diving) {
      beepBlocking(4, 80); g_lastBeepMs = now;
    } else if (g_diving && g_remainingAirL > 0 && barFromAirL(g_remainingAirL) < 50.0f) {
      // 剩余压力 < 50 bar (相当于 1/5 - 1/4 满瓶) 报警
      beepBlocking(5, 60); g_lastBeepMs = now;
    } else if (g_diving && g_temp < (float)g_lowTempC) {
      // 低温报警 (短)
      beepBlocking(2, 50); g_lastBeepMs = now;
    }
  }

  // ---- Telemetry ----
  Serial.printf("D=%5.2fm Mx=%5.2fm T=%4.1fC P=%6.1f Up=%+5.1f NDL=%4.1f SS=%d Pg=%u Dive=%c Bat=%.2fV/%u%% BTN=%d%d%d\n",
                g_depthSmooth, g_maxDepth, g_temp, g_pressureRaw, g_ascentMpm,
                ndl, g_ss, g_page, g_diving ? 'Y' : 'N',
                g_batVoltage, g_batPct,
                digitalRead(BTN_MODE_PIN), digitalRead(BTN_UP_PIN), digitalRead(BTN_DOWN_PIN));

  // ---- Light Sleep: 屏保期间深度省电 (~5-10mA, 比 80MHz 待机更省) ----
  // 仅在: 屏保激活 + 非潜水 + 非编辑/设置 模式下进入
  if (g_screenOff && !g_diving && !g_editingTime && !g_inSettings) {
    Serial.flush();
    // 唤醒源: 任一按键 (GPIO LOW) 或 5 秒 timer (定期醒来更新电池)
    esp_sleep_enable_ext1_wakeup(
      (1ULL << BTN_MODE_PIN) | (1ULL << BTN_UP_PIN) | (1ULL << BTN_DOWN_PIN),
      ESP_EXT1_WAKEUP_ANY_LOW
    );
    esp_sleep_enable_timer_wakeup(5ULL * 1000 * 1000);
    esp_light_sleep_start();
    // 醒来后 loop 自然继续到下一帧
    // 按键醒 -> 下次 loop 顶部 anyBtnEvent 触发屏幕唤醒
    // Timer 醒 -> 更新一次电池/MS5837, 再次睡
  }
}
