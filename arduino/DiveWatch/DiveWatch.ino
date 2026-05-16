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
#include <esp_sleep.h>
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

// 平均深度统计 (本次潜水)
double   g_diveDepthSum  = 0.0;     // 累积 (depth * dt_ms)
uint32_t g_diveTimeMs    = 0;       // 累积时间 ms

// NDL 计划器
uint8_t  g_planDepthM    = 18;      // 用户在 PAGE_PLAN 设的计划深度

// Power saving: dim OLED after idle period (g_screenOffMs is configurable)
uint32_t g_lastInteractMs  = 0;
bool     g_screenOff       = false;

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
#define FONT_CN     u8g2_font_wqy12_t_gb2312
#define FONT_BIG    u8g2_font_logisoso28_tn
#define FONT_MID    u8g2_font_logisoso24_tn

// 128x64 布局规范:
//   行 baseline y: 10, 22, 34, 46, 58 (12 像素间距, 不重叠)
//   大字 baseline: 36 (logisoso24, 字符占 12-36)
// HUD 4 段: 顶 y=10 / 大字 y=36 / 中 y=48 / 底 y=60

// 通用页眉分隔线 (标题下方 y=12, 横跨整屏)
inline void drawDivider() {
  u8g2.drawHLine(0, 12, 128);
}
void drawHud(float depth, float maxDepth, float temp, float ndl, float ascentMpm, uint32_t diveSec) {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);

  char buf[40];
  uint8_t hh, mm, ss;
  getCurrentTime(hh, mm, ss);

  // 顶部 y=10 (字符占 1-11)
  // 左: 时钟 (0-30)
  snprintf(buf, sizeof(buf), "%02u:%02u", hh, mm);
  u8g2.drawUTF8(0, 10, buf);

  // 海/淡 标识固定在最右 (118-128)
  u8g2.drawUTF8(118, 10, g_fluidDensity > 1010 ? "海" : "淡");

  if (g_diving) {
    // 潜水时: 中央显示潜水时长 + 右侧显示纯数字百分比 (无图标避免重叠)
    snprintf(buf, sizeof(buf), "潜%lu:%02lu", (unsigned long)(diveSec / 60), (unsigned long)(diveSec % 60));
    int w = u8g2.getUTF8Width(buf);
    u8g2.drawUTF8(36, 10, buf);  // 紧贴时钟右边
    if (g_batPresent) {
      snprintf(buf, sizeof(buf), "%d%%", g_batPct);
      int pw = u8g2.getUTF8Width(buf);
      u8g2.drawUTF8(116 - pw, 10, buf);  // 紧贴海/淡左
    }
  } else {
    // 水面时: 显示电池图标 + 百分比 (中央留空)
    if (g_batPresent) {
      drawBatteryIcon(72, 2);
      snprintf(buf, sizeof(buf), "%d%%", g_batPct);
      u8g2.drawUTF8(92, 10, buf);
    }
  }

  // 报警状态检测 (用于大字反色显示)
  bool depthAlarm = depth > g_alarmDepth;
  bool decoAlarm  = (ndl <= 0.0f) && g_diving;
  bool alarmRev   = depthAlarm || decoAlarm;

  // 大字深度 y=36 (logisoso24, 字符占 12-36)
  if (depth < 100.0f) snprintf(buf, sizeof(buf), "%4.1f", depth);
  else                snprintf(buf, sizeof(buf), "%4.0f", depth);
  u8g2.setFont(u8g2_font_logisoso24_tn);
  int dw = u8g2.getStrWidth(buf);
  int dx = (128 - dw - 14) / 2;
  if (dx < 0) dx = 0;

  // 警告时大字区域反色 (黑底白字)
  if (alarmRev) {
    u8g2.drawBox(0, 13, 128, 25);          // 实心黑色块覆盖大字区
    u8g2.setDrawColor(0);                  // 之后画的内容反色 (变白)
  }
  u8g2.drawStr(dx, 36, buf);
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(dx + dw + 2, 34, "米");
  if (alarmRev) u8g2.setDrawColor(1);     // 恢复正常颜色

  // 中部 y=48: NDL / 状态 (左) + 最深 (右)
  // 安全停留/减压等关键状态时让出整行, 不挤"最深"
  bool ssActive = (g_ss == SS_RUNNING || g_ss == SS_ARMED);
  bool decoNow  = (ndl <= 0.0f) && g_diving;

  if (ssActive) {
    uint32_t remain = (g_ssAccumMs >= g_safetyStopSec * 1000UL) ? 0 : (g_safetyStopSec * 1000UL - g_ssAccumMs);
    snprintf(buf, sizeof(buf), "安全停留 %lu 秒", (unsigned long)(remain / 1000));
    u8g2.drawUTF8(0, 48, buf);
  } else if (g_ss == SS_DONE) {
    u8g2.drawUTF8(0, 48, "安全停留完成");
  } else if (decoNow) {
    u8g2.drawUTF8(0, 48, "需要减压!");
  } else if (ndl >= 99.0f) {
    u8g2.drawUTF8(0, 48, "NDL >99 分");
  } else {
    snprintf(buf, sizeof(buf), "NDL %2.0f 分", ndl);
    u8g2.drawUTF8(0, 48, buf);
  }

  // 中部右: N2 负荷% (潜水时) 或 最深 (非潜水时)
  // 关键状态 (安全停留/减压/SS完成) 时不显示右侧, 让左边状态字占整行
  if (!ssActive && g_ss != SS_DONE && !decoNow) {
    if (g_diving) {
      // 潜水中: 显示氮气负荷, 更关键
      float n2Pct = computeTissueLoadPct(depth);
      snprintf(buf, sizeof(buf), "N2 %.0f%%", n2Pct);
    } else {
      snprintf(buf, sizeof(buf), "最深%.1f", maxDepth);
    }
    int w = u8g2.getUTF8Width(buf);
    u8g2.drawUTF8(126 - w, 48, buf);
  }

  // 底部 y=60 (字符占 48-60, 与中部 36-48 间距 0): 温度 + 上升速率
  snprintf(buf, sizeof(buf), "%.1f度", temp);
  u8g2.drawUTF8(0, 60, buf);

  snprintf(buf, sizeof(buf), "%+.1f米/分", ascentMpm);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(126 - w, 60, buf);

  // 警告标志 (右边缘 x=118 留 4px 边距)
  if (depth > g_alarmDepth)            u8g2.drawUTF8(118, 22, "!");
  if (ascentMpm > g_ascentLimit)       u8g2.drawUTF8(118, 34, "^");  // 上升过快
  if (-ascentMpm > g_descentLimit && g_diving)
                                        u8g2.drawUTF8(118, 60, "v");  // 下降过快

  u8g2.sendBuffer();
}

void drawTissue(float depth) {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  // y=10 标题 + 最大百分比
  u8g2.drawUTF8(0, 10, "组织负荷");
  drawDivider();

  float pct = computeTissueLoadPct(depth);
  char buf[32];
  snprintf(buf, sizeof(buf), "最大%3.0f%%", pct);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(126 - w, 10, buf);

  // 16 房间柱状图 y=14-46 (高 32)
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

  // y=58 底部: NDL
  float ndl = computeNDL(depth);
  if (ndl >= 99.0f)      snprintf(buf, sizeof(buf), "NDL >99 分");
  else if (ndl <= 0.0f)  snprintf(buf, sizeof(buf), "需要减压");
  else                   snprintf(buf, sizeof(buf), "NDL %.0f 分钟", ndl);
  u8g2.drawUTF8(0, 58, buf);
  u8g2.sendBuffer();
}

void drawLastDive() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  // y=10 标题 + 累计
  u8g2.drawUTF8(0, 10, "上次潜水");
  drawDivider();
  char buf[40];
  snprintf(buf, sizeof(buf), "累计%u次", g_diveTotal);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(126 - w, 10, buf);

  if (g_logCount == 0) {
    u8g2.drawUTF8(0, 36, "无潜水记录");
  } else {
    DiveRecord &r = g_log[0];
    // y=22/34/46/58 (4 行, 12 间距, 不重叠)
    snprintf(buf, sizeof(buf), "最深  %.1f 米", r.maxDepth);
    u8g2.drawUTF8(0, 22, buf);
    if (r.avgDepth > 0.1f) {
      snprintf(buf, sizeof(buf), "时长 %u:%02u 均%.1fm", r.durationSec/60, r.durationSec%60, r.avgDepth);
    } else {
      snprintf(buf, sizeof(buf), "时长  %u:%02u", r.durationSec/60, r.durationSec%60);
    }
    u8g2.drawUTF8(0, 34, buf);
    snprintf(buf, sizeof(buf), "温度  %.1f 度",  r.minTemp);
    u8g2.drawUTF8(0, 46, buf);

    // y=58 底部: 海/淡 + 距今时间
    if (r.endEpoch > 0) {
      time_t nowEpoch;
      time(&nowEpoch);
      int32_t intervalSec = (int32_t)((uint32_t)nowEpoch - r.endEpoch);
      if (intervalSec >= 0 && intervalSec < 99 * 3600) {
        snprintf(buf, sizeof(buf), "%s 距今%ldh%02ldm",
                 r.saltwater ? "海水" : "淡水",
                 (long)(intervalSec / 3600), (long)((intervalSec / 60) % 60));
      } else {
        snprintf(buf, sizeof(buf), "%s", r.saltwater ? "海水" : "淡水");
      }
    } else {
      snprintf(buf, sizeof(buf), "%s", r.saltwater ? "海水" : "淡水");
    }
    u8g2.drawUTF8(0, 58, buf);
  }
  u8g2.sendBuffer();
}

void drawLogList() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  // y=10 标题 + N/M
  u8g2.drawUTF8(0, 10, "潜水日志");
  drawDivider();
  char buf[40];
  snprintf(buf, sizeof(buf), "%u/%u", g_logViewIdx + 1, g_logCount);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(126 - w, 10, buf);

  if (g_logCount == 0) {
    u8g2.drawUTF8(0, 36, "日志为空");
  } else {
    if (g_logViewIdx >= g_logCount) g_logViewIdx = g_logCount - 1;
    DiveRecord &r = g_log[g_logViewIdx];
    // y=22/34/46/58 (4 行, 12 间距)
    snprintf(buf, sizeof(buf), "#%u 最深%.1f米", g_diveTotal - g_logViewIdx, r.maxDepth);
    u8g2.drawUTF8(0, 22, buf);
    snprintf(buf, sizeof(buf), "时长 %u:%02u  %s", r.durationSec/60, r.durationSec%60, r.saltwater ? "海" : "淡");
    u8g2.drawUTF8(0, 34, buf);
    if (r.batVoltage > 0.1f) {
      snprintf(buf, sizeof(buf), "温%.1f度 电%.2fV", r.minTemp, r.batVoltage);
    } else {
      snprintf(buf, sizeof(buf), "温度 %.1f 度", r.minTemp);
    }
    u8g2.drawUTF8(0, 46, buf);
    if (r.airUsedL > 1.0f) {
      snprintf(buf, sizeof(buf), "耗气 %.0fL", r.airUsedL);
    } else {
      snprintf(buf, sizeof(buf), "上下键浏览");
    }
    u8g2.drawUTF8(0, 58, buf);
  }
  u8g2.sendBuffer();
}

void drawAir() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);

  char buf[40];
  uint16_t total_L = totalAirL();   // 当前满瓶总气量 L
  // y=10 标题 + 当前消耗速率 (bar/min)
  u8g2.drawUTF8(0, 10, "气量监控");
  drawDivider();
  if (g_diving && g_currentSacL > 0.1f) {
    float currentBarMin = (g_tankVolL > 0) ? (g_currentSacL / g_tankVolL) : 0;
    snprintf(buf, sizeof(buf), "%.1fbar/分", currentBarMin);
    int w = u8g2.getUTF8Width(buf);
    u8g2.drawUTF8(126 - w, 10, buf);
  } else {
    int w = u8g2.getUTF8Width("水面");
    u8g2.drawUTF8(126 - w, 10, "水面");
  }

  // y=22 剩余压力 + 百分比 (主要数据)
  float remainingBar = barFromAirL(g_remainingAirL);
  uint8_t pct = (total_L > 0) ? (uint8_t)((g_remainingAirL / total_L) * 100.0f) : 0;
  snprintf(buf, sizeof(buf), "剩压 %3d bar  %3d%%", (int)remainingBar, pct);
  u8g2.drawUTF8(0, 22, buf);

  // y=34 可潜时间 (Air Time)
  if (g_diving && g_currentSacL > 0.1f) {
    int airMin = (int)(g_remainingAirL / g_currentSacL);
    if (airMin > 999) airMin = 999;
    snprintf(buf, sizeof(buf), "Air Time  %d 分钟", airMin);
  } else {
    snprintf(buf, sizeof(buf), "Air Time  --");
  }
  u8g2.drawUTF8(0, 34, buf);

  // y=46 瓶配置
  snprintf(buf, sizeof(buf), "瓶 %dbar*%dL  SAC%d", g_initialBar, g_tankVolL, g_sacLmin);
  u8g2.drawUTF8(0, 46, buf);

  // y=52-60 压力条 (按总压力 0-g_initialBar)
  int barY = 54;
  int barH = 8;
  u8g2.drawFrame(0, barY, 128, barH);
  int fillW = (g_initialBar > 0)
      ? (int)((126.0f) * (remainingBar / g_initialBar))
      : 0;
  if (fillW < 0) fillW = 0;
  if (fillW > 126) fillW = 126;
  if (fillW > 0) u8g2.drawBox(1, barY + 1, fillW, barH - 2);

  u8g2.sendBuffer();
}

// 体内氮气安全页 - 基于 ZHL-16C 衍生指标
void drawN2() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 10, "体内氮气 ZHL-16");
  drawDivider();

  char buf[40];
  // 1. 氮气负荷 % + 风险等级 (左/右)
  float loadPct = computeTissueLoadPct(g_depthSmooth);
  const char* risk;
  if (loadPct < 60.0f)      risk = "低";
  else if (loadPct < 80.0f) risk = "中";
  else                       risk = "高";

  snprintf(buf, sizeof(buf), "负荷 %.0f%%", loadPct);
  u8g2.drawUTF8(0, 24, buf);
  snprintf(buf, sizeof(buf), "风险 %s", risk);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(126 - w, 24, buf);

  // 2. 最快(4min) + 最慢(635min) 组织室饱和度
  float fastSat = computeSaturationPct(0,  g_depthSmooth);
  float slowSat = computeSaturationPct(15, g_depthSmooth);
  snprintf(buf, sizeof(buf), "快 %.0f%%  慢 %.0f%%", fastSat, slowSat);
  u8g2.drawUTF8(0, 36, buf);

  // 3. 禁飞时间 (NoFly time)
  float noFlyH = computeNoFlyTimeHours();
  if (noFlyH < 0.05f) {
    snprintf(buf, sizeof(buf), "禁飞   可飞行");
  } else if (noFlyH < 100.0f) {
    int h = (int)noFlyH;
    int m = (int)((noFlyH - h) * 60);
    snprintf(buf, sizeof(buf), "禁飞   %dh%02dm", h, m);
  } else {
    snprintf(buf, sizeof(buf), "禁飞   >100h");
  }
  u8g2.drawUTF8(0, 48, buf);

  // 4. 完全脱饱和时间
  float desatH = computeFullDesaturationHours();
  if (desatH < 0.05f) {
    snprintf(buf, sizeof(buf), "脱饱和 已完成");
  } else if (desatH < 100.0f) {
    snprintf(buf, sizeof(buf), "脱饱和 %.0fh", desatH);
  } else {
    snprintf(buf, sizeof(buf), "脱饱和 >100h");
  }
  u8g2.drawUTF8(0, 60, buf);

  u8g2.sendBuffer();
}

void drawPlan() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  // y=10 标题 + 当前深度
  u8g2.drawUTF8(0, 10, "潜水规划");
  drawDivider();
  char buf[40];
  snprintf(buf, sizeof(buf), "当前 %.1fm", g_depthSmooth);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(126 - w, 10, buf);

  // y=22 计划深度
  snprintf(buf, sizeof(buf), "计划深度 %u 米", g_planDepthM);
  u8g2.drawUTF8(0, 22, buf);

  // y=34 该深度的 NDL
  float ndl = computeNDL((float)g_planDepthM);
  if (ndl >= 99.0f)      snprintf(buf, sizeof(buf), "NDL %d 米 >99 分", g_planDepthM);
  else if (ndl <= 0.0f)  snprintf(buf, sizeof(buf), "NDL %d 米 需减压!", g_planDepthM);
  else                   snprintf(buf, sizeof(buf), "NDL %d 米 %.0f 分", g_planDepthM, ndl);
  u8g2.drawUTF8(0, 34, buf);

  // y=46 该深度的 Air Time (满瓶估算)
  uint16_t total_L = totalAirL();
  float sacAtDepth = g_sacLmin * (1.0f + g_planDepthM / 10.0f);
  if (sacAtDepth > 0) {
    int airMin = (int)(total_L / sacAtDepth);
    if (airMin > 999) airMin = 999;
    snprintf(buf, sizeof(buf), "满瓶气量 %d 分钟", airMin);
  } else {
    snprintf(buf, sizeof(buf), "满瓶气量 --");
  }
  u8g2.drawUTF8(0, 46, buf);

  // y=58 操作提示
  u8g2.drawUTF8(0, 58, "上下键改深度");
  u8g2.sendBuffer();
}

void drawTempChart() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);

  char buf[24];
  // y=10 标题 + 当前温度
  snprintf(buf, sizeof(buf), "温度 %.1f度", g_temp);
  u8g2.drawUTF8(0, 10, buf);
  drawDivider();

  // y=10 右上: 采样进度
  snprintf(buf, sizeof(buf), "%u/60", g_tempHistCount);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(126 - w, 10, buf);

  if (g_tempHistCount < 2) {
    u8g2.drawUTF8(0, 34, "数据收集中...");
    u8g2.drawUTF8(0, 46, "每分钟一个样本");
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

  // Y 轴标签 (左侧, 紧凑)
  snprintf(buf, sizeof(buf), "%4.1f", maxT);
  u8g2.drawUTF8(0, 22, buf);
  snprintf(buf, sizeof(buf), "%4.1f", minT);
  u8g2.drawUTF8(0, 58, buf);

  // 绘图区: x=28-126, y=16-58 (宽 98, 高 42)
  int x0 = 28, y0 = 16, gw = 98, gh = 42;
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
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  // y=10 标题 + 当前电压/百分比
  char buf[40];
  u8g2.drawUTF8(0, 10, "电池趋势");
  drawDivider();
  if (g_batPresent) {
    snprintf(buf, sizeof(buf), "%.2fV %u%%", g_batVoltage, g_batPct);
  } else {
    snprintf(buf, sizeof(buf), "未接入");
  }
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(126 - w, 10, buf);

  // 中部说明
  if (g_batHistCount < 2) {
    snprintf(buf, sizeof(buf), "已采 %u/24 小时", g_batHistCount);
    u8g2.drawUTF8(0, 34, buf);
    u8g2.drawUTF8(0, 46, "每小时采样一次");
    u8g2.sendBuffer();
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
  u8g2.drawUTF8(0, 22, buf);
  snprintf(buf, sizeof(buf), "%.2f", minV);
  u8g2.drawUTF8(0, 58, buf);

  // 绘图区 x=28-126 y=16-58
  int x0 = 28, y0 = 16, gw = 98, gh = 42;
  u8g2.drawFrame(x0, y0, gw, gh);

  // 折线
  for (int i = 0; i < g_batHistCount - 1; i++) {
    int idx1 = (g_batHistIdx + BAT_HIST_LEN - g_batHistCount + i) % BAT_HIST_LEN;
    int idx2 = (g_batHistIdx + BAT_HIST_LEN - g_batHistCount + i + 1) % BAT_HIST_LEN;
    int x1 = x0 + 1 + i * (gw - 2) / (g_batHistCount - 1);
    int x2 = x0 + 1 + (i + 1) * (gw - 2) / (g_batHistCount - 1);
    int y1 = y0 + gh - 2 - (int)((g_batHist[idx1] - minV) * (gh - 4) / (maxV - minV));
    int y2 = y0 + gh - 2 - (int)((g_batHist[idx2] - minV) * (gh - 4) / (maxV - minV));
    u8g2.drawLine(x1, y1, x2, y2);
  }

  u8g2.sendBuffer();
}

void drawStats() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  // y=10 标题 + 电池
  u8g2.drawUTF8(0, 10, "总览统计");
  drawDivider();

  char buf[40];
  // 标题右: 海拔 (海平面附近不显示)
  float alt = estimateAltitudeM();
  if (alt > 50.0f) snprintf(buf, sizeof(buf), "海拔%.0fm", alt);
  else if (g_batPresent) snprintf(buf, sizeof(buf), "%.2fV %u%%", g_batVoltage, g_batPct);
  else snprintf(buf, sizeof(buf), "未接电池");
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(126 - w, 10, buf);

  // y=22/34/46/58 (4 行)
  snprintf(buf, sizeof(buf), "潜水次数  %u 次", g_diveTotal);
  u8g2.drawUTF8(0, 22, buf);

  snprintf(buf, sizeof(buf), "历史最深  %.1f 米", g_diveTotalMaxD);
  u8g2.drawUTF8(0, 34, buf);

  uint32_t totMin = g_lifetimeUnderwaterSec / 60;
  uint32_t totH   = totMin / 60;
  snprintf(buf, sizeof(buf), "累计水下  %luh%02lum", (unsigned long)totH, (unsigned long)(totMin % 60));
  u8g2.drawUTF8(0, 46, buf);

  // 距上次潜水时长 (水面间隔)
  if (g_logCount > 0 && g_log[0].endEpoch > 0) {
    time_t nowEpoch;
    time(&nowEpoch);
    int32_t intervalSec = (int32_t)((uint32_t)nowEpoch - g_log[0].endEpoch);
    if (intervalSec >= 0 && intervalSec < 99 * 3600) {
      snprintf(buf, sizeof(buf), "距上次潜水  %ldh%02ldm",
               (long)(intervalSec / 3600), (long)((intervalSec / 60) % 60));
      u8g2.drawUTF8(0, 58, buf);
    }
  }

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
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);

  // y=10 标题 + 当前项序号
  char buf[24];
  u8g2.drawUTF8(0, 10, "设置");
  snprintf(buf, sizeof(buf), "%u/%u", g_settingsItem + 1, SETTINGS_COUNT);
  int w = u8g2.getUTF8Width(buf);
  u8g2.drawUTF8(126 - w, 10, buf);
  drawDivider();

  // y=22/34/46/58 (4 项可见, 12px 间距, 不重叠)
  const int VISIBLE = 4;
  int first = 0;
  if (g_settingsItem >= VISIBLE) first = g_settingsItem - VISIBLE + 1;
  if (first > SETTINGS_COUNT - VISIBLE) first = SETTINGS_COUNT - VISIBLE;
  if (first < 0) first = 0;

  for (int i = 0; i < VISIBLE && first + i < SETTINGS_COUNT; i++) {
    uint8_t idx = first + i;
    int y = 22 + i * 12;
    if (idx == g_settingsItem) u8g2.drawUTF8(0, y, ">");
    u8g2.drawUTF8(10, y, settingsItemName(idx));
    char vbuf[20];
    settingsItemValue(idx, vbuf, sizeof(vbuf));
    int vw = u8g2.getUTF8Width(vbuf);
    u8g2.drawUTF8(126 - vw, y, vbuf);
  }

  // 滚动指示器: 右侧 ^ / v 表示上下还有更多项 (x=118 留 4px 边距)
  if (first > 0)                            u8g2.drawUTF8(118, 22, "^");
  if (first + VISIBLE < SETTINGS_COUNT)     u8g2.drawUTF8(118, 58, "v");
}

void drawSettingsAndSend() {
  drawSettings();
  u8g2.sendBuffer();
}

void drawTimeEdit() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  // y=10 标题 + MODE 提示
  u8g2.drawUTF8(0, 10, "设置时间");
  // 右对齐: "MODE OK" 7 字符 = 42px, x=82 -> 82-124 留 4px 边距
  u8g2.drawUTF8(82, 10, "MODE OK");
  drawDivider();

  // 大字时间 y=40 (logisoso24, 字符 16-40)
  char buf[8];
  u8g2.setFont(u8g2_font_logisoso24_tn);
  snprintf(buf, sizeof(buf), "%02u:%02u", g_editHH, g_editMM);
  int w = u8g2.getStrWidth(buf);
  int x = (128 - w) / 2;
  u8g2.drawStr(x, 40, buf);

  // 下划线指示当前编辑字段
  int colonOffset = u8g2.getStrWidth("00");
  int afterColon  = u8g2.getStrWidth("00:");
  int hhX = x;
  int mmX = x + afterColon;
  if (g_editField == 0) u8g2.drawHLine(hhX, 44, colonOffset);
  else                  u8g2.drawHLine(mmX, 44, colonOffset);

  // y=58 底部提示
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 58, "上下增减   MODE切换");
  u8g2.sendBuffer();
}

void drawSplash() {
  u8g2.clearBuffer();
  // y=22 大字标题 (logisoso24, 字符 0-22)
  u8g2.setFont(u8g2_font_logisoso24_tn);
  // logisoso 字体不支持中文, 标题用 wqy12 大些字代替
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 14, "潜水表 DiveWatch");
  // y=28/40/52 副标题 + 水面 + 密度
  char buf[40];
  u8g2.drawUTF8(0, 28, "v3.2  ZHL-16C");
  snprintf(buf, sizeof(buf), "水面: %.1f mbar", g_surfacePressure);
  u8g2.drawUTF8(0, 42, buf);
  snprintf(buf, sizeof(buf), "密度: %.0f kg/m3", g_fluidDensity);
  u8g2.drawUTF8(0, 54, buf);
  u8g2.sendBuffer();
}

void drawError(const char *l1, const char *l2 = nullptr) {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  // y=14 大字标题 + y=34 错误信息行
  u8g2.drawUTF8(0, 14, "*** 错误 ***");
  u8g2.drawUTF8(0, 34, l1);
  if (l2) u8g2.drawUTF8(0, 50, l2);
  u8g2.sendBuffer();
}

// 进入深度睡眠 (软关机): ~10μA 功耗
// 唤醒源 = MODE 按键 (BTN_MODE_PIN = GPIO 6, RTC IO, LOW 触发)
void enterDeepSleep() {
  Serial.println("[POWER] 准备关机, 进入深度睡眠...");
  saveClockToNVS();
  saveSettingsToNVS();

  // 显示"关机中"画面
  u8g2.setPowerSave(0);
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 16, "正在关机...");
  u8g2.drawUTF8(0, 36, "再次长按 MODE");
  u8g2.drawUTF8(0, 50, "2 秒以上即可开机");
  u8g2.sendBuffer();

  // 蜂鸣 3 长声做关机提示
  beepBlocking(3, 250, 150);
  delay(800);

  // 关 OLED + WiFi
  u8g2.setPowerSave(1);
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

// 开机自检: I2C 设备 + 按钮 + 蜂鸣器
void runSelfTest() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_CN);
  u8g2.drawUTF8(0, 12, "系统自检");
  u8g2.drawUTF8(0, 28, "正在检测...");
  u8g2.sendBuffer();

  // 检查 OLED I2C (能跑到这一步说明 OLED OK)
  bool oledOK = true;

  // 检查 MS5837 0x76
  Wire.beginTransmission(0x76);
  bool ms5837OK = (Wire.endTransmission() == 0);

  // 蜂鸣器响 1 声测试
  digitalWrite(BUZZER_PIN, HIGH);
  delay(60);
  digitalWrite(BUZZER_PIN, LOW);
  bool buzzerTested = true;

  // 按钮检查: 长按开机时按键仍可能按住; 抖动也可能瞬时 LOW
  // 策略: 提示松开 + 等待至少 500ms 连续 HIGH 才算 OK (容忍弹簧抖动)
  auto checkBtnStable = [](uint8_t pin, uint32_t timeoutMs) -> bool {
    unsigned long deadline = millis() + timeoutMs;
    unsigned long stableStart = millis();
    while (millis() < deadline) {
      if (digitalRead(pin) == LOW) {
        stableStart = millis();   // 检测到按下/抖动, 重置稳定计时
      } else if (millis() - stableStart >= 500) {
        return true;              // 连续 500ms HIGH = 真的释放
      }
      delay(20);
    }
    return false;
  };

  // 显示"请松开按键"提示, 给用户最多 4 秒释放时间
  u8g2.clearBuffer();
  u8g2.drawUTF8(0, 12, "系统自检");
  u8g2.drawUTF8(0, 28, "正在检测...");
  u8g2.drawUTF8(0, 50, "请松开按键");
  u8g2.sendBuffer();

  bool btnMOK = checkBtnStable(BTN_MODE_PIN, 4000);
  bool btnUOK = checkBtnStable(BTN_UP_PIN,   1000);
  bool btnDOK = checkBtnStable(BTN_DOWN_PIN, 1000);
  bool btnOK  = btnMOK && btnUOK && btnDOK;

  // 显示自检结果
  u8g2.clearBuffer();
  u8g2.drawUTF8(0, 12, "系统自检结果");
  char buf[32];
  snprintf(buf, sizeof(buf), "OLED       %s", oledOK ? "正常" : "异常");
  u8g2.drawUTF8(0, 26, buf);
  snprintf(buf, sizeof(buf), "MS5837     %s", ms5837OK ? "正常" : "异常");
  u8g2.drawUTF8(0, 38, buf);
  snprintf(buf, sizeof(buf), "键 M:%s U:%s D:%s",
           btnMOK ? "OK" : "X",
           btnUOK ? "OK" : "X",
           btnDOK ? "OK" : "X");
  u8g2.drawUTF8(0, 50, buf);
  snprintf(buf, sizeof(buf), "蜂鸣器     %s", buzzerTested ? "已测试" : "异常");
  u8g2.drawUTF8(0, 62, buf);
  u8g2.sendBuffer();
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
  Wire.setClock(800000);  // SH1106 标 400k, 实测多数模块能稳跑 800k, 帧时间 23→11ms

  u8g2.begin();
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
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, 46, 128, 18);
    u8g2.setDrawColor(1);
    u8g2.setFont(FONT_CN);
    u8g2.drawUTF8(0, 58, "WiFi 同步中...");
    u8g2.sendBuffer();
    bool ok = trySyncNTP();
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, 46, 128, 18);
    u8g2.setDrawColor(1);
    u8g2.setFont(FONT_CN);
    u8g2.drawUTF8(0, 58, ok ? "时间同步完成" : "WiFi 同步失败");
    u8g2.sendBuffer();
    delay(1200);
  }

  beepBlocking(2, 70);
  delay(400);

  // 显式回到主 HUD 页, 重置任何残留状态 (防止开机进错页)
  g_page = PAGE_HUD;
  g_editingTime = false;
  g_inSettings  = false;
  g_screenOff   = false;
  // 清空可能因 setup 期间按键抖动产生的事件
  g_btnMode.evShort = g_btnMode.evLong = false;
  g_btnUp.evShort   = g_btnUp.evLong   = false;
  g_btnDown.evShort = g_btnDown.evLong = false;

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
      u8g2.setPowerSave(0);
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
    u8g2.setPowerSave(1);
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
