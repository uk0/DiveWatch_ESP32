// =====================================================================
//  DiveWatch v1.0  -  ESP32-S3 + MS5837-30BA + OLED SH1106 + Buzzer
//  Author: DIY
//
//  Board:  ESP32S3 Dev Module (Arduino-ESP32 core >= 2.0.14)
//  Libs:
//    - U8g2          (Library Manager: search "U8g2" by oliver)
//    - MS5837        (https://github.com/bluerobotics/BlueRobotics_MS5837_Library)
//                     Sketch -> Include Library -> Add .ZIP Library...
//
//  Wiring (ESP32-S3 R8):
//    SDA  = GPIO 8  -> MS5837 SDA + OLED SDA
//    SCL  = GPIO 9  -> MS5837 SCL + OLED SCL
//    3V3                -> MS5837 VCC + OLED VCC
//    GND                -> MS5837 GND + OLED GND
//    GPIO 5 -> 1k -> S8050 base
//    S8050 collector -> buzzer(-);  buzzer(+) -> 3V3
//    S8050 emitter   -> GND
//
//  Upload:
//    Board     : "ESP32S3 Dev Module"
//    USB CDC On Boot : Enabled  (so Serial works over USB)
//    Flash Size: 8MB
//    PSRAM     : OPI PSRAM
//    Upload Speed: 921600
// =====================================================================

#include <Wire.h>
#include <U8g2lib.h>
#include "MS5837.h"

// -------- Pin map --------
#define I2C_SDA     8
#define I2C_SCL     9
#define BUZZER_PIN  5

// -------- Tunables --------
static const float ALARM_DEPTH        = 30.0f;   // m
static const float ASCENT_LIMIT_MPM   = 9.0f;    // m/min  (recreational safe rate ~9 m/min)
static const float FLUID_DENSITY      = 1029.0f; // 1029 seawater, 997 freshwater
static const uint32_t SAMPLE_MS       = 200;
static const uint32_t REARM_BEEP_MS   = 1500;

// -------- Globals --------
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
MS5837 sensor;

float    g_surfacePressure = 1013.25f;  // mbar, replaced by calibration in setup()
float    g_lastDepth       = 0.0f;
float    g_maxDepth        = 0.0f;
uint32_t g_lastSampleMs    = 0;
uint32_t g_lastBeepMs      = 0;
uint32_t g_diveStartMs     = 0;
bool     g_diving          = false;
bool     g_sensorOk        = false;

// -------- Helpers --------
void beepBlocking(int times, int onMs, int offMs = 80) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < times - 1) delay(offMs);
  }
}

void drawError(const char *line1, const char *line2 = nullptr) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(0, 16, "ERROR");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 36, line1);
  if (line2) u8g2.drawStr(0, 50, line2);
  u8g2.sendBuffer();
}

void drawSplash(float surfaceMbar) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB12_tr);
  u8g2.drawStr(0, 18, "DiveWatch");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 34, "v1.0  MS5837-30BA");
  char buf[32];
  snprintf(buf, sizeof(buf), "Surface: %.1f mbar", surfaceMbar);
  u8g2.drawStr(0, 48, buf);
  snprintf(buf, sizeof(buf), "Density: %.0f kg/m^3", FLUID_DENSITY);
  u8g2.drawStr(0, 62, buf);
  u8g2.sendBuffer();
}

void drawHud(float depth, float maxDepth, float temp, float ascentMpm, uint32_t diveSec) {
  u8g2.clearBuffer();

  // Title bar
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 8, "DEPTH (m)");
  // Top-right: dive timer
  char tbuf[16];
  snprintf(tbuf, sizeof(tbuf), "%02lu:%02lu", (unsigned long)(diveSec / 60), (unsigned long)(diveSec % 60));
  int tw = u8g2.getStrWidth(tbuf);
  u8g2.drawStr(128 - tw, 8, tbuf);

  // Big depth
  char dbuf[16];
  if (depth < 100.0f) snprintf(dbuf, sizeof(dbuf), "%4.1f", depth);
  else                snprintf(dbuf, sizeof(dbuf), "%4.0f", depth);
  u8g2.setFont(u8g2_font_logisoso28_tn);
  int dw = u8g2.getStrWidth(dbuf);
  u8g2.drawStr((128 - dw) / 2, 38, dbuf);

  // Bottom row: max + temp + ascent
  u8g2.setFont(u8g2_font_6x10_tr);
  char bbuf[24];
  snprintf(bbuf, sizeof(bbuf), "Max %.1fm", maxDepth);
  u8g2.drawStr(0, 52, bbuf);
  snprintf(bbuf, sizeof(bbuf), "%.1f C", temp);
  u8g2.drawStr(85, 52, bbuf);

  snprintf(bbuf, sizeof(bbuf), "Ascent %+5.1f m/min", ascentMpm);
  u8g2.drawStr(0, 62, bbuf);

  // Alarm icons on the right edge
  if (depth > ALARM_DEPTH)            u8g2.drawStr(120, 62, "!");
  if (ascentMpm > ASCENT_LIMIT_MPM)   u8g2.drawStr(112, 62, "^");

  u8g2.sendBuffer();
}

float depthFromPressure(float pressureMbar) {
  // depth (m) = (P - P0) * 100 / (rho * g);  P in mbar, *100 -> Pa
  float d = (pressureMbar - g_surfacePressure) * 100.0f / (FLUID_DENSITY * 9.80665f);
  return d < 0 ? 0 : d;
}

// -------- Setup --------
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!u8g2.begin()) {
    // OLED init can't really fail in U8g2 hardware path, but guard anyway.
    Serial.println("[OLED] begin() failed");
  }
  u8g2.enableUTF8Print();

  drawSplash(0);

  // ---- MS5837 init with retry ----
  uint8_t tries = 0;
  while (!sensor.init()) {
    Serial.println("[MS5837] init failed - check SDA/SCL/3V3/GND, addr 0x76");
    drawError("MS5837 init fail", "Check wiring/3V3");
    tries++;
    if (tries >= 5) {
      Serial.println("[MS5837] giving up; HUD will run with zeros");
      g_sensorOk = false;
      break;
    }
    delay(1000);
  }
  if (tries < 5) {
    sensor.setModel(MS5837::MS5837_30BA);
    sensor.setFluidDensity(FLUID_DENSITY);
    g_sensorOk = true;
  }

  // ---- Surface pressure calibration (10 samples averaged) ----
  if (g_sensorOk) {
    float sum = 0;
    const int N = 10;
    for (int i = 0; i < N; i++) {
      sensor.read();
      sum += sensor.pressure();
      delay(60);
    }
    g_surfacePressure = sum / N;
    Serial.printf("[CAL] Surface pressure = %.2f mbar\n", g_surfacePressure);
  }

  drawSplash(g_surfacePressure);
  beepBlocking(2, 80);
  delay(800);
}

// -------- Loop --------
void loop() {
  uint32_t now = millis();
  if (now - g_lastSampleMs < SAMPLE_MS) return;
  uint32_t dtMs = now - g_lastSampleMs;
  g_lastSampleMs = now;

  float depth = 0, temp = 0, ascentMpm = 0, pressMbar = 0;

  if (g_sensorOk) {
    sensor.read();
    pressMbar = sensor.pressure();
    temp      = sensor.temperature();
    depth     = depthFromPressure(pressMbar);

    // Ascent rate (positive = ascending)
    ascentMpm = (g_lastDepth - depth) * 60000.0f / (float)dtMs;
    g_lastDepth = depth;

    if (depth > g_maxDepth) g_maxDepth = depth;

    // Dive state machine: deeper than 1.2m -> diving; resurface -> stop timer
    if (!g_diving && depth > 1.2f) {
      g_diving = true;
      g_diveStartMs = now;
      Serial.println("[DIVE] Started");
    } else if (g_diving && depth < 0.5f) {
      g_diving = false;
      Serial.printf("[DIVE] Ended. Max=%.1fm  Duration=%lus\n",
                    g_maxDepth, (unsigned long)((now - g_diveStartMs) / 1000));
    }
  }

  uint32_t diveSec = g_diving ? (now - g_diveStartMs) / 1000 : 0;
  drawHud(depth, g_maxDepth, temp, ascentMpm, diveSec);

  // ---- Alarms ----
  if (g_sensorOk && now - g_lastBeepMs > REARM_BEEP_MS) {
    if (depth > ALARM_DEPTH) {
      beepBlocking(3, 70);
      g_lastBeepMs = now;
    } else if (ascentMpm > ASCENT_LIMIT_MPM && depth > 3.0f) {
      beepBlocking(1, 250);
      g_lastBeepMs = now;
    }
  }

  // ---- Telemetry ----
  Serial.printf("D=%6.2fm  Max=%5.2fm  T=%5.1fC  P=%7.1fmbar  Up=%+5.1fm/min  Dive=%s\n",
                depth, g_maxDepth, temp, pressMbar, ascentMpm,
                g_diving ? "Y" : "N");
}
