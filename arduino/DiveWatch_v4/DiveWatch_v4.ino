// =====================================================================
// DiveWatch v4.0 MVP - ST7789 2.4" 横屏 320×240 橘黄背景
// 屏的颜色模式: invertDisplay(true) + swap RB
// =====================================================================

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <SPI.h>

#define TFT_CS    21
#define TFT_DC    18
#define TFT_RST   17
#define TFT_MOSI  38
#define TFT_SCK   48

// 原始 RGB565 颜色 (但屏需要 swap RB)
#define RGB_ORANGE  0xFD20    // 橘黄
#define RGB_BLACK   0x0000
#define RGB_WHITE   0xFFFF
#define RGB_RED     0xF800
#define RGB_GREEN   0x07E0
#define RGB_BLUE    0x001F
#define RGB_YELLOW  0xFFE0
#define RGB_CYAN    0x07FF
#define RGB_DARK    0x4208

// swap R/B (5/5 互换, G 6 位不变)
uint16_t swapRB(uint16_t c) {
  return ((c & 0x001F) << 11) | (c & 0x07E0) | ((c >> 11) & 0x001F);
}
#define C(x) swapRB(x)

SPIClass mySPI(HSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&mySPI, TFT_CS, TFT_DC, TFT_RST);
U8G2_FOR_ADAFRUIT_GFX u8g2;

void drawCN(int x, int y, const char *text, const uint8_t *font, uint16_t color) {
  u8g2.setFont(font);
  u8g2.setForegroundColor(color);
  u8g2.setCursor(x, y);
  u8g2.print(text);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[DiveWatch v4 HUD]");

  mySPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  tft.init(240, 320);
  tft.setRotation(1);               // 横屏 320 宽 × 240 高
  tft.invertDisplay(true);          // 关键! 这块屏需要反色
  tft.fillScreen(C(RGB_ORANGE));    // 橘黄背景 (swap)

  u8g2.begin(tft);
  u8g2.setFontMode(1);              // 透明背景
  u8g2.setFontDirection(0);

  // ===== 顶部状态栏 (y=0-30, 暗灰底白字) =====
  tft.fillRect(0, 0, 320, 30, C(RGB_DARK));
  drawCN(10,  22, "12:34",       u8g2_font_wqy16_t_gb2312, C(RGB_WHITE));
  drawCN(120, 22, "潜水中 5:23", u8g2_font_wqy16_t_gb2312, C(RGB_GREEN));
  drawCN(250, 22, "87% 海",      u8g2_font_wqy16_t_gb2312, C(RGB_WHITE));

  // ===== 大字深度区 (y=40-130) =====
  drawCN(10, 105, "深度", u8g2_font_wqy16_t_gb2312, C(RGB_BLACK));

  // 大字 99.9 用 logisoso50_tn (50px)
  u8g2.setFont(u8g2_font_logisoso50_tn);
  u8g2.setForegroundColor(C(RGB_BLACK));
  u8g2.setCursor(80, 120);
  u8g2.print("99.9");

  drawCN(260, 105, "米", u8g2_font_wqy16_t_gb2312, C(RGB_BLACK));

  // 分隔线
  tft.drawFastHLine(0, 140, 320, C(RGB_BLACK));

  // ===== 数据区 (y=155-205) =====
  drawCN(10,  165, "NDL",  u8g2_font_wqy16_t_gb2312, C(RGB_GREEN));
  drawCN(55,  165, "23 分", u8g2_font_wqy16_t_gb2312, C(RGB_BLACK));

  drawCN(180, 165, "N2",   u8g2_font_wqy16_t_gb2312, C(RGB_BLUE));
  drawCN(215, 165, "67%",  u8g2_font_wqy16_t_gb2312, C(RGB_BLACK));

  drawCN(10,  190, "最深",  u8g2_font_wqy16_t_gb2312, C(RGB_BLACK));
  drawCN(60,  190, "25.3 米", u8g2_font_wqy16_t_gb2312, C(RGB_BLACK));

  drawCN(180, 190, "温度",  u8g2_font_wqy16_t_gb2312, C(RGB_BLACK));
  drawCN(225, 190, "18.5度", u8g2_font_wqy16_t_gb2312, C(RGB_BLACK));

  // ===== 底部状态条 (y=210-240) =====
  tft.fillRect(0, 210, 320, 30, C(RGB_DARK));
  drawCN(8, 232, "MODE:翻页  UP:重置  DOWN:计时",
         u8g2_font_wqy13_t_gb2312, C(RGB_WHITE));

  Serial.println("[v4] HUD 显示完成");
}

void loop() {
  // 右上闪烁红点 (确认 loop 在跑)
  static uint32_t last = 0;
  static bool on = false;
  if (millis() - last > 500) {
    last = millis();
    on = !on;
    tft.fillCircle(305, 15, 5, on ? C(RGB_RED) : C(RGB_DARK));
  }
}
