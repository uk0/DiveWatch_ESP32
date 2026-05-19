// =====================================================================
// ST7789 2.4" 240x320 SPI 屏 - 点亮测试
// 用 Adafruit 库 (配置最简单, 不用改库文件)
//
// 接线 (ESP32-S3-Nano):
//   GND  -> GND
//   VCC  -> 3V3
//   SCL  -> D13 (GPIO 48 = HSPI SCK)
//   SDA  -> D11 (GPIO 38 = HSPI MOSI)
//   RES  -> D8  (GPIO 17)
//   DC   -> D9  (GPIO 18)
//   CS   -> D10 (GPIO 21)
//   BLK  -> 3V3 (背光常亮)
//
// 需要安装的库 (Arduino IDE 库管理器搜):
//   - "Adafruit GFX Library"     by Adafruit
//   - "Adafruit ST7735 and ST7789 Library" by Adafruit
//   - "Adafruit BusIO"            (自动依赖)
// =====================================================================

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS    21    // D10
#define TFT_DC    18    // D9
#define TFT_RST   17    // D8
// MOSI=38, SCK=48 用 ESP32-S3-Nano 默认硬件 SPI (HSPI)

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[ST7789] init...");

  tft.init(240, 320);                  // 分辨率 240×320
  tft.setRotation(0);                   // 0/1/2/3 旋转 (0=竖屏)
  tft.fillScreen(ST77XX_BLACK);

  // 1. 大字标题
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(4);
  tft.setCursor(20, 30);
  tft.println("DiveWatch");

  // 2. 副标题
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(20, 90);
  tft.println("ST7789 240x320");

  // 3. 接线确认 (彩色彩虹横条)
  tft.fillRect(0, 130, 240, 10, ST77XX_RED);
  tft.fillRect(0, 145, 240, 10, ST77XX_ORANGE);
  tft.fillRect(0, 160, 240, 10, ST77XX_YELLOW);
  tft.fillRect(0, 175, 240, 10, ST77XX_GREEN);
  tft.fillRect(0, 190, 240, 10, ST77XX_CYAN);
  tft.fillRect(0, 205, 240, 10, ST77XX_BLUE);
  tft.fillRect(0, 220, 240, 10, ST77XX_MAGENTA);

  // 4. 操作提示
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 250);
  tft.println("接线 OK!");
  tft.setTextSize(1);
  tft.setCursor(20, 280);
  tft.println("下一步: 迁移 DiveWatch UI");
  tft.setCursor(20, 295);
  tft.println("库: TFT_eSPI / LovyanGFX");

  Serial.println("[ST7789] 显示完成. 应该看到 DiveWatch + 彩虹条");
}

void loop() {
  // 简单闪烁右上角小红点表示在跑
  static uint32_t last = 0;
  static bool blink = false;
  if (millis() - last > 500) {
    last = millis();
    blink = !blink;
    tft.fillCircle(220, 20, 6, blink ? ST77XX_RED : ST77XX_BLACK);
  }
}
