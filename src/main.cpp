#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// SH1106 1.3" OLED mit I2C (SDA, SCL)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() {
  Wire.begin();
  u8g2.begin();
}

void loop() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(0, 24, "Hallo OLED!");

  u8g2.drawFrame(0, 32, 128, 32);
  u8g2.drawCircle(64, 48, 10, U8G2_DRAW_ALL);
  u8g2.drawStr(28, 56, "I2C SH1106");

  u8g2.sendBuffer();
  delay(1000);
}
