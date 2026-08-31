/**************************************************************************
  TSL2585 live light dashboard for the Adafruit ESP32-S2 TFT Feather.

  Connect a TSL2585 breakout to the Feather's STEMMA QT port. The display
  shows photopic, infrared, and factory-calibrated UVA measurements normalized
  to 1x gain. The logarithmic bars make both dim and bright changes visible.

  Adafruit invests time and resources providing this open source code.
  Please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution.
 **************************************************************************/

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_TSL2585.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <SPI.h>

Adafruit_TSL2585 tsl2585;
Adafruit_ST7789 display(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

const uint16_t photopicColor = 0xFFE0;
const uint16_t infraredColor = 0xF980;
const uint16_t uvaColor = 0xA81F;

void drawDashboard(const tsl2585_data_t& data);
void drawReading(const char* label, float value, bool saturated,
                 int16_t baseline, uint16_t color);
void drawMessage(const char* line1, const char* line2, uint16_t color);
uint16_t getBarWidth(float value);

void setup() {
  Serial.begin(115200);

  // The Feather's TFT and STEMMA QT connector share a switched power supply.
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);

  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, LOW);
  delay(10);

  display.init(135, 240);
  display.setRotation(3);
  display.fillScreen(ST77XX_BLACK);
  digitalWrite(TFT_BACKLITE, HIGH);

  drawMessage("Adafruit TSL2585 Demo", "Looking for sensor...", ST77XX_CYAN);

  if (!tsl2585.begin()) {
    Serial.println("Could not find a TSL2585. Check the STEMMA QT cable.");
    drawMessage("NO TSL2585", "Check STEMMA QT cable", ST77XX_RED);
    while (true) {
      delay(10);
    }
  }

  if (!tsl2585.setIntegrationTime(50)) {
    Serial.println("Could not set the integration time.");
    drawMessage("TSL2585 ERROR", "Configuration failed", ST77XX_RED);
    while (true) {
      delay(10);
    }
  }

  Serial.println("Adafruit TSL2585 Feather ESP32-S2 TFT demo");
  Serial.println("Photopic\tInfrared\tUVA");
}

void loop() {
  if (!tsl2585.dataReady()) {
    delay(20);
    return;
  }

  tsl2585_data_t data;
  if (tsl2585.readData(&data)) {
    drawDashboard(data);

    Serial.print(data.photopic_1x, 1);
    Serial.print('\t');
    Serial.print(data.infrared_1x, 1);
    Serial.print('\t');
    Serial.println(data.uva_1x, 1);
  }

  delay(20);
}

void drawDashboard(const tsl2585_data_t& data) {
  canvas.fillScreen(ST77XX_BLACK);

  canvas.setTextWrap(false);
  canvas.setFont(&FreeSansBold9pt7b);
  canvas.setTextSize(1);
  canvas.setTextColor(ST77XX_WHITE);
  canvas.setCursor(4, 18);
  canvas.print("Adafruit TSL2585 Demo");

  canvas.drawFastHLine(0, 23, 240, 0x4208);

  drawReading("PHOTOPIC", data.photopic_1x, data.photopic_saturated, 42,
              photopicColor);
  drawReading("INFRARED", data.infrared_1x, data.infrared_saturated, 74,
              infraredColor);
  drawReading("UVA", data.uva_1x, data.uva_saturated, 106, uvaColor);

  if (data.photopic_saturated || data.infrared_saturated ||
      data.uva_saturated) {
    canvas.setFont();
    canvas.setTextSize(1);
    canvas.setTextColor(ST77XX_RED);
    canvas.setCursor(4, 126);
    canvas.print("SATURATED");
  }

  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
}

void drawReading(const char* label, float value, bool saturated,
                 int16_t baseline, uint16_t color) {
  canvas.setFont(&FreeSansBold9pt7b);
  canvas.setTextSize(1);
  canvas.setTextColor(color);
  canvas.setCursor(4, baseline);
  canvas.print(label);

  canvas.setFont(&FreeSansBold12pt7b);
  canvas.setTextColor(saturated ? ST77XX_RED : ST77XX_WHITE);
  canvas.setCursor(120, baseline);
  canvas.print(value, 1);

  canvas.drawRect(4, baseline + 5, 232, 7, 0x4208);
  uint16_t width = getBarWidth(value);
  if (width > 0) {
    canvas.fillRect(6, baseline + 7, width, 3, color);
  }
}

void drawMessage(const char* line1, const char* line2, uint16_t color) {
  canvas.fillScreen(ST77XX_BLACK);
  canvas.setTextWrap(false);
  canvas.setFont(&FreeSansBold9pt7b);
  canvas.setTextColor(color);
  canvas.setTextSize(1);
  canvas.setCursor(8, 53);
  canvas.print(line1);
  canvas.setFont(&FreeSans9pt7b);
  canvas.setTextColor(ST77XX_WHITE);
  canvas.setCursor(8, 83);
  canvas.print(line2);
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
}

uint16_t getBarWidth(float value) {
  // A fixed logarithmic scale keeps the display stable while showing useful
  // movement over the sensor's wide dynamic range. 131070 is the largest
  // possible 16-bit result normalized from the sensor's minimum 0.5x gain.
  const float maxLogValue = 5.1175;
  float fraction = log10f(value + 1.0) / maxLogValue;

  if (fraction < 0.0) {
    fraction = 0.0;
  } else if (fraction > 1.0) {
    fraction = 1.0;
  }

  return (uint16_t)(fraction * 228.0);
}
