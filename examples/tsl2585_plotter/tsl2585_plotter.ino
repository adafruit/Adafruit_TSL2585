#include <Adafruit_TSL2585.h>

Adafruit_TSL2585 tsl2585;

void setup() {
  Serial.begin(115200);
  // Wait for the Serial Monitor to open on native USB boards.
  // Remove this while (!Serial) loop to run without a USB connection.
  while (!Serial) {
    delay(10);
  }
  delay(250);

  Serial.println("Adafruit TSL2585 Serial Plotter example");

  if (!tsl2585.begin()) {
    Serial.println("Could not find a TSL2585. Check the wiring and I2C address.");
    while (true) {
      delay(10);
    }
  }

  // Register-based results support integration times from 0.25 ms to 90 ms.
  if (!tsl2585.setIntegrationTime(50)) {
    Serial.println("Could not set the integration time.");
    while (true) {
      delay(10);
    }
  }
}

void loop() {
  if (!tsl2585.dataReady()) {
    delay(100);
    return;
  }

  tsl2585_data_t data;
  if (tsl2585.readData(&data)) {
    uint8_t saturated = 0;
    if (data.photopic_saturated || data.infrared_saturated ||
        data.uva_saturated) {
      saturated = 1;
    }

    // Keep the same numeric label:value fields on every line so the Arduino
    // Serial Plotter can graph each series.
    Serial.print("Photopic_1x:");
    Serial.print(data.photopic_normalized, 1);
    Serial.print(",Infrared_1x:");
    Serial.print(data.infrared_normalized, 1);
    Serial.print(",UVA_1x:");
    Serial.print(data.uva_normalized, 1);
    Serial.print(",Saturated:");
    Serial.println(saturated);
  }

  delay(100);
}
