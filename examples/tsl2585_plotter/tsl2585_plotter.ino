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
    // Keep the same numeric label:value fields on every line so the Arduino
    // Serial Plotter can graph each series.
    Serial.print("Photopic:");
    Serial.print(data.photopic_1x, 1);
    Serial.print(",\tInfrared:");
    Serial.print(data.infrared_1x, 1);
    Serial.print(",\tUVA:");
    Serial.println(data.uva_1x, 1);
  }

  delay(100);
}
