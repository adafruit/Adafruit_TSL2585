#include <Adafruit_TSL2585.h>

const uint16_t INTEGRATION_STEPS = 360;
const float INTEGRATION_STEP_MS = 0.25;
const float INTEGRATION_TOLERANCE_MS = 0.01;

Adafruit_TSL2585 tsl2585;

void haltWithFailure(const __FlashStringHelper *message);
void haltWithSuccess();

void setup() {
  Serial.begin(115200);
  // Wait for the Serial Monitor to open on native USB boards.
  // Remove this while (!Serial) loop to run without a USB connection.
  while (!Serial) {
    delay(10);
  }
  delay(250);

  Serial.println(F("TSL2585 integration time setter/getter hardware test"));
  Serial.println(
      F("Integration time is shared by the photopic, IR, and UVA channels."));
  Serial.println(F("Testing every 0.25 ms setting from 0.25 ms to 90 ms."));

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println(F("Begin succeeded"));

  for (uint16_t step = 1; step <= INTEGRATION_STEPS; step++) {
    float requested_ms = step * INTEGRATION_STEP_MS;
    if (!tsl2585.setIntegrationTime(requested_ms)) {
      haltWithFailure(F("Setting an integration time failed"));
    }

    float readback_ms = tsl2585.getIntegrationTime();
    if (abs(readback_ms - requested_ms) > INTEGRATION_TOLERANCE_MS) {
      haltWithFailure(F("Integration time readback did not match"));
    }

    Serial.print(F("  "));
    Serial.print(readback_ms, 2);
    Serial.println(F(" ms set and verified"));
  }

  haltWithSuccess();
}

void loop() {}

void haltWithFailure(const __FlashStringHelper *message) {
  Serial.print(F("FAIL: "));
  Serial.println(message);
  while (true) {
    delay(100);
  }
}

void haltWithSuccess() {
  Serial.println();
  Serial.println(F("ALL INTEGRATION TIME SETTER/GETTER TESTS PASSED"));
  while (true) {
    delay(100);
  }
}
