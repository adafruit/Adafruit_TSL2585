#include <Adafruit_TSL2585.h>

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

  Serial.println("TSL2585 basic hardware test");

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println("Begin succeeded");

  if (!tsl2585.reset()) {
    haltWithFailure(F("Software reset failed"));
  }
  Serial.println("Software reset succeeded");
  if (tsl2585.getDeviceID() != TSL2585_DEVICE_ID) {
    haltWithFailure(F("Sensor did not respond after software reset"));
  }
  Serial.println("Sensor responded after software reset");

  if (!tsl2585.begin()) {
    haltWithFailure(F("Second begin failed after software reset"));
  }
  Serial.println("Second begin succeeded after software reset");

  if (!tsl2585.enableAGC(false)) {
    haltWithFailure(F("Disabling AGC for the manual gain checks failed"));
  }
  Serial.println("AGC disabled for the manual gain checks");

  if (tsl2585.getDeviceID() != TSL2585_DEVICE_ID) {
    haltWithFailure(F("Device ID did not match 0x5C"));
  }
  Serial.println("Device ID matched 0x5C");

  Serial.print("Revision ID: 0x");
  Serial.println(tsl2585.getRevisionID(), HEX);
  Serial.print("Auxiliary ID: 0x");
  Serial.println(tsl2585.getAuxiliaryID(), HEX);
  Serial.print("Factory UVA calibration byte: ");
  Serial.println(tsl2585.getUVCalibration());

  Serial.println();
  if (!tsl2585.setIntegrationTime(50)) {
    haltWithFailure(F("Setting the integration time failed"));
  }
  if (abs(tsl2585.getIntegrationTime() - 50) > 0.01F) {
    haltWithFailure(F("Integration time readback did not match 50 ms"));
  }
  Serial.println("50 ms integration time set and verified");

  if (!tsl2585.setGain(TSL2585_CHANNEL_PHOTOPIC, TSL2585_GAIN_128X) ||
      !tsl2585.setGain(TSL2585_CHANNEL_IR, TSL2585_GAIN_128X) ||
      !tsl2585.setGain(TSL2585_CHANNEL_UVA, TSL2585_GAIN_128X)) {
    haltWithFailure(F("Setting 128x channel gains failed"));
  }
  if (tsl2585.getGain(TSL2585_CHANNEL_PHOTOPIC) != TSL2585_GAIN_128X ||
      tsl2585.getGain(TSL2585_CHANNEL_IR) != TSL2585_GAIN_128X ||
      tsl2585.getGain(TSL2585_CHANNEL_UVA) != TSL2585_GAIN_128X) {
    haltWithFailure(F("Channel gain readback failed"));
  }
  Serial.println("128x channel gains set and verified");

  Serial.println();
  if (!tsl2585.enable(false)) {
    haltWithFailure(F("Disabling the sensor failed"));
  }
  Serial.println("Disable succeeded");
  if (!tsl2585.enable(true)) {
    haltWithFailure(F("Re-enabling the sensor failed"));
  }
  Serial.println("Re-enable succeeded");

  delay(100);
  if (!tsl2585.dataReady()) {
    haltWithFailure(F("Fresh measurement was not available"));
  }

  tsl2585_data_t data;
  if (!tsl2585.readData(&data)) {
    haltWithFailure(F("Reading the coherent measurement failed"));
  }
  Serial.println("Coherent three-channel measurement succeeded");
  Serial.print("Photopic: ");
  Serial.print(data.photopic);
  Serial.print("    IR: ");
  Serial.print(data.infrared);
  Serial.print("    UVA: ");
  Serial.println(data.uva);

  haltWithSuccess();
}

void loop() {}

void haltWithFailure(const __FlashStringHelper *message) {
  Serial.print("FAIL: ");
  Serial.println(message);
  while (true) {
    delay(100);
  }
}

void haltWithSuccess() {
  Serial.println();
  Serial.println("ALL BASIC TESTS PASSED");
  while (true) {
    delay(100);
  }
}
