#include <Adafruit_TSL2585.h>

#define INT_PIN 2
#define UVA_LED_PIN 4

// The 24-bit threshold registers align 16-bit full counts at the MSB.
const uint8_t ALS_THRESHOLD_COUNT_SHIFT = 8;
const float MINIMUM_UVA_INCREASE_COUNTS = 25;

Adafruit_TSL2585 tsl2585;

bool readFreshData(tsl2585_data_t *data);
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

  Serial.println(F("TSL2585 D2 UVA interrupt hardware test"));
  Serial.println(
      F("Aim the D4 365 nm LED directly at the sensor before testing."));

  pinMode(INT_PIN, INPUT);
  pinMode(UVA_LED_PIN, OUTPUT);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println(F("D2 is an input; D4 UVA LED is safely off"));

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println(F("Begin succeeded"));

  if (!tsl2585.enableAGC(false)) {
    haltWithFailure(F("Disabling AGC for the fixed threshold test failed"));
  }
  Serial.println(F("AGC disabled for the fixed threshold test"));

  if (!tsl2585.setIntegrationTime(50)) {
    haltWithFailure(F("Setting the integration time failed"));
  }
  Serial.println(F("50 ms integration time set"));

  if (!tsl2585.setGain(TSL2585_CHANNEL_UVA, TSL2585_GAIN_64X)) {
    haltWithFailure(F("Setting the UVA gain failed"));
  }
  Serial.println(F("UVA gain set to 64x"));

  if (!tsl2585.enableALSInterrupt(false)) {
    haltWithFailure(F("Disabling ALS interrupts failed"));
  }

  tsl2585_data_t led_off_data;
  tsl2585_data_t led_on_data;

  digitalWrite(UVA_LED_PIN, LOW);
  if (!readFreshData(&led_off_data)) {
    haltWithFailure(F("Could not read the LED-off UVA measurement"));
  }

  digitalWrite(UVA_LED_PIN, HIGH);
  if (!readFreshData(&led_on_data)) {
    haltWithFailure(F("Could not read the LED-on UVA measurement"));
  }
  digitalWrite(UVA_LED_PIN, LOW);

  Serial.println();
  Serial.print(F("LED off: "));
  Serial.print(led_off_data.uva);
  Serial.print(F(" counts  LED on: "));
  Serial.print(led_on_data.uva);
  Serial.println(F(" counts"));

  if (led_off_data.uva_saturated || led_on_data.uva_saturated) {
    haltWithFailure(F("UVA channel saturated; move the LED farther away"));
  }
  if (led_on_data.uva < led_off_data.uva + MINIMUM_UVA_INCREASE_COUNTS) {
    haltWithFailure(F("D4 did not produce a clear UVA increase"));
  }
  Serial.println(F("PASS: D4 produced a clear UVA increase"));

  uint16_t uva_count_threshold =
      led_off_data.uva + ((led_on_data.uva - led_off_data.uva) / 2);
  uint32_t high_threshold =
      (uint32_t)uva_count_threshold << ALS_THRESHOLD_COUNT_SHIFT;
  Serial.print(F("UVA interrupt threshold: "));
  Serial.print(uva_count_threshold);
  Serial.println(F(" counts"));

  if (!tsl2585.setALSThresholds(TSL2585_CHANNEL_UVA, 0, high_threshold, 1)) {
    haltWithFailure(F("Configuring the UVA interrupt threshold failed"));
  }

  digitalWrite(UVA_LED_PIN, LOW);
  if (!readFreshData(&led_off_data)) {
    haltWithFailure(F("Could not read UVA with D4 off"));
  }
  if (!tsl2585.clearALSInterrupt()) {
    haltWithFailure(F("Clearing the ALS interrupt failed"));
  }

  if (!tsl2585.enableALSInterrupt(true)) {
    haltWithFailure(F("Enabling ALS interrupts failed"));
  }
  delay(10);
  if (digitalRead(INT_PIN) != HIGH) {
    haltWithFailure(F("D2 was not high while D4 was off"));
  }
  Serial.println(F("PASS: D2 was high while D4 was off"));

  digitalWrite(UVA_LED_PIN, HIGH);
  delay(150);
  if (digitalRead(INT_PIN) != LOW) {
    haltWithFailure(F("D2 was not low while D4 was on"));
  }
  Serial.println(F("PASS: D2 went low while D4 was on"));

  if (!tsl2585.enableALSInterrupt(false)) {
    haltWithFailure(F("Disabling ALS interrupts failed"));
  }
  digitalWrite(UVA_LED_PIN, LOW);
  if (!tsl2585.clearALSInterrupt()) {
    haltWithFailure(F("Clearing the ALS interrupt failed"));
  }
  delay(10);
  if (digitalRead(INT_PIN) != HIGH) {
    haltWithFailure(F("D2 did not return high after the interrupt was cleared"));
  }
  Serial.println(F("PASS: D2 returned high after the interrupt was cleared"));

  haltWithSuccess();
}

void loop() {}

bool readFreshData(tsl2585_data_t *data) {
  delay((uint32_t)tsl2585.getIntegrationTime() + 10);
  if (!tsl2585.dataReady()) {
    return false;
  }
  return tsl2585.readData(data);
}

void haltWithFailure(const __FlashStringHelper *message) {
  digitalWrite(UVA_LED_PIN, LOW);
  tsl2585.enableALSInterrupt(false);
  tsl2585.clearALSInterrupt();
  Serial.print(F("FAIL: "));
  Serial.println(message);
  while (true) {
    delay(100);
  }
}

void haltWithSuccess() {
  digitalWrite(UVA_LED_PIN, LOW);
  tsl2585.enableALSInterrupt(false);
  tsl2585.clearALSInterrupt();
  Serial.println();
  Serial.println(F("ALL UVA INTERRUPT TESTS PASSED"));
  while (true) {
    delay(100);
  }
}
