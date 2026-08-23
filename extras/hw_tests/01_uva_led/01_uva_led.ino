#include <Adafruit_TSL2585.h>

#define UVA_LED_PIN 4

const uint8_t SAMPLES_PER_LEVEL = 6;
const float MINIMUM_UVA_INCREASE_COUNTS = 25.0F;
const float MINIMUM_UVA_INCREASE_FRACTION = 0.20F;

const tsl2585_gain_t UVA_TEST_GAINS[] = {
    TSL2585_GAIN_128X, TSL2585_GAIN_64X, TSL2585_GAIN_32X,
    TSL2585_GAIN_16X,  TSL2585_GAIN_8X,  TSL2585_GAIN_4X,
    TSL2585_GAIN_2X,   TSL2585_GAIN_1X,   TSL2585_GAIN_0_5X};

Adafruit_TSL2585 tsl2585;

bool readAverageUVA(bool led_on, float *average, bool *saturated);
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

  Serial.println("TSL2585 D4 UVA LED hardware test");
  Serial.println("Aim the 365 nm LED directly at the sensor before testing.");

  pinMode(UVA_LED_PIN, OUTPUT);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println("D4 UVA LED is safely off");

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println("Begin succeeded");

  if (!tsl2585.enableAGC(false)) {
    haltWithFailure(F("Disabling AGC for the fixed-gain test failed"));
  }
  Serial.println("AGC disabled for the fixed-gain test");

  if (!tsl2585.setIntegrationTime(50)) {
    haltWithFailure(F("Setting the integration time failed"));
  }
  Serial.println("50 ms integration time set");

  if (!tsl2585.setGain(TSL2585_CHANNEL_PHOTOPIC, TSL2585_GAIN_4X) ||
      !tsl2585.setGain(TSL2585_CHANNEL_IR, TSL2585_GAIN_4X)) {
    haltWithFailure(F("Setting photopic and IR gains failed"));
  }
  Serial.println("Photopic and IR gains set to 4x");

  for (uint8_t gain_index = 0;
       gain_index < sizeof(UVA_TEST_GAINS) / sizeof(UVA_TEST_GAINS[0]);
       gain_index++) {
    tsl2585_gain_t gain = UVA_TEST_GAINS[gain_index];
    if (!tsl2585.setGain(TSL2585_CHANNEL_UVA, gain)) {
      haltWithFailure(F("Setting the UVA test gain failed"));
    }

    Serial.println();
    Serial.print("Testing UVA gain code 0x");
    Serial.println((uint8_t)gain, HEX);

    float led_off_average;
    float led_on_average;
    bool led_off_saturated;
    bool led_on_saturated;

    if (!readAverageUVA(false, &led_off_average, &led_off_saturated)) {
      haltWithFailure(F("Could not read the LED-off UVA samples"));
    }
    if (!readAverageUVA(true, &led_on_average, &led_on_saturated)) {
      haltWithFailure(F("Could not read the LED-on UVA samples"));
    }
    digitalWrite(UVA_LED_PIN, LOW);

    Serial.print("LED off average: ");
    Serial.println(led_off_average, 1);
    Serial.print("LED on average:  ");
    Serial.println(led_on_average, 1);

    if (led_off_saturated || led_on_saturated) {
      Serial.println("UVA channel saturated; reducing the gain and retrying");
      continue;
    }

    float increase = led_on_average - led_off_average;
    float required_increase =
        led_off_average * MINIMUM_UVA_INCREASE_FRACTION;
    if (required_increase < MINIMUM_UVA_INCREASE_COUNTS) {
      required_increase = MINIMUM_UVA_INCREASE_COUNTS;
    }

    Serial.print("Measured increase: ");
    Serial.print(increase, 1);
    Serial.print(" counts; required: ");
    Serial.print(required_increase, 1);
    Serial.println(" counts");

    if (increase < required_increase) {
      haltWithFailure(
          F("UVA did not increase enough; check D4, polarity, resistor, and aim"));
    }

    Serial.println("365 nm LED produced a clear UVA response");
    haltWithSuccess();
  }

  haltWithFailure(F("UVA remained saturated at every gain setting"));
}

void loop() {}

bool readAverageUVA(bool led_on, float *average, bool *saturated) {
  digitalWrite(UVA_LED_PIN, led_on ? HIGH : LOW);
  delay(100);

  tsl2585_data_t data;
  if (!tsl2585.dataReady() || !tsl2585.readData(&data)) {
    return false;
  }

  uint32_t sum = 0;
  *saturated = false;
  for (uint8_t sample = 0; sample < SAMPLES_PER_LEVEL; sample++) {
    delay(100);
    if (!tsl2585.dataReady() || !tsl2585.readData(&data)) {
      return false;
    }
    sum += data.uva;
    if (data.uva_saturated) {
      *saturated = true;
    }
  }

  *average = sum / (float)SAMPLES_PER_LEVEL;
  return true;
}

void haltWithFailure(const __FlashStringHelper *message) {
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.print("FAIL: ");
  Serial.println(message);
  while (true) {
    delay(100);
  }
}

void haltWithSuccess() {
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println();
  Serial.println("ALL UVA LED TESTS PASSED");
  while (true) {
    delay(100);
  }
}
