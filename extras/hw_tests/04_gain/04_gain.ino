#include <Adafruit_TSL2585.h>

const tsl2585_gain_t TEST_GAINS[] = {
    TSL2585_GAIN_0_5X,  TSL2585_GAIN_1X,    TSL2585_GAIN_2X,
    TSL2585_GAIN_4X,    TSL2585_GAIN_8X,    TSL2585_GAIN_16X,
    TSL2585_GAIN_32X,   TSL2585_GAIN_64X,   TSL2585_GAIN_128X,
    TSL2585_GAIN_256X,  TSL2585_GAIN_512X,  TSL2585_GAIN_1024X,
    TSL2585_GAIN_2048X, TSL2585_GAIN_4096X};

Adafruit_TSL2585 tsl2585;

void printChannel(tsl2585_channel_t channel);
void printGain(tsl2585_gain_t gain);
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

  Serial.println(F("TSL2585 gain setter/getter hardware test"));

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println(F("Begin succeeded"));

  if (!tsl2585.enableAGC(false)) {
    haltWithFailure(F("Disabling AGC for the manual gain test failed"));
  }
  Serial.println(F("AGC disabled for the manual gain test"));

  for (uint8_t channel = 0; channel < 3; channel++) {
    tsl2585_channel_t test_channel = (tsl2585_channel_t)channel;
    Serial.println();
    Serial.print(F("Testing every gain for the "));
    printChannel(test_channel);
    Serial.println(F(" channel"));

    for (uint8_t gain_index = 0;
         gain_index < sizeof(TEST_GAINS) / sizeof(TEST_GAINS[0]);
         gain_index++) {
      tsl2585_gain_t requested_gain = TEST_GAINS[gain_index];
      if (!tsl2585.setGain(test_channel, requested_gain)) {
        haltWithFailure(F("Setting a channel gain failed"));
      }
      if (tsl2585.getGain(test_channel) != requested_gain) {
        haltWithFailure(F("Channel gain readback did not match"));
      }

      Serial.print(F("  "));
      printGain(requested_gain);
      Serial.println(F(" set and verified"));
    }

    Serial.print(F("PASS: every "));
    printChannel(test_channel);
    Serial.println(F(" gain set and read back correctly"));
  }

  haltWithSuccess();
}

void loop() {}

void printChannel(tsl2585_channel_t channel) {
  if (channel == TSL2585_CHANNEL_PHOTOPIC) {
    Serial.print(F("photopic"));
  } else if (channel == TSL2585_CHANNEL_IR) {
    Serial.print(F("IR"));
  } else {
    Serial.print(F("UVA"));
  }
}

void printGain(tsl2585_gain_t gain) {
  if (gain == TSL2585_GAIN_0_5X) {
    Serial.print(F("0.5x"));
  } else {
    Serial.print(1UL << ((uint8_t)gain - 1));
    Serial.print(F("x"));
  }
}

void haltWithFailure(const __FlashStringHelper *message) {
  Serial.print(F("FAIL: "));
  Serial.println(message);
  while (true) {
    delay(100);
  }
}

void haltWithSuccess() {
  Serial.println();
  Serial.println(F("ALL GAIN SETTER/GETTER TESTS PASSED"));
  while (true) {
    delay(100);
  }
}
