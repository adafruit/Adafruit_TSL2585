#include <Adafruit_TSL2585.h>

#define UVA_LED_PIN 4

const uint8_t SAMPLES_PER_GAIN = 4;
const uint8_t MEASUREMENT_READ_RETRIES = 2;
const uint16_t DATA_READY_TIMEOUT_MS = 1000;
const float GAIN_TEST_INTEGRATION_MS = 0.25;

const tsl2585_gain_t TEST_GAINS[] = {
    TSL2585_GAIN_0_5X,  TSL2585_GAIN_1X,    TSL2585_GAIN_2X,
    TSL2585_GAIN_4X,    TSL2585_GAIN_8X,    TSL2585_GAIN_16X,
    TSL2585_GAIN_32X,   TSL2585_GAIN_64X,   TSL2585_GAIN_128X,
    TSL2585_GAIN_256X,  TSL2585_GAIN_512X,  TSL2585_GAIN_1024X,
    TSL2585_GAIN_2048X, TSL2585_GAIN_4096X};

typedef struct {
  bool started;
  float first;
  float previous;
  float last;
  uint8_t levels;
  uint8_t clear_increases;
} trend_t;

Adafruit_TSL2585 tsl2585;

bool testChannel(tsl2585_channel_t channel);
bool readFreshData(tsl2585_data_t *data);
bool readChannelAverage(tsl2585_channel_t channel,
                        tsl2585_gain_t expected_gain, float *average,
                        bool *saturated);
bool updateTrend(trend_t *trend, float count);
bool trendPassed(const trend_t *trend);
uint16_t channelCount(const tsl2585_data_t *data,
                      tsl2585_channel_t channel);
tsl2585_gain_t channelGain(const tsl2585_data_t *data,
                           tsl2585_channel_t channel);
bool channelSaturated(const tsl2585_data_t *data,
                      tsl2585_channel_t channel);
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

  Serial.println(F("TSL2585 gain hardware test"));
  Serial.println(F("Room light tests the photopic and IR channels."));
  Serial.println(F("D4 provides a 365 nm signal for the UVA channel."));

  pinMode(UVA_LED_PIN, OUTPUT);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println(F("D4 UVA LED is safely off"));

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println(F("Begin succeeded"));

  if (!tsl2585.enableAGC(false)) {
    haltWithFailure(F("Disabling AGC for the manual gain test failed"));
  }
  Serial.println(F("AGC disabled for the manual gain test"));

  if (!tsl2585.setIntegrationTime(GAIN_TEST_INTEGRATION_MS) ||
      abs(tsl2585.getIntegrationTime() - GAIN_TEST_INTEGRATION_MS) > 0.01) {
    haltWithFailure(F("Setting the gain test integration time failed"));
  }
  Serial.println(F("0.25 ms integration time set and verified"));

  if (!testChannel(TSL2585_CHANNEL_PHOTOPIC) ||
      !testChannel(TSL2585_CHANNEL_IR) ||
      !testChannel(TSL2585_CHANNEL_UVA)) {
    haltWithFailure(F("A channel gain test failed"));
  }

  haltWithSuccess();
}

void loop() {}

bool testChannel(tsl2585_channel_t channel) {
  Serial.println();
  if (channel == TSL2585_CHANNEL_UVA) {
    digitalWrite(UVA_LED_PIN, HIGH);
    delay(100);
    Serial.println(F("D4 UVA LED is on for the UVA gain test"));
  } else {
    digitalWrite(UVA_LED_PIN, LOW);
  }

  for (uint8_t reset_channel = 0; reset_channel < 3; reset_channel++) {
    if (!tsl2585.setGain((tsl2585_channel_t)reset_channel,
                         TSL2585_GAIN_0_5X)) {
      Serial.println(F("Could not reset all gains before the test."));
      return false;
    }
  }

  Serial.print(F("Testing gain response for the "));
  printChannel(channel);
  Serial.println(F(" channel"));

  trend_t trend = {false, 0, 0, 0, 0, 0};
  for (uint8_t gain_index = 0;
       gain_index < sizeof(TEST_GAINS) / sizeof(TEST_GAINS[0]);
       gain_index++) {
    tsl2585_gain_t requested_gain = TEST_GAINS[gain_index];
    if (!tsl2585.setGain(channel, requested_gain) ||
        tsl2585.getGain(channel) != requested_gain) {
      Serial.println(F("Gain setter/getter verification failed."));
      return false;
    }

    float average;
    bool saturated;
    if (!readChannelAverage(channel, requested_gain, &average, &saturated)) {
      return false;
    }

    Serial.print(F("  "));
    printGain(requested_gain);
    Serial.print(F(": "));
    if (saturated && average == 65535) {
      Serial.println(F("analog saturation (0xFFFF)"));
    } else {
      Serial.print(average, 1);
      Serial.print(F(" counts"));
      if (saturated) {
        Serial.print(F(" (saturated)"));
      }
      Serial.println();
    }

    if (!saturated && !updateTrend(&trend, average)) {
      Serial.println(F("Counts decreased beyond the allowed noise margin."));
      return false;
    }
  }

  if (!trendPassed(&trend)) {
    Serial.println(F("Counts did not increase across enough gain settings."));
    return false;
  }

  Serial.print(F("PASS: "));
  printChannel(channel);
  Serial.println(F(" counts increased as gain increased"));
  return true;
}

bool readFreshData(tsl2585_data_t *data) {
  for (uint8_t attempt = 0; attempt <= MEASUREMENT_READ_RETRIES; attempt++) {
    uint32_t start_ms = millis();
    while (!tsl2585.dataReady()) {
      if (millis() - start_ms >= DATA_READY_TIMEOUT_MS) {
        return false;
      }
      delay(1);
    }

    if (tsl2585.readData(data)) {
      if (attempt > 0) {
        Serial.println(F("  Recovered a transient measurement read."));
      }
      return true;
    }
    delay(1);
  }
  return false;
}

bool readChannelAverage(tsl2585_channel_t channel,
                        tsl2585_gain_t expected_gain, float *average,
                        bool *saturated) {
  tsl2585_data_t data;
  if (!readFreshData(&data)) {
    Serial.println(F("Could not read the first post-change measurement."));
    return false;
  }

  uint32_t sum = 0;
  *saturated = false;
  for (uint8_t sample = 0; sample < SAMPLES_PER_GAIN; sample++) {
    if (!readFreshData(&data)) {
      Serial.println(F("Could not read a measurement during averaging."));
      return false;
    }
    if (channelGain(&data, channel) != expected_gain) {
      Serial.println(F("Measurement reported the wrong hardware gain."));
      return false;
    }
    sum += channelCount(&data, channel);
    if (channelSaturated(&data, channel)) {
      *saturated = true;
    }
  }

  *average = sum / (float)SAMPLES_PER_GAIN;
  return true;
}

bool updateTrend(trend_t *trend, float count) {
  if (!trend->started) {
    trend->started = true;
    trend->first = count;
    trend->previous = count;
    trend->last = count;
    trend->levels = 1;
    return true;
  }

  float decrease_margin = trend->previous * 0.15;
  if (decrease_margin < 2) {
    decrease_margin = 2;
  }
  if (count + decrease_margin < trend->previous) {
    return false;
  }

  float increase_margin = trend->previous * 0.12;
  if (increase_margin < 2) {
    increase_margin = 2;
  }
  if (count >= trend->previous + increase_margin) {
    trend->clear_increases++;
  }
  trend->previous = count;
  trend->last = count;
  trend->levels++;
  return true;
}

bool trendPassed(const trend_t *trend) {
  if (trend->levels < 4 || trend->clear_increases < 3) {
    return false;
  }
  float required_growth = trend->first * 0.50;
  if (required_growth < 8) {
    required_growth = 8;
  }
  return trend->last >= trend->first + required_growth;
}

uint16_t channelCount(const tsl2585_data_t *data,
                      tsl2585_channel_t channel) {
  if (channel == TSL2585_CHANNEL_PHOTOPIC) {
    return data->photopic;
  }
  if (channel == TSL2585_CHANNEL_IR) {
    return data->infrared;
  }
  return data->uva;
}

tsl2585_gain_t channelGain(const tsl2585_data_t *data,
                           tsl2585_channel_t channel) {
  if (channel == TSL2585_CHANNEL_PHOTOPIC) {
    return data->photopic_gain;
  }
  if (channel == TSL2585_CHANNEL_IR) {
    return data->infrared_gain;
  }
  return data->uva_gain;
}

bool channelSaturated(const tsl2585_data_t *data,
                      tsl2585_channel_t channel) {
  if (channel == TSL2585_CHANNEL_PHOTOPIC) {
    return data->photopic_saturated;
  }
  if (channel == TSL2585_CHANNEL_IR) {
    return data->infrared_saturated;
  }
  return data->uva_saturated;
}

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
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.print(F("FAIL: "));
  Serial.println(message);
  while (true) {
    delay(100);
  }
}

void haltWithSuccess() {
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println();
  Serial.println(F("ALL GAIN HARDWARE TESTS PASSED"));
  while (true) {
    delay(100);
  }
}
