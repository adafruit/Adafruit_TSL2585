#include <Adafruit_TSL2585.h>

#define UVA_LED_PIN 4

const uint8_t SAMPLES_PER_LEVEL = 4;
const uint8_t MEASUREMENT_READ_RETRIES = 2;
const uint16_t DATA_READY_TIMEOUT_MS = 1000;

const tsl2585_gain_t TEST_GAINS[] = {
    TSL2585_GAIN_4X, TSL2585_GAIN_8X, TSL2585_GAIN_64X};

typedef struct {
  bool started;
  float first;
  float previous;
  float last;
  uint8_t levels;
  uint8_t clear_increases;
} trend_t;

Adafruit_TSL2585 tsl2585;

bool readFreshData(tsl2585_data_t *data);
bool readAverages(float *averages, bool *saturated);
bool updateTrend(trend_t *trend, float count);
bool trendPassed(const trend_t *trend);
uint16_t channelCount(const tsl2585_data_t *data,
                      tsl2585_channel_t channel);
tsl2585_gain_t channelGain(const tsl2585_data_t *data,
                           tsl2585_channel_t channel);
bool channelSaturated(const tsl2585_data_t *data,
                      tsl2585_channel_t channel);
void printChannel(tsl2585_channel_t channel);
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

  Serial.println(F("TSL2585 integration time hardware test"));
  Serial.println(F("Keep the sensor and room lighting steady."));
  Serial.println(F("Aim the D4 365 nm LED at the sensor for the UVA signal."));

  pinMode(UVA_LED_PIN, OUTPUT);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println(F("D4 UVA LED is safely off"));

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println(F("Begin succeeded"));

  if (!tsl2585.enableAGC(false)) {
    haltWithFailure(F("Disabling AGC for the integration test failed"));
  }
  Serial.println(F("AGC disabled for the integration test"));

  for (uint8_t channel = 0; channel < 3; channel++) {
    if (!tsl2585.setGain((tsl2585_channel_t)channel,
                         TEST_GAINS[channel]) ||
        tsl2585.getGain((tsl2585_channel_t)channel) != TEST_GAINS[channel]) {
      haltWithFailure(F("Setting a channel gain failed"));
    }
  }
  Serial.println(F("Fixed gains set and verified for all three channels"));

  digitalWrite(UVA_LED_PIN, HIGH);
  delay(100);
  Serial.println(F("D4 UVA LED is on for the integration test"));

  trend_t trends[3] = {{false, 0, 0, 0, 0, 0},
                       {false, 0, 0, 0, 0, 0},
                       {false, 0, 0, 0, 0, 0}};

  for (uint8_t integration_ms = 5; integration_ms <= 90;
       integration_ms += 5) {
    if (!tsl2585.setIntegrationTime(integration_ms) ||
        abs(tsl2585.getIntegrationTime() - integration_ms) > 0.01) {
      haltWithFailure(F("Integration time setter/getter verification failed"));
    }

    float averages[3];
    bool saturated[3];
    if (!readAverages(averages, saturated)) {
      haltWithFailure(F("Could not average fresh measurements"));
    }

    Serial.print(F("  "));
    Serial.print(integration_ms);
    Serial.print(F(" ms: photopic "));
    Serial.print(averages[TSL2585_CHANNEL_PHOTOPIC], 1);
    Serial.print(F(", IR "));
    Serial.print(averages[TSL2585_CHANNEL_IR], 1);
    Serial.print(F(", UVA "));
    Serial.println(averages[TSL2585_CHANNEL_UVA], 1);

    for (uint8_t channel = 0; channel < 3; channel++) {
      if (saturated[channel]) {
        Serial.print(F("Unexpected saturation on the "));
        printChannel((tsl2585_channel_t)channel);
        Serial.println(F(" channel"));
        haltWithFailure(F("A channel saturated during the integration test"));
      }
      if (!updateTrend(&trends[channel], averages[channel])) {
        Serial.print(F("Counts decreased too far on the "));
        printChannel((tsl2585_channel_t)channel);
        Serial.println(F(" channel"));
        haltWithFailure(F("An integration response decreased"));
      }
    }
  }

  for (uint8_t channel = 0; channel < 3; channel++) {
    if (!trendPassed(&trends[channel])) {
      Serial.print(F("Integration response was too small on the "));
      printChannel((tsl2585_channel_t)channel);
      Serial.println(F(" channel"));
      haltWithFailure(F("A channel did not respond to integration time"));
    }
    Serial.print(F("PASS: "));
    printChannel((tsl2585_channel_t)channel);
    Serial.println(F(" counts increased with integration time"));
  }

  haltWithSuccess();
}

void loop() {}

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

bool readAverages(float *averages, bool *saturated) {
  tsl2585_data_t data;
  if (!readFreshData(&data)) {
    return false;
  }

  uint32_t sums[3] = {0, 0, 0};
  for (uint8_t channel = 0; channel < 3; channel++) {
    saturated[channel] = false;
  }

  for (uint8_t sample = 0; sample < SAMPLES_PER_LEVEL; sample++) {
    if (!readFreshData(&data)) {
      return false;
    }
    for (uint8_t channel = 0; channel < 3; channel++) {
      tsl2585_channel_t test_channel = (tsl2585_channel_t)channel;
      if (channelGain(&data, test_channel) != TEST_GAINS[channel]) {
        Serial.println(F("Measurement reported the wrong hardware gain."));
        return false;
      }
      sums[channel] += channelCount(&data, test_channel);
      if (channelSaturated(&data, test_channel)) {
        saturated[channel] = true;
      }
    }
  }

  for (uint8_t channel = 0; channel < 3; channel++) {
    averages[channel] = sums[channel] / (float)SAMPLES_PER_LEVEL;
  }
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

  float increase_margin = trend->previous * 0.04;
  if (increase_margin < 1) {
    increase_margin = 1;
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
  if (trend->levels != 18 || trend->clear_increases < 6) {
    return false;
  }
  float required_growth = trend->first * 4;
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
  Serial.println(F("ALL INTEGRATION TIME HARDWARE TESTS PASSED"));
  while (true) {
    delay(100);
  }
}
