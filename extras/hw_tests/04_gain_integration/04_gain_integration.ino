#include <Adafruit_TSL2585.h>

#define UVA_LED_PIN 4

const uint8_t SAMPLES_PER_LEVEL = 6;
const uint16_t DATA_READY_TIMEOUT_MS = 1000;
const float GAIN_TEST_INTEGRATION_MS = 10;
const float MAXIMUM_INTEGRATION_GAIN_COUNT = 3500;

const float GAIN_TEST_INTEGRATION_TIMES[] = {10, 5, 2.5, 1.25, 0.5,
                                             0.25};

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

bool testGain(tsl2585_channel_t channel, tsl2585_gain_t *integration_gain);
bool testIntegrationTime(const tsl2585_gain_t *gains);
bool readFreshData(tsl2585_data_t *data);
bool readAverages(const tsl2585_gain_t *expected_gains, float *averages,
                  bool *saturated);
bool updateTrend(trend_t *trend, float count, float decrease_fraction,
                 float increase_fraction, float minimum_change);
bool gainTrendPassed(const trend_t *trend);
bool integrationTrendPassed(const trend_t *trend);
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

  Serial.println(F("TSL2585 gain and integration time hardware test"));
  Serial.println(
      F("Keep the sensor and room lighting steady during this test."));
  Serial.println(F("Aim the D4 365 nm LED at the sensor for a UVA signal."));
  Serial.println(F("The 4096x preflight must not saturate."));
  Serial.println(
      F("If asked, reduce room light or move the D4 LED and restart."));

  pinMode(UVA_LED_PIN, OUTPUT);
  digitalWrite(UVA_LED_PIN, LOW);
  Serial.println(F("D4 UVA LED is safely off"));

  if (!tsl2585.begin()) {
    haltWithFailure(F("Begin failed: check sensor power and I2C wiring"));
  }
  Serial.println(F("Begin succeeded"));

  if (!tsl2585.enableAGC(false)) {
    haltWithFailure(F("Disabling AGC for the manual gain sweep failed"));
  }
  Serial.println(F("AGC disabled for the manual gain sweep"));

  digitalWrite(UVA_LED_PIN, HIGH);
  delay(100);
  Serial.println(F("D4 UVA LED is on for a steady optical stimulus"));

  tsl2585_gain_t integration_gains[3];
  if (!testGain(TSL2585_CHANNEL_PHOTOPIC, &integration_gains[0]) ||
      !testGain(TSL2585_CHANNEL_IR, &integration_gains[1]) ||
      !testGain(TSL2585_CHANNEL_UVA, &integration_gains[2])) {
    haltWithFailure(F("A channel gain sweep failed"));
  }

  if (!testIntegrationTime(integration_gains)) {
    haltWithFailure(F("The integration time sweep failed"));
  }

  haltWithSuccess();
}

void loop() {}

bool testGain(tsl2585_channel_t channel, tsl2585_gain_t *integration_gain) {
  Serial.println();
  Serial.print(F("Checking maximum gain for the "));
  printChannel(channel);
  Serial.println(F(" channel"));

  tsl2585_gain_t expected_gains[3] = {
      TSL2585_GAIN_0_5X, TSL2585_GAIN_0_5X, TSL2585_GAIN_0_5X};
  for (uint8_t reset_channel = 0; reset_channel < 3; reset_channel++) {
    if (!tsl2585.setGain((tsl2585_channel_t)reset_channel,
                         TSL2585_GAIN_0_5X)) {
      Serial.println(F("Could not reset all gains before the sweep."));
      return false;
    }
  }

  expected_gains[channel] = TSL2585_GAIN_4096X;
  if (!tsl2585.setGain(channel, TSL2585_GAIN_4096X) ||
      tsl2585.getGain(channel) != TSL2585_GAIN_4096X) {
    Serial.println(F("Could not set 4096x for the preflight."));
    return false;
  }

  float gain_sweep_integration_ms = GAIN_TEST_INTEGRATION_MS;
  bool maximum_gain_ready = false;
  for (uint8_t time_index = 0;
       time_index < sizeof(GAIN_TEST_INTEGRATION_TIMES) /
                        sizeof(GAIN_TEST_INTEGRATION_TIMES[0]);
       time_index++) {
    if (!tsl2585.setIntegrationTime(
            GAIN_TEST_INTEGRATION_TIMES[time_index])) {
      Serial.println(F("Could not set the integration time."));
      return false;
    }
    gain_sweep_integration_ms = tsl2585.getIntegrationTime();

    float averages[3];
    bool saturated[3];
    if (!readAverages(expected_gains, averages, saturated)) {
      return false;
    }

    Serial.print(F("  4096x at "));
    Serial.print(gain_sweep_integration_ms, 2);
    Serial.print(F(" ms: "));
    Serial.print(averages[channel], 1);
    Serial.print(F(" counts"));
    if (saturated[channel]) {
      Serial.print(F(" (saturated)"));
    }
    Serial.println();

    if (!saturated[channel]) {
      maximum_gain_ready = true;
      break;
    }
  }

  if (!maximum_gain_ready) {
    Serial.println(F("4096x is still saturated at minimum integration."));
    Serial.println(
        F("Reduce room light or move the D4 LED farther away and restart."));
    return false;
  }

  expected_gains[channel] = TSL2585_GAIN_0_5X;
  if (!tsl2585.setGain(channel, TSL2585_GAIN_0_5X) ||
      tsl2585.getGain(channel) != TSL2585_GAIN_0_5X) {
    Serial.println(F("Could not reset the gain after the preflight."));
    return false;
  }

  Serial.print(F("Sweeping all gains for the "));
  printChannel(channel);
  Serial.print(F(" channel at "));
  Serial.print(gain_sweep_integration_ms, 2);
  Serial.println(F(" ms, starting from 0.5x"));

  trend_t trend = {false, 0, 0, 0, 0, 0};
  *integration_gain = TSL2585_GAIN_0_5X;
  bool integration_gain_selected = false;
  float maximum_integration_gain_count =
      MAXIMUM_INTEGRATION_GAIN_COUNT * gain_sweep_integration_ms /
      GAIN_TEST_INTEGRATION_MS;

  for (uint8_t gain_index = 0;
       gain_index < sizeof(TEST_GAINS) / sizeof(TEST_GAINS[0]); gain_index++) {
    tsl2585_gain_t gain = TEST_GAINS[gain_index];
    expected_gains[channel] = gain;

    if (!tsl2585.setGain(channel, gain) || tsl2585.getGain(channel) != gain) {
      Serial.print(F("Gain setter/getter failed for "));
      printChannel(channel);
      Serial.print(F(" at "));
      printGain(gain);
      Serial.println();
      return false;
    }

    float averages[3];
    bool saturated[3];
    if (!readAverages(expected_gains, averages, saturated)) {
      return false;
    }

    Serial.print(F("  "));
    printGain(gain);
    Serial.print(F(": "));
    Serial.print(averages[channel], 1);
    Serial.print(F(" counts"));
    if (saturated[channel]) {
      Serial.print(F(" (saturated)"));
    }
    Serial.println();
    Serial.println(F("  PASS: requested gain was reported by the sensor"));

    if (!saturated[channel]) {
      if (!updateTrend(&trend, averages[channel], 0.12F, 0.15F, 3)) {
        Serial.println(
            F("Counts decreased beyond the allowed noise margin."));
        return false;
      }
      if (averages[channel] <= maximum_integration_gain_count) {
        *integration_gain = gain;
        integration_gain_selected = true;
      }
    }
  }

  if (!gainTrendPassed(&trend)) {
    Serial.print(F("The "));
    printChannel(channel);
    Serial.println(F(" counts did not show enough clear gain response."));
    Serial.println(
        F("Check the light level and keep the illumination steady."));
    return false;
  }
  if (!integration_gain_selected) {
    Serial.println(
        F("No unsaturated gain was available for integration testing."));
    return false;
  }

  Serial.print(F("PASS: "));
  printChannel(channel);
  Serial.print(F(" counts increased across the gain sweep; using "));
  printGain(*integration_gain);
  Serial.println(F(" for the integration test"));
  return true;
}

bool testIntegrationTime(const tsl2585_gain_t *gains) {
  Serial.println();
  Serial.println(F("Sweeping integration time from 5 ms to 90 ms"));

  for (uint8_t channel = 0; channel < 3; channel++) {
    if (!tsl2585.setGain((tsl2585_channel_t)channel, gains[channel])) {
      Serial.println(F("Could not set a gain for the integration sweep."));
      return false;
    }
  }

  trend_t trends[3] = {{false, 0, 0, 0, 0, 0},
                       {false, 0, 0, 0, 0, 0},
                       {false, 0, 0, 0, 0, 0}};

  for (uint8_t integration_ms = 5; integration_ms <= 90;
       integration_ms += 5) {
    if (!tsl2585.setIntegrationTime(integration_ms) ||
        abs(tsl2585.getIntegrationTime() - integration_ms) > 0.01F) {
      Serial.print(F("Integration setter/getter failed at "));
      Serial.print(integration_ms);
      Serial.println(F(" ms"));
      return false;
    }

    float averages[3];
    bool saturated[3];
    if (!readAverages(gains, averages, saturated)) {
      return false;
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
        Serial.print(F("Channel saturated during integration sweep: "));
        printChannel((tsl2585_channel_t)channel);
        Serial.println();
        Serial.println(F("Reduce the light level and run the test again."));
        return false;
      }
      if (!updateTrend(&trends[channel], averages[channel], 0.10F, 0.04F, 3)) {
        Serial.print(
            F("Counts decreased beyond the allowed noise margin for "));
        printChannel((tsl2585_channel_t)channel);
        Serial.println();
        return false;
      }
    }
    Serial.println(
        F("  PASS: time and all three hardware gains were reported"));
  }

  for (uint8_t channel = 0; channel < 3; channel++) {
    if (!integrationTrendPassed(&trends[channel])) {
      Serial.print(F("The integration response was too small for "));
      printChannel((tsl2585_channel_t)channel);
      Serial.println();
      Serial.println(
          F("Check the light level and keep the illumination steady."));
      return false;
    }
    Serial.print(F("PASS: "));
    printChannel((tsl2585_channel_t)channel);
    Serial.println(F(" counts increased with integration time"));
  }
  return true;
}

bool readFreshData(tsl2585_data_t *data) {
  uint32_t start_ms = millis();
  while (!tsl2585.dataReady()) {
    if (millis() - start_ms >= DATA_READY_TIMEOUT_MS) {
      return false;
    }
    delay(1);
  }
  return tsl2585.readData(data);
}

bool readAverages(const tsl2585_gain_t *expected_gains, float *averages,
                  bool *saturated) {
  uint32_t sums[3] = {0, 0, 0};
  for (uint8_t channel = 0; channel < 3; channel++) {
    saturated[channel] = false;
  }

  // Discard the first result after every configuration change.
  tsl2585_data_t data;
  if (!readFreshData(&data)) {
    Serial.println(F("Could not read the first post-change measurement."));
    return false;
  }

  for (uint8_t sample = 0; sample < SAMPLES_PER_LEVEL; sample++) {
    if (!readFreshData(&data)) {
      Serial.println(F("Could not read a measurement during averaging."));
      return false;
    }
    for (uint8_t channel = 0; channel < 3; channel++) {
      if (channelGain(&data, (tsl2585_channel_t)channel) !=
          expected_gains[channel]) {
        Serial.print(F("Sensor reported the wrong hardware gain for "));
        printChannel((tsl2585_channel_t)channel);
        Serial.println();
        return false;
      }
      sums[channel] += channelCount(&data, (tsl2585_channel_t)channel);
      if (channelSaturated(&data, (tsl2585_channel_t)channel)) {
        saturated[channel] = true;
      }
    }
  }

  for (uint8_t channel = 0; channel < 3; channel++) {
    averages[channel] = sums[channel] / (float)SAMPLES_PER_LEVEL;
  }
  return true;
}

bool updateTrend(trend_t *trend, float count, float decrease_fraction,
                 float increase_fraction, float minimum_change) {
  if (!trend->started) {
    trend->started = true;
    trend->first = count;
    trend->previous = count;
    trend->last = count;
    trend->levels = 1;
    return true;
  }

  float decrease_margin = trend->previous * decrease_fraction;
  if (decrease_margin < minimum_change) {
    decrease_margin = minimum_change;
  }
  if (count + decrease_margin < trend->previous) {
    return false;
  }

  float increase_margin = trend->previous * increase_fraction;
  if (increase_margin < minimum_change) {
    increase_margin = minimum_change;
  }
  if (count >= trend->previous + increase_margin) {
    trend->clear_increases++;
  }
  trend->previous = count;
  trend->last = count;
  trend->levels++;
  return true;
}

bool gainTrendPassed(const trend_t *trend) {
  if (trend->levels < 3 || trend->clear_increases < 2) {
    return false;
  }
  float required_growth = trend->first * 0.50F;
  if (required_growth < 8) {
    required_growth = 8;
  }
  return trend->last >= trend->first + required_growth;
}

bool integrationTrendPassed(const trend_t *trend) {
  if (trend->levels != 18 || trend->clear_increases < 8) {
    return false;
  }
  float required_growth = trend->first * 4;
  if (required_growth < 20) {
    required_growth = 20;
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
  Serial.println(F("ALL GAIN AND INTEGRATION TIME TESTS PASSED"));
  while (true) {
    delay(100);
  }
}
