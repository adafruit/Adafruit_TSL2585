/*!
 * @file tsl2585_fulltest.ino
 *
 * Demonstrate TSL2585 device information, configuration, and measurements.
 * Adjust the settings below to explore the sensor's features.
 *
 * Written by Limor Fried and the Adafruit team for Adafruit Industries.
 * MIT license, all text above must be included in any redistribution.
 */

#include <Adafruit_TSL2585.h>

Adafruit_TSL2585 tsl2585;

// Common settings to adjust.
const float integrationTimeMs = 50; // 0.25 ms to 90 ms in 0.25 ms steps
const bool useAutomaticGain = true;
const tsl2585_gain_t maximumAutomaticGain = TSL2585_GAIN_4096X;
const uint8_t calibrationInterval = 1; // Run AGC every sequencer round

// These gains are used when useAutomaticGain is false.
const tsl2585_gain_t photopicGain = TSL2585_GAIN_128X;
const tsl2585_gain_t infraredGain = TSL2585_GAIN_128X;
const tsl2585_gain_t uvaGain = TSL2585_GAIN_128X;

// Optional ALS threshold settings. The interrupt stays disabled in this example
// because using the INT pin requires an external pull-up and wiring.
const tsl2585_channel_t thresholdChannel = TSL2585_CHANNEL_PHOTOPIC;
const uint32_t lowThreshold = 100;
const uint32_t highThreshold = 50000;
const uint8_t thresholdPersistence = 1; // 0 through 15 measurements

void setup() {
  Serial.begin(115200);
  // Wait for the Serial Monitor to open on native USB boards.
  // Remove this while (!Serial) loop to run without a USB connection.
  while (!Serial) {
    delay(10);
  }
  delay(250);

  Serial.println(F("Adafruit TSL2585 full test"));

  if (!tsl2585.begin()) {
    haltWithMessage(
        F("Could not find a TSL2585. Check the wiring and I2C address."));
  }

  Serial.println(F("\n--- Device information ---"));
  Serial.print(F("Device ID: 0x"));
  Serial.println(tsl2585.getDeviceID(), HEX);
  Serial.print(F("Revision ID: 0x"));
  Serial.println(tsl2585.getRevisionID(), HEX);
  Serial.print(F("Auxiliary ID: 0x"));
  Serial.println(tsl2585.getAuxiliaryID(), HEX);
  Serial.print(F("Factory UVA calibration byte: "));
  Serial.println(tsl2585.getUVCalibration());

  Serial.println(F("\n--- Integration time ---"));
  if (!tsl2585.setIntegrationTime(integrationTimeMs)) {
    haltWithMessage(F("Could not set the integration time."));
  }
  Serial.print(F("Integration time: "));
  Serial.print(tsl2585.getIntegrationTime(), 2);
  Serial.println(F(" ms"));

  Serial.println(F("\n--- Gain control ---"));
  if (useAutomaticGain) {
    // The advanced setters below do not change the measurement state. Stop ALS
    // while selecting the AGC ceiling and how often AGC runs.
    if (!tsl2585.enable(false)) {
      haltWithMessage(F("Could not stop measurements."));
    }
    if (!tsl2585.setMaximumGain(maximumAutomaticGain)) {
      haltWithMessage(F("Could not set the maximum automatic gain."));
    }
    if (!tsl2585.setCalibrationInterval(calibrationInterval)) {
      haltWithMessage(F("Could not set the calibration schedule."));
    }
    if (!tsl2585.enableAGC(true)) {
      haltWithMessage(F("Could not enable automatic gain control."));
    }

    Serial.print(F("Automatic gain control enabled, maximum gain "));
    printGain(maximumAutomaticGain);
    Serial.print(F(", calibration every "));
    Serial.print(calibrationInterval);
    Serial.println(F(" sequencer round(s)"));
  } else {
    if (!tsl2585.enableAGC(false)) {
      haltWithMessage(F("Could not disable automatic gain control."));
    }
    if (!tsl2585.setGain(TSL2585_CHANNEL_PHOTOPIC, photopicGain) ||
        !tsl2585.setGain(TSL2585_CHANNEL_IR, infraredGain) ||
        !tsl2585.setGain(TSL2585_CHANNEL_UVA, uvaGain)) {
      haltWithMessage(F("Could not set the manual gains."));
    }

    Serial.print(F("Photopic gain: "));
    printGain(tsl2585.getGain(TSL2585_CHANNEL_PHOTOPIC));
    Serial.print(F("   Infrared gain: "));
    printGain(tsl2585.getGain(TSL2585_CHANNEL_IR));
    Serial.print(F("   UVA gain: "));
    printGain(tsl2585.getGain(TSL2585_CHANNEL_UVA));
    Serial.println();
  }

  Serial.println(F("\n--- ALS threshold interrupt settings ---"));
  // Thresholds are 24-bit values from 0 through 16,777,215. Persistence is
  // 0 through 15 consecutive out-of-range measurements.
  if (!tsl2585.setALSThresholds(thresholdChannel, lowThreshold, highThreshold,
                                thresholdPersistence)) {
    haltWithMessage(F("Could not set the ALS thresholds."));
  }
  if (!tsl2585.enableALSInterrupt(false)) {
    haltWithMessage(F("Could not disable the ALS interrupt output."));
  }
  if (!tsl2585.clearALSInterrupt()) {
    haltWithMessage(F("Could not clear the ALS interrupt status."));
  }
  Serial.print(F("Threshold channel: "));
  printChannel(thresholdChannel);
  Serial.print(F("   Low: "));
  Serial.print(lowThreshold);
  Serial.print(F("   High: "));
  Serial.print(highThreshold);
  Serial.print(F("   Persistence: "));
  Serial.println(thresholdPersistence);
  Serial.println(F("External ALS interrupt output disabled"));

  Serial.println(F("\n--- GPIO input ---"));
  if (!tsl2585.enableGPIOInput(true)) {
    haltWithMessage(F("Could not enable the GPIO input."));
  }
  Serial.print(F("GPIO input is currently "));
  Serial.println(tsl2585.readGPIOInput() ? F("high") : F("low"));
  // setGPIOOutput(true) releases the open-drain output and
  // setGPIOOutput(false) pulls it low. Do not use output mode until you have
  // checked that it is safe for the circuit connected to the GPIO pin.

  // Uncomment this only when you need to replace begin()'s recommended
  // register setup with raw sequencer, SMUX, and result-format settings.
  // configureAdvancedSettings();

  Serial.println(F("\n--- Measurements ---"));
}

void loop() {
  if (!tsl2585.dataReady()) {
    delay(100);
    return;
  }

  tsl2585_data_t data;
  if (tsl2585.readData(&data)) {
    Serial.print(F("Photopic 1x: "));
    Serial.print(data.photopic_1x, 1);
    Serial.print(F(" ("));
    printGain(data.photopic_gain);
    Serial.print(F(")   Infrared 1x: "));
    Serial.print(data.infrared_1x, 1);
    Serial.print(F(" ("));
    printGain(data.infrared_gain);
    Serial.print(F(")   UVA 1x: "));
    Serial.print(data.uva_1x, 1);
    Serial.print(F(" ("));
    printGain(data.uva_gain);
    Serial.print(F(")   Saturated: "));
    if (data.photopic_saturated || data.infrared_saturated ||
        data.uva_saturated) {
      Serial.println(F("yes"));
    } else {
      Serial.println(F("no"));
    }
  }

  delay(100);
}

void haltWithMessage(const __FlashStringHelper *message) {
  Serial.print(F("Stopped: "));
  Serial.println(message);
  while (true) {
    delay(10);
  }
}

void printChannel(tsl2585_channel_t channel) {
  switch (channel) {
  case TSL2585_CHANNEL_PHOTOPIC:
    Serial.print(F("photopic"));
    break;
  case TSL2585_CHANNEL_IR:
    Serial.print(F("infrared"));
    break;
  case TSL2585_CHANNEL_UVA:
    Serial.print(F("UVA"));
    break;
  default:
    Serial.print(F("unknown"));
    break;
  }
}

void printGain(tsl2585_gain_t gain) {
  switch (gain) {
  case TSL2585_GAIN_0_5X:
    Serial.print(F("0.5x"));
    break;
  case TSL2585_GAIN_1X:
    Serial.print(F("1x"));
    break;
  case TSL2585_GAIN_2X:
    Serial.print(F("2x"));
    break;
  case TSL2585_GAIN_4X:
    Serial.print(F("4x"));
    break;
  case TSL2585_GAIN_8X:
    Serial.print(F("8x"));
    break;
  case TSL2585_GAIN_16X:
    Serial.print(F("16x"));
    break;
  case TSL2585_GAIN_32X:
    Serial.print(F("32x"));
    break;
  case TSL2585_GAIN_64X:
    Serial.print(F("64x"));
    break;
  case TSL2585_GAIN_128X:
    Serial.print(F("128x"));
    break;
  case TSL2585_GAIN_256X:
    Serial.print(F("256x"));
    break;
  case TSL2585_GAIN_512X:
    Serial.print(F("512x"));
    break;
  case TSL2585_GAIN_1024X:
    Serial.print(F("1024x"));
    break;
  case TSL2585_GAIN_2048X:
    Serial.print(F("2048x"));
    break;
  case TSL2585_GAIN_4096X:
    Serial.print(F("4096x"));
    break;
  default:
    Serial.print(F("unknown"));
    break;
  }
}

void configureAdvancedSettings() {
  // These raw setters do not stop or restart ALS. Stop measurements first so
  // every register is applied together before the next conversion.
  if (!tsl2585.enable(false)) {
    haltWithMessage(F("Could not stop measurements."));
  }

  // MEAS_MODE0 and MEAS_MODE1 select the result format and bit alignment.
  if (!tsl2585.setResultFormat(TSL2585_MEAS_MODE0_FULL_COUNTS,
                               TSL2585_MEAS_MODE1_MSB_POSITION_12)) {
    haltWithMessage(F("Could not set the result format."));
  }

  // The default sample time is 179 register counts, or 250 us. Integration is
  // 1 through 2048 samples; 200 samples at 250 us gives 50 ms.
  if (!tsl2585.setSampleTime(TSL2585_SAMPLE_TIME_250_US) ||
      !tsl2585.setIntegrationSamples(TSL2585_DEFAULT_ALS_SAMPLE_COUNT)) {
    haltWithMessage(F("Could not set the raw sample timing."));
  }
  int16_t sampleTimeRegisterValue = tsl2585.getSampleTime();
  uint16_t integrationSamples = tsl2585.getIntegrationSamples();
  if (sampleTimeRegisterValue < 0 || integrationSamples == 0) {
    haltWithMessage(F("Could not read the raw sample timing."));
  }
  Serial.print(F("Sample-time register value: "));
  Serial.print(sampleTimeRegisterValue);
  Serial.print(F("   Integration samples: "));
  Serial.println(integrationSamples);

  // Each nibble is a four-step pattern. These values enable ALS and interrupt
  // persistence on step 0 without flicker, residual, VSYNC, or wait steps.
  if (!tsl2585.setSequencer(TSL2585_SEQUENCER_DISABLED,
                            TSL2585_SEQUENCER_STEP0,
                            TSL2585_SEQUENCER_STEP0,
                            TSL2585_SEQUENCER_DISABLED,
                            TSL2585_SEQUENCER_DISABLED)) {
    haltWithMessage(F("Could not set the raw sequencer patterns."));
  }

  if (!tsl2585.setMaximumGain(maximumAutomaticGain) ||
      !tsl2585.setGainValue(TSL2585_CHANNEL_PHOTOPIC, photopicGain) ||
      !tsl2585.setGainValue(TSL2585_CHANNEL_IR, infraredGain) ||
      !tsl2585.setGainValue(TSL2585_CHANNEL_UVA, uvaGain)) {
    haltWithMessage(F("Could not set the raw gain fields."));
  }

  // The recommended SMUX maps photopic, infrared, and UVA photodiodes to the
  // three result channels.
  if (!tsl2585.setSMUX(TSL2585_RECOMMENDED_SMUX_L,
                       TSL2585_RECOMMENDED_SMUX_H)) {
    haltWithMessage(F("Could not set the photodiode mapping."));
  }

  if (!tsl2585.setCalibrationInterval(calibrationInterval)) {
    haltWithMessage(F("Could not set the calibration schedule."));
  }

  // enableAGC() completes the configuration and restarts measurements.
  if (!tsl2585.enableAGC(useAutomaticGain)) {
    haltWithMessage(F("Could not finish the advanced configuration."));
  }
}
