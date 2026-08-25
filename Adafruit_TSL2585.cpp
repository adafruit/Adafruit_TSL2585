/*!
 * @file Adafruit_TSL2585.cpp
 *
 * @mainpage Adafruit TSL2585 ambient light, infrared, and UVA sensor library
 *
 * @section intro_sec Introduction
 *
 * Arduino driver for the ams OSRAM TSL2585 / TSL25853 optical sensor.
 *
 * @section author Author
 *
 * Written by Limor Fried and the Adafruit team for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 */

#include "Adafruit_TSL2585.h"

/*! @brief Construct a new TSL2585 driver. */
Adafruit_TSL2585::Adafruit_TSL2585() {}

/*! @brief Release the allocated I2C device. */
Adafruit_TSL2585::~Adafruit_TSL2585() {
  if (i2c_dev != nullptr) {
    delete i2c_dev;
  }
}

/*!
 * @brief Initialize the I2C connection and configure three-channel ALS.
 * @param i2c_addr The sensor I2C address. The TSL25853 uses 0x39.
 * @param wire The I2C bus to use.
 * @return True when the device was identified and configured.
 */
bool Adafruit_TSL2585::begin(uint8_t i2c_addr, TwoWire* wire) {
  if (i2c_dev != nullptr) {
    delete i2c_dev;
    i2c_dev = nullptr;
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
  if (!i2c_dev->begin()) {
    return false;
  }

  // ID confirms that the responding I2C device is a TSL2585.
  if (getDeviceID() != TSL2585_DEVICE_ID) {
    return false;
  }

  if (!reset()) {
    return false;
  }

  // UV_CALIB supplies the factory correction used for calibrated UVA results.
  _uv_calibration = getUVCalibration();
  if (_uv_calibration == 0) {
    return false;
  }

  // ENABLE register Figure 20 says to set PON only after configuration.
  if (!enable(false)) {
    return false;
  }

  // Configure unscaled 16-bit full counts with a 250 us sample period and
  // 200 samples for a 50 ms integration. Sequencer step 0 runs ALS and
  // interrupt persistence without flicker, residual, VSYNC, or wait
  // measurements. Application note Table 1 routes the photopic, IR, and UVA
  // diodes to modulators 0, 1, and 2. Each channel starts at 128x gain, with
  // AGC allowed to select gains up to 4096x before every sequencer round.
  if (!setResultFormat(TSL2585_MEAS_MODE0_FULL_COUNTS,
                       TSL2585_MEAS_MODE1_MSB_POSITION_12) ||
      !setSampleTime(TSL2585_SAMPLE_TIME_250_US) ||
      !setIntegrationSamples(TSL2585_DEFAULT_ALS_SAMPLE_COUNT) ||
      !setSequencer(TSL2585_SEQUENCER_DISABLED, TSL2585_SEQUENCER_STEP0,
                    TSL2585_SEQUENCER_STEP0, TSL2585_SEQUENCER_DISABLED,
                    TSL2585_SEQUENCER_DISABLED) ||
      !setCalibrationInterval(TSL2585_CALIBRATION_EVERY_ROUND) ||
      !setMaximumGain(TSL2585_GAIN_4096X) ||
      !setGainValue(TSL2585_CHANNEL_PHOTOPIC, TSL2585_GAIN_128X) ||
      !setGainValue(TSL2585_CHANNEL_IR, TSL2585_GAIN_128X) ||
      !setGainValue(TSL2585_CHANNEL_UVA, TSL2585_GAIN_128X) ||
      !setSMUX(TSL2585_RECOMMENDED_SMUX_L, TSL2585_RECOMMENDED_SMUX_H)) {
    return false;
  }

  // Enable AGC for all three channels before starting measurements.
  return enableAGC(true);
}

/*!
 * @brief Reset the sensor registers to their power-on values.
 *
 * Datasheet Figure 54 requires ENABLE.PON to be set before writing
 * CONTROL.SOFT_RESET. The reset then initializes the device in the same way as
 * a hardware reset.
 *
 * @return True when the reset command was written successfully.
 */
bool Adafruit_TSL2585::reset() {
  if (i2c_dev == nullptr) {
    return false;
  }

  // ENABLE.PON starts the oscillator required to execute a software reset.
  Adafruit_BusIO_Register enable_reg(i2c_dev, TSL2585_REG_ENABLE);
  Adafruit_BusIO_RegisterBits power_on_bit(&enable_reg, 1,
                                           TSL2585_ENABLE_PON_BIT);
  if (!power_on_bit.write(1)) {
    return false;
  }
  delay(TSL2585_STARTUP_DELAY_MS);

  // CONTROL.SOFT_RESET restores the device's power-on register values.
  Adafruit_BusIO_Register control_reg(i2c_dev, TSL2585_REG_CONTROL);
  Adafruit_BusIO_RegisterBits soft_reset_bit(&control_reg, 1,
                                             TSL2585_CONTROL_SOFT_RESET_BIT);
  if (!soft_reset_bit.write(1)) {
    return false;
  }

  // The device temporarily rejects I2C transactions while it initializes.
  delay(TSL2585_RESET_DELAY_MS);
  return true;
}

/*!
 * @brief Enable or disable continuous ALS measurements.
 *
 * Datasheet Figure 16 sequences PON from SLEEP to IDLE before AEN starts ALS.
 * Figure 20 identifies PON as the oscillator enable and AEN as the ALS enable.
 *
 * @param enabled True to enable measurements, false to disable them.
 * @return True when the register writes succeeded.
 */
bool Adafruit_TSL2585::enable(bool enabled) {
  if (i2c_dev == nullptr) {
    return false;
  }

  Adafruit_BusIO_Register enable_reg(i2c_dev, TSL2585_REG_ENABLE);
  Adafruit_BusIO_RegisterBits power_on_bit(&enable_reg, 1,
                                           TSL2585_ENABLE_PON_BIT);
  Adafruit_BusIO_RegisterBits als_enable_bit(&enable_reg, 1,
                                             TSL2585_ENABLE_AEN_BIT);
  if (!enabled) {
    // Reverse Figure 16's sequence: stop ALS before stopping its oscillator.
    if (!als_enable_bit.write(0)) {
      return false;
    }
    return power_on_bit.write(0);
  }

  // Figures 16 and 20 start the oscillator and enter IDLE before enabling ALS.
  if (!power_on_bit.write(1)) {
    return false;
  }

  // No PON-to-AEN delay is specified; allow a conservative startup margin.
  delay(TSL2585_STARTUP_DELAY_MS);
  return als_enable_bit.write(1);
}

/*!
 * @brief Set ALS integration time for the register-result path.
 * The time is rounded to the nearest whole sample using the sample period
 * currently stored in SAMPLE_TIME. See TSL2585 datasheet Figures 23 through
 * 26.
 *
 * @param milliseconds Integration time from 0.25 ms through 90 ms that rounds
 * to 1 through 2048 samples at the current sample period.
 * @return True when the sample-time register was read and the resulting sample
 * count was in range and written successfully.
 */
bool Adafruit_TSL2585::setIntegrationTime(float milliseconds) {
  if (i2c_dev == nullptr || milliseconds < 0.25F || milliseconds > 90.0F) {
    return false;
  }

  int16_t sample_time_register_value = getSampleTime();
  if (sample_time_register_value < 0) {
    return false;
  }

  float sample_period_ms = (sample_time_register_value + 1) *
                           TSL2585_MODULATOR_CLOCK_PERIOD_US / 1000.0F;
  float requested_sample_count = milliseconds / sample_period_ms;
  if (requested_sample_count < 0.5F ||
      requested_sample_count >= TSL2585_MAX_INTEGRATION_SAMPLES + 0.5F) {
    return false;
  }
  uint16_t sample_count = (uint16_t)(requested_sample_count + 0.5F);

  // Figures 25 and 26 do not define a live update for this two-byte field.
  // Stop ALS so the next conversion uses one complete integration setting.
  if (!enable(false)) {
    return false;
  }
  bool success = setIntegrationSamples(sample_count);

  if (!enable(true)) {
    return false;
  }
  return success;
}

/*!
 * @brief Get the configured ALS integration time.
 * @return Integration time in milliseconds, or 0 if the register read failed.
 */
float Adafruit_TSL2585::getIntegrationTime() {
  if (i2c_dev == nullptr) {
    return 0;
  }

  int16_t sample_time_register_value = getSampleTime();
  uint16_t sample_count = getIntegrationSamples();
  if (sample_time_register_value < 0 || sample_count == 0) {
    return 0;
  }

  float sample_period_ms = (sample_time_register_value + 1) *
                           TSL2585_MODULATOR_CLOCK_PERIOD_US / 1000.0F;
  return sample_count * sample_period_ms;
}

/*!
 * @brief Set the starting gain for one optical channel.
 *
 * This safely stops ALS, writes the selected step-0 modulator gain, and starts
 * ALS again. Photopic and IR share MEAS_SEQR_STEP0_MOD_GAINX_L; UVA uses
 * MEAS_SEQR_STEP0_MOD_GAINX_H. See TSL2585 datasheet Figures 63 and 64. When
 * AGC is enabled, the sensor may replace this value before a measurement. Use
 * the gain fields returned by readData() for the gain actually used.
 *
 * @param channel The photopic, infrared, or UVA channel.
 * @param gain Gain from 0.5x through 4096x.
 * @return True when the register writes succeeded.
 */
bool Adafruit_TSL2585::setGain(tsl2585_channel_t channel, tsl2585_gain_t gain) {
  if (i2c_dev == nullptr) {
    return false;
  }

  // Figures 63 and 64 do not define when a running cycle adopts a new gain.
  // Stop ALS so the next conversion uses the requested gain from its start.
  if (!enable(false)) {
    return false;
  }

  bool success = setGainValue(channel, gain);

  if (!enable(true)) {
    return false;
  }
  return success;
}

/*!
 * @brief Get the configured manual gain for one channel.
 * @param channel The photopic, infrared, or UVA channel.
 * @return The configured gain, or 0.5x if the register read failed.
 */
tsl2585_gain_t Adafruit_TSL2585::getGain(tsl2585_channel_t channel) {
  if (i2c_dev == nullptr) {
    return TSL2585_GAIN_0_5X;
  }

  uint16_t gain_register_address = TSL2585_REG_STEP0_GAIN_L;
  if (channel == TSL2585_CHANNEL_UVA) {
    gain_register_address = TSL2585_REG_STEP0_GAIN_H;
  }

  // A direct RegisterBits read cannot distinguish an I2C failure from 0x0F.
  tsl2585_gain_register_t gain_register_value;
  Adafruit_BusIO_Register gain_reg(i2c_dev, gain_register_address);
  if (!gain_reg.read((uint8_t*)&gain_register_value)) {
    return TSL2585_GAIN_0_5X;
  }

  if (channel == TSL2585_CHANNEL_IR) {
    return (tsl2585_gain_t)gain_register_value.upper_nibble;
  }
  return (tsl2585_gain_t)gain_register_value.lower_nibble;
}

/*!
 * @brief Enable or disable automatic gain control.
 *
 * Enabling selects both predictive AGC and analog-saturation AGC for the
 * library's active sequencer step 0. Predictive AGC chooses a gain near its
 * target level, while saturation AGC can reduce that gain and repeat an AGC
 * measurement if analog saturation remains. See the TSL2585 application note
 * section 2.5 and datasheet Figures 46, 74, 76, and 80.
 *
 * MOD_CALIB_CFG0 controls how often AGC runs. The library default is every
 * sequencer round. Use setCalibrationInterval() while ALS is disabled to
 * select another schedule. That schedule is shared with auto-zero calibration,
 * which can add measurement time.
 *
 * @param enabled True to enable both AGC methods, false for manual gain.
 * @return True when all register writes succeeded.
 */
bool Adafruit_TSL2585::enableAGC(bool enabled) {
  if (i2c_dev == nullptr) {
    return false;
  }

  // The AGC registers do not define coherent live-update behavior. Stop ALS so
  // the next measurement starts with one complete AGC configuration.
  if (!enable(false)) {
    return false;
  }

  // CFG4 selects per-round or per-step calibration. Figure 46 requires
  // per-round calibration when AGC is enabled.
  Adafruit_BusIO_Register cfg4_reg(i2c_dev, TSL2585_REG_CFG4);
  Adafruit_BusIO_RegisterBits calibration_per_step_bit(
      &cfg4_reg, 1, TSL2585_CFG4_CALIBRATION_PER_STEP_BIT);

  // STEP1_SMUX_H selects the sequencer steps that use saturation AGC.
  Adafruit_BusIO_Register step1_smux_high_reg(i2c_dev,
                                              TSL2585_REG_STEP1_SMUX_H);
  Adafruit_BusIO_RegisterBits saturation_agc_pattern_bits(
      &step1_smux_high_reg, TSL2585_AGC_PATTERN_BITS,
      TSL2585_AGC_PATTERN_SHIFT);

  // STEP2_SMUX_H selects the sequencer steps that use predictive AGC.
  Adafruit_BusIO_Register step2_smux_high_reg(i2c_dev,
                                              TSL2585_REG_STEP2_SMUX_H);
  Adafruit_BusIO_RegisterBits predictive_agc_pattern_bits(
      &step2_smux_high_reg, TSL2585_AGC_PATTERN_BITS,
      TSL2585_AGC_PATTERN_SHIFT);

  // MOD_CALIB_CFG2 links the selected AGC methods to the calibration schedule.
  Adafruit_BusIO_Register mod_calib_cfg2_reg(i2c_dev,
                                             TSL2585_REG_MOD_CALIB_CFG2);
  Adafruit_BusIO_RegisterBits agc_enable_bit(&mod_calib_cfg2_reg, 1,
                                             TSL2585_MOD_CALIB_AGC_ENABLE_BIT);

  bool success;
  if (enabled) {
    success = calibration_per_step_bit.write(0) &&
              saturation_agc_pattern_bits.write(TSL2585_AGC_STEP0_PATTERN) &&
              predictive_agc_pattern_bits.write(TSL2585_AGC_STEP0_PATTERN) &&
              agc_enable_bit.write(1);
  } else {
    success = agc_enable_bit.write(0) &&
              saturation_agc_pattern_bits.write(TSL2585_SEQUENCER_DISABLED) &&
              predictive_agc_pattern_bits.write(TSL2585_SEQUENCER_DISABLED);
  }

  if (!enable(true)) {
    return false;
  }
  return success;
}

/*!
 * @brief Check whether a new ALS measurement is available.
 * @return True when ALS_DATA_VALID is set.
 */
bool Adafruit_TSL2585::dataReady() {
  if (i2c_dev == nullptr) {
    return false;
  }

  // A direct RegisterBits read cannot report an I2C failure.
  uint8_t status2;
  Adafruit_BusIO_Register status2_reg(i2c_dev, TSL2585_REG_STATUS2);
  if (!status2_reg.read(&status2)) {
    return false;
  }
  return bitRead(status2, TSL2585_STATUS2_DATA_VALID_BIT);
}

/*!
 * @brief Read the latest coherent three-channel measurement.
 *
 * STATUS2 is read before ALS_STATUS as required by the device. The complete
 * ALS_STATUS through ALS_STATUS3 block is then read in one transaction so the
 * channel values, saturation flags, and actual gains belong to one cycle. The
 * actual gains are then used to calculate typical 1x-equivalent counts.
 *
 * @param data Destination for the result.
 * @return True when the register data was read successfully.
 */
bool Adafruit_TSL2585::readData(tsl2585_data_t* data) {
  if (i2c_dev == nullptr || data == nullptr) {
    return false;
  }

  // STATUS2 reports digital saturation for the current result.
  uint8_t device_status2;
  Adafruit_BusIO_Register device_status2_reg(i2c_dev, TSL2585_REG_STATUS2);
  if (!device_status2_reg.read(&device_status2)) {
    return false;
  }

  // ALS_STATUS starts the atomic status, counts, and actual-gains result frame.
  tsl2585_result_buffer_t result;
  Adafruit_BusIO_Register als_result_reg(i2c_dev, TSL2585_REG_ALS_STATUS,
                                         sizeof(result.buffer), LSBFIRST);
  if (!als_result_reg.read(result.buffer, sizeof(result.buffer))) {
    return false;
  }

  uint8_t als_status = result.registers.als_status;
  data->photopic = result.registers.als_data0;
  data->infrared = result.registers.als_data1;
  data->uva = result.registers.als_data2;
  data->uva_calibrated = calibrateUVA(data->uva);

  data->photopic_gain =
      (tsl2585_gain_t)result.registers.als_data01_gain_status.lower_nibble;
  data->infrared_gain =
      (tsl2585_gain_t)result.registers.als_data01_gain_status.upper_nibble;
  data->uva_gain =
      (tsl2585_gain_t)result.registers.als_data2_gain_status.lower_nibble;

  data->photopic_1x = normalizeGainTo1x(data->photopic, data->photopic_gain);
  data->infrared_1x = normalizeGainTo1x(data->infrared, data->infrared_gain);
  data->uva_1x = normalizeGainTo1x(data->uva_calibrated, data->uva_gain);

  bool als_digital_saturation =
      (device_status2 & TSL2585_STATUS2_DIGITAL_SATURATION) != 0;
  data->photopic_saturated =
      als_digital_saturation ||
      (als_status & TSL2585_ALS_STATUS_PHOTOPIC_SATURATION) != 0;
  data->infrared_saturated =
      als_digital_saturation ||
      (als_status & TSL2585_ALS_STATUS_IR_SATURATION) != 0;
  data->uva_saturated = als_digital_saturation ||
                        (als_status & TSL2585_ALS_STATUS_UVA_SATURATION) != 0;

  return true;
}

/*!
 * @brief Apply the factory OTP correction to a raw UVA count.
 * @param raw_uva Raw UVA full counts from the coherent result block.
 * @return Factory-corrected UVA counts.
 */
float Adafruit_TSL2585::calibrateUVA(uint16_t raw_uva) {
  float uv_calibration_divisor = 1.0F - ((_uv_calibration - 127.0F) / 100.0F);
  if (uv_calibration_divisor <= 0.0F) {
    return raw_uva;
  }
  return raw_uva / uv_calibration_divisor;
}

/*!
 * @brief Normalize counts to the typical response at 1x gain.
 *
 * The high gain stages are not exact powers of two, so use the typical gain
 * multipliers derived from the TSL2585 datasheet Figure 6 instead of the
 * nominal gain labels. The result remains in counts at the configured
 * integration time; it is not lux or irradiance.
 *
 * @param counts Raw or factory-corrected counts at the reported gain.
 * @param gain Gain reported with the coherent measurement.
 * @return Typical 1x-equivalent counts.
 */
float Adafruit_TSL2585::normalizeGainTo1x(float counts, tsl2585_gain_t gain) {
  float typical_response_multiplier = 1.0F;

  switch (gain) {
    case TSL2585_GAIN_0_5X:
      typical_response_multiplier = 0.49713F;
      break;
    case TSL2585_GAIN_1X:
      typical_response_multiplier = 1.0F;
      break;
    case TSL2585_GAIN_2X:
      typical_response_multiplier = 1.96681F;
      break;
    case TSL2585_GAIN_4X:
      typical_response_multiplier = 3.90448F;
      break;
    case TSL2585_GAIN_8X:
      typical_response_multiplier = 7.97489F;
      break;
    case TSL2585_GAIN_16X:
      typical_response_multiplier = 15.53952F;
      break;
    case TSL2585_GAIN_32X:
      typical_response_multiplier = 31.0401F;
      break;
    case TSL2585_GAIN_64X:
      typical_response_multiplier = 61.61692F;
      break;
    case TSL2585_GAIN_128X:
      typical_response_multiplier = 123.85F;
      break;
    case TSL2585_GAIN_256X:
      typical_response_multiplier = 239.0305F;
      break;
    case TSL2585_GAIN_512X:
      typical_response_multiplier = 470.63F;
      break;
    case TSL2585_GAIN_1024X:
      typical_response_multiplier = 918.967F;
      break;
    case TSL2585_GAIN_2048X:
      typical_response_multiplier = 1741.331F;
      break;
    case TSL2585_GAIN_4096X:
      typical_response_multiplier = 3139.5975F;
      break;
    default:
      break;
  }

  return counts / typical_response_multiplier;
}

/*!
 * @brief Configure ALS interrupt thresholds and persistence.
 * @param channel Optical channel used for threshold comparisons.
 * @param low_threshold Inclusive low threshold from 0 through 0xFFFFFF.
 * @param high_threshold Inclusive high threshold from 0 through 0xFFFFFF.
 * @param persistence Consecutive out-of-range results required, from 0 to 15.
 * @return True when the numeric settings were valid and all writes succeeded.
 */
bool Adafruit_TSL2585::setALSThresholds(tsl2585_channel_t channel,
                                        uint32_t low_threshold,
                                        uint32_t high_threshold,
                                        uint8_t persistence) {
  if (i2c_dev == nullptr || low_threshold > high_threshold ||
      high_threshold > TSL2585_MAX_INTERRUPT_THRESHOLD ||
      persistence > TSL2585_MAX_INTERRUPT_PERSISTENCE) {
    return false;
  }

  // Figures 30, 31, and 47 spread interrupt setup across multiple writes.
  // Stop ALS so no result is tested against a partly updated configuration.
  if (!enable(false)) {
    return false;
  }

  // ALS_THRESHOLD_LOW and ALS_THRESHOLD_HIGH hold the 24-bit window limits.
  Adafruit_BusIO_Register low_threshold_reg(
      i2c_dev, TSL2585_REG_ALS_THRESHOLD_LOW, 3, LSBFIRST);
  Adafruit_BusIO_Register high_threshold_reg(
      i2c_dev, TSL2585_REG_ALS_THRESHOLD_HIGH, 3, LSBFIRST);

  // CFG5 selects which optical channel is compared and its persistence count.
  Adafruit_BusIO_Register cfg5_reg(i2c_dev, TSL2585_REG_CFG5);
  Adafruit_BusIO_RegisterBits threshold_channel_bits(
      &cfg5_reg, TSL2585_CFG5_THRESHOLD_CHANNEL_BITS,
      TSL2585_CFG5_THRESHOLD_CHANNEL_SHIFT);
  Adafruit_BusIO_RegisterBits persistence_bits(
      &cfg5_reg, TSL2585_CFG5_PERSISTENCE_BITS, TSL2585_CFG5_PERSISTENCE_SHIFT);

  bool success = low_threshold_reg.write(low_threshold) &&
                 high_threshold_reg.write(high_threshold) &&
                 threshold_channel_bits.write((uint8_t)channel) &&
                 persistence_bits.write(persistence);

  if (!enable(true)) {
    return false;
  }
  return success;
}

/*!
 * @brief Enable or disable ALS threshold events on the open-drain INT pin.
 * @param enabled True to enable the interrupt, false to disable it.
 * @return True when the requested interrupt state was configured.
 */
bool Adafruit_TSL2585::enableALSInterrupt(bool enabled) {
  if (i2c_dev == nullptr) {
    return false;
  }

  // INTENAB enables or disables ALS threshold events as an interrupt source.
  Adafruit_BusIO_Register interrupt_enable_reg(i2c_dev, TSL2585_REG_INTENAB);
  Adafruit_BusIO_RegisterBits als_interrupt_enable_bit(
      &interrupt_enable_reg, 1, TSL2585_INTENAB_AIEN_BIT);
  if (!enabled) {
    return als_interrupt_enable_bit.write(0);
  }

  // CFG3 routes the interrupt signal to the external INT pin.
  Adafruit_BusIO_Register cfg3_reg(i2c_dev, TSL2585_REG_CFG3);
  Adafruit_BusIO_RegisterBits int_pinmap_bits(
      &cfg3_reg, TSL2585_CFG3_INT_PINMAP_BITS, TSL2585_CFG3_INT_PINMAP_SHIFT);

  // VSYNC_GPIO_INT makes INT an active-low output instead of an input.
  Adafruit_BusIO_Register vsync_gpio_int_reg(i2c_dev,
                                             TSL2585_REG_VSYNC_GPIO_INT);
  Adafruit_BusIO_RegisterBits int_input_enable_bit(
      &vsync_gpio_int_reg, 1, TSL2585_INT_INPUT_ENABLE_BIT);
  Adafruit_BusIO_RegisterBits int_invert_bit(&vsync_gpio_int_reg, 1,
                                             TSL2585_INT_INVERT_BIT);
  return int_pinmap_bits.write(TSL2585_CFG3_INT_PINMAP_INTERRUPT) &&
         int_input_enable_bit.write(0) && int_invert_bit.write(0) &&
         als_interrupt_enable_bit.write(1);
}

/*!
 * @brief Check whether an ALS threshold interrupt is pending.
 * @return True when the sensor's ALS interrupt status bit is set.
 */
bool Adafruit_TSL2585::alsInterruptActive() {
  if (i2c_dev == nullptr) {
    return false;
  }

  // A direct RegisterBits read cannot report an I2C failure.
  uint8_t status;
  Adafruit_BusIO_Register status_reg(i2c_dev, TSL2585_REG_STATUS);
  if (!status_reg.read(&status)) {
    return false;
  }
  return bitRead(status, TSL2585_STATUS_AINT_BIT);
}

/*!
 * @brief Clear the pending ALS threshold interrupt.
 * @return True when the write-one-to-clear operation succeeded.
 */
bool Adafruit_TSL2585::clearALSInterrupt() {
  if (i2c_dev == nullptr) {
    return false;
  }

  Adafruit_BusIO_Register status_reg(i2c_dev, TSL2585_REG_STATUS);
  return status_reg.write(TSL2585_STATUS_AINT);
}

/*!
 * @brief Release or pull low the open-drain VSYNC/GPIO output.
 * @param released True to release the output, false to pull it low.
 * @return True when the GPIO routing and output writes succeeded.
 */
bool Adafruit_TSL2585::setGPIOOutput(bool released) {
  if (i2c_dev == nullptr) {
    return false;
  }

  // CFG3 routes the GPIO output register to the external GPIO pin.
  Adafruit_BusIO_Register cfg3_reg(i2c_dev, TSL2585_REG_CFG3);
  Adafruit_BusIO_RegisterBits gpio_pinmap_bits(
      &cfg3_reg, TSL2585_CFG3_GPIO_PINMAP_BITS, TSL2585_CFG3_GPIO_PINMAP_SHIFT);

  // VSYNC_GPIO_INT sets the GPIO direction, polarity, and open-drain state.
  Adafruit_BusIO_Register vsync_gpio_int_reg(i2c_dev,
                                             TSL2585_REG_VSYNC_GPIO_INT);
  Adafruit_BusIO_RegisterBits gpio_invert_bit(&vsync_gpio_int_reg, 1,
                                              TSL2585_GPIO_INVERT_BIT);
  Adafruit_BusIO_RegisterBits gpio_input_enable_bit(
      &vsync_gpio_int_reg, 1, TSL2585_GPIO_INPUT_ENABLE_BIT);
  Adafruit_BusIO_RegisterBits gpio_output_bit(&vsync_gpio_int_reg, 1,
                                              TSL2585_GPIO_OUTPUT_BIT);

  return gpio_pinmap_bits.write(TSL2585_CFG3_GPIO_PINMAP_OUTPUT) &&
         gpio_invert_bit.write(0) && gpio_input_enable_bit.write(0) &&
         gpio_output_bit.write(released ? 1 : 0);
}

/*!
 * @brief Enable or disable the VSYNC/GPIO pin input.
 * @param enabled True to enable the input, false to disable it.
 * @return True when the GPIO direction writes succeeded.
 */
bool Adafruit_TSL2585::enableGPIOInput(bool enabled) {
  if (i2c_dev == nullptr) {
    return false;
  }

  // VSYNC_GPIO_INT releases the open-drain output before enabling input mode.
  Adafruit_BusIO_Register vsync_gpio_int_reg(i2c_dev,
                                             TSL2585_REG_VSYNC_GPIO_INT);
  Adafruit_BusIO_RegisterBits gpio_input_enable_bit(
      &vsync_gpio_int_reg, 1, TSL2585_GPIO_INPUT_ENABLE_BIT);
  Adafruit_BusIO_RegisterBits gpio_output_bit(&vsync_gpio_int_reg, 1,
                                              TSL2585_GPIO_OUTPUT_BIT);

  return gpio_output_bit.write(1) &&
         gpio_input_enable_bit.write(enabled ? 1 : 0);
}

/*!
 * @brief Read the external state applied to the VSYNC/GPIO pin.
 * @return True when the GPIO input is high, false when it is low.
 */
bool Adafruit_TSL2585::readGPIOInput() {
  if (i2c_dev == nullptr) {
    return false;
  }

  // VSYNC_GPIO_INT reports the logic level currently present on the GPIO pin.
  // A direct RegisterBits read cannot report an I2C failure.
  uint8_t vsync_gpio_int;
  Adafruit_BusIO_Register vsync_gpio_int_reg(i2c_dev,
                                             TSL2585_REG_VSYNC_GPIO_INT);
  if (!vsync_gpio_int_reg.read(&vsync_gpio_int)) {
    return false;
  }
  return bitRead(vsync_gpio_int, TSL2585_GPIO_INPUT_BIT);
}

/*! @return The TSL2585 device identification byte, or 0 on read failure. */
uint8_t Adafruit_TSL2585::getDeviceID() {
  if (i2c_dev == nullptr) {
    return 0;
  }

  uint8_t device_id = 0;
  Adafruit_BusIO_Register id_reg(i2c_dev, TSL2585_REG_ID);
  if (!id_reg.read(&device_id)) {
    return 0;
  }
  return device_id;
}

/*! @return The silicon revision identification byte, or 0 on read failure. */
uint8_t Adafruit_TSL2585::getRevisionID() {
  if (i2c_dev == nullptr) {
    return 0;
  }

  uint8_t revision_id = 0;
  Adafruit_BusIO_Register revision_id_reg(i2c_dev, TSL2585_REG_REV_ID);
  if (!revision_id_reg.read(&revision_id)) {
    return 0;
  }
  return revision_id;
}

/*! @return The auxiliary identification byte, or 0 on read failure. */
uint8_t Adafruit_TSL2585::getAuxiliaryID() {
  if (i2c_dev == nullptr) {
    return 0;
  }

  uint8_t auxiliary_id = 0;
  Adafruit_BusIO_Register auxiliary_id_reg(i2c_dev, TSL2585_REG_AUX_ID);
  if (!auxiliary_id_reg.read(&auxiliary_id)) {
    return 0;
  }
  return auxiliary_id;
}

/*!
 * @brief Read the factory UVA calibration byte.
 * @return The calibration byte, where 127 is nominal, or 0 if the register read
 * failed.
 */
uint8_t Adafruit_TSL2585::getUVCalibration() {
  if (i2c_dev == nullptr) {
    return 0;
  }

  uint8_t calibration = 0;
  Adafruit_BusIO_Register uv_calibration_reg(i2c_dev, TSL2585_REG_UV_CALIB);
  if (!uv_calibration_reg.read(&calibration)) {
    return 0;
  }
  return calibration;
}

/*!
 * @brief Set the raw ALS result format and bit alignment.
 *
 * This advanced setter writes MEAS_MODE0 and MEAS_MODE1 exactly as supplied.
 * MEAS_MODE0 controls ALS scaling and result behavior; MEAS_MODE1 controls the
 * most-significant result-bit position and flicker FIFO metadata. See TSL2585
 * datasheet Figures 21 and 22. Disable ALS before changing these registers;
 * this function does not change PON or AEN.
 *
 * @param mode0 Complete MEAS_MODE0 register value.
 * @param mode1 Complete MEAS_MODE1 register value.
 * @return True when both register writes succeeded.
 */
bool Adafruit_TSL2585::setResultFormat(uint8_t mode0, uint8_t mode1) {
  if (i2c_dev == nullptr) {
    return false;
  }

  // MEAS_MODE0 selects the result width and residual-data mode.
  Adafruit_BusIO_Register meas_mode0_reg(i2c_dev, TSL2585_REG_MEAS_MODE0);

  // MEAS_MODE1 selects the most-significant result-bit position.
  Adafruit_BusIO_Register meas_mode1_reg(i2c_dev, TSL2585_REG_MEAS_MODE1);

  return meas_mode0_reg.write(mode0) && meas_mode1_reg.write(mode1);
}

/*!
 * @brief Set the raw 11-bit modulator sample-time value.
 *
 * SAMPLE_TIME spans SAMPLE_TIME0 and SAMPLE_TIME1. With the default CFG7 clock
 * divider, one count is a 1.388889 us modulator-clock step, so the period is
 * (sample_time_register_value + 1) steps. See TSL2585 datasheet Figures 23 and
 * 24. Disable ALS before changing this field; this function does not change PON
 * or AEN.
 *
 * @param sample_time_register_value Sample-time register value from 0 through
 * 2047.
 * @return True when the value was valid and the register write succeeded.
 */
bool Adafruit_TSL2585::setSampleTime(uint16_t sample_time_register_value) {
  if (i2c_dev == nullptr ||
      sample_time_register_value > TSL2585_MAX_SAMPLE_TIME) {
    return false;
  }

  // SAMPLE_TIME0 sets each modulator sample period.
  Adafruit_BusIO_Register sample_time_reg(i2c_dev, TSL2585_REG_SAMPLE_TIME0, 2,
                                          LSBFIRST);
  return sample_time_reg.write(sample_time_register_value);
}

/*!
 * @brief Read the raw 11-bit modulator sample-time value.
 *
 * SAMPLE_TIME spans SAMPLE_TIME0 and SAMPLE_TIME1. With the default CFG7 clock
 * divider, the sample period is the returned value plus one, multiplied by
 * 1.388889 us. See TSL2585 datasheet Figures 23 and 24.
 *
 * @return The register value from 0 through 2047, or -1 if the register read
 * failed.
 */
int16_t Adafruit_TSL2585::getSampleTime() {
  if (i2c_dev == nullptr) {
    return -1;
  }

  uint16_t sample_time_register_value = 0;
  Adafruit_BusIO_Register sample_time_reg(i2c_dev, TSL2585_REG_SAMPLE_TIME0, 2,
                                          LSBFIRST);
  if (!sample_time_reg.read(&sample_time_register_value) ||
      sample_time_register_value > TSL2585_MAX_SAMPLE_TIME) {
    return -1;
  }
  return sample_time_register_value;
}

/*!
 * @brief Set the ALS integration length as an actual sample count.
 *
 * ALS_NR_SAMPLES stores one less than the requested count across registers
 * ALS_NR_SAMPLES0 and ALS_NR_SAMPLES1. ALS integration time is the requested
 * count multiplied by the sample period selected by setSampleTime(). See
 * TSL2585 datasheet Figures 25 and 26. Disable ALS before changing this field;
 * this function does not change PON or AEN.
 *
 * @param sample_count Number of samples from 1 through 2048.
 * @return True when the value was valid and the register write succeeded.
 */
bool Adafruit_TSL2585::setIntegrationSamples(uint16_t sample_count) {
  if (i2c_dev == nullptr || sample_count == 0 ||
      sample_count > TSL2585_MAX_INTEGRATION_SAMPLES) {
    return false;
  }

  Adafruit_BusIO_Register als_samples_reg(i2c_dev, TSL2585_REG_ALS_NR_SAMPLES0,
                                          2, LSBFIRST);
  return als_samples_reg.write(sample_count - 1);
}

/*!
 * @brief Read the ALS integration length as an actual sample count.
 *
 * ALS_NR_SAMPLES spans ALS_NR_SAMPLES0 and ALS_NR_SAMPLES1 and stores one less
 * than the sample count. See TSL2585 datasheet Figures 25 and 26.
 *
 * @return The sample count from 1 through 2048, or 0 if the register read
 * failed or contained an invalid value.
 */
uint16_t Adafruit_TSL2585::getIntegrationSamples() {
  if (i2c_dev == nullptr) {
    return 0;
  }

  uint16_t sample_count_register_value;
  Adafruit_BusIO_Register als_samples_reg(i2c_dev, TSL2585_REG_ALS_NR_SAMPLES0,
                                          2, LSBFIRST);
  if (!als_samples_reg.read(&sample_count_register_value) ||
      sample_count_register_value >= TSL2585_MAX_INTEGRATION_SAMPLES) {
    return 0;
  }

  return sample_count_register_value + 1;
}

/*!
 * @brief Set one channel's gain field without changing the ALS state.
 *
 * This advanced setter writes the same step-0 gain fields as setGain(), but it
 * does not change PON or AEN. Use it while ALS is disabled when several setup
 * registers must be programmed together. Photopic and IR occupy the low and
 * high nibbles of MEAS_SEQR_STEP0_MOD_GAINX_L; UVA occupies the low nibble of
 * MEAS_SEQR_STEP0_MOD_GAINX_H. See TSL2585 datasheet Figures 63 and 64.
 *
 * @param channel The photopic, infrared, or UVA channel.
 * @param gain Gain from 0.5x through 4096x.
 * @return True when the register-field write succeeded.
 */
bool Adafruit_TSL2585::setGainValue(tsl2585_channel_t channel,
                                    tsl2585_gain_t gain) {
  if (i2c_dev == nullptr) {
    return false;
  }

  uint16_t gain_register_address = TSL2585_REG_STEP0_GAIN_L;
  uint8_t gain_field_shift = TSL2585_PHOTOPIC_GAIN_SHIFT;
  if (channel == TSL2585_CHANNEL_UVA) {
    gain_register_address = TSL2585_REG_STEP0_GAIN_H;
    gain_field_shift = TSL2585_UVA_GAIN_SHIFT;
  } else if (channel == TSL2585_CHANNEL_IR) {
    gain_field_shift = TSL2585_IR_GAIN_SHIFT;
  }

  Adafruit_BusIO_Register gain_reg(i2c_dev, gain_register_address);
  Adafruit_BusIO_RegisterBits gain_bits(&gain_reg, TSL2585_GAIN_BITS,
                                        gain_field_shift);
  return gain_bits.write((uint8_t)gain);
}

/*!
 * @brief Set the raw ALS and flicker sequencer patterns.
 *
 * Each nibble is a four-step bit pattern: bit 0 selects sequencer step 0 and
 * bit 3 selects step 3. The five bytes control flicker use by modulators 0/1,
 * ALS and modulator-2 flicker use, persistence and VSYNC wait, residual use by
 * modulators 0/1, and residual use by modulator 2 plus timer wait. See TSL2585
 * datasheet Figures 58 through 62. Disable ALS before changing these registers;
 * this function does not change PON or AEN.
 *
 * @param fd_mod01_pattern Complete MEAS_SEQR_FD_0 register value.
 * @param als_fd_mod2_pattern Complete MEAS_SEQR_ALS_FD_1 register value.
 * @param persistence_vsync_pattern Complete MEAS_SEQR_APERS_AND_VSYNC_WAIT
 * register value.
 * @param residual_mod01_pattern Complete MEAS_SEQR_RESIDUAL_0 register value.
 * @param residual_mod2_wait_pattern Complete MEAS_SEQR_RESIDUAL_1_AND_WAIT
 * register value.
 * @return True when all five register writes succeeded.
 */
bool Adafruit_TSL2585::setSequencer(uint8_t fd_mod01_pattern,
                                    uint8_t als_fd_mod2_pattern,
                                    uint8_t persistence_vsync_pattern,
                                    uint8_t residual_mod01_pattern,
                                    uint8_t residual_mod2_wait_pattern) {
  if (i2c_dev == nullptr) {
    return false;
  }

  // MEAS_SEQR_FD_0 selects flicker steps. MEAS_SEQR_ALS_FD_1 selects ALS steps,
  // and MEAS_SEQR_APERS selects which steps use interrupt persistence.
  Adafruit_BusIO_Register meas_seqr_fd_0_reg(i2c_dev,
                                             TSL2585_REG_MEAS_SEQR_FD_0);
  Adafruit_BusIO_Register meas_seqr_als_fd_1_reg(
      i2c_dev, TSL2585_REG_MEAS_SEQR_ALS_FD_1);
  Adafruit_BusIO_Register meas_seqr_apers_reg(i2c_dev,
                                              TSL2585_REG_MEAS_SEQR_APERS);

  // MEAS_SEQR_RESIDUAL_0 and _1 select residual measurements for all three
  // modulators.
  Adafruit_BusIO_Register meas_seqr_residual_0_reg(
      i2c_dev, TSL2585_REG_MEAS_SEQR_RESIDUAL_0);
  Adafruit_BusIO_Register meas_seqr_residual_1_reg(
      i2c_dev, TSL2585_REG_MEAS_SEQR_RESIDUAL_1);

  return meas_seqr_fd_0_reg.write(fd_mod01_pattern) &&
         meas_seqr_als_fd_1_reg.write(als_fd_mod2_pattern) &&
         meas_seqr_apers_reg.write(persistence_vsync_pattern) &&
         meas_seqr_residual_0_reg.write(residual_mod01_pattern) &&
         meas_seqr_residual_1_reg.write(residual_mod2_wait_pattern);
}

/*!
 * @brief Set the maximum gain available to every sequencer channel.
 *
 * This writes MEASUREMENT_SEQUENCER_MAX_MOD_GAIN in CFG8 without changing the
 * AGC prediction reduction field. See TSL2585 datasheet Figure 50. Disable ALS
 * before changing this field; this function does not change PON or AEN.
 *
 * @param maximum_gain Maximum permitted gain from 0.5x through 4096x.
 * @return True when the register-field write succeeded.
 */
bool Adafruit_TSL2585::setMaximumGain(tsl2585_gain_t maximum_gain) {
  if (i2c_dev == nullptr) {
    return false;
  }

  // CFG8 sets the sequencer gain ceiling.
  Adafruit_BusIO_Register cfg8_reg(i2c_dev, TSL2585_REG_CFG8);
  Adafruit_BusIO_RegisterBits maximum_gain_bits(
      &cfg8_reg, TSL2585_CFG8_MAX_GAIN_BITS, TSL2585_CFG8_MAX_GAIN_SHIFT);
  return maximum_gain_bits.write(maximum_gain);
}

/*!
 * @brief Set the raw step-0 photodiode-to-modulator routing.
 *
 * The low byte contains four two-bit fields for photodiodes 0 through 3. The
 * low nibble of the high byte contains the fields for photodiodes 4 and 5;
 * each field selects no connection or modulator 0, 1, or 2. See TSL2585
 * datasheet Figures 71 and 72. Disable ALS before changing these registers;
 * this function does not change PON or AEN.
 *
 * @param smux_low Complete MEAS_SEQR_STEP0_MOD_PHDX_SMUX_L register value.
 * @param smux_high MEAS_SEQR_STEP0_MOD_PHDX_SMUX_H value from 0 through 0x0F.
 * @return True when the value was valid and both register writes succeeded.
 */
bool Adafruit_TSL2585::setSMUX(uint8_t smux_low, uint8_t smux_high) {
  if (i2c_dev == nullptr || smux_high > TSL2585_SMUX_HIGH_MAX) {
    return false;
  }

  // STEP0_SMUX_L and _H route the photopic, IR, and UVA photodiodes to their
  // three modulators.
  Adafruit_BusIO_Register smux_low_reg(i2c_dev, TSL2585_REG_STEP0_SMUX_L);
  Adafruit_BusIO_Register smux_high_reg(i2c_dev, TSL2585_REG_STEP0_SMUX_H);

  return smux_low_reg.write(smux_low) && smux_high_reg.write(smux_high);
}

/*!
 * @brief Set how often modulator calibration features run.
 *
 * MOD_CALIB_CFG0 schedules every enabled calibration feature, including AGC
 * and auto-zero, by sequencer rounds. Write 0 to disable scheduled calibration,
 * 1 through 254 to run every nth round, or 255 to run once when measurements
 * start. See TSL2585 datasheet Figure 79. Disable ALS before changing this
 * register; this function does not change PON or AEN.
 *
 * @param calibration_interval Calibration schedule from 0 through 255.
 * @return True when the register write succeeded.
 */
bool Adafruit_TSL2585::setCalibrationInterval(uint8_t calibration_interval) {
  if (i2c_dev == nullptr) {
    return false;
  }

  Adafruit_BusIO_Register calibration_interval_reg(i2c_dev,
                                                   TSL2585_REG_MOD_CALIB_CFG0);
  return calibration_interval_reg.write(calibration_interval);
}
