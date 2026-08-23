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

  // UV_CALIB supplies the factory correction used for calibrated UVA results.
  _uv_calibration = getUVCalibration();

  return configure();
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
  delay(1);
  return als_enable_bit.write(1);
}

/*!
 * @brief Set ALS integration time for the register-result path.
 * @param milliseconds Integration time from 0.25 ms through 90 ms.
 * @return True when the value was in range and written successfully.
 */
bool Adafruit_TSL2585::setIntegrationTime(float milliseconds) {
  if (i2c_dev == nullptr || milliseconds < 0.25F || milliseconds > 90.0F) {
    return false;
  }

  uint16_t sample_count = (uint16_t)(milliseconds * 4.0F + 0.5F);
  uint16_t register_value = sample_count - 1;

  // Figures 25 and 26 do not define a live update for this two-byte field.
  // Stop ALS so the next conversion uses one complete integration setting.
  if (!enable(false)) {
    return false;
  }
  Adafruit_BusIO_Register als_samples_reg(i2c_dev, TSL2585_REG_ALS_NR_SAMPLES0,
                                          2, LSBFIRST);
  bool success = als_samples_reg.write(register_value);

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

  uint16_t register_value;
  Adafruit_BusIO_Register als_samples_reg(i2c_dev, TSL2585_REG_ALS_NR_SAMPLES0,
                                          2, LSBFIRST);
  if (!als_samples_reg.read(&register_value)) {
    return 0;
  }
  return (register_value + 1) * 0.25F;
}

/*!
 * @brief Set the manual gain for one optical channel.
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

  uint16_t register_address = TSL2585_REG_STEP0_GAIN_L;
  uint8_t shift = TSL2585_PHOTOPIC_GAIN_SHIFT;
  if (channel == TSL2585_CHANNEL_UVA) {
    register_address = TSL2585_REG_STEP0_GAIN_H;
    shift = TSL2585_UVA_GAIN_SHIFT;
  } else if (channel == TSL2585_CHANNEL_IR) {
    shift = TSL2585_IR_GAIN_SHIFT;
  }

  Adafruit_BusIO_Register gain_reg(i2c_dev, register_address);
  Adafruit_BusIO_RegisterBits gain_bits(&gain_reg, TSL2585_GAIN_BITS, shift);
  bool success = gain_bits.write((uint8_t)gain);

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

  uint16_t register_address = TSL2585_REG_STEP0_GAIN_L;
  uint8_t shift = TSL2585_PHOTOPIC_GAIN_SHIFT;
  if (channel == TSL2585_CHANNEL_UVA) {
    register_address = TSL2585_REG_STEP0_GAIN_H;
    shift = TSL2585_UVA_GAIN_SHIFT;
  } else if (channel == TSL2585_CHANNEL_IR) {
    shift = TSL2585_IR_GAIN_SHIFT;
  }

  Adafruit_BusIO_Register gain_reg(i2c_dev, register_address);
  Adafruit_BusIO_RegisterBits gain_bits(&gain_reg, TSL2585_GAIN_BITS, shift);
  return (tsl2585_gain_t)gain_bits.read();
}

/*!
 * @brief Check whether a new ALS measurement is available.
 * @return True when ALS_DATA_VALID is set.
 */
bool Adafruit_TSL2585::dataReady() {
  if (i2c_dev == nullptr) {
    return false;
  }

  Adafruit_BusIO_Register status2_reg(i2c_dev, TSL2585_REG_STATUS2);
  Adafruit_BusIO_RegisterBits data_valid_bit(&status2_reg, 1,
                                             TSL2585_STATUS2_DATA_VALID_BIT);
  return data_valid_bit.read();
}

/*!
 * @brief Read the latest coherent three-channel measurement.
 *
 * STATUS2 is read before ALS_STATUS as required by the device. The complete
 * ALS_STATUS through ALS_STATUS3 block is then read in one transaction so the
 * channel values, saturation flags, and actual gains belong to one cycle.
 *
 * @param data Destination for the result.
 * @return True when the register data was read successfully.
 */
bool Adafruit_TSL2585::readData(tsl2585_data_t* data) {
  if (i2c_dev == nullptr || data == nullptr) {
    return false;
  }

  // STATUS2 reports digital saturation for the current result.
  uint8_t status2;
  Adafruit_BusIO_Register status2_reg(i2c_dev, TSL2585_REG_STATUS2);
  if (!status2_reg.read(&status2)) {
    return false;
  }

  // ALS_STATUS starts the atomic status, counts, and actual-gains result frame.
  tsl2585_result_buffer_t result;
  Adafruit_BusIO_Register als_result_reg(i2c_dev, TSL2585_REG_ALS_STATUS,
                                         sizeof(result.buffer), LSBFIRST);
  if (!als_result_reg.read(result.buffer, sizeof(result.buffer))) {
    return false;
  }

  uint8_t als_status = result.registers.status;
  data->photopic = result.registers.photopic;
  data->infrared = result.registers.infrared;
  data->uva = result.registers.uva;
  data->uva_calibrated = calibrateUVA(data->uva);

  data->photopic_gain = (tsl2585_gain_t)result.registers.photopic_ir_gain.lower;
  data->infrared_gain = (tsl2585_gain_t)result.registers.photopic_ir_gain.upper;
  data->uva_gain = (tsl2585_gain_t)result.registers.uva_gain.lower;

  bool digital_saturation = (status2 & TSL2585_STATUS2_DIGITAL_SATURATION) != 0;
  data->photopic_saturated =
      digital_saturation ||
      (als_status & TSL2585_ALS_STATUS_PHOTOPIC_SATURATION) != 0;
  data->infrared_saturated =
      digital_saturation ||
      (als_status & TSL2585_ALS_STATUS_IR_SATURATION) != 0;
  data->uva_saturated = digital_saturation ||
                        (als_status & TSL2585_ALS_STATUS_UVA_SATURATION) != 0;

  return true;
}

/*!
 * @brief Apply the factory OTP correction to a raw UVA count.
 * @param raw_uva Raw UVA full counts from the coherent result block.
 * @return Factory-corrected UVA counts.
 */
float Adafruit_TSL2585::calibrateUVA(uint16_t raw_uva) {
  float correction = 1.0F - ((_uv_calibration - 127.0F) / 100.0F);
  if (correction <= 0.0F) {
    return raw_uva;
  }
  return raw_uva / correction;
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
  Adafruit_BusIO_Register gpio_reg(i2c_dev, TSL2585_REG_VSYNC_GPIO_INT);
  Adafruit_BusIO_RegisterBits int_input_enable_bit(
      &gpio_reg, 1, TSL2585_INT_INPUT_ENABLE_BIT);
  Adafruit_BusIO_RegisterBits int_invert_bit(&gpio_reg, 1,
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

  Adafruit_BusIO_Register status_reg(i2c_dev, TSL2585_REG_STATUS);
  Adafruit_BusIO_RegisterBits als_interrupt_status_bit(&status_reg, 1,
                                                       TSL2585_STATUS_AINT_BIT);
  return als_interrupt_status_bit.read() != 0;
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
 * @param high True to release the output, false to pull it low.
 * @return True when the GPIO routing and output writes succeeded.
 */
bool Adafruit_TSL2585::setGPIOOutput(bool high) {
  if (i2c_dev == nullptr) {
    return false;
  }

  // CFG3 routes the GPIO output register to the external GPIO pin.
  Adafruit_BusIO_Register cfg3_reg(i2c_dev, TSL2585_REG_CFG3);
  Adafruit_BusIO_RegisterBits gpio_pinmap_bits(
      &cfg3_reg, TSL2585_CFG3_GPIO_PINMAP_BITS, TSL2585_CFG3_GPIO_PINMAP_SHIFT);

  // VSYNC_GPIO_INT sets the GPIO direction, polarity, and open-drain state.
  Adafruit_BusIO_Register gpio_reg(i2c_dev, TSL2585_REG_VSYNC_GPIO_INT);
  Adafruit_BusIO_RegisterBits gpio_invert_bit(&gpio_reg, 1,
                                              TSL2585_GPIO_INVERT_BIT);
  Adafruit_BusIO_RegisterBits gpio_input_enable_bit(
      &gpio_reg, 1, TSL2585_GPIO_INPUT_ENABLE_BIT);
  Adafruit_BusIO_RegisterBits gpio_output_bit(&gpio_reg, 1,
                                              TSL2585_GPIO_OUTPUT_BIT);

  return gpio_pinmap_bits.write(TSL2585_CFG3_GPIO_PINMAP_OUTPUT) &&
         gpio_invert_bit.write(0) && gpio_input_enable_bit.write(0) &&
         gpio_output_bit.write(high ? 1 : 0);
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
  Adafruit_BusIO_Register gpio_reg(i2c_dev, TSL2585_REG_VSYNC_GPIO_INT);
  Adafruit_BusIO_RegisterBits gpio_input_enable_bit(
      &gpio_reg, 1, TSL2585_GPIO_INPUT_ENABLE_BIT);
  Adafruit_BusIO_RegisterBits gpio_output_bit(&gpio_reg, 1,
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
  Adafruit_BusIO_Register gpio_reg(i2c_dev, TSL2585_REG_VSYNC_GPIO_INT);
  Adafruit_BusIO_RegisterBits gpio_input_bit(&gpio_reg, 1,
                                             TSL2585_GPIO_INPUT_BIT);
  return gpio_input_bit.read();
}

/*! @return The TSL2585 device identification byte. */
uint8_t Adafruit_TSL2585::getDeviceID() {
  Adafruit_BusIO_Register id_reg(i2c_dev, TSL2585_REG_ID);
  return id_reg.read();
}

/*! @return The silicon revision identification byte. */
uint8_t Adafruit_TSL2585::getRevisionID() {
  Adafruit_BusIO_Register revision_id_reg(i2c_dev, TSL2585_REG_REV_ID);
  return revision_id_reg.read();
}

/*! @return The auxiliary identification byte. */
uint8_t Adafruit_TSL2585::getAuxiliaryID() {
  Adafruit_BusIO_Register auxiliary_id_reg(i2c_dev, TSL2585_REG_AUX_ID);
  return auxiliary_id_reg.read();
}

/*! @return The factory UVA calibration byte, where 127 is nominal. */
uint8_t Adafruit_TSL2585::getUVCalibration() {
  Adafruit_BusIO_Register uv_calibration_reg(i2c_dev, TSL2585_REG_UV_CALIB);
  return uv_calibration_reg.read();
}

/*! @brief Configure the recommended one-step, three-channel ALS sequence. */
bool Adafruit_TSL2585::configure() {
  // ENABLE register Figure 20 says to set PON only after configuration.
  if (!enable(false)) {
    return false;
  }

  // MEAS_MODE0 and MEAS_MODE1 select the result format and bit alignment.
  Adafruit_BusIO_Register meas_mode0_reg(i2c_dev, TSL2585_REG_MEAS_MODE0);
  Adafruit_BusIO_Register meas_mode1_reg(i2c_dev, TSL2585_REG_MEAS_MODE1);

  // SAMPLE_TIME0 sets each modulator sample period; ALS_NR_SAMPLES0 sets how
  // many samples are accumulated into one ALS result.
  Adafruit_BusIO_Register sample_time_reg(i2c_dev, TSL2585_REG_SAMPLE_TIME0, 2,
                                          LSBFIRST);
  Adafruit_BusIO_Register als_samples_reg(i2c_dev, TSL2585_REG_ALS_NR_SAMPLES0,
                                          2, LSBFIRST);

  // MEAS_SEQR_FD_0 disables flicker steps. MEAS_SEQR_ALS_FD_1 enables ALS step
  // 0, and MEAS_SEQR_APERS applies interrupt persistence to that step.
  Adafruit_BusIO_Register sequencer_fd_reg(i2c_dev, TSL2585_REG_MEAS_SEQR_FD_0);
  Adafruit_BusIO_Register sequencer_als_reg(i2c_dev,
                                            TSL2585_REG_MEAS_SEQR_ALS_FD_1);
  Adafruit_BusIO_Register sequencer_persistence_reg(
      i2c_dev, TSL2585_REG_MEAS_SEQR_APERS);

  // MEAS_SEQR_RESIDUAL_0 and _1 disable residual measurements on all three
  // modulators.
  Adafruit_BusIO_Register sequencer_residual0_reg(
      i2c_dev, TSL2585_REG_MEAS_SEQR_RESIDUAL_0);
  Adafruit_BusIO_Register sequencer_residual1_reg(
      i2c_dev, TSL2585_REG_MEAS_SEQR_RESIDUAL_1);

  // CFG8 sets the sequencer gain ceiling.
  Adafruit_BusIO_Register cfg8_reg(i2c_dev, TSL2585_REG_CFG8);
  Adafruit_BusIO_RegisterBits maximum_gain_bits(
      &cfg8_reg, TSL2585_CFG8_MAX_GAIN_BITS, TSL2585_CFG8_MAX_GAIN_SHIFT);

  // STEP0_GAIN_L sets photopic and IR gain; STEP0_GAIN_H sets UVA gain.
  Adafruit_BusIO_Register gain_low_reg(i2c_dev, TSL2585_REG_STEP0_GAIN_L);
  Adafruit_BusIO_Register gain_high_reg(i2c_dev, TSL2585_REG_STEP0_GAIN_H);

  // STEP0_SMUX_L and _H route the photopic, IR, and UVA photodiodes to their
  // three modulators.
  Adafruit_BusIO_Register smux_low_reg(i2c_dev, TSL2585_REG_STEP0_SMUX_L);
  Adafruit_BusIO_Register smux_high_reg(i2c_dev, TSL2585_REG_STEP0_SMUX_H);

  if (!meas_mode0_reg.write(TSL2585_MEAS_MODE0_FULL_COUNTS) ||
      !meas_mode1_reg.write(TSL2585_MEAS_MODE1_MSB_POSITION_12) ||
      !sample_time_reg.write(TSL2585_SAMPLE_TIME_250_US) ||
      !als_samples_reg.write(TSL2585_DEFAULT_ALS_SAMPLES) ||
      !sequencer_fd_reg.write(TSL2585_SEQUENCER_DISABLED) ||
      !sequencer_als_reg.write(TSL2585_SEQUENCER_STEP0) ||
      !sequencer_persistence_reg.write(TSL2585_SEQUENCER_STEP0) ||
      !sequencer_residual0_reg.write(TSL2585_SEQUENCER_DISABLED) ||
      !sequencer_residual1_reg.write(TSL2585_SEQUENCER_DISABLED) ||
      !maximum_gain_bits.write(TSL2585_GAIN_4096X) ||
      !gain_low_reg.write(TSL2585_DEFAULT_GAIN_L) ||
      !gain_high_reg.write(TSL2585_DEFAULT_GAIN_H) ||
      !smux_low_reg.write(TSL2585_RECOMMENDED_SMUX_L) ||
      !smux_high_reg.write(TSL2585_RECOMMENDED_SMUX_H)) {
    return false;
  }

  return enable(true);
}
