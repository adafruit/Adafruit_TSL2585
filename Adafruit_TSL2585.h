/*!
 * @file Adafruit_TSL2585.h
 *
 * This is a library for the ams OSRAM TSL2585 / TSL25853 ambient light,
 * infrared, and UVA sensor.
 *
 * These sensors use I2C to communicate, 2 pins are required to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing products
 * from Adafruit!
 *
 * Written by Limor Fried and the Adafruit team for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 */

#ifndef ADAFRUIT_TSL2585_H
#define ADAFRUIT_TSL2585_H

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h>

#include "Arduino.h"

#define TSL2585_DEFAULT_ADDR 0x39 ///< Fixed TSL2585 I2C address
#define TSL2585_DEVICE_ID 0x5C    ///< Expected value of the ID register

#define TSL2585_REG_UV_CALIB 0x08          ///< Factory UVA calibration register
#define TSL2585_REG_ENABLE 0x80            ///< Power and measurement enable
#define TSL2585_REG_MEAS_MODE0 0x81        ///< Measurement mode register 0
#define TSL2585_REG_MEAS_MODE1 0x82        ///< Measurement mode register 1
#define TSL2585_REG_SAMPLE_TIME0 0x83      ///< Sample time low byte
#define TSL2585_REG_ALS_NR_SAMPLES0 0x85   ///< ALS sample count low byte
#define TSL2585_REG_ALS_THRESHOLD_LOW 0x8A ///< ALS low threshold, 24-bit
#define TSL2585_REG_ALS_THRESHOLD_HIGH 0x8D ///< ALS high threshold, 24-bit
#define TSL2585_REG_AUX_ID 0x90             ///< Auxiliary identification
#define TSL2585_REG_REV_ID 0x91             ///< Silicon revision identification
#define TSL2585_REG_ID 0x92                 ///< Device identification
#define TSL2585_REG_STATUS 0x93             ///< Main interrupt status
#define TSL2585_REG_ALS_STATUS 0x94     ///< Start of coherent ALS result block
#define TSL2585_REG_STATUS2 0x9D        ///< ALS validity and saturation status
#define TSL2585_REG_STATUS4 0x9F        ///< Initialization and trigger status
#define TSL2585_REG_CFG3 0xA4           ///< INT and GPIO pin mapping
#define TSL2585_REG_CFG4 0xA5           ///< Sequencer and AGC configuration
#define TSL2585_REG_CFG5 0xA6           ///< ALS interrupt channel/persistence
#define TSL2585_REG_CFG8 0xA9           ///< Maximum sequencer modulator gain
#define TSL2585_REG_CONTROL 0xB1        ///< Reset and status-clear controls
#define TSL2585_REG_INTENAB 0xBA        ///< External interrupt enables
#define TSL2585_REG_MEAS_SEQR_FD_0 0xCF ///< Modulator 0/1 flicker patterns
#define TSL2585_REG_MEAS_SEQR_ALS_FD_1 0xD0   ///< ALS and modulator 2 patterns
#define TSL2585_REG_MEAS_SEQR_APERS 0xD1      ///< ALS persistence step pattern
#define TSL2585_REG_MEAS_SEQR_RESIDUAL_0 0xD2 ///< Modulator 0/1 residuals
#define TSL2585_REG_MEAS_SEQR_RESIDUAL_1 0xD3 ///< Modulator 2 residuals/wait
#define TSL2585_REG_STEP0_GAIN_L 0xD4         ///< Step 0 modulator 0/1 gains
#define TSL2585_REG_STEP0_GAIN_H 0xD5         ///< Step 0 modulator 2 gain
#define TSL2585_REG_STEP0_SMUX_L 0xDC   ///< Step 0 photodiode map low byte
#define TSL2585_REG_STEP0_SMUX_H 0xDD   ///< Step 0 photodiode map high byte
#define TSL2585_REG_STEP1_SMUX_H 0xDF   ///< Saturation AGC step pattern
#define TSL2585_REG_STEP2_SMUX_H 0xE1   ///< Predictive AGC step pattern
#define TSL2585_REG_MOD_CALIB_CFG0 0xE4 ///< Calibration repetition rate
#define TSL2585_REG_MOD_CALIB_CFG2 0xE6 ///< Calibration feature enables
#define TSL2585_REG_VSYNC_GPIO_INT 0xF8 ///< INT and GPIO direction/value

#define TSL2585_ENABLE_PON_BIT 0 ///< Oscillator and power enable bit position
#define TSL2585_ENABLE_AEN_BIT 1 ///< Ambient light enable bit position

#define TSL2585_CONTROL_SOFT_RESET_BIT 3 ///< Software-reset bit position
#define TSL2585_STARTUP_DELAY_MS 1       ///< Oscillator startup delay
#define TSL2585_RESET_DELAY_MS 1         ///< Reset initialization delay

#define TSL2585_STATUS2_DATA_VALID_BIT 6        ///< ALS data-valid bit position
#define TSL2585_STATUS2_DIGITAL_SATURATION 0x10 ///< ALS result overflowed
#define TSL2585_STATUS_AINT 0x08  ///< ALS threshold interrupt asserted
#define TSL2585_STATUS_AINT_BIT 3 ///< Position of ALS interrupt status

#define TSL2585_INTENAB_AIEN_BIT 3 ///< Position of ALS interrupt enable

#define TSL2585_CFG3_GPIO_PINMAP_BITS 2   ///< Width of GPIO pin-map field
#define TSL2585_CFG3_GPIO_PINMAP_SHIFT 0  ///< Position of GPIO pin-map field
#define TSL2585_CFG3_GPIO_PINMAP_OUTPUT 0 ///< Route GPIO output register to pin
#define TSL2585_CFG3_INT_PINMAP_BITS 2    ///< Width of INT pin-map field
#define TSL2585_CFG3_INT_PINMAP_SHIFT 4   ///< Position of INT pin-map field
#define TSL2585_CFG3_INT_PINMAP_INTERRUPT 0 ///< Route interrupt signal to INT

#define TSL2585_CFG4_CALIBRATION_PER_STEP_BIT 6 ///< Schedule by sequencer step

#define TSL2585_CFG5_THRESHOLD_CHANNEL_BITS 2  ///< Width of channel field
#define TSL2585_CFG5_THRESHOLD_CHANNEL_SHIFT 4 ///< Position of channel field
#define TSL2585_CFG5_PERSISTENCE_BITS 4        ///< Width of persistence field
#define TSL2585_CFG5_PERSISTENCE_SHIFT 0 ///< Position of persistence field
#define TSL2585_MAX_INTERRUPT_PERSISTENCE 0x0F ///< Largest persistence code

#define TSL2585_CFG8_MAX_GAIN_BITS 4  ///< Width of maximum gain field
#define TSL2585_CFG8_MAX_GAIN_SHIFT 4 ///< Position of maximum gain field

#define TSL2585_AGC_PATTERN_BITS 4     ///< Width of each AGC sequencer pattern
#define TSL2585_AGC_PATTERN_SHIFT 4    ///< Position of each AGC pattern
#define TSL2585_AGC_STEP0_PATTERN 0x01 ///< Enable AGC for sequencer step 0
#define TSL2585_MOD_CALIB_AGC_ENABLE_BIT 5 ///< Link AGC to calibration cycle

#define TSL2585_INT_INPUT_ENABLE_BIT 5  ///< INT direction bit position
#define TSL2585_INT_INVERT_BIT 6        ///< INT polarity bit position
#define TSL2585_GPIO_INVERT_BIT 3       ///< GPIO polarity bit position
#define TSL2585_GPIO_INPUT_ENABLE_BIT 2 ///< GPIO direction bit position
#define TSL2585_GPIO_OUTPUT_BIT 1       ///< GPIO open-drain value bit position
#define TSL2585_GPIO_INPUT_BIT 0        ///< GPIO input value bit position

#define TSL2585_GAIN_BITS 4           ///< Width of one gain configuration field
#define TSL2585_PHOTOPIC_GAIN_SHIFT 0 ///< Photopic gain field position
#define TSL2585_IR_GAIN_SHIFT 4       ///< IR gain field position
#define TSL2585_UVA_GAIN_SHIFT 0      ///< UVA gain field position

#define TSL2585_ALS_STATUS_PHOTOPIC_SATURATION 0x20 ///< Modulator 0 saturated
#define TSL2585_ALS_STATUS_IR_SATURATION 0x10       ///< Modulator 1 saturated
#define TSL2585_ALS_STATUS_UVA_SATURATION 0x08      ///< Modulator 2 saturated

#define TSL2585_RECOMMENDED_SMUX_L 0xE6 ///< PHO->0, IR->1, UVA->2 mapping
#define TSL2585_RECOMMENDED_SMUX_H 0x07 ///< PHO->0 and UVA->2 mapping

#define TSL2585_MEAS_MODE0_FULL_COUNTS 0x00     ///< 16-bit counts, no residuals
#define TSL2585_MEAS_MODE1_MSB_POSITION_12 0x0C ///< Default 20-bit MSB
#define TSL2585_SEQUENCER_DISABLED 0x00         ///< Disable a sequencer feature
#define TSL2585_SEQUENCER_STEP0 0x01         ///< Enable sequencer step 0 only
#define TSL2585_CALIBRATION_EVERY_ROUND 0x01 ///< Run calibration every round

#define TSL2585_SAMPLE_TIME_250_US 179 ///< 250 us sample time register value
#define TSL2585_MODULATOR_CLOCK_PERIOD_US 1.388889F ///< One clock period in us
#define TSL2585_DEFAULT_ALS_SAMPLE_COUNT 200        ///< 200 samples, or 50 ms
#define TSL2585_MAX_SAMPLE_TIME 2047 ///< Largest 11-bit sample-time value
#define TSL2585_MAX_INTEGRATION_SAMPLES 2048 ///< Largest ALS sample count
#define TSL2585_SMUX_HIGH_MAX 0x0F ///< Largest valid step-0 SMUX high value
#define TSL2585_MAX_INTERRUPT_THRESHOLD 0xFFFFFFUL ///< Largest ALS threshold

/*! @brief Packed lower and upper gain fields from a gain-status register. */
#pragma pack(push, 1)
typedef struct {
  uint8_t lower_nibble : 4; ///< Gain code stored in bits 3 through 0
  uint8_t upper_nibble : 4; ///< Gain code stored in bits 7 through 4
} tsl2585_gain_register_t;

/*! @brief Packed register layout for one coherent ALS result block. */
typedef struct {
  uint8_t als_status; ///< Saturation status for all three modulators
  uint16_t als_data0; ///< Raw result from modulator 0
  uint16_t als_data1; ///< Raw result from modulator 1
  uint16_t als_data2; ///< Raw result from modulator 2
  tsl2585_gain_register_t
      als_data01_gain_status; ///< Gain codes for modulators 0 and 1
  tsl2585_gain_register_t als_data2_gain_status; ///< Gain code for modulator 2
} tsl2585_result_registers_t;
#pragma pack(pop)

/*! @brief Byte buffer and register view for one coherent ALS result block. */
typedef union {
  tsl2585_result_registers_t registers; ///< Structured register view
  uint8_t buffer[sizeof(tsl2585_result_registers_t)]; ///< I2C read buffer
} tsl2585_result_buffer_t;

static_assert(sizeof(tsl2585_result_buffer_t) == 9,
              "TSL2585 result block must be 9 bytes");
static_assert(sizeof(tsl2585_gain_register_t) == 1,
              "TSL2585 gain register must be 1 byte");

/*!
 * @brief Optical channels after applying the recommended TSL2585 SMUX map.
 */
typedef enum {
  TSL2585_CHANNEL_PHOTOPIC = 0, ///< Human-eye response channel, modulator 0
  TSL2585_CHANNEL_IR = 1,       ///< Near-infrared channel, modulator 1
  TSL2585_CHANNEL_UVA = 2,      ///< 315 nm to 400 nm channel, modulator 2
} tsl2585_channel_t;

/*!
 * @brief Programmable modulator gain settings.
 */
typedef enum {
  TSL2585_GAIN_0_5X = 0x00,  ///< 0.5x gain
  TSL2585_GAIN_1X = 0x01,    ///< 1x gain
  TSL2585_GAIN_2X = 0x02,    ///< 2x gain
  TSL2585_GAIN_4X = 0x03,    ///< 4x gain
  TSL2585_GAIN_8X = 0x04,    ///< 8x gain
  TSL2585_GAIN_16X = 0x05,   ///< 16x gain
  TSL2585_GAIN_32X = 0x06,   ///< 32x gain
  TSL2585_GAIN_64X = 0x07,   ///< 64x gain
  TSL2585_GAIN_128X = 0x08,  ///< 128x gain
  TSL2585_GAIN_256X = 0x09,  ///< 256x gain
  TSL2585_GAIN_512X = 0x0A,  ///< 512x gain
  TSL2585_GAIN_1024X = 0x0B, ///< 1024x gain
  TSL2585_GAIN_2048X = 0x0C, ///< 2048x gain
  TSL2585_GAIN_4096X = 0x0D, ///< 4096x gain
} tsl2585_gain_t;

/*!
 * @brief One coherent three-channel TSL2585 measurement.
 */
typedef struct {
  uint16_t photopic;    ///< Raw full-count photopic result
  uint16_t infrared;    ///< Raw full-count infrared result
  uint16_t uva;         ///< Raw full-count UVA result
  float uva_calibrated; ///< UVA counts corrected with the part's OTP factor
  float photopic_1x;    ///< Typical photopic counts at 1x-equivalent gain
  float infrared_1x;    ///< Typical infrared counts at 1x-equivalent gain
  float uva_1x;         ///< Corrected UVA counts at 1x-equivalent gain
  tsl2585_gain_t photopic_gain; ///< Gain used for the photopic result
  tsl2585_gain_t infrared_gain; ///< Gain used for the infrared result
  tsl2585_gain_t uva_gain;      ///< Gain used for the UVA result
  bool photopic_saturated;      ///< True when the photopic result is invalid
  bool infrared_saturated;      ///< True when the infrared result is invalid
  bool uva_saturated;           ///< True when the UVA result is invalid
} tsl2585_data_t;

/*!
 * @brief Class that stores state and functions for interacting with TSL2585.
 */
class Adafruit_TSL2585 {
 public:
  Adafruit_TSL2585();
  ~Adafruit_TSL2585();

  bool begin(uint8_t i2c_addr = TSL2585_DEFAULT_ADDR, TwoWire* wire = &Wire);
  bool reset();
  bool enable(bool enabled);

  bool setIntegrationTime(float milliseconds);
  float getIntegrationTime();

  bool setGain(tsl2585_channel_t channel, tsl2585_gain_t gain);
  tsl2585_gain_t getGain(tsl2585_channel_t channel);
  bool enableAGC(bool enabled);

  bool setResultFormat(uint8_t mode0, uint8_t mode1);
  bool setSampleTime(uint16_t sample_time_register_value);
  int16_t getSampleTime();
  bool setIntegrationSamples(uint16_t sample_count);
  uint16_t getIntegrationSamples();
  bool setGainValue(tsl2585_channel_t channel, tsl2585_gain_t gain);
  bool setSequencer(uint8_t fd_mod01_pattern, uint8_t als_fd_mod2_pattern,
                    uint8_t persistence_vsync_pattern,
                    uint8_t residual_mod01_pattern,
                    uint8_t residual_mod2_wait_pattern);
  bool setMaximumGain(tsl2585_gain_t maximum_gain);
  bool setSMUX(uint8_t smux_low, uint8_t smux_high);
  bool setCalibrationInterval(uint8_t calibration_interval);

  bool dataReady();
  bool readData(tsl2585_data_t* data);
  float calibrateUVA(uint16_t raw_uva);

  bool setALSThresholds(tsl2585_channel_t channel, uint32_t low_threshold,
                        uint32_t high_threshold, uint8_t persistence = 1);
  bool enableALSInterrupt(bool enabled);
  bool alsInterruptActive();
  bool clearALSInterrupt();

  bool setGPIOOutput(bool released);
  bool enableGPIOInput(bool enabled);
  bool readGPIOInput();

  uint8_t getDeviceID();
  uint8_t getRevisionID();
  uint8_t getAuxiliaryID();
  uint8_t getUVCalibration();

 private:
  Adafruit_TSL2585(const Adafruit_TSL2585&) = delete;
  Adafruit_TSL2585& operator=(const Adafruit_TSL2585&) = delete;

  Adafruit_I2CDevice* i2c_dev = nullptr;
  uint8_t _uv_calibration = 127;

  float normalizeGainTo1x(float counts, tsl2585_gain_t gain);
};

#endif
