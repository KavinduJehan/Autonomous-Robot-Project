/**
 * @file vl53l0x_pololu.h
 * @brief VL53L0X Driver - Pololu-based Implementation for STM32 HAL
 * 
 * This implementation is based on Pololu's proven VL53L0X Arduino library,
 * which avoids the state machine corruption bugs in ST's official API.
 * 
 * Advantages:
 * - Simple, direct I2C register access
 * - Works reliably with dual sensors and address changes
 * - No complex state machine validation
 * - Each read/write is independent
 * 
 * @date November 18, 2025
 */

#ifndef VL53L0X_POLOLU_H
#define VL53L0X_POLOLU_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ==================== Data Structures ==================== */

typedef struct {
    I2C_HandleTypeDef *bus;
    uint8_t address;        // I2C address (7-bit)
    uint16_t io_timeout;    // Timeout in ms (0 = disabled)
    bool did_timeout;
    
    // Calibration data
    uint16_t stop_variable;
    uint32_t measurement_timing_budget_us;
} VL53L0X_Pololu_t;

typedef struct {
    uint8_t tcc;
    uint8_t msrc;
    uint8_t pre_range;
    uint8_t final_range;
    uint8_t dss;
} VL53L0X_SequenceStepEnables;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
    uint16_t final_range_mclks;
    
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_us;
    uint32_t final_range_us;
} VL53L0X_SequenceStepTimeouts;

typedef enum { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;

/* ==================== Register Definitions ==================== */

/* IDENTIFICATION_MODEL_ID should have this value */
#define IDENTIFICATION_MODEL_ID_VALUE      0xEE
#define IDENTIFICATION_MODEL_ID            0xC0  /* Register address for model ID */

/* Register definitions */
#define I2C_SLAVE_DEVICE_ADDRESS          0x8A
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV 0x89
#define MSRC_CONFIG_CONTROL               0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0  0xB0
#define DYNAMIC_SPAD_REF_EN_START_OFFSET  0x4F
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E
#define GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH 0x69
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW  0x6A
#define PRE_RANGE_CONFIG_VCSEL_PERIOD     0x6E
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71
#define MSRC_CONFIG_TIMEOUT_MACROP        0x46
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH 0x73
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW  0x74
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD   0x7C
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x7F
#define GLOBAL_CONFIG_VCSEL_WIDTH         0x32
#define ALGO_PHASECAL_CONFIG_TIMEOUT      0x30
#define ALGO_PHASECAL_LIM                 0x55
#define SYSTEM_INTERRUPT_CONFIG_GPIO      0x0A
#define GPIO_HV_MUX_ACTIVE_HIGH           0x84
#define SYSTEM_INTERRUPT_CLEAR            0x0B
#define SYSTEM_SEQUENCE_CONFIG            0x01
#define SYSRANGE_START                    0x00
#define RESULT_INTERRUPT_STATUS           0x13
#define RESULT_RANGE_STATUS               0x14
#define IDENTIFICATION_REVISION_ID        0xC1  /* Register address for revision ID */
#define OSC_CALIBRATE_VAL                 0xF8

/* ==================== Public API ==================== */

/**
 * @brief Initialize VL53L0X sensor instance
 * @param sensor Pointer to sensor structure
 * @param hi2c I2C handle
 * @param address 7-bit I2C address (0x29 default)
 */
void VL53L0X_Pololu_Init(VL53L0X_Pololu_t *sensor, I2C_HandleTypeDef *hi2c, uint8_t address);

/**
 * @brief Change I2C address of sensor
 * @param sensor Pointer to sensor structure
 * @param new_addr New 7-bit I2C address
 */
void VL53L0X_Pololu_SetAddress(VL53L0X_Pololu_t *sensor, uint8_t new_addr);

/**
 * @brief Full sensor initialization sequence (equivalent to Pololu init())
 * @param sensor Pointer to sensor structure
 * @param io_2v8 Use 2.8V mode (true = 2V8, false = 1V8)
 * @return true if successful, false if failed
 */
bool VL53L0X_Pololu_Setup(VL53L0X_Pololu_t *sensor, bool io_2v8);

/**
 * @brief Perform single-shot range measurement
 * @param sensor Pointer to sensor structure
 * @return Distance in millimeters
 */
uint16_t VL53L0X_Pololu_ReadRangeSingleMillimeters(VL53L0X_Pololu_t *sensor);

/**
 * @brief Start continuous ranging measurements
 * @param sensor Pointer to sensor structure
 * @param period_ms Inter-measurement period (0 = back-to-back mode)
 */
void VL53L0X_Pololu_StartContinuous(VL53L0X_Pololu_t *sensor, uint32_t period_ms);

/**
 * @brief Stop continuous ranging measurements
 * @param sensor Pointer to sensor structure
 */
void VL53L0X_Pololu_StopContinuous(VL53L0X_Pololu_t *sensor);

/**
 * @brief Read range in continuous mode
 * @param sensor Pointer to sensor structure
 * @return Distance in millimeters
 */
uint16_t VL53L0X_Pololu_ReadRangeContinuousMillimeters(VL53L0X_Pololu_t *sensor);

/**
 * @brief Set measurement timing budget
 * @param sensor Pointer to sensor structure
 * @param budget_us Timing budget in microseconds
 * @return true if successful
 */
bool VL53L0X_Pololu_SetMeasurementTimingBudget(VL53L0X_Pololu_t *sensor, uint32_t budget_us);

/**
 * @brief Get measurement timing budget
 * @param sensor Pointer to sensor structure
 * @return Timing budget in microseconds
 */
uint32_t VL53L0X_Pololu_GetMeasurementTimingBudget(VL53L0X_Pololu_t *sensor);

/**
 * @brief Set VCSEL pulse period
 * @param sensor Pointer to sensor structure
 * @param type Pre-range or final range
 * @param period_pclks Period in PCLKs
 * @return true if successful
 */
bool VL53L0X_Pololu_SetVcselPulsePeriod(VL53L0X_Pololu_t *sensor, vcselPeriodType type, uint8_t period_pclks);

/**
 * @brief Set signal rate limit
 * @param sensor Pointer to sensor structure
 * @param limit_Mcps Signal rate limit in MCPS
 * @return true if successful
 */
bool VL53L0X_Pololu_SetSignalRateLimit(VL53L0X_Pololu_t *sensor, float limit_Mcps);

/**
 * @brief Get signal rate limit
 * @param sensor Pointer to sensor structure
 * @return Signal rate limit in MCPS
 */
float VL53L0X_Pololu_GetSignalRateLimit(VL53L0X_Pololu_t *sensor);

/**
 * @brief Check if timeout occurred
 * @param sensor Pointer to sensor structure
 * @return true if timeout occurred
 */
bool VL53L0X_Pololu_TimeoutOccurred(VL53L0X_Pololu_t *sensor);

#endif /* VL53L0X_POLOLU_H */
