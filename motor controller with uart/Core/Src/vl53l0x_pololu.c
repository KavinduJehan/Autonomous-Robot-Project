/**
 * @file vl53l0x_pololu.c
 * @brief VL53L0X Driver Implementation - Pololu-based for STM32 HAL
 * 
 * Based on Pololu's VL53L0X Arduino library with STM32 HAL adaptation
 * 
 * @date November 18, 2025
 */

#include "vl53l0x_pololu.h"
#include <stdio.h>
#include "uart_comm.h"
#include "cmsis_os.h"
#include <string.h>

/* ==================== Macros ==================== */

#define startTimeout() (timeout_start_ms = HAL_GetTick())
#define checkTimeoutExpired() (sensor->io_timeout > 0 && ((HAL_GetTick() - timeout_start_ms) > sensor->io_timeout))

#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

/* ==================== I2C Read/Write Functions ==================== */

/**
 * @brief Read 8-bit register
 */
static uint8_t VL53L0X_ReadReg8(VL53L0X_Pololu_t *sensor, uint8_t reg) {
    uint8_t value = 0;
    
    HAL_I2C_Mem_Read(sensor->bus, (sensor->address << 1), reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
    
    return value;
}

/**
 * @brief Read 16-bit register (big-endian)
 */
static uint16_t VL53L0X_ReadReg16(VL53L0X_Pololu_t *sensor, uint8_t reg) {
    uint8_t buffer[2];
    
    HAL_I2C_Mem_Read(sensor->bus, (sensor->address << 1), reg, I2C_MEMADD_SIZE_8BIT, buffer, 2, 1000);
    
    return ((uint16_t)buffer[0] << 8) | buffer[1];
}

/**
 * @brief Read 32-bit register (big-endian)
 */
static uint32_t VL53L0X_ReadReg32(VL53L0X_Pololu_t *sensor, uint8_t reg) {
    uint8_t buffer[4];
    
    HAL_I2C_Mem_Read(sensor->bus, (sensor->address << 1), reg, I2C_MEMADD_SIZE_8BIT, buffer, 4, 1000);
    
    return ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | 
           ((uint16_t)buffer[2] << 8) | buffer[3];
}

/**
 * @brief Read multiple bytes
 */
static void VL53L0X_ReadMulti(VL53L0X_Pololu_t *sensor, uint8_t reg, uint8_t *buffer, uint8_t count) {
    HAL_I2C_Mem_Read(sensor->bus, (sensor->address << 1), reg, I2C_MEMADD_SIZE_8BIT, buffer, count, 1000);
}

/**
 * @brief Write 8-bit register
 */
static void VL53L0X_WriteReg8(VL53L0X_Pololu_t *sensor, uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(sensor->bus, (sensor->address << 1), reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
}

/**
 * @brief Write 16-bit register (big-endian)
 */
static void VL53L0X_WriteReg16(VL53L0X_Pololu_t *sensor, uint8_t reg, uint16_t value) {
    uint8_t buffer[2] = {(uint8_t)(value >> 8), (uint8_t)value};
    
    HAL_I2C_Mem_Write(sensor->bus, (sensor->address << 1), reg, I2C_MEMADD_SIZE_8BIT, buffer, 2, 1000);
}

/**
 * @brief Write 32-bit register (big-endian)
 */
static void VL53L0X_WriteReg32(VL53L0X_Pololu_t *sensor, uint8_t reg, uint32_t value) {
    uint8_t buffer[4] = {
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)value
    };
    
    HAL_I2C_Mem_Write(sensor->bus, (sensor->address << 1), reg, I2C_MEMADD_SIZE_8BIT, buffer, 4, 1000);
}

/**
 * @brief Write multiple bytes
 */
static void VL53L0X_WriteMulti(VL53L0X_Pololu_t *sensor, uint8_t reg, uint8_t const *buffer, uint8_t count) {
    HAL_I2C_Mem_Write(sensor->bus, (sensor->address << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buffer, count, 1000);
}

/* ==================== Utility Functions ==================== */

/**
 * @brief Decode sequence step timeout
 */
static uint16_t decodeTimeout(uint16_t reg_val) {
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

/**
 * @brief Encode sequence step timeout
 */
static uint16_t encodeTimeout(uint32_t timeout_mclks) {
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;
    
    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;
        
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }
        
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return 0;
}

/**
 * @brief Convert MCLKs to microseconds
 */
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

/**
 * @brief Convert microseconds to MCLKs
 */
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

/* ==================== Sensor Calibration Functions ==================== */

/**
 * @brief Get SPAD information
 */
static bool getSpadInfo(VL53L0X_Pololu_t *sensor, uint8_t *count, bool *type_is_aperture) {
    uint8_t tmp;
    uint32_t timeout_start_ms;
    uint32_t loop_count = 0;
    
    VL53L0X_WriteReg8(sensor, 0x80, 0x01);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x00, 0x00);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x06);
    VL53L0X_WriteReg8(sensor, 0x83, VL53L0X_ReadReg8(sensor, 0x83) | 0x04);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x07);
    VL53L0X_WriteReg8(sensor, 0x81, 0x01);
    
    VL53L0X_WriteReg8(sensor, 0x80, 0x01);
    
    VL53L0X_WriteReg8(sensor, 0x94, 0x6b);
    VL53L0X_WriteReg8(sensor, 0x83, 0x00);
    
    startTimeout();
    while (VL53L0X_ReadReg8(sensor, 0x83) == 0x00) {
        loop_count++;
        if (loop_count > 10000) {  // Safety: max 10k iterations
            return false;
        }
        if (checkTimeoutExpired()) return false;
    }
    
    VL53L0X_WriteReg8(sensor, 0x83, 0x01);
    tmp = VL53L0X_ReadReg8(sensor, 0x92);
    
    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;
    
    VL53L0X_WriteReg8(sensor, 0x81, 0x00);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x06);
    VL53L0X_WriteReg8(sensor, 0x83, VL53L0X_ReadReg8(sensor, 0x83) & ~0x04);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x00, 0x01);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x80, 0x00);
    
    return true;
}

/**
 * @brief Get sequence step enables
 */
static void getSequenceStepEnables(VL53L0X_Pololu_t *sensor, VL53L0X_SequenceStepEnables *enables) {
    uint8_t sequence_config = VL53L0X_ReadReg8(sensor, SYSTEM_SEQUENCE_CONFIG);
    
    enables->tcc         = (sequence_config >> 4) & 0x1;
    enables->dss         = (sequence_config >> 3) & 0x1;
    enables->msrc        = (sequence_config >> 2) & 0x1;
    enables->pre_range   = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

/**
 * @brief Get sequence step timeouts
 */
static void getSequenceStepTimeouts(VL53L0X_Pololu_t *sensor, 
                                    VL53L0X_SequenceStepEnables const *enables,
                                    VL53L0X_SequenceStepTimeouts *timeouts) {
    timeouts->pre_range_vcsel_period_pclks = decodeVcselPeriod(VL53L0X_ReadReg8(sensor, PRE_RANGE_CONFIG_VCSEL_PERIOD));
    
    timeouts->msrc_dss_tcc_mclks = VL53L0X_ReadReg8(sensor, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                                                           timeouts->pre_range_vcsel_period_pclks);
    
    timeouts->pre_range_mclks = decodeTimeout(VL53L0X_ReadReg16(sensor, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                                                        timeouts->pre_range_vcsel_period_pclks);
    
    timeouts->final_range_vcsel_period_pclks = decodeVcselPeriod(VL53L0X_ReadReg8(sensor, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    
    timeouts->final_range_mclks = decodeTimeout(VL53L0X_ReadReg16(sensor, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    
    if (enables->pre_range) {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }
    
    timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                                                          timeouts->final_range_vcsel_period_pclks);
}

/**
 * @brief Perform single reference calibration
 */
static bool performSingleRefCalibration(VL53L0X_Pololu_t *sensor, uint8_t vhv_init_byte) {
    uint32_t timeout_start_ms;
    
    VL53L0X_WriteReg8(sensor, SYSRANGE_START, 0x01 | vhv_init_byte);
    
    startTimeout();
    while ((VL53L0X_ReadReg8(sensor, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired()) return false;
    }
    
    VL53L0X_WriteReg8(sensor, SYSTEM_INTERRUPT_CLEAR, 0x01);
    VL53L0X_WriteReg8(sensor, SYSRANGE_START, 0x00);
    
    return true;
}

/* ==================== Public API Implementation ==================== */

void VL53L0X_Pololu_Init(VL53L0X_Pololu_t *sensor, I2C_HandleTypeDef *hi2c, uint8_t address) {
    sensor->bus = hi2c;
    sensor->address = address;
    sensor->io_timeout = 0;  // No timeout by default
    sensor->did_timeout = false;
    sensor->measurement_timing_budget_us = 33000;  // 33ms default
    sensor->stop_variable = 0;
}

void VL53L0X_Pololu_SetAddress(VL53L0X_Pololu_t *sensor, uint8_t new_addr) {
    VL53L0X_WriteReg8(sensor, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    sensor->address = new_addr;
}

bool VL53L0X_Pololu_Setup(VL53L0X_Pololu_t *sensor, bool io_2v8) {
    uint32_t timeout_start_ms;
    uint8_t spad_count;
    bool spad_type_is_aperture;
    uint8_t ref_spad_map[6];
    uint8_t first_spad_to_enable;
    uint8_t spads_enabled = 0;
    char msg[80];
    
    UART_SendString("  [Setup] Starting initialization sequence\r\n");
    UART_SendString("  [Setup] Attempting Model ID read from 0x");
    
    uint8_t addr_byte = (sensor->address << 1);
    snprintf(msg, sizeof(msg), "%02X\r\n", addr_byte);
    UART_SendString(msg);
    
    /* Check model ID */
    uint8_t model_id = VL53L0X_ReadReg8(sensor, IDENTIFICATION_MODEL_ID);
    snprintf(msg, sizeof(msg), "  [Setup] Model ID read complete: 0x%02X\r\n", model_id);
    UART_SendString(msg);
    
    if (model_id != 0xEE) {
        snprintf(msg, sizeof(msg), "  ✗ Model ID mismatch: got 0x%02X, expected 0xEE\r\n", model_id);
        UART_SendString(msg);
        return false;  // Device not responding or wrong chip
    }
    UART_SendString("  ✓ Model ID verified (0xEE)\r\n");
    
    /* === VL53L0X_DataInit() begin === */
    
    /* Sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary */
    if (io_2v8) {
        VL53L0X_WriteReg8(sensor, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                         VL53L0X_ReadReg8(sensor, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
    }
    
    /* Set I2C standard mode */
    VL53L0X_WriteReg8(sensor, 0x88, 0x00);
    
    VL53L0X_WriteReg8(sensor, 0x80, 0x01);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x00, 0x00);
    sensor->stop_variable = VL53L0X_ReadReg8(sensor, 0x91);
    VL53L0X_WriteReg8(sensor, 0x00, 0x01);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x80, 0x00);
    
    /* Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks */
    VL53L0X_WriteReg8(sensor, MSRC_CONFIG_CONTROL, VL53L0X_ReadReg8(sensor, MSRC_CONFIG_CONTROL) | 0x12);
    
    /* Set final range signal rate limit to 0.25 MCPS */
    VL53L0X_Pololu_SetSignalRateLimit(sensor, 0.25);
    
    VL53L0X_WriteReg8(sensor, SYSTEM_SEQUENCE_CONFIG, 0xFF);
    
    /* === VL53L0X_DataInit() end === */
    
    /* === VL53L0X_StaticInit() begin === */
    
    if (!getSpadInfo(sensor, &spad_count, &spad_type_is_aperture)) {
        UART_SendString("  ✗ getSpadInfo failed!\r\n");
        return false;
    }
    
    snprintf(msg, sizeof(msg), "  ✓ SPAD info: count=%d, aperture=%d\r\n", spad_count, spad_type_is_aperture);
    UART_SendString(msg);
    
    VL53L0X_ReadMulti(sensor, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    
    /* Set reference SPADs */
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    VL53L0X_WriteReg8(sensor, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
    
    first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    spads_enabled = 0;
    
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }
    
    VL53L0X_WriteMulti(sensor, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    
    /* Load tuning settings - DefaultTuningSettings from vl53l0x_tuning.h */
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x00, 0x00);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x09, 0x00);
    VL53L0X_WriteReg8(sensor, 0x10, 0x00);
    VL53L0X_WriteReg8(sensor, 0x11, 0x00);
    
    VL53L0X_WriteReg8(sensor, 0x24, 0x01);
    VL53L0X_WriteReg8(sensor, 0x25, 0xFF);
    VL53L0X_WriteReg8(sensor, 0x75, 0x00);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x4E, 0x2C);
    VL53L0X_WriteReg8(sensor, 0x48, 0x00);
    VL53L0X_WriteReg8(sensor, 0x30, 0x20);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x30, 0x09);
    VL53L0X_WriteReg8(sensor, 0x54, 0x00);
    VL53L0X_WriteReg8(sensor, 0x31, 0x04);
    VL53L0X_WriteReg8(sensor, 0x32, 0x03);
    VL53L0X_WriteReg8(sensor, 0x40, 0x83);
    VL53L0X_WriteReg8(sensor, 0x46, 0x25);
    VL53L0X_WriteReg8(sensor, 0x60, 0x00);
    VL53L0X_WriteReg8(sensor, 0x27, 0x00);
    VL53L0X_WriteReg8(sensor, 0x50, 0x06);
    VL53L0X_WriteReg8(sensor, 0x51, 0x00);
    VL53L0X_WriteReg8(sensor, 0x52, 0x96);
    VL53L0X_WriteReg8(sensor, 0x56, 0x08);
    VL53L0X_WriteReg8(sensor, 0x57, 0x30);
    VL53L0X_WriteReg8(sensor, 0x61, 0x00);
    VL53L0X_WriteReg8(sensor, 0x62, 0x00);
    VL53L0X_WriteReg8(sensor, 0x64, 0x00);
    VL53L0X_WriteReg8(sensor, 0x65, 0x00);
    VL53L0X_WriteReg8(sensor, 0x66, 0xA0);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x22, 0x32);
    VL53L0X_WriteReg8(sensor, 0x47, 0x14);
    VL53L0X_WriteReg8(sensor, 0x49, 0xFF);
    VL53L0X_WriteReg8(sensor, 0x4A, 0x00);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x7A, 0x0A);
    VL53L0X_WriteReg8(sensor, 0x7B, 0x00);
    VL53L0X_WriteReg8(sensor, 0x78, 0x21);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x23, 0x34);
    VL53L0X_WriteReg8(sensor, 0x42, 0x00);
    VL53L0X_WriteReg8(sensor, 0x44, 0xFF);
    VL53L0X_WriteReg8(sensor, 0x45, 0x26);
    VL53L0X_WriteReg8(sensor, 0x46, 0x05);
    VL53L0X_WriteReg8(sensor, 0x40, 0x40);
    VL53L0X_WriteReg8(sensor, 0x0E, 0x06);
    VL53L0X_WriteReg8(sensor, 0x20, 0x1A);
    VL53L0X_WriteReg8(sensor, 0x43, 0x40);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x34, 0x03);
    VL53L0X_WriteReg8(sensor, 0x35, 0x44);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x31, 0x04);
    VL53L0X_WriteReg8(sensor, 0x4B, 0x09);
    VL53L0X_WriteReg8(sensor, 0x4C, 0x05);
    VL53L0X_WriteReg8(sensor, 0x4D, 0x04);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x44, 0x00);
    VL53L0X_WriteReg8(sensor, 0x45, 0x20);
    VL53L0X_WriteReg8(sensor, 0x47, 0x08);
    VL53L0X_WriteReg8(sensor, 0x48, 0x28);
    VL53L0X_WriteReg8(sensor, 0x67, 0x00);
    VL53L0X_WriteReg8(sensor, 0x70, 0x04);
    VL53L0X_WriteReg8(sensor, 0x71, 0x01);
    VL53L0X_WriteReg8(sensor, 0x72, 0xFE);
    VL53L0X_WriteReg8(sensor, 0x76, 0x00);
    VL53L0X_WriteReg8(sensor, 0x77, 0x00);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x0D, 0x01);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x80, 0x01);
    VL53L0X_WriteReg8(sensor, 0x01, 0xF8);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x8E, 0x01);
    VL53L0X_WriteReg8(sensor, 0x00, 0x01);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x80, 0x00);
    
    /* Set interrupt config to new sample ready */
    VL53L0X_WriteReg8(sensor, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    VL53L0X_WriteReg8(sensor, GPIO_HV_MUX_ACTIVE_HIGH, VL53L0X_ReadReg8(sensor, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    VL53L0X_WriteReg8(sensor, SYSTEM_INTERRUPT_CLEAR, 0x01);
    
    sensor->measurement_timing_budget_us = VL53L0X_Pololu_GetMeasurementTimingBudget(sensor);
    
    /* Disable MSRC and TCC by default */
    VL53L0X_WriteReg8(sensor, SYSTEM_SEQUENCE_CONFIG, 0xE8);
    
    /* Recalculate timing budget */
    VL53L0X_Pololu_SetMeasurementTimingBudget(sensor, sensor->measurement_timing_budget_us);
    
    /* === VL53L0X_StaticInit() end === */
    
    /* === VL53L0X_PerformRefCalibration() begin === */
    
    VL53L0X_WriteReg8(sensor, SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(sensor, 0x40)) {
        UART_SendString("  ✗ RefCalib 1 failed!\r\n");
        return false;
    }
    
    VL53L0X_WriteReg8(sensor, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(sensor, 0x00)) {
        UART_SendString("  ✗ RefCalib 2 failed!\r\n");
        return false;
    }
    
    VL53L0X_WriteReg8(sensor, SYSTEM_SEQUENCE_CONFIG, 0xE8);
    UART_SendString("  ✓ Setup complete!\r\n");
    
    /* === VL53L0X_PerformRefCalibration() end === */
    
    return true;
}

uint16_t VL53L0X_Pololu_ReadRangeSingleMillimeters(VL53L0X_Pololu_t *sensor) {
    uint32_t timeout_start_ms;
    
    VL53L0X_WriteReg8(sensor, 0x80, 0x01);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x00, 0x00);
    VL53L0X_WriteReg8(sensor, 0x91, sensor->stop_variable);
    VL53L0X_WriteReg8(sensor, 0x00, 0x01);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x80, 0x00);
    
    VL53L0X_WriteReg8(sensor, SYSRANGE_START, 0x01);
    
    /* Wait until start bit has been cleared */
    startTimeout();
    while (VL53L0X_ReadReg8(sensor, SYSRANGE_START) & 0x01) {
        if (checkTimeoutExpired()) {
            sensor->did_timeout = true;
            return 65535;
        }
    }
    
    return VL53L0X_Pololu_ReadRangeContinuousMillimeters(sensor);
}

uint16_t VL53L0X_Pololu_ReadRangeContinuousMillimeters(VL53L0X_Pololu_t *sensor) {
    uint32_t timeout_start_ms;
    uint16_t range;
    
    startTimeout();
    while ((VL53L0X_ReadReg8(sensor, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired()) {
            sensor->did_timeout = true;
            return 65535;
        }
    }
    
    range = VL53L0X_ReadReg16(sensor, RESULT_RANGE_STATUS + 10);
    VL53L0X_WriteReg8(sensor, SYSTEM_INTERRUPT_CLEAR, 0x01);
    
    return range;
}

void VL53L0X_Pololu_StartContinuous(VL53L0X_Pololu_t *sensor, uint32_t period_ms) {
    VL53L0X_WriteReg8(sensor, 0x80, 0x01);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x00, 0x00);
    VL53L0X_WriteReg8(sensor, 0x91, sensor->stop_variable);
    VL53L0X_WriteReg8(sensor, 0x00, 0x01);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
    VL53L0X_WriteReg8(sensor, 0x80, 0x00);
    
    if (period_ms != 0) {
        uint16_t osc_calibrate_val = VL53L0X_ReadReg16(sensor, OSC_CALIBRATE_VAL);
        
        if (osc_calibrate_val != 0) {
            period_ms *= osc_calibrate_val;
        }
        
        VL53L0X_WriteReg32(sensor, 0x04, period_ms);
        VL53L0X_WriteReg8(sensor, SYSRANGE_START, 0x04);
    } else {
        VL53L0X_WriteReg8(sensor, SYSRANGE_START, 0x02);
    }
}

void VL53L0X_Pololu_StopContinuous(VL53L0X_Pololu_t *sensor) {
    VL53L0X_WriteReg8(sensor, SYSRANGE_START, 0x01);
    
    VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
    VL53L0X_WriteReg8(sensor, 0x00, 0x00);
    VL53L0X_WriteReg8(sensor, 0x91, 0x00);
    VL53L0X_WriteReg8(sensor, 0x00, 0x01);
    VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
}

bool VL53L0X_Pololu_SetSignalRateLimit(VL53L0X_Pololu_t *sensor, float limit_Mcps) {
    if (limit_Mcps < 0 || limit_Mcps > 511.99) return false;
    
    VL53L0X_WriteReg16(sensor, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
    return true;
}

float VL53L0X_Pololu_GetSignalRateLimit(VL53L0X_Pololu_t *sensor) {
    return (float)VL53L0X_ReadReg16(sensor, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool VL53L0X_Pololu_SetMeasurementTimingBudget(VL53L0X_Pololu_t *sensor, uint32_t budget_us) {
    VL53L0X_SequenceStepEnables enables;
    VL53L0X_SequenceStepTimeouts timeouts;
    
    uint16_t const StartOverhead     = 1910;
    uint16_t const EndOverhead       = 960;
    uint16_t const MsrcOverhead      = 660;
    uint16_t const TccOverhead       = 590;
    uint16_t const DssOverhead       = 690;
    uint16_t const PreRangeOverhead  = 660;
    uint16_t const FinalRangeOverhead = 550;
    
    uint32_t used_budget_us = StartOverhead + EndOverhead;
    
    getSequenceStepEnables(sensor, &enables);
    getSequenceStepTimeouts(sensor, &enables, &timeouts);
    
    if (enables.tcc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }
    
    if (enables.dss) {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }
    
    if (enables.pre_range) {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }
    
    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;
        
        if (used_budget_us > budget_us) {
            return false;
        }
        
        uint32_t final_range_timeout_us = budget_us - used_budget_us;
        
        uint32_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us,
                                                                         timeouts.final_range_vcsel_period_pclks);
        
        if (enables.pre_range) {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }
        
        VL53L0X_WriteReg16(sensor, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                          encodeTimeout(final_range_timeout_mclks));
        
        sensor->measurement_timing_budget_us = budget_us;
    }
    return true;
}

uint32_t VL53L0X_Pololu_GetMeasurementTimingBudget(VL53L0X_Pololu_t *sensor) {
    VL53L0X_SequenceStepEnables enables;
    VL53L0X_SequenceStepTimeouts timeouts;
    
    uint16_t const StartOverhead     = 1910;
    uint16_t const EndOverhead       = 960;
    uint16_t const MsrcOverhead      = 660;
    uint16_t const TccOverhead       = 590;
    uint16_t const DssOverhead       = 690;
    uint16_t const PreRangeOverhead  = 660;
    uint16_t const FinalRangeOverhead = 550;
    
    uint32_t budget_us = StartOverhead + EndOverhead;
    
    getSequenceStepEnables(sensor, &enables);
    getSequenceStepTimeouts(sensor, &enables, &timeouts);
    
    if (enables.tcc) {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }
    
    if (enables.dss) {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }
    
    if (enables.pre_range) {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }
    
    if (enables.final_range) {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }
    
    sensor->measurement_timing_budget_us = budget_us;
    return budget_us;
}

bool VL53L0X_Pololu_SetVcselPulsePeriod(VL53L0X_Pololu_t *sensor, vcselPeriodType type, uint8_t period_pclks) {
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
    
    VL53L0X_SequenceStepEnables enables;
    VL53L0X_SequenceStepTimeouts timeouts;
    
    getSequenceStepEnables(sensor, &enables);
    getSequenceStepTimeouts(sensor, &enables, &timeouts);
    
    if (type == VcselPeriodPreRange) {
        switch (period_pclks) {
            case 12: VL53L0X_WriteReg8(sensor, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
            case 14: VL53L0X_WriteReg8(sensor, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
            case 16: VL53L0X_WriteReg8(sensor, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
            case 18: VL53L0X_WriteReg8(sensor, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
            default: return false;
        }
        VL53L0X_WriteReg8(sensor, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        VL53L0X_WriteReg8(sensor, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
        
        uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
        VL53L0X_WriteReg16(sensor, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));
        
        uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
        VL53L0X_WriteReg8(sensor, MSRC_CONFIG_TIMEOUT_MACROP,
                         (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
    } else if (type == VcselPeriodFinalRange) {
        switch (period_pclks) {
            case 8:
                VL53L0X_WriteReg8(sensor, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                VL53L0X_WriteReg8(sensor, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_WriteReg8(sensor, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                VL53L0X_WriteReg8(sensor, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
                VL53L0X_WriteReg8(sensor, ALGO_PHASECAL_LIM, 0x30);
                VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
                break;
            
            case 10:
                VL53L0X_WriteReg8(sensor, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                VL53L0X_WriteReg8(sensor, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_WriteReg8(sensor, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_WriteReg8(sensor, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
                VL53L0X_WriteReg8(sensor, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
                break;
            
            case 12:
                VL53L0X_WriteReg8(sensor, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                VL53L0X_WriteReg8(sensor, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_WriteReg8(sensor, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_WriteReg8(sensor, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
                VL53L0X_WriteReg8(sensor, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
                break;
            
            case 14:
                VL53L0X_WriteReg8(sensor, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                VL53L0X_WriteReg8(sensor, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_WriteReg8(sensor, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_WriteReg8(sensor, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                VL53L0X_WriteReg8(sensor, 0xFF, 0x01);
                VL53L0X_WriteReg8(sensor, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_WriteReg8(sensor, 0xFF, 0x00);
                break;
            
            default:
                return false;
        }
        
        VL53L0X_WriteReg8(sensor, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
        
        uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
        
        if (enables.pre_range) {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }
        
        VL53L0X_WriteReg16(sensor, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
    } else {
        return false;
    }
    
    VL53L0X_Pololu_SetMeasurementTimingBudget(sensor, sensor->measurement_timing_budget_us);
    
    uint8_t sequence_config = VL53L0X_ReadReg8(sensor, SYSTEM_SEQUENCE_CONFIG);
    VL53L0X_WriteReg8(sensor, SYSTEM_SEQUENCE_CONFIG, 0x02);
    performSingleRefCalibration(sensor, 0x0);
    VL53L0X_WriteReg8(sensor, SYSTEM_SEQUENCE_CONFIG, sequence_config);
    
    return true;
}

bool VL53L0X_Pololu_TimeoutOccurred(VL53L0X_Pololu_t *sensor) {
    bool tmp = sensor->did_timeout;
    sensor->did_timeout = false;
    return tmp;
}
