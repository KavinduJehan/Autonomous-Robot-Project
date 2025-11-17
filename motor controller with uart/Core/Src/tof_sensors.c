/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : tof_sensors.c
  * @brief          : VL53L0X ToF Sensors Module Implementation
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "tof_sensors.h"
#include "main.h"
#include "cmsis_os.h"
#include "vl53l0x_api.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
/* Software I2C implementation - no HAL I2C driver needed */

/* Private software I2C functions */
static void SW_I2C_Init(void);
static void SW_I2C_Start(void);
static void SW_I2C_Stop(void);
static void SW_I2C_WriteByte(uint8_t data);
static uint8_t SW_I2C_ReadByte(uint8_t ack);
static uint8_t SW_I2C_WaitAck(void);
static void SW_I2C_Delay(void);
static void SW_I2C_SDA_OUT(void);
static void SW_I2C_SDA_IN(void);
static void SW_I2C_SDA_SET(void);
static void SW_I2C_SDA_CLR(void);
static void SW_I2C_SCL_SET(void);
static void SW_I2C_SCL_CLR(void);
static uint8_t SW_I2C_SDA_READ(void);

#define VL53L0X_SINGLE_RANGING_BUDGET_US 33000U

#define SW_I2C_DELAY_US 5  // Delay for ~100kHz I2C speed
#define SW_I2C_DELAY_US_SLOW 20  // Slow mode for debugging (~25kHz)

/* Debug: slow I2C mode for troubleshooting */
static bool debug_slow_i2c = true;  // Start in slow mode by default

/* Global sensor data array */
ToF_Data_t tof_sensors[TOF_SENSOR_COUNT];

/* ST API device instances */
static VL53L0X_Dev_t vl53_devices[TOF_SENSOR_COUNT];

/* Shared measurement buffer */
static VL53L0X_RangingMeasurementData_t vl53_measurement;

/* Sensor configuration arrays */
static const uint8_t sensor_addresses[TOF_SENSOR_COUNT] = {
    TOF_FL_ADDR,    // Front-Left
    TOF_FR_ADDR,    // Front-Right
    TOF_L_ADDR,     // Left
    TOF_R_ADDR      // Right
};

static const uint16_t xshut_pins[TOF_SENSOR_COUNT] = {
    TOF_FL_XSHUT_PIN,   // Front-Left XSHUT
    TOF_FR_XSHUT_PIN,   // Front-Right XSHUT
    TOF_L_XSHUT_PIN,    // Left XSHUT
    TOF_R_XSHUT_PIN     // Right XSHUT
};

/* Private function prototypes -----------------------------------------------*/
static void ToF_PowerDown(ToF_Sensor_ID_t sensor);
static void ToF_PowerUp(ToF_Sensor_ID_t sensor);
static HAL_StatusTypeDef ToF_ConfigureSensor(uint8_t addr);
static HAL_StatusTypeDef ToF_WaitForDataReady(uint8_t addr, uint32_t timeout_ms);
static uint8_t ToF_DecodeStatus(uint8_t range_status);
static void ToF_LogError(ToF_Sensor_ID_t sensor, const char* stage, int32_t detail);
static VL53L0X_Error ToF_ConfigureWithStApi(ToF_Sensor_ID_t sensor);
static VL53L0X_DEV ToF_GetDevice(ToF_Sensor_ID_t sensor);

/* External function declarations for UART */
extern void UART_SendString(const char* s);
extern void UART_SendUInt(uint32_t v);
extern void UART_SendCRLF(void);

/* Error logging throttle counters */
static uint32_t tof_last_error_log[TOF_SENSOR_COUNT] = {0};

/**
  * @brief  Initialize ToF sensors system
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_Init(void)
{
    HAL_StatusTypeDef status;
    
    // Initialize GPIO pins
    status = ToF_GPIO_Init();
    if (status != HAL_OK) return status;
    
    // Initialize I2C interface
    status = ToF_I2C_Init();
    if (status != HAL_OK) return status;
    
    // Initialize all sensors
    status = ToF_Sensors_Init();
    if (status != HAL_OK) return status;
    
    // Initialize sensor data structures
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        tof_sensors[i].distance_mm = 0;
        tof_sensors[i].status = TOF_STATUS_NOT_READY;
        tof_sensors[i].valid = false;
        tof_sensors[i].timestamp = 0;
    }
    
    return HAL_OK;
}

/**
  * @brief  Initialize GPIO pins for ToF sensors
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure I2C GPIO pins (PB9=SDA, PB10=SCL) for software I2C */
    GPIO_InitStruct.Pin = TOF_I2C_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // SCL as output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TOF_I2C_GPIO_PORT, &GPIO_InitStruct);
    
    /* SDA will be configured dynamically in software I2C functions */
    
    /* Configure XSHUT control pins (PA4-PA7) as outputs */
    GPIO_InitStruct.Pin = TOF_FL_XSHUT_PIN | TOF_FR_XSHUT_PIN | 
                          TOF_L_XSHUT_PIN | TOF_R_XSHUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TOF_XSHUT_GPIO_PORT, &GPIO_InitStruct);
    
    /* Initialize all XSHUT pins low (sensors in shutdown) */
    HAL_GPIO_WritePin(TOF_XSHUT_GPIO_PORT, TOF_FL_XSHUT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOF_XSHUT_GPIO_PORT, TOF_FR_XSHUT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOF_XSHUT_GPIO_PORT, TOF_L_XSHUT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOF_XSHUT_GPIO_PORT, TOF_R_XSHUT_PIN, GPIO_PIN_RESET);
    
    return HAL_OK;
}

/**
  * @brief  Initialize I2C interface using software implementation
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_I2C_Init(void)
{
    /* Initialize software I2C */
    SW_I2C_Init();
    return HAL_OK;
}

/**
  * @brief  Initialize all VL53L0X sensors with unique addresses
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_Sensors_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t model_id;
    uint8_t sensors_found = 0;
    const uint32_t power_cycle_delay_ms = debug_slow_i2c ? 40U : 10U;
    const uint32_t boot_delay_ms = debug_slow_i2c ? 35U : 7U;
    const uint32_t detect_retry_delay_ms = debug_slow_i2c ? 15U : 3U;
    const uint32_t address_settle_delay_ms = debug_slow_i2c ? 25U : 5U;
    
    #ifdef DEBUG_UART_HEARTBEAT
    UART_SendString("ToF Init: Starting in ");
    if (debug_slow_i2c) {
        UART_SendString("SLOW I2C mode");
    } else {
        UART_SendString("NORMAL I2C mode");
    }
    UART_SendCRLF();
    #endif
    
    /* Power down all sensors first */
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        ToF_PowerDown(i);
    }
    HAL_Delay(power_cycle_delay_ms);
    
    /* Initialize sensors one by one - skip missing ones gracefully */
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        vl53_devices[i].I2cDevAddr = 0;
        #ifdef DEBUG_UART_HEARTBEAT
        UART_SendString("Init ToF sensor ");
        UART_SendUInt(i);
        UART_SendString("...");
        UART_SendCRLF();
        #endif
        
        /* Power up this sensor only */
        ToF_PowerUp(i);
        HAL_Delay(boot_delay_ms);
        
        /* Try multiple times to read sensor ID - important for reliability */
        uint8_t attempts = 0;
        bool sensor_detected = false;
        
        for (attempts = 0; attempts < 5; attempts++) {
            HAL_Delay(detect_retry_delay_ms);
            status = ToF_ReadReg(VL53L0X_DEFAULT_ADDR, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id);
            
            if (status == HAL_OK && (model_id == 0xEE || model_id == 0x238)) {
                sensor_detected = true;
                break;
            }
        }
        
        /* Verify sensor is responding */
        if (!sensor_detected) {
            #ifdef DEBUG_UART_HEARTBEAT
            UART_SendString("ToF sensor ");
            UART_SendUInt(i);
            UART_SendString(" not found (Status=");
            UART_SendUInt(status);
            UART_SendString(", Model=0x");
            UART_SendUInt(model_id);
            UART_SendString(") - skipping");
            UART_SendCRLF();
            #endif
            
            /* Power down failed sensor and continue */
            ToF_PowerDown(i);
            tof_sensors[i].valid = false;
            tof_sensors[i].status = TOF_STATUS_ERROR;
            vl53_devices[i].I2cDevAddr = 0;
            continue;  // ✅ Continue with next sensor instead of failing
        }
        
        /* Program new I2C address */
        status = ToF_SetAddress(i, sensor_addresses[i]);
        if (status != HAL_OK) {
            #ifdef DEBUG_UART_HEARTBEAT
            UART_SendString("Failed to set address for ToF sensor ");
            UART_SendUInt(i);
            UART_SendCRLF();
            #endif
            ToF_PowerDown(i);
            tof_sensors[i].valid = false;
            tof_sensors[i].status = TOF_STATUS_ERROR;
            vl53_devices[i].I2cDevAddr = 0;
            continue;  // ✅ Continue instead of failing
        }
        
        HAL_Delay(address_settle_delay_ms);
        
        /* Verify new address works with retry */
        bool address_verified = false;
        for (attempts = 0; attempts < 3; attempts++) {
            HAL_Delay(detect_retry_delay_ms);
            status = ToF_ReadReg(sensor_addresses[i], VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id);
            if (status == HAL_OK && (model_id == 0xEE || model_id == 0x238)) {
                address_verified = true;
                break;
            }
        }
        
        if (!address_verified) {
            #ifdef DEBUG_UART_HEARTBEAT
            UART_SendString("ToF sensor ");
            UART_SendUInt(i);
            UART_SendString(" not responding at new address (Status=");
            UART_SendUInt(status);
            UART_SendString(", Model=0x");
            UART_SendUInt(model_id);
            UART_SendString(")");
            UART_SendCRLF();
            #endif
            ToF_PowerDown(i);
            tof_sensors[i].valid = false;
            tof_sensors[i].status = TOF_STATUS_ERROR;
            continue;  // ✅ Continue instead of failing
        }

        /* Configure interrupt behavior so data-ready polling works */
        if (ToF_ConfigureSensor(sensor_addresses[i]) != HAL_OK) {
            #ifdef DEBUG_UART_HEARTBEAT
            UART_SendString("Failed to configure ToF sensor ");
            UART_SendUInt(i);
            UART_SendCRLF();
            #endif
            ToF_PowerDown(i);
            tof_sensors[i].valid = false;
            tof_sensors[i].status = TOF_STATUS_ERROR;
            vl53_devices[i].I2cDevAddr = 0;
            continue;
        }

        VL53L0X_Error st_status = ToF_ConfigureWithStApi(i);
        if (st_status != VL53L0X_ERROR_NONE) {
            ToF_LogError(i, "st_init", st_status);
            ToF_PowerDown(i);
            tof_sensors[i].valid = false;
            tof_sensors[i].status = TOF_STATUS_ERROR;
            vl53_devices[i].I2cDevAddr = 0;
            continue;
        }
        
        /* Sensor initialized successfully */
        sensors_found++;
        tof_sensors[i].valid = false;  // Will be set by first measurement
        tof_sensors[i].status = TOF_STATUS_OK;
        tof_sensors[i].distance_mm = 0;
        tof_sensors[i].timestamp = 0;
        
        #ifdef DEBUG_UART_HEARTBEAT
        UART_SendString("ToF sensor ");
        UART_SendUInt(i);
        UART_SendString(" initialized at address 0x");
        UART_SendUInt(sensor_addresses[i]);
        UART_SendString(" I2cAddr=0x");
        UART_SendUInt(vl53_devices[i].I2cDevAddr);
        UART_SendCRLF();
        #endif
    }
    
    #ifdef DEBUG_UART_HEARTBEAT
    UART_SendString("ToF initialization complete: ");
    UART_SendUInt(sensors_found);
    UART_SendString(" of ");
    UART_SendUInt(TOF_SENSOR_COUNT);
    UART_SendString(" sensors found");
    UART_SendCRLF();
    #endif
    
    /* Return success if at least one sensor is working */
    return (sensors_found > 0) ? HAL_OK : HAL_ERROR;
}

/**
  * @brief  Power down a specific sensor
  * @param  sensor: Sensor ID to power down
  * @retval None
  */
static void ToF_PowerDown(ToF_Sensor_ID_t sensor)
{
    if (sensor < TOF_SENSOR_COUNT) {
        HAL_GPIO_WritePin(TOF_XSHUT_GPIO_PORT, xshut_pins[sensor], GPIO_PIN_RESET);
    }
}

/**
  * @brief  Power up a specific sensor
  * @param  sensor: Sensor ID to power up
  * @retval None
  */
static void ToF_PowerUp(ToF_Sensor_ID_t sensor)
{
    if (sensor < TOF_SENSOR_COUNT) {
        HAL_GPIO_WritePin(TOF_XSHUT_GPIO_PORT, xshut_pins[sensor], GPIO_PIN_SET);
    }
}

/**
  * @brief  Configure interrupt behavior for a VL53L0X sensor
  * @param  addr: Sensor I2C address (8-bit bus value)
  * @retval HAL status
  */
static HAL_StatusTypeDef ToF_ConfigureSensor(uint8_t addr)
{
    /* Set GPIO/interrupt to "new sample ready" (0x04) */
    if (ToF_WriteReg(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Clear any pending interrupts before first measurement */
    if (ToF_WriteReg(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static VL53L0X_DEV ToF_GetDevice(ToF_Sensor_ID_t sensor)
{
    if (sensor >= TOF_SENSOR_COUNT) {
        return NULL;
    }
    return &vl53_devices[sensor];
}

static VL53L0X_Error ToF_ConfigureWithStApi(ToF_Sensor_ID_t sensor)
{
    VL53L0X_DEV dev = ToF_GetDevice(sensor);
    if (dev == NULL) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }

    memset(dev, 0, sizeof(VL53L0X_Dev_t));
    dev->I2cDevAddr = sensor_addresses[sensor];
    dev->comms_type = 1;
    dev->comms_speed_khz = debug_slow_i2c ? 25U : 100U;

    uint32_t ref_spad_count = 0;
    uint8_t is_aperture = 0;
    uint8_t vhv_settings = 0;
    uint8_t phase_cal = 0;

    VL53L0X_Error status = VL53L0X_DataInit(dev);
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_StaticInit(dev);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_PerformRefSpadManagement(dev, &ref_spad_count, &is_aperture);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_PerformRefCalibration(dev, &vhv_settings, &phase_cal);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, VL53L0X_SINGLE_RANGING_BUDGET_US);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    }

    return status;
}

/**
  * @brief  Write a register to VL53L0X sensor
  * @param  addr: I2C address
  * @param  reg: Register address
  * @param  value: Value to write
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_WriteReg(uint8_t addr, uint8_t reg, uint8_t value)
{
    SW_I2C_Start();
    SW_I2C_WriteByte(addr);  // Device address + write bit
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }
    
    SW_I2C_WriteByte(reg);   // Register address
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }
    
    SW_I2C_WriteByte(value); // Register value
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }
    
    SW_I2C_Stop();
    return HAL_OK;
}

/**
  * @brief  Read a register from VL53L0X sensor
  * @param  addr: I2C address
  * @param  reg: Register address
  * @param  value: Pointer to store read value
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_ReadReg(uint8_t addr, uint8_t reg, uint8_t* value)
{
    /* Write register address */
    SW_I2C_Start();
    SW_I2C_WriteByte(addr);  // Device address + write bit
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }
    
    SW_I2C_WriteByte(reg);   // Register address
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }
    
    /* Read register value */
    SW_I2C_Start();          // Repeated start
    SW_I2C_WriteByte(addr | 0x01);  // Device address + read bit
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }
    
    *value = SW_I2C_ReadByte(0);  // Read with NACK
    SW_I2C_Stop();
    return HAL_OK;
}

/**
  * @brief  Read multiple registers from VL53L0X sensor
  * @param  addr: I2C address
  * @param  reg: Starting register address
  * @param  data: Pointer to data buffer
  * @param  length: Number of bytes to read
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_ReadMulti(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t length)
{
    /* Write register address */
    SW_I2C_Start();
    SW_I2C_WriteByte(addr);  // Device address + write bit
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }
    
    SW_I2C_WriteByte(reg);   // Register address
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }
    
    /* Read multiple bytes */
    SW_I2C_Start();          // Repeated start
    SW_I2C_WriteByte(addr | 0x01);  // Device address + read bit
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }
    
    for (uint16_t i = 0; i < length; i++) {
        data[i] = SW_I2C_ReadByte((i == length - 1) ? 0 : 1);  // ACK for all except last
    }
    
    SW_I2C_Stop();
    return HAL_OK;
}

/**
  * @brief  Write multiple bytes starting at a given register
  * @param  addr: I2C address
  * @param  reg: Starting register address
  * @param  data: Pointer to data buffer
  * @param  length: Number of bytes to write
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_WriteMulti(uint8_t addr, uint8_t reg, const uint8_t* data, uint16_t length)
{
    SW_I2C_Start();
    SW_I2C_WriteByte(addr);
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }

    SW_I2C_WriteByte(reg);
    if (SW_I2C_WaitAck() != 0) {
        SW_I2C_Stop();
        return HAL_ERROR;
    }

    for (uint16_t i = 0; i < length; i++) {
        SW_I2C_WriteByte(data[i]);
        if (SW_I2C_WaitAck() != 0) {
            SW_I2C_Stop();
            return HAL_ERROR;
        }
    }

    SW_I2C_Stop();
    return HAL_OK;
}

/**
  * @brief  Set new I2C address for VL53L0X sensor
  * @param  sensor: Sensor ID
  * @param  new_addr: New I2C address (7-bit, unshifted)
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_SetAddress(ToF_Sensor_ID_t sensor, uint8_t new_addr)
{
    /* new_addr is the 8-bit bus value (write/read bit in LSB); sensor register wants 7-bit */
    uint8_t seven_bit_addr = new_addr >> 1;  // e.g., 0x54 -> 0x2A
    return ToF_WriteReg(VL53L0X_DEFAULT_ADDR, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, seven_bit_addr);
}

/**
  * @brief  Start distance measurement
  * @param  addr: I2C address of sensor
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_StartMeasurement(uint8_t addr)
{
    return ToF_WriteReg(addr, VL53L0X_REG_SYSRANGE_START, 0x01);
}

/**
  * @brief  Check if measurement data is ready
  * @param  addr: I2C address of sensor
  * @retval true if data ready, false otherwise
  */
bool ToF_IsDataReady(uint8_t addr)
{
    uint8_t status;
    if (ToF_ReadReg(addr, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &status) == HAL_OK) {
        return (status & 0x07) != 0;
    }
    return false;
}

/**
  * @brief  Read distance measurement
  * @param  addr: I2C address of sensor
  * @param  distance: Pointer to store distance (mm)
  * @param  status: Pointer to store measurement status
  * @retval HAL status
  */
HAL_StatusTypeDef ToF_ReadDistance(uint8_t addr, uint16_t* distance, uint8_t* status)
{
    HAL_StatusTypeDef hal_status;
    uint8_t data[14];
    
    /* Read range status and distance */
    hal_status = ToF_ReadMulti(addr, VL53L0X_REG_RESULT_RANGE_STATUS, data, 14);
    if (hal_status != HAL_OK) return hal_status;
    
    /* Extract distance (bytes 10-11) */
    *distance = (uint16_t)((data[10] << 8) | data[11]);
    
    /* Decode and return status */
    *status = ToF_DecodeStatus(data[0] & 0x78);
    
    /* Clear GPIO/interrupt flag so next measurement can signal completion */
    ToF_WriteReg(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    
    return HAL_OK;
}

/**
  * @brief  Wait for measurement to complete
  * @param  addr: I2C address of sensor
  * @param  timeout_ms: Timeout in milliseconds
  * @retval HAL status
  */
static HAL_StatusTypeDef ToF_WaitForDataReady(uint8_t addr, uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();
    
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (ToF_IsDataReady(addr)) {
            return HAL_OK;
        }
        osDelay(1);  // Yield to other tasks
    }
    
    return HAL_TIMEOUT;
}

/**
  * @brief  Decode VL53L0X range status
  * @param  range_status: Raw range status from sensor
  * @retval Decoded status code
  */
static uint8_t ToF_DecodeStatus(uint8_t range_status)
{
    uint8_t status_code = (range_status <= 7) ? range_status : (range_status >> 3);

    switch (status_code) {
        case 0x00: return TOF_STATUS_OK;           // Good measurement
        case 0x01: return TOF_STATUS_TIMEOUT;      // Sigma fail
        case 0x02: return TOF_STATUS_ERROR;        // Signal fail
        case 0x03: return TOF_STATUS_OK;           // Min range fail (still valid)
        case 0x04: return TOF_STATUS_TIMEOUT;      // Phase fail
        case 0x05: return TOF_STATUS_ERROR;        // Hardware fail
        default:   return TOF_STATUS_ERROR;        // Unknown status
    }
}

/**
  * @brief  Update all ToF sensors (blocking)
  * @retval None
  */
void ToF_UpdateAll(void)
{
    uint32_t current_time = HAL_GetTick();
    static uint32_t debug_counter = 0;
    debug_counter++;
    
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        /* Skip sensors that failed during initialization */
        if (tof_sensors[i].status == TOF_STATUS_ERROR) {
            continue;  // ✅ Skip missing sensors gracefully
        }

        VL53L0X_DEV dev = ToF_GetDevice(i);
        if (dev == NULL || dev->I2cDevAddr == 0U) {
            tof_sensors[i].valid = false;
            tof_sensors[i].status = TOF_STATUS_ERROR;
            tof_sensors[i].distance_mm = 0;
            /* Reduced logging frequency to every 50 cycles (~1.5s) */
            if ((debug_counter % 50) == 0) {
                UART_SendString("ToF S");
                UART_SendUInt(i);
                UART_SendString(" dev=NULL or addr=0");
                UART_SendCRLF();
            }
            continue;
        }

        VL53L0X_Error st_status = VL53L0X_PerformSingleRangingMeasurement(dev, &vl53_measurement);
        if (st_status != VL53L0X_ERROR_NONE) {
            tof_sensors[i].status = TOF_STATUS_ERROR;
            tof_sensors[i].valid = false;
            tof_sensors[i].distance_mm = 0;
            ToF_LogError(i, "st_meas", st_status);
            continue;
        }

        uint16_t distance = vl53_measurement.RangeMilliMeter;
        uint8_t status = ToF_DecodeStatus(vl53_measurement.RangeStatus);
        bool distance_in_range = (distance >= TOF_MIN_DISTANCE_MM) &&
                                 (distance <= TOF_MAX_DISTANCE_MM);

        tof_sensors[i].distance_mm = distance;
        tof_sensors[i].status = status;
        tof_sensors[i].valid = (status == TOF_STATUS_OK) && distance_in_range;
        tof_sensors[i].timestamp = current_time;

        /* Debug: log first successful measurement ONLY */
        if ((debug_counter == 1) && (status == TOF_STATUS_OK)) {
            UART_SendString("ToF S");
            UART_SendUInt(i);
            UART_SendString(" first: ");
            UART_SendUInt(distance);
            UART_SendString("mm stat=");
            UART_SendUInt(vl53_measurement.RangeStatus);
            UART_SendCRLF();
        }

        if (!distance_in_range || status != TOF_STATUS_OK) {
            ToF_LogError(i, (status == TOF_STATUS_OK) ? "range" : "status",
                         (status << 16) | distance);
        }
    }
}

/**
  * @brief  ToF sensors task (FreeRTOS)
  * @param  argument: Task argument (unused)
  * @retval None
  */
void ToF_Task(void const * argument)
{
    /* Task initialization */
    const uint32_t cycle_time_ms = TOF_MEASUREMENT_INTERVAL_MS;
    uint32_t iteration = 0;
    
    /* Always log startup - remove ifdef */
    UART_SendString("Task: ToF started");
    UART_SendCRLF();
    
    /* Wait for initialization to complete */
    osDelay(100);
    
    for(;;) {
        iteration++;
        
        /* Removed simple heartbeat to reduce UART spam */
        
        /* Debug: Log every 10th iteration */
        if ((iteration % 10) == 0) {
            UART_SendString("ToF update #");
            UART_SendUInt(iteration);
            UART_SendCRLF();
        }
        
        /* Update all sensors */
        ToF_UpdateAll();
        
        /* Wait for next cycle using CMSIS-RTOS */
        osDelay(cycle_time_ms);
    }
}

/**
  * @brief  Get distance from specific sensor
  * @param  sensor: Sensor ID
  * @retval Distance in millimeters (0 if invalid)
  */
uint16_t ToF_GetDistance(ToF_Sensor_ID_t sensor)
{
    if (sensor < TOF_SENSOR_COUNT && tof_sensors[sensor].valid) {
        return tof_sensors[sensor].distance_mm;
    }
    return 0;
}

/**
  * @brief  Check if sensor data is valid
  * @param  sensor: Sensor ID
  * @retval true if valid, false otherwise
  */
bool ToF_IsValid(ToF_Sensor_ID_t sensor)
{
    return (sensor < TOF_SENSOR_COUNT) && tof_sensors[sensor].valid;
}

/**
  * @brief  Get age of sensor data in milliseconds
  * @param  sensor: Sensor ID
  * @retval Age in milliseconds
  */
uint32_t ToF_GetAge(ToF_Sensor_ID_t sensor)
{
    if (sensor < TOF_SENSOR_COUNT) {
        return HAL_GetTick() - tof_sensors[sensor].timestamp;
    }
    return UINT32_MAX;
}

/**
  * @brief  Detect junction (opening on both sides)
  * @retval true if junction detected
  */
bool ToF_DetectJunction(void)
{
    return ToF_DetectLeftJunction() && ToF_DetectRightJunction();
}

/**
  * @brief  Detect left junction (opening on left side)
  * @retval true if left junction detected
  */
bool ToF_DetectLeftJunction(void)
{
    uint16_t left_dist = ToF_GetDistance(TOF_SENSOR_L);
    return (left_dist > TOF_JUNCTION_DETECT_THRESHOLD) && ToF_IsValid(TOF_SENSOR_L);
}

/**
  * @brief  Detect right junction (opening on right side)
  * @retval true if right junction detected
  */
bool ToF_DetectRightJunction(void)
{
    /* Handle missing right sensor gracefully */
    if (TOF_SENSOR_R >= TOF_ACTIVE_COUNT || !ToF_IsValid(TOF_SENSOR_R)) {
        return false;  // No right sensor available or sensor failed
    }
    
    uint16_t right_dist = ToF_GetDistance(TOF_SENSOR_R);
    return (right_dist > TOF_JUNCTION_DETECT_THRESHOLD);
}

/**
  * @brief  Detect front obstacle requiring action
  * @retval true if obstacle detected
  */
bool ToF_DetectFrontObstacle(void)
{
    uint16_t fl_dist = ToF_GetDistance(TOF_SENSOR_FL);
    uint16_t fr_dist = ToF_GetDistance(TOF_SENSOR_FR);
    
    bool fl_obstacle = ToF_IsValid(TOF_SENSOR_FL) && (fl_dist < TOF_OBSTACLE_SLOW_THRESHOLD);
    bool fr_obstacle = ToF_IsValid(TOF_SENSOR_FR) && (fr_dist < TOF_OBSTACLE_SLOW_THRESHOLD);
    
    return fl_obstacle || fr_obstacle;
}

/**
  * @brief  Get closest distance from all sensors
  * @retval Closest distance in millimeters
  */
uint16_t ToF_GetClosestDistance(void)
{
    uint16_t closest = TOF_MAX_DISTANCE_MM;
    
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (ToF_IsValid(i)) {
            uint16_t dist = ToF_GetDistance(i);
            if (dist < closest) {
                closest = dist;
            }
        }
    }
    
    return closest;
}

/**
  * @brief  Get ID of sensor with closest reading
  * @retval Sensor ID with closest reading
  */
ToF_Sensor_ID_t ToF_GetClosestSensor(void)
{
    ToF_Sensor_ID_t closest_sensor = TOF_SENSOR_FL;
    uint16_t closest_distance = TOF_MAX_DISTANCE_MM;
    
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (ToF_IsValid(i)) {
            uint16_t dist = ToF_GetDistance(i);
            if (dist < closest_distance) {
                closest_distance = dist;
                closest_sensor = i;
            }
        }
    }
    
    return closest_sensor;
}

/**
  * @brief  Get left wall distance for wall following
  * @retval Distance to left wall in millimeters
  */
uint16_t ToF_GetLeftWallDistance(void)
{
    return ToF_GetDistance(TOF_SENSOR_L);
}

/**
  * @brief  Get right wall distance for wall following
  * @retval Distance to right wall in millimeters (0 if sensor not available)
  */
uint16_t ToF_GetRightWallDistance(void)
{
    /* Handle missing right sensor gracefully */
    if (TOF_SENSOR_R >= TOF_ACTIVE_COUNT || !ToF_IsValid(TOF_SENSOR_R)) {
        return 0;  // No right sensor data available
    }
    return ToF_GetDistance(TOF_SENSOR_R);
}

/**
  * @brief  Calculate centering error for corridor navigation
  * @retval Error in millimeters (negative = too far left, positive = too far right)
  */
int16_t ToF_GetCenteringError(void)
{
    /* Check if both sensors are available */
    bool left_valid = ToF_IsValid(TOF_SENSOR_L);
    bool right_valid = (TOF_SENSOR_R < TOF_ACTIVE_COUNT) && ToF_IsValid(TOF_SENSOR_R);
    
    if (left_valid && right_valid) {
        uint16_t left_dist = ToF_GetDistance(TOF_SENSOR_L);
        uint16_t right_dist = ToF_GetDistance(TOF_SENSOR_R);
        
        /* Positive error means robot is too far right (right distance > left distance) */
        return (int16_t)right_dist - (int16_t)left_dist;
    }
    
    /* Only left sensor available - use simplified navigation */
    if (left_valid) {
        uint16_t left_dist = ToF_GetDistance(TOF_SENSOR_L);
        /* Try to maintain ~20cm from left wall */
        return (int16_t)left_dist - TOF_WALL_FOLLOW_THRESHOLD;
    }
    
    return 0;  // No correction if no sensors available
}

/**
  * @brief  Send ToF distances via UART
  * @retval None
  */
void ToF_SendDistances(void)
{
    UART_SendString("ToF: FL=");
    UART_SendUInt(ToF_GetDistance(TOF_SENSOR_FL));
    UART_SendString("mm FR=");
    UART_SendUInt(ToF_GetDistance(TOF_SENSOR_FR));
    UART_SendString("mm L=");
    UART_SendUInt(ToF_GetDistance(TOF_SENSOR_L));
    UART_SendString("mm R=");
    UART_SendUInt(ToF_GetDistance(TOF_SENSOR_R));
    UART_SendString("mm");
    UART_SendCRLF();
}

/**
  * @brief  Process ToF-related commands
  * @param  cmd: Command character
  * @retval None
  */
void ToF_ProcessCommand(uint8_t cmd)
{
    switch (cmd) {
        case 'G':  // ToF distances (changed from T to avoid conflict with CMD_RIGHT)
            ToF_SendDistances();
            break;
            
        case 'J':  // Junction detection status
            UART_SendString("Junction: L=");
            UART_SendUInt(ToF_DetectLeftJunction() ? 1 : 0);
            UART_SendString(" R=");
            UART_SendUInt(ToF_DetectRightJunction() ? 1 : 0);
            UART_SendString(" Both=");
            UART_SendUInt(ToF_DetectJunction() ? 1 : 0);
            UART_SendCRLF();
            break;
            
        case 'O':  // Obstacle detection status
            UART_SendString("Obstacle: Front=");
            UART_SendUInt(ToF_DetectFrontObstacle() ? 1 : 0);
            UART_SendString(" Closest=");
            UART_SendUInt(ToF_GetClosestDistance());
            UART_SendString("mm");
            UART_SendCRLF();
            break;

        case 'X':  // ToF hardware test (debug command)
        {
            bool previous_speed_slow = debug_slow_i2c;
            UART_SendString("=== ToF Hardware Test ===");
            UART_SendCRLF();
            
            // Test 1: I2C Bus Check (all sensors powered down)
            UART_SendString("Step 1: Power down all sensors");
            UART_SendCRLF();
            for (int i = 0; i < 4; i++) {
                ToF_PowerDown(i);
            }
            HAL_Delay(20);
            
            // Test 2: Try I2C communication with no sensors (should fail)
            UART_SendString("Step 2: I2C bus test (no sensors)");
            UART_SendCRLF();
            uint8_t test_data = 0;
            HAL_StatusTypeDef status = ToF_ReadReg(VL53L0X_DEFAULT_ADDR, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &test_data);
            UART_SendString("I2C (no sensors): Status=");
            UART_SendUInt(status);
            UART_SendString(" (should be HAL_ERROR=1)");
            UART_SendCRLF();
            
            // Test 3: Test each sensor individually
            UART_SendString("Step 3: Individual sensor tests");
            UART_SendCRLF();
            bool any_success = false;
            for (int i = 0; i < 3; i++) {  // Test first 3 sensors
                UART_SendString("Testing sensor ");
                UART_SendUInt(i);
                UART_SendString(": ");
                
                // Power up only this sensor
                ToF_PowerUp(i);
                HAL_Delay(10);  // Boot delay
                
                // Try multiple reads with increasing delays
                bool success = false;
                for (int attempt = 0; attempt < 3; attempt++) {
                    HAL_Delay(20 * (attempt + 1));  // 20ms, 40ms, 60ms
                    status = ToF_ReadReg(VL53L0X_DEFAULT_ADDR, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &test_data);
                    if (status == HAL_OK && (test_data == 0xEE || test_data == 0x238)) {
                        success = true;
                        any_success = true;
                        break;
                    }
                }
                
                if (success) {
                    UART_SendString("OK (Model=0x");
                    UART_SendUInt(test_data);
                    UART_SendString(")");
                } else {
                    UART_SendString("FAIL (Status=");
                    UART_SendUInt(status);
                    UART_SendString(", Data=0x");
                    UART_SendUInt(test_data);
                    UART_SendString(")");
                }
                UART_SendCRLF();
                
                ToF_PowerDown(i);  // Power down after test
                HAL_Delay(10);
            }
            
            // Test 4: Check I2C timing
            UART_SendString("Step 4: I2C timing test");
            UART_SendCRLF();
            UART_SendString("Current I2C delay: ");
            if (debug_slow_i2c) {
                UART_SendUInt(SW_I2C_DELAY_US_SLOW);
                UART_SendString("us (SLOW mode, ~");
                UART_SendUInt(1000000 / (SW_I2C_DELAY_US_SLOW * 20));
            } else {
                UART_SendUInt(SW_I2C_DELAY_US);
                UART_SendString("us (NORMAL mode, ~");
                UART_SendUInt(1000000 / (SW_I2C_DELAY_US * 20));
            }
            UART_SendString("Hz)");
            UART_SendCRLF();
            
            // Test 5: Try slow I2C mode if normal failed
            if (!any_success) {
                UART_SendString("Step 5: Trying SLOW I2C mode");
                UART_SendCRLF();
                debug_slow_i2c = true;
                
                // Retry sensor 0 with slow I2C
                ToF_PowerUp(0);
                HAL_Delay(50);
                status = ToF_ReadReg(VL53L0X_DEFAULT_ADDR, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &test_data);
                
                UART_SendString("Slow I2C test: ");
                if (status == HAL_OK && (test_data == 0xEE || test_data == 0x238)) {
                    UART_SendString("SUCCESS! Use slow mode.");
                } else {
                    UART_SendString("Still failing (Status=");
                    UART_SendUInt(status);
                    UART_SendString(", Data=0x");
                    UART_SendUInt(test_data);
                    UART_SendString(")");
                }
                UART_SendCRLF();
                
                ToF_PowerDown(0);
                debug_slow_i2c = false;  // Reset to normal
            }
            
            UART_SendString("=== Test Complete ===");
            UART_SendCRLF();

            /* Restore sensors to operational state after the diagnostic run */
            UART_SendString("Restoring ToF sensors...");
            UART_SendCRLF();
            debug_slow_i2c = previous_speed_slow;
            HAL_StatusTypeDef restore_status = ToF_Sensors_Init();
            if (restore_status == HAL_OK) {
                ToF_UpdateAll();
                ToF_SendDistances();
            } else {
                UART_SendString("ToF restore failed (status=");
                UART_SendUInt(restore_status);
                UART_SendString(")");
                UART_SendCRLF();
            }
            break;
        }
            
        case 'Y':  // Toggle slow I2C mode
            debug_slow_i2c = !debug_slow_i2c;
            UART_SendString("I2C Speed: ");
            if (debug_slow_i2c) {
                UART_SendString("SLOW mode (~25kHz) - Use for initialization");
            } else {
                UART_SendString("NORMAL mode (~100kHz) - Use after initialization");
            }
            UART_SendCRLF();
            break;
            
        case 'N':  // Reinitialize ToF sensors (N for New init)
            UART_SendString("=== Reinitializing ToF Sensors ===");
            UART_SendCRLF();
            
            HAL_StatusTypeDef init_status = ToF_Sensors_Init();
            
            if (init_status == HAL_OK) {
                UART_SendString("ToF reinitialization: SUCCESS");
            } else {
                UART_SendString("ToF reinitialization: FAILED");
            }
            UART_SendCRLF();
            
            // Show how many sensors are now valid
            uint8_t valid_count = 0;
            for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
                if (tof_sensors[i].valid) {
                    valid_count++;
                }
            }
            UART_SendString("Valid sensors: ");
            UART_SendUInt(valid_count);
            UART_SendString("/");
            UART_SendUInt(TOF_SENSOR_COUNT);
            UART_SendCRLF();

            /* Immediately update and report distances so the host sees fresh data */
            if (init_status == HAL_OK && valid_count > 0) {
                ToF_UpdateAll();
                ToF_SendDistances();
            }
            break;
            
        default:
            /* Command not handled by ToF module */
            break;
    }
}

/**
  * @brief  Print detailed status of all sensors
  * @retval None
  */
void ToF_PrintStatus(void)
{
    UART_SendString("=== ToF Sensor Status ===");
    UART_SendCRLF();
    
    const char* sensor_names[TOF_SENSOR_COUNT] = {"FL", "FR", "L ", "R "};
    
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        UART_SendString(sensor_names[i]);
        UART_SendString(": ");
        
        if (ToF_IsValid(i)) {
            UART_SendUInt(ToF_GetDistance(i));
            UART_SendString("mm (");
            UART_SendUInt(ToF_GetAge(i));
            UART_SendString("ms ago)");
        } else {
            UART_SendString("INVALID (status=");
            UART_SendUInt(tof_sensors[i].status);
            UART_SendString(")");
        }
        UART_SendCRLF();
    }
    
    UART_SendString("========================");
    UART_SendCRLF();
}

/**
  * @brief  Perform self-test of ToF sensors
  * @retval None
  */
void ToF_SelfTest(void)
{
    UART_SendString("Starting ToF self-test...");
    UART_SendCRLF();
    
    /* Test I2C communication with each sensor */
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        uint8_t model_id;
        HAL_StatusTypeDef status = ToF_ReadReg(sensor_addresses[i], VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id);
        
        UART_SendString("Sensor ");
        UART_SendUInt(i);
        UART_SendString(" (0x");
        UART_SendUInt(sensor_addresses[i]);
        UART_SendString("): ");
        
        if (status == HAL_OK && model_id == 0xEE) {
            UART_SendString("OK");
        } else {
            UART_SendString("FAIL (model=0x");
            UART_SendUInt(model_id);
            UART_SendString(", status=");
            UART_SendUInt(status);
            UART_SendString(")");
        }
        UART_SendCRLF();
    }
    
    /* Update all sensors once */
    ToF_UpdateAll();
    
    /* Print results */
    ToF_PrintStatus();
    
    UART_SendString("ToF self-test complete");
    UART_SendCRLF();
}

/* Private Software I2C Implementation --------------------------------------- */

/**
  * @brief  Software I2C initialization
  * @retval None
  */
static void SW_I2C_Init(void)
{
    /* Set SDA and SCL high (idle state) */
    SW_I2C_SDA_OUT();
    SW_I2C_SDA_SET();
    SW_I2C_SCL_SET();
    SW_I2C_Delay();
}

/**
  * @brief  Generate I2C start condition
  * @retval None
  */
static void SW_I2C_Start(void)
{
    SW_I2C_SDA_OUT();
    SW_I2C_SDA_SET();
    SW_I2C_SCL_SET();
    SW_I2C_Delay();
    SW_I2C_SDA_CLR();  // SDA goes low while SCL is high
    SW_I2C_Delay();
    SW_I2C_SCL_CLR();  // Prepare for data transmission
    SW_I2C_Delay();
}

/**
  * @brief  Generate I2C stop condition
  * @retval None
  */
static void SW_I2C_Stop(void)
{
    SW_I2C_SDA_OUT();
    SW_I2C_SCL_CLR();
    SW_I2C_SDA_CLR();
    SW_I2C_Delay();
    SW_I2C_SCL_SET();  // SCL goes high first
    SW_I2C_Delay();
    SW_I2C_SDA_SET();  // Then SDA goes high
    SW_I2C_Delay();
}

/**
  * @brief  Write one byte via I2C
  * @param  data: Byte to send
  * @retval None
  */
static void SW_I2C_WriteByte(uint8_t data)
{
    SW_I2C_SDA_OUT();
    
    for (uint8_t i = 0; i < 8; i++) {
        SW_I2C_SCL_CLR();
        SW_I2C_Delay();
        
        if (data & 0x80) {
            SW_I2C_SDA_SET();
        } else {
            SW_I2C_SDA_CLR();
        }
        
        data <<= 1;
        SW_I2C_Delay();
        SW_I2C_SCL_SET();
        SW_I2C_Delay();
    }
    
    SW_I2C_SCL_CLR();
    SW_I2C_Delay();
}

/**
  * @brief  Read one byte via I2C
  * @param  ack: 1 to send ACK, 0 to send NACK
  * @retval Received byte
  */
static uint8_t SW_I2C_ReadByte(uint8_t ack)
{
    uint8_t data = 0;
    
    SW_I2C_SDA_IN();  // Set SDA as input
    
    for (uint8_t i = 0; i < 8; i++) {
        SW_I2C_SCL_CLR();
        SW_I2C_Delay();
        SW_I2C_SCL_SET();
        SW_I2C_Delay();
        
        data <<= 1;
        if (SW_I2C_SDA_READ()) {
            data |= 0x01;
        }
    }
    
    SW_I2C_SCL_CLR();
    SW_I2C_Delay();
    
    /* Send ACK/NACK */
    SW_I2C_SDA_OUT();
    if (ack) {
        SW_I2C_SDA_CLR();  // ACK
    } else {
        SW_I2C_SDA_SET();  // NACK
    }
    SW_I2C_Delay();
    SW_I2C_SCL_SET();
    SW_I2C_Delay();
    SW_I2C_SCL_CLR();
    SW_I2C_Delay();
    
    return data;
}

/**
  * @brief  Wait for ACK from slave
  * @retval 0 if ACK received, 1 if NACK/timeout
  */
static uint8_t SW_I2C_WaitAck(void)
{
    uint8_t ack_timeout = 0;
    
    SW_I2C_SDA_IN();   // Set SDA as input
    SW_I2C_SDA_SET();  // Release SDA
    SW_I2C_Delay();
    SW_I2C_SCL_SET();  // Clock high
    SW_I2C_Delay();
    
    while (SW_I2C_SDA_READ()) {
        ack_timeout++;
        if (ack_timeout > 250) {
            SW_I2C_Stop();
            return 1;  // Timeout
        }
    }
    
    SW_I2C_SCL_CLR();
    SW_I2C_Delay();
    return 0;  // ACK received
}

/**
  * @brief  I2C timing delay
  * @retval None
  */
static void SW_I2C_Delay(void)
{
    /* Variable delay for I2C timing - slow mode for debugging */
    uint32_t delay_cycles = debug_slow_i2c ? (SW_I2C_DELAY_US_SLOW * 16) : (SW_I2C_DELAY_US * 16);
    for (volatile uint32_t i = 0; i < delay_cycles; i++) {
        __NOP();
    }
}

/**
  * @brief  Configure SDA as output
  * @retval None
  */
static void SW_I2C_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = TOF_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TOF_I2C_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Configure SDA as input
  * @retval None
  */
static void SW_I2C_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = TOF_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TOF_I2C_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Set SDA pin high
  * @retval None
  */
static void SW_I2C_SDA_SET(void)
{
    HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SDA_PIN, GPIO_PIN_SET);
}

/**
  * @brief  Set SDA pin low
  * @retval None
  */
static void SW_I2C_SDA_CLR(void)
{
    HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SDA_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Set SCL pin high
  * @retval None
  */
static void SW_I2C_SCL_SET(void)
{
    HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SCL_PIN, GPIO_PIN_SET);
}

/**
  * @brief  Set SCL pin low
  * @retval None
  */
static void SW_I2C_SCL_CLR(void)
{
    HAL_GPIO_WritePin(TOF_I2C_GPIO_PORT, TOF_I2C_SCL_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Read SDA pin state
  * @retval Pin state (0 or 1)
  */
static uint8_t SW_I2C_SDA_READ(void)
{
    return HAL_GPIO_ReadPin(TOF_I2C_GPIO_PORT, TOF_I2C_SDA_PIN) ? 1 : 0;
}

/**
  * @brief  Log ToF error messages with throttling
  * @param  sensor: Sensor index
  * @param  stage: Message describing failure point
  * @param  detail: Additional numeric detail (HAL status or reason code)
  * @retval None
  */
static void ToF_LogError(ToF_Sensor_ID_t sensor, const char* stage, int32_t detail)
{
    uint32_t now = HAL_GetTick();
    if (sensor >= TOF_SENSOR_COUNT) {
        return;
    }
    if ((now - tof_last_error_log[sensor]) < 250U) {
        return;  // limit spam to ~4 Hz per sensor
    }
    tof_last_error_log[sensor] = now;

    UART_SendString("ToF ERR S");
    UART_SendUInt(sensor);
    UART_SendString(" ");
    UART_SendString(stage);
    UART_SendString(" detail=");
    UART_SendUInt((uint32_t)detail);
    UART_SendCRLF();
}