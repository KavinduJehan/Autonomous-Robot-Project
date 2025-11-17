/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : tof_sensors.h
  * @brief          : VL53L0X ToF Sensors Module Header
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
  * 
  * @project       ToF Sensors Module - Autonomous Robot
  * @target        STM32F401RCT6
  * @sensors       4x VL53L0X ToF Distance Sensors
  * 
  * @description
  * This module implements precision distance measurement using 4 VL53L0X ToF sensors
  * positioned for enhanced obstacle detection and junction recognition:
  * - ToF_FL: Front-Left sensor (30° diagonal coverage)
  * - ToF_FR: Front-Right sensor (30° diagonal coverage)  
  * - ToF_L:  Left sensor (60° lateral coverage)
  * - ToF_R:  Right sensor (60° lateral coverage)
  * 
 * @hardware_connections
 * I2C1 Bus Configuration:
 *   PB9  -> SDA (I2C1_SDA) - Data line for all 4 sensors
 *   PB10 -> SCL (I2C1_SCL) - Clock line for all 4 sensors
 * 
 * XSHUT Control Pins (for address programming):
 *   PA4 -> ToF_FL_XSHUT (Front-Left sensor shutdown)
 *   PA5 -> ToF_FR_XSHUT (Front-Right sensor shutdown)
 *   PA6 -> ToF_L_XSHUT  (Left sensor shutdown)
 *   PA7 -> ToF_R_XSHUT  (Right sensor shutdown)
  * 
  * @i2c_addresses
  * Default: 0x52 (all sensors ship with this address)
  * Programmed addresses after initialization:
  *   ToF_FL: 0x54 (Front-Left)
  *   ToF_FR: 0x56 (Front-Right)  
  *   ToF_L:  0x58 (Left)
  *   ToF_R:  0x5A (Right)
  * 
  * @integration_notes
  * - Works alongside existing HC-SR04 ultrasonic sensors (collision backup)
  * - 30ms measurement cycle per sensor (120ms full sweep)
  * - Designed for YOLO vision system integration
  * - Junction detection with precise angular coverage
  * 
  * @author        Robot Project Team
  * @date          November 16, 2025
  * @version       1.0 - Initial ToF Implementation
  * 
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TOF_SENSORS_H
#define __TOF_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ToF Sensor Configuration --------------------------------------------------*/

/* I2C Configuration (Software Implementation) */
#define TOF_I2C_CLOCK_SPEED     100000      // 100kHz standard mode (target)
#define TOF_I2C_TIMEOUT_MS      1000        // I2C operation timeout

/* I2C GPIO Pins (Software I2C) */
#define TOF_I2C_SDA_PIN         GPIO_PIN_9   // PB9 - I2C1_SDA (Software)
#define TOF_I2C_SCL_PIN         GPIO_PIN_10  // PB10 - I2C1_SCL (Software)
#define TOF_I2C_GPIO_PORT       GPIOB

/* XSHUT Control Pins for Address Programming */
#define TOF_FL_XSHUT_PIN        GPIO_PIN_4   // PA4 - Front-Left XSHUT (freed from IR)
#define TOF_FR_XSHUT_PIN        GPIO_PIN_5   // PA5 - Front-Right XSHUT (freed from IR)
#define TOF_L_XSHUT_PIN         GPIO_PIN_6   // PA6 - Left XSHUT
#define TOF_R_XSHUT_PIN         GPIO_PIN_7   // PA7 - Right XSHUT
#define TOF_XSHUT_GPIO_PORT     GPIOA

/* VL53L0X I2C Addresses */
#define VL53L0X_DEFAULT_ADDR    0x52         // Default address (factory)
#define TOF_FL_ADDR            0x54         // Front-Left programmed address
#define TOF_FR_ADDR            0x56         // Front-Right programmed address
#define TOF_L_ADDR             0x58         // Left programmed address
#define TOF_R_ADDR             0x5A         // Right programmed address

/* Sensor Identifiers */
typedef enum {
    TOF_SENSOR_FL = 0,    // Front-Left
    TOF_SENSOR_FR = 1,    // Front-Right
    TOF_SENSOR_L  = 2,    // Left
    TOF_SENSOR_R  = 3,    // Right (may not be connected - PA7 not soldered)
    TOF_SENSOR_COUNT = 4, // Keep at 4 for array sizing
    TOF_ACTIVE_COUNT = 3  // Currently only 3 sensors connected (FL, FR, L)
} ToF_Sensor_ID_t;

/* Sensor Data Structure */
typedef struct {
    uint16_t distance_mm;     // Distance in millimeters
    uint8_t  status;          // Measurement status
    bool     valid;           // Data validity flag
    uint32_t timestamp;       // Last update timestamp (ms)
} ToF_Data_t;

/* Global Sensor Data Array */
extern ToF_Data_t tof_sensors[TOF_SENSOR_COUNT];

/* VL53L0X Register Definitions */
/*
 * Register addresses now come from ST's VL53L0X API headers (vl53l0x_device.h),
 * which are included transitively via vl53l0x_api.h. Keep this section for
 * contextual reference when wiring code, but avoid redefining the macros to
 * prevent compiler warnings if the vendor headers change.
 */

/* Measurement Configuration */
#define TOF_MEASUREMENT_INTERVAL_MS   30         // 30ms per sensor
#define TOF_FULL_CYCLE_MS            120        // 4 sensors * 30ms
#define TOF_MAX_DISTANCE_MM          2000       // 2m maximum range
#define TOF_MIN_DISTANCE_MM          30         // 3cm minimum range
#define TOF_TIMEOUT_MS               60         // Measurement timeout (allow slower SW I2C)

/* Status Codes */
#define TOF_STATUS_OK                0x00
#define TOF_STATUS_TIMEOUT           0x01
#define TOF_STATUS_ERROR             0x02
#define TOF_STATUS_NOT_READY         0x03

/* Detection Thresholds (millimeters) */
#define TOF_JUNCTION_DETECT_THRESHOLD    300     // 30cm for junction detection
#define TOF_OBSTACLE_STOP_THRESHOLD      150     // 15cm for emergency stop
#define TOF_OBSTACLE_SLOW_THRESHOLD      300     // 30cm for speed reduction
#define TOF_WALL_FOLLOW_THRESHOLD        200     // 20cm for wall following

/* Function Prototypes ------------------------------------------------------- */

/* Initialization */
HAL_StatusTypeDef ToF_Init(void);
HAL_StatusTypeDef ToF_GPIO_Init(void);
HAL_StatusTypeDef ToF_I2C_Init(void);
HAL_StatusTypeDef ToF_Sensors_Init(void);

/* Low-level I2C Operations */
HAL_StatusTypeDef ToF_WriteReg(uint8_t addr, uint8_t reg, uint8_t value);
HAL_StatusTypeDef ToF_ReadReg(uint8_t addr, uint8_t reg, uint8_t* value);
HAL_StatusTypeDef ToF_ReadMulti(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t length);
HAL_StatusTypeDef ToF_WriteMulti(uint8_t addr, uint8_t reg, const uint8_t* data, uint16_t length);

/* Sensor Management */
HAL_StatusTypeDef ToF_SetAddress(ToF_Sensor_ID_t sensor, uint8_t new_addr);
HAL_StatusTypeDef ToF_StartMeasurement(uint8_t addr);
HAL_StatusTypeDef ToF_ReadDistance(uint8_t addr, uint16_t* distance, uint8_t* status);
bool ToF_IsDataReady(uint8_t addr);

/* High-level API */
void ToF_UpdateAll(void);
void ToF_Task(void const * argument);
uint16_t ToF_GetDistance(ToF_Sensor_ID_t sensor);
bool ToF_IsValid(ToF_Sensor_ID_t sensor);
uint32_t ToF_GetAge(ToF_Sensor_ID_t sensor);

/* Junction Detection */
bool ToF_DetectJunction(void);
bool ToF_DetectLeftJunction(void);
bool ToF_DetectRightJunction(void);

/* Obstacle Detection */
bool ToF_DetectFrontObstacle(void);
uint16_t ToF_GetClosestDistance(void);
ToF_Sensor_ID_t ToF_GetClosestSensor(void);

/* Wall Following Support */
uint16_t ToF_GetLeftWallDistance(void);
uint16_t ToF_GetRightWallDistance(void);
int16_t ToF_GetCenteringError(void);

/* Diagnostics */
void ToF_PrintStatus(void);
void ToF_SelfTest(void);

/* UART Integration */
void ToF_SendDistances(void);
void ToF_ProcessCommand(uint8_t cmd);  // Handles G/J/O commands

#ifdef __cplusplus
}
#endif

#endif /* __TOF_SENSORS_H */