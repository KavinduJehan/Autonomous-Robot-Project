/*
 * tof_sensors.h
 *
 * VL53L0X Time-of-Flight Sensor Application Layer
 * Manages two VL53L0X sensors at 45° angles for:
 * - Junction detection (corridor turns)
 * - YOLO distance fusion
 * - Enhanced obstacle awareness
 *
 * Created: November 18, 2025
 */

#ifndef TOF_SENSORS_H
#define TOF_SENSORS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ToF Sensor Configuration */
#define TOF_MEASURE_INTERVAL_MS    20           // 50Hz reading rate

/* Global distance readings (in millimeters) */
extern volatile uint16_t tof_left45_mm;   // Left 45° distance
extern volatile uint16_t tof_right45_mm;  // Right 45° distance

/* Sensor status flags */
extern volatile bool tof_sensor1_ready;
extern volatile bool tof_sensor2_ready;
extern volatile bool tof_junction_detected;

/* Junction detection structure */
typedef struct {
    bool left_open;      // Left side has opening (> threshold)
    bool right_open;     // Right side has opening (> threshold)
    bool is_junction;    // Both sides open = junction/intersection
    bool is_left_turn;   // Only left open = left turn available
    bool is_right_turn;  // Only right open = right turn available
    uint16_t left_mm;    // Left distance reading
    uint16_t right_mm;   // Right distance reading
} ToF_JunctionInfo;

/* Initialization Functions */
void ToF_GPIO_Init(void);
void ToF_Init(void);
bool ToF_InitSensors(I2C_HandleTypeDef *hi2c);

/* Sensor Control */
void ToF_EnableSensor1(void);
void ToF_EnableSensor2(void);
void ToF_DisableSensor1(void);
void ToF_DisableSensor2(void);
void ToF_DisableAll(void);
void ToF_EnableAll(void);

/* Reading Functions */
uint16_t ToF_ReadSensor1(void);  // Left 45° sensor (blocking)
uint16_t ToF_ReadSensor2(void);  // Right 45° sensor (blocking)
bool ToF_ReadBothSensors(uint16_t *left_mm, uint16_t *right_mm);

/* Non-blocking reads (for FreeRTOS task) */
bool ToF_ReadSensor1_NonBlocking(uint16_t *range_mm);
bool ToF_ReadSensor2_NonBlocking(uint16_t *range_mm);

/* Junction Detection */
bool ToF_DetectJunction(void);
ToF_JunctionInfo ToF_GetJunctionInfo(void);
void ToF_UpdateJunctionDetection(void);

/* Utility Functions */
bool ToF_SensorsReady(void);
void ToF_GetDistances(uint16_t *left_mm, uint16_t *right_mm);
const char* ToF_StatusToString(int status);

/* FreeRTOS Task */
void ToF_Task(void const * argument);

#endif /* TOF_SENSORS_H */
