/**
  ******************************************************************************
  * @file           : ultrasonic.h
  * @brief          : Ultrasonic sensor module header
  * @description    : HC-SR04 distance measurement and collision detection
  ******************************************************************************
  */

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern volatile uint16_t ultrasonic_left_cm;
extern volatile uint16_t ultrasonic_right_cm;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize GPIO pins for ultrasonic sensors
 * @retval None
 */
void Ultrasonic_GPIO_Init(void);

/**
 * @brief  Initialize DWT cycle counter for microsecond timing
 * @retval None
 */
void Ultrasonic_Init(void);

/**
 * @brief  Measure distance from left sensor (Sensor A)
 * @retval Distance in centimeters (0-400), 0 on timeout
 */
uint16_t Ultrasonic_MeasureA(void);

/**
 * @brief  Measure distance from right sensor (Sensor B)
 * @retval Distance in centimeters (0-400), 0 on timeout
 */
uint16_t Ultrasonic_MeasureB(void);

/**
 * @brief  Check if either sensor detects collision threshold
 * @retval true if collision imminent, false otherwise
 */
bool Ultrasonic_CheckCollision(void);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_H */
