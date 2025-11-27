/**
  ******************************************************************************
  * @file           : wall_avoidance.h
  * @brief          : Wall avoidance module header
  * @description    : PID-based wall following and collision avoidance
  ******************************************************************************
  */

#ifndef __WALL_AVOIDANCE_H
#define __WALL_AVOIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Runtime-adjustable PID gains (can be tuned via UART commands) */
extern float CENTER_PID_KP;
extern float CENTER_PID_KI;
extern float CENTER_PID_KD;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  FreeRTOS task for ultrasonic sensing and wall avoidance
 * @param  argument: Not used
 * @retval None
 */
void WallAvoidance_Task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif /* __WALL_AVOIDANCE_H */
