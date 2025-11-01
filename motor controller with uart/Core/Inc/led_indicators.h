/**
  ******************************************************************************
  * @file           : led_indicators.h
  * @brief          : LED indicators module header
  * @description    : Control for RX/TX, heartbeat, and wall detection LEDs
  ******************************************************************************
  */

#ifndef __LED_INDICATORS_H
#define __LED_INDICATORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize all LED GPIO pins
 * @retval None
 */
void LED_Init(void);

/**
 * @brief  Toggle RX activity LED
 * @retval None
 */
void LED_Toggle_RX(void);

/**
 * @brief  Toggle TX activity LED
 * @retval None
 */
void LED_Toggle_TX(void);

/**
 * @brief  Toggle heartbeat LED
 * @retval None
 */
void LED_Toggle_Heartbeat(void);

/**
 * @brief  Set wall detection LED states
 * @param  left_state: GPIO_PIN_SET or GPIO_PIN_RESET
 * @param  right_state: GPIO_PIN_SET or GPIO_PIN_RESET
 * @retval None
 */
void LED_SetWallIndicators(GPIO_PinState left_state, GPIO_PinState right_state);

/**
 * @brief  Run LED self-test (brief blink sequence)
 * @retval None
 */
void LED_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_INDICATORS_H */
