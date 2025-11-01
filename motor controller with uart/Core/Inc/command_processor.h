/**
  ******************************************************************************
  * @file           : command_processor.h
  * @brief          : Command processing module header
  * @description    : UART command processing and safety checks
  ******************************************************************************
  */

#ifndef __COMMAND_PROCESSOR_H
#define __COMMAND_PROCESSOR_H

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
extern volatile uint32_t last_command_time;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Process received UART command
 * @param  cmd: Command character received via UART
 * @retval None
 */
void Command_Process(uint8_t cmd);

/**
 * @brief  Safety check - emergency stop if no command received
 * @retval None
 */
void Command_SafetyCheck(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMMAND_PROCESSOR_H */
