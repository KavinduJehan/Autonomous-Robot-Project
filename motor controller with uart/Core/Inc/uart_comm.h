/**
  ******************************************************************************
  * @file           : uart_comm.h
  * @brief          : UART communication module header
  * @description    : UART initialization, interrupt handling, and debug output
  ******************************************************************************
  */

#ifndef __UART_COMM_H
#define __UART_COMM_H

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
extern volatile uint8_t rx_data;
extern volatile bool uart_error;
extern volatile bool uart_rx_seen;
extern osMessageQId uartCmdQueueHandle;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize GPIO pins for UART
 * @retval None
 */
void UART_GPIO_Init(void);

/**
 * @brief  Initialize USART1 peripheral
 * @retval None
 */
void UART_Init(void);

/**
 * @brief  USART1 Interrupt Handler
 * @retval None
 */
void UART_IRQHandler(void);

/**
 * @brief  Send string via UART (polling mode)
 * @param  s: Null-terminated string
 * @retval None
 */
void UART_SendString(const char* s);

/**
 * @brief  Send unsigned integer via UART (polling mode)
 * @param  v: Value to send
 * @retval None
 */
void UART_SendUInt(uint32_t v);

/**
 * @brief  Send carriage return + line feed
 * @retval None
 */
void UART_SendCRLF(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_COMM_H */
