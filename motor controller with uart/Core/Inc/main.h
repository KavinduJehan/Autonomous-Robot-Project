/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* Motor Control Pin Definitions - MX1508 Motor Driver */
#define MOTOR1_IN1_PIN    GPIO_PIN_0   // PA0 - Motor 1 IN1
#define MOTOR1_IN2_PIN    GPIO_PIN_1   // PA1 - Motor 1 IN2
#define MOTOR2_IN3_PIN    GPIO_PIN_2   // PA2 - Motor 2 IN3
#define MOTOR2_IN4_PIN    GPIO_PIN_3   // PA3 - Motor 2 IN4
#define MOTOR_PORT        GPIOA

/* LED Indicator Pin Definitions */
#define LED_RX_PIN        GPIO_PIN_13  // PC13 - RX Activity LED
#define LED_TX_PIN        GPIO_PIN_14  // PC14 - TX Activity LED
#define LED_PORT          GPIOC

/* Heartbeat LED (separate from RX/TX indicators) */
#define HEARTBEAT_LED_PIN   GPIO_PIN_12   // PB12 - Heartbeat LED
#define HEARTBEAT_LED_PORT  GPIOB

/* UART Pin Definitions - USART1 */
#define UART_TX_PIN       GPIO_PIN_9   // PA9 - USART1_TX
#define UART_RX_PIN       GPIO_PIN_10  // PA10 - USART1_RX
#define UART_PORT         GPIOA
#define UART_AF           GPIO_AF7_USART1

/* UART Configuration */
#define UART_BAUDRATE     9600
#define UART_TIMEOUT_MS   1000

/* Command Definitions */
#define CMD_FORWARD       'F'
#define CMD_REVERSE       'R'
#define CMD_LEFT          'L'
#define CMD_RIGHT         'T'
#define CMD_STOP          'S'

/* Safety Configuration */
#define SAFETY_TIMEOUT_MS 2000  // Emergency stop if no command for 2 seconds

/* Function Prototypes */
void GPIO_Init(void);
void USART1_Init(void);
void Motor_Forward(void);
void Motor_Reverse(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);
void Process_Command(uint8_t cmd);
void Safety_Check(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
