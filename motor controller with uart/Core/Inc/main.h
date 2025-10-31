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

/* Motor Control Pin Definitions - MX1508 Motor Driver with PWM */
#define MOTOR1_IN1_PIN    GPIO_PIN_0   // PA0 - Motor 1 IN1 - TIM3_CH1
#define MOTOR1_IN2_PIN    GPIO_PIN_1   // PA1 - Motor 1 IN2 - TIM3_CH2
#define MOTOR2_IN3_PIN    GPIO_PIN_2   // PA2 - Motor 2 IN3 - TIM3_CH3
#define MOTOR2_IN4_PIN    GPIO_PIN_3   // PA3 - Motor 2 IN4 - TIM3_CH4
#define MOTOR_PORT        GPIOA

/* PWM Configuration */
#define PWM_TIMER_FREQ    16000000      // 16 MHz (APB1 timer clock)
#define PWM_FREQUENCY     1000          // 1 kHz PWM frequency
#define PWM_PRESCALER     15            // 16MHz / (15+1) = 1MHz timer clock
#define PWM_PERIOD        999           // 1MHz / (999+1) = 1kHz PWM
#define PWM_MAX_DUTY      PWM_PERIOD    // 100% duty cycle

/* Motor Speed Levels (0-100%) */
#define SPEED_STOP        0
#define SPEED_SLOW        40
#define SPEED_MEDIUM      70
#define SPEED_FAST        100

/* Motor Kick-Start Configuration (helps overcome static friction at low speeds) */
#define KICKSTART_ENABLED   1             // Enable kick-start pulse for low speeds
#define KICKSTART_DUTY      80            // Initial boost PWM (%) for kick-start
#define KICKSTART_DURATION  150           // Kick-start pulse duration (ms)

/* Acceleration/Deceleration Configuration */
#define ACCEL_ENABLED     1             // Set to 0 to disable ramping
#define ACCEL_STEP        5             // Speed increment per step (0-100%)
#define ACCEL_DELAY_MS    20            // Delay between steps (milliseconds)
#define DECEL_STEP        10            // Deceleration step (faster than accel)
#define DECEL_DELAY_MS    15            // Deceleration delay (shorter for quick stop)

/* LED Indicator Pin Definitions */
#define LED_RX_PIN        GPIO_PIN_13  // PC13 - RX Activity LED
#define LED_TX_PIN        GPIO_PIN_14  // PC14 - TX Activity LED
#define LED_PORT          GPIOC

/* Heartbeat LED (separate from RX/TX indicators) */
#define HEARTBEAT_LED_PIN   GPIO_PIN_12   // PB12 - Heartbeat LED
#define HEARTBEAT_LED_PORT  GPIOB

/* IR Wall Sensors (Analog) - use ADC1 on PA4/PA5 */
#define IR_LEFT_PIN         GPIO_PIN_4    // PA4 - ADC1_IN4
#define IR_RIGHT_PIN        GPIO_PIN_5    // PA5 - ADC1_IN5
#define IR_SENSOR_PORT      GPIOA

/* Ultrasonic Sensors (HC-SR04) - Wall Collision Detection */
// Sensor A faces LEFT wall (while moving forward)
// Sensor B faces RIGHT wall (while moving forward)
#define US_TRIG_A_PIN       GPIO_PIN_0    // PB0 - Trigger A
#define US_TRIG_B_PIN       GPIO_PIN_1    // PB1 - Trigger B
#define US_ECHO_A_PIN       GPIO_PIN_6    // PB6 - Echo A
#define US_ECHO_B_PIN       GPIO_PIN_7    // PB7 - Echo B
#define US_GPIO_PORT        GPIOB

/* Ultrasonic configuration */
#define ULTRASONIC_ENABLED            1      // Master enable for ultrasonic logic
#define ULTRASONIC_TRIGGER_US         10     // Trigger pulse width (us)
#define ULTRASONIC_TIMEOUT_US         30000  // Echo wait timeout (us)
#define ULTRASONIC_MEASURE_INTERVAL_MS 50    // Measure at 20 Hz

/* Collision avoidance thresholds (cm) */
#define COLLISION_DISTANCE_STOP       15     // Hard stop if closer than this
#define COLLISION_DISTANCE_SLOW       30     // Apply steering correction below this
#define COLLISION_DISTANCE_WARN       50     // Optional warning distance

/* Wall-following steering gain */
#define WALL_CORR_GAIN_PCT_PER_CM      2     // % speed correction per cm inside threshold

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
#define CMD_SPEED_SLOW    '1'   // Set speed to 40%
#define CMD_SPEED_MEDIUM  '2'   // Set speed to 70%
#define CMD_SPEED_FAST    '3'   // Set speed to 100%
#define CMD_ACCEL_ENABLE  'M'   // Enable smooth acceleration/deceleration (M for sMooth)
#define CMD_ACCEL_DISABLE 'Z'   // Disable acceleration (instant speed change)
#define CMD_ACCEL_DISABLE_ALT 'D' // Alternate disable (was used for this before)
/* Self-test command to cycle PWM speeds/directions (manual only) */
#define CMD_SELF_TEST     'X'

/* Enable a short PWM self-test at boot (runs once inside motor task). Set to 0 to disable. */
#define ENABLE_PWM_SELF_TEST   0

/* Safety Configuration */
#define SAFETY_TIMEOUT_MS 2000  // Emergency stop if no command for 2 seconds

/* Function Prototypes */
void GPIO_Init(void);
void TIM5_PWM_Init(void);
void USART1_Init(void);
void Motor_SetSpeed(uint8_t motor1_in1, uint8_t motor1_in2, uint8_t motor2_in3, uint8_t motor2_in4);
void Motor_SetSpeed_Smooth(uint8_t target_m1_in1, uint8_t target_m1_in2, uint8_t target_m2_in3, uint8_t target_m2_in4);
void Motor_Forward(uint8_t speed);
void Motor_Reverse(uint8_t speed);
void Motor_Left(uint8_t speed);
void Motor_Right(uint8_t speed);
void Motor_Stop(void);
void Motor_Stop_Smooth(void);
void Process_Command(uint8_t cmd);
void Safety_Check(void);

/* IR Sensors API */
void IR_Init(void);
void IR_ReadRaw(uint16_t* left, uint16_t* right);

/* Ultrasonic Sensor API */
void Ultrasonic_Init(void);
uint16_t Ultrasonic_MeasureA(void); // Left-facing sensor (A)
uint16_t Ultrasonic_MeasureB(void); // Right-facing sensor (B)
bool Ultrasonic_CheckCollision(void);
void Ultrasonic_Task(void const * argument);

/* Differential forward drive (arc steering) */
void Motor_ForwardDifferential(uint8_t left_speed, uint8_t right_speed);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
