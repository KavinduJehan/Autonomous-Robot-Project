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

/* Ultrasonic Sensors (HC-SR04) - Collision Detection */
// Sensor A faces LEFT wall (while moving forward)
// Sensor B faces RIGHT wall (while moving forward)  
// Sensor C faces FRONT (obstacle detection for YOLO integration)
#define US_TRIG_A_PIN       GPIO_PIN_0    // PB0 - Trigger A (Left)
#define US_TRIG_B_PIN       GPIO_PIN_1    // PB1 - Trigger B (Right)
#define US_TRIG_C_PIN       GPIO_PIN_2    // PB2 - Trigger C (Front)
#define US_ECHO_A_PIN       GPIO_PIN_6    // PB6 - Echo A (Left)
#define US_ECHO_B_PIN       GPIO_PIN_7    // PB7 - Echo B (Right)
#define US_ECHO_C_PIN       GPIO_PIN_8    // PB8 - Echo C (Front)
#define US_GPIO_PORT        GPIOB

/* Ultrasonic configuration */
#define ULTRASONIC_ENABLED            1      // Master enable for ultrasonic logic
#define ULTRASONIC_TRIGGER_US         10     // Trigger pulse width (us)
#define ULTRASONIC_TIMEOUT_US         30000  // Echo wait timeout (us)
#define ULTRASONIC_MEASURE_INTERVAL_MS 50    // Measure at 20 Hz
/* Debug: print periodic ultrasonic readings + LED states from the task */
#define ULTRASONIC_DEBUG              1      // Set to 0 to disable UART debug spam
/* Extra debug: boot banner and UART heartbeat (to verify TX wiring) */
#define DEBUG_BOOT_BANNER             1
#define DEBUG_UART_HEARTBEAT          1

/* Collision avoidance thresholds (cm)
 * Note: HC-SR04 minimum reliable range is ~2cm. We set STOP at 3cm to avoid
 * constant e-stops when running very close (~2.5cm) to the walls.
 */
#define COLLISION_DISTANCE_STOP       2.5      // Hard stop if closer than this
#define COLLISION_DISTANCE_SLOW       5       // Apply steering/centering below this
#define COLLISION_DISTANCE_WARN       50      // Optional warning distance

/* Front obstacle detection thresholds (for YOLO integration) */
#define FRONT_OBSTACLE_STOP           15      // Stop if front obstacle closer than 15cm
#define FRONT_OBSTACLE_SLOW           30      // Reduce speed if obstacle closer than 30cm
#define FRONT_OBSTACLE_WARN           50      // Report to RPI if obstacle closer than 50cm

/* Wall-following steering gain (fallback when only one sensor is valid) */
#define WALL_CORR_GAIN_PCT_PER_CM      2     // % speed correction per cm inside threshold

/* Center-seeking PID control (active only while moving forward) */
#define CENTERING_PID_ENABLED          1
#define CENTER_PID_KP                  1.0f   // % per cm
#define CENTER_PID_KI                  0.00f  // % per (cm*s) (start 0 to avoid windup)
#define CENTER_PID_KD                  0.20f  // % per (cm/s)
#define CENTER_DEADBAND_CM             1.0f   // ignore tiny error band to avoid chatter
#define CENTER_CORR_MAX                40.0f  // max magnitude of correction (%)

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
#define CMD_SELF_TEST     'W'
/* Ultrasonic debug command */
#define CMD_ULTRASONIC_PING 'U'
/* Front sensor query command (for RPI integration) */
#define CMD_FRONT_DISTANCE  'Q'
/* Status report command (sends all sensor distances) */
#define CMD_STATUS_REPORT   'I'
/* ToF sensor commands */
#define CMD_TOF_DISTANCES   'G'   // Get ToF sensor distances (changed from T to avoid conflict)
#define CMD_TOF_JUNCTIONS   'J'   // Get junction detection status
#define CMD_TOF_OBSTACLES   'O'   // Get obstacle detection status

/* Enable a short PWM self-test at boot (runs once inside motor task). Set to 0 to disable. */
#define ENABLE_PWM_SELF_TEST   0

/* Safety Configuration */
/* Emergency stop timeout
 * Note: temporarily relaxed for debugging so motors don't stop while testing sensors. */
#define SAFETY_TIMEOUT_MS 10000  // Emergency stop if no command for 10 seconds (set back to 2000 after debug)

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

/* Ultrasonic Sensor API */
void Ultrasonic_Init(void);
uint16_t Ultrasonic_MeasureA(void); // Left-facing sensor (A)
uint16_t Ultrasonic_MeasureB(void); // Right-facing sensor (B)
uint16_t Ultrasonic_MeasureC(void); // Front-facing sensor (C)
bool Ultrasonic_CheckCollision(void);
void Ultrasonic_Task(void const * argument);

/* Differential forward drive (arc steering) */
void Motor_ForwardDifferential(uint8_t left_speed, uint8_t right_speed);

/* UART debug helpers (optional) */
void UART_SendString(const char* s);
void UART_SendUInt(uint32_t v);
void UART_SendCRLF(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
