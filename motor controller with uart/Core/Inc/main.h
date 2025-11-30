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

#/* USER CODE BEGIN Private defines */

/* Motor Control Pin Definitions - MX1508 Motor Driver with PWM */
#define MOTOR1_IN1_PIN    GPIO_PIN_0   // PA0 - Motor 1 IN1 - TIM5_CH1 (PWM)
#define MOTOR1_IN2_PIN    GPIO_PIN_1   // PA1 - Motor 1 IN2 - TIM5_CH2 (PWM)
#define MOTOR1_IN2_PIN    GPIO_PIN_1   // PA1 - Motor 1 IN2 - TIM5_CH2 (PWM)
#define MOTOR2_IN3_PIN    GPIO_PIN_2   // PA2 - Motor 2 IN3 - TIM5_CH3 (PWM)
#define MOTOR2_IN4_PIN    GPIO_PIN_3   // PA3 - Motor 2 IN4 - TIM5_CH4 (PWM)
#define MOTOR_PORT        GPIOA

/* PWM Configuration */
#define PWM_TIMER_FREQ    16000000      // 16 MHz (APB1 timer clock)
#define PWM_FREQUENCY     1000          // 1 kHz PWM frequency
#define PWM_PRESCALER     15            // 16MHz / (15+1) = 1MHz timer clock
#define PWM_PERIOD        999           // 1MHz / (999+1) = 1kHz PWM
#define PWM_MAX_DUTY      PWM_PERIOD    // 100% duty cycle

/* Per-motor PWM scaling to compensate hardware differences (percentage)
 * Example: set right motor to 95 to reduce its effective PWM to 95% of commands.
 * Range: 0-200 (where 100 means no change)
 */
#ifndef MOTOR1_PWM_SCALE
#define MOTOR1_PWM_SCALE 99 //right
#endif
#ifndef MOTOR2_PWM_SCALE
#define MOTOR2_PWM_SCALE 99 //left........
#endif

#/* Motor Speed Levels (0-100%) */
#define SPEED_STOP        0
#define SPEED_SLOW        50
#define SPEED_MEDIUM      70
#define SPEED_FAST        100

/* Motor Kick-Start Configuration (helps overcome static friction at low speeds) */
#define KICKSTART_ENABLED   0             // Disable kick-start pulse for low speeds (removed per user request)
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

/* Wall Avoidance Debug LEDs */
/* PB14 lights when LEFT sensor reports inside slow/stop zone
  PB15 lights when RIGHT sensor reports inside slow/stop zone */
#define WALL_LEFT_LED_PIN   GPIO_PIN_14   // PB14 - Left wall debug LED
#define WALL_RIGHT_LED_PIN  GPIO_PIN_15   // PB15 - Right wall debug LED
#define WALL_LED_PORT       GPIOB
/* Set to 1 if your LEDs are wired active-low (LED to Vcc, pin sinks current) */
#define WALL_LED_ACTIVE_LOW 0

/* Heartbeat LED (separate from RX/TX indicators) */
#define HEARTBEAT_LED_PIN   GPIO_PIN_12   // PB12 - Heartbeat LED
#define HEARTBEAT_LED_PORT  GPIOB

/* VL53L0X Time-of-Flight Sensors - I2C2 on PB3/PB10 */
// Two sensors mounted at 45° angles for junction detection and YOLO distance fusion
// Sensor 1: Left 45° (I2C address 0x30 after init)
// Sensor 2: Right 45° (I2C address 0x31 after init)
#define TOF_I2C_SDA_PIN     GPIO_PIN_3    // PB3 - I2C2_SDA
#define TOF_I2C_SCL_PIN     GPIO_PIN_10   // PB10 - I2C2_SCL
#define TOF_I2C_PORT        GPIOB
#define TOF_I2C_AF          GPIO_AF4_I2C2

#define TOF_XSHUT1_PIN      GPIO_PIN_4    // PA4 - XSHUT for left sensor
#define TOF_XSHUT2_PIN      GPIO_PIN_5    // PA5 - XSHUT for right sensor
#define TOF_XSHUT_PORT      GPIOA

/* ToF Sensor Configuration */
#define TOF_ENABLED                1        // Master enable for ToF sensors
#define TOF_I2C_TIMEOUT_MS         100      // I2C transaction timeout
#define TOF_MEASURE_INTERVAL_MS    20       // Read at 50Hz (faster than ultrasonic)
#define TOF_DEFAULT_ADDR           0x29     // Factory default I2C address (shifted: 0x52)
#define TOF_SENSOR1_ADDR           0x30     // Reassigned address for left sensor
#define TOF_SENSOR2_ADDR           0x31     // Reassigned address for right sensor

/* ToF detection thresholds for junction detection (mm) */
#define TOF_JUNCTION_THRESHOLD     1500     // > 150cm indicates open space (possible turn)
#define TOF_CORRIDOR_THRESHOLD     800      // 80cm typical corridor side clearance
#define TOF_OBSTACLE_THRESHOLD     300      // < 30cm obstacle detected

/* Junction detection tuning */
#define TOF_JUMP_DELTA_MM         100      // sudden ToF increase (mm) indicating opening
#define US_JUMP_DELTA_CM          30       // sudden ultrasonic increase (cm) confirming opening
#define JUNCTION_ACK_TIMEOUT_MS   2000     // wait up to 2s for Pi ACK
#define JUNCTION_DIRECTION_TIMEOUT_MS 5000  // wait up to 5s for Pi direction command

/* Ultrasonic Sensors (HC-SR04) - Wall Collision & Obstacle Detection */
// Sensor A faces LEFT wall (while moving forward)
// Sensor B faces RIGHT wall (while moving forward)
// Sensor C faces FRONT (obstacle detection)
// User mapping (verified):
//  - LEFT (Sensor A):  TRIG = PA8,  ECHO = PB8
//  - RIGHT (Sensor B): TRIG = PB1,  ECHO = PB7
//  - FRONT (Sensor C): leave as PB0 trig / PB6 echo (default)

/* Sensor A (LEFT) */
#define US_TRIG_A_PORT      GPIOA
#define US_TRIG_A_PIN       GPIO_PIN_8    // PA8 - Trigger A (LEFT)
#define US_ECHO_A_PORT      GPIOB
#define US_ECHO_A_PIN       GPIO_PIN_8    // PB8 - Echo A (LEFT)

/* Sensor B (RIGHT) */
#define US_TRIG_B_PORT      GPIOB
#define US_TRIG_B_PIN       GPIO_PIN_1    // PB1 - Trigger B (RIGHT)
#define US_ECHO_B_PORT      GPIOB
#define US_ECHO_B_PIN       GPIO_PIN_7    // PB7 - Echo B (RIGHT)

/* Sensor C (FRONT) - keep defaults */
#define US_TRIG_C_PORT      GPIOB
#define US_TRIG_C_PIN       GPIO_PIN_0    // PB0 - Trigger C (FRONT)
#define US_ECHO_C_PORT      GPIOB
#define US_ECHO_C_PIN       GPIO_PIN_6    // PB6 - Echo C (FRONT)

/* Ultrasonic configuration */
#define ULTRASONIC_ENABLED            1      // Master enable for ultrasonic logic
#define ULTRASONIC_TRIGGER_US         10     // Trigger pulse width (us)
#define ULTRASONIC_TIMEOUT_US         30000  // Echo wait timeout (us)
#define ULTRASONIC_MEASURE_INTERVAL_MS 50    // Measure at 20 Hz
/* Debug: print periodic ultrasonic readings + LED states from the task
 * Set to 0 to disable periodic ultrasonic UART messages; explicit 'U' ping
 * will still trigger a full A/B/C report via the command processor.
 */
#define ULTRASONIC_DEBUG              0      // Disabled: only send on explicit ping
/* Extra debug: boot banner and UART heartbeat (to verify TX wiring) */
#define DEBUG_BOOT_BANNER             1
#define DEBUG_UART_HEARTBEAT          1

/* Collision avoidance thresholds (cm)
 * Note: HC-SR04 minimum reliable range is ~2cm. We set STOP at 3cm to avoid
 * constant e-stops when running very close (~2.5cm) to the walls.
 */
#define COLLISION_DISTANCE_STOP       3.0      // Hard stop if closer than this (side walls)
#define COLLISION_DISTANCE_SLOW       5     // Apply steering/centering below this (side walls)
#define COLLISION_DISTANCE_WARN       50     // Optional warning distance

/* Front obstacle detection thresholds (cm) */
#define OBSTACLE_DISTANCE_STOP        15    // Emergency stop if obstacle closer than 15cm
#define OBSTACLE_DISTANCE_SLOW        30    // Reduce speed if obstacle within 30cm
#define OBSTACLE_DISTANCE_WARN        50    // Warning distance

/* Wall-following steering gain (fallback when only one sensor is valid) */
#define WALL_CORR_GAIN_PCT_PER_CM      2     // % speed correction per cm inside threshold

/* Center-seeking PID control (active only while moving forward) */
#define CENTERING_PID_ENABLED          1
/* Default PID values (can be tuned via UART commands P/I/D) */
#define CENTER_PID_KP_DEFAULT          1.2f   // % per cm (increased for faster response)
#define CENTER_PID_KI_DEFAULT          0.05f  // % per (cm*s) (small integral to eliminate steady-state error)
#define CENTER_PID_KD_DEFAULT          0.3f   // % per (cm/s) (increased damping)
#define CENTER_DEADBAND_CM             0.5f   // ignore tiny error band to avoid chatter (reduced for precision)
#define CENTER_CORR_MAX                50.0f  // max magnitude of correction (%) (increased range)

/* Allow compile-time inversion of motor forward/reverse mapping.
  Set to 1 if your H-bridge wiring or motor driver inverts directions.
  This flips the logical Motor_Forward()/Motor_Reverse() behavior so
  the UART command mapping remains unchanged. */
#ifndef INVERT_MOTOR_DIRECTION
#define INVERT_MOTOR_DIRECTION 0
#endif

/* Per-motor inversion flags. If one motor was wired reversed during assembly
  you can invert only that motor to correct direction without swapping wires.
  By default these follow INVERT_MOTOR_DIRECTION for backward compatibility. */
#ifndef INVERT_MOTOR1_DIRECTION
#define INVERT_MOTOR1_DIRECTION INVERT_MOTOR_DIRECTION
#endif
#ifndef INVERT_MOTOR2_DIRECTION
#define INVERT_MOTOR2_DIRECTION INVERT_MOTOR_DIRECTION
#endif

/* Swap left/right motor logical mapping. Set to 1 if motor1/motor2 were
  installed swapped (left motor connected to motor2 pins and vice versa).
  Enabling this flips logical motor assignments only; it does NOT change
  PID, centering, or other control logic. Use this when wiring cannot be
  changed. */
#ifndef SWAP_MOTORS
#define SWAP_MOTORS 1
#endif

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
/* Alternate single-key controls (common keyboard layout):
 * 'A'/'a' -> turn left, 'D'/'d' -> turn right
 */
#define CMD_LEFT_ALT      'A'
#define CMD_RIGHT_ALT     'D'
#define CMD_STOP          'S'
#define CMD_SPEED_SLOW    '1'   // Set speed to 50%
#define CMD_SPEED_MEDIUM  '2'   // Set speed to 70%
#define CMD_SPEED_FAST    '3'   // Set speed to 100%

#if 0
/* PID Tuning Commands (removed) - runtime PID tweaking not needed in this build.
 * If you need to re-enable, change `#if 0` to `#if 1` and implement handlers.
 */
#define CMD_PID_KP_UP     'P'   // Increase Kp by 0.1
#define CMD_PID_KP_DOWN   'p'   // Decrease Kp by 0.1
#define CMD_PID_KI_UP     'I'   // Increase Ki by 0.01
#define CMD_PID_KI_DOWN   'i'   // Decrease Ki by 0.01
#define CMD_PID_KD_UP     'J'   // Increase Kd by 0.05 (changed from 'D' to avoid conflict)
#define CMD_PID_KD_DOWN   'j'   // Decrease Kd by 0.05 (changed from 'd')
#define CMD_PID_RESET     'K'   // Reset PID to defaults
#define CMD_PID_REPORT    'Q'   // Report current PID values
#endif
#define CMD_ACCEL_ENABLE  'M'   // Enable smooth acceleration/deceleration (M for sMooth)
#define CMD_ACCEL_DISABLE 'Z'   // Disable acceleration (instant speed change)
#define CMD_ACCEL_DISABLE_ALT 'Y' // Alternate disable (changed to avoid conflict with CMD_RIGHT_ALT)
/* Self-test command to cycle PWM speeds/directions (manual only) */
#define CMD_SELF_TEST     'X'
/* Ultrasonic debug command */
#define CMD_ULTRASONIC_PING 'U'
/* ToF ping command - request current ToF distances (left/right 45° sensors) */
#define CMD_TOF_PING 'o'

/* Enable a short PWM self-test at boot (runs once inside motor task). Set to 0 to disable. */
#define ENABLE_PWM_SELF_TEST   0

/* Safety Configuration */
/* Emergency stop timeout
 * Note: temporarily relaxed for debugging so motors don't stop while testing sensors. */
#define SAFETY_TIMEOUT_MS 10000  // Emergency stop if no command for 10 seconds (set back to 2000 after debug)

/* Timed turn configuration
 * When receiving a turn-left/turn-right command from UART the controller will
 * perform a one-shot spot turn for this duration (ms) and then stop. This
 * provides an easy way to request ~90 degree turns without holding the
 * command or wiring an IMU. Tune this value for your robot (depends on
 * wheelbase, speed and motor power).
 */
#ifndef TURN_90_MS
#define TURN_90_MS 350
#endif

/* Minimum and maximum allowed turn durations when scaling with speed */
#ifndef TURN_90_MS_MIN
#define TURN_90_MS_MIN 150
#endif
#ifndef TURN_90_MS_MAX
#define TURN_90_MS_MAX 5000
#endif

/* Function Prototypes */
void GPIO_Init(void);
void TIM5_PWM_Init(void);
void USART1_Init(void);
void I2C2_Init(void);
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

/* ToF Sensors API */
void ToF_Init(void);
uint16_t ToF_ReadSensor1(void);  // Left 45° sensor (mm)
uint16_t ToF_ReadSensor2(void);  // Right 45° sensor (mm)
bool ToF_DetectJunction(void);   // Check if junction is detected
void ToF_Task(void const * argument);

/* Junction coordination flags (shared across modules) */
extern volatile bool junction_ack_received;
extern volatile bool junction_mode_active;
extern volatile bool junction_direction_received;
extern volatile uint8_t junction_direction_cmd;

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
