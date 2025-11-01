/**
  ******************************************************************************
  * @file           : motor_control.h
  * @brief          : Motor control module header
  * @description    : PWM-based motor control with smooth acceleration/deceleration
  ******************************************************************************
  */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

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
extern TIM_HandleTypeDef htim5;
extern volatile uint8_t current_speed;
extern volatile bool accel_enabled;
extern volatile uint8_t current_m1_in1;
extern volatile uint8_t current_m1_in2;
extern volatile uint8_t current_m2_in3;
extern volatile uint8_t current_m2_in4;
extern volatile bool motors_moving;
extern volatile uint8_t last_movement_cmd;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize TIM5 for PWM generation on motor control pins
 * @retval None
 */
void Motor_TIM5_PWM_Init(void);

/**
 * @brief  Initialize GPIO pins for motor control
 * @retval None
 */
void Motor_GPIO_Init(void);

/**
 * @brief  Set motor speeds using PWM
 * @param  motor1_in1: PWM duty cycle for Motor 1 Forward (0-100%)
 * @param  motor1_in2: PWM duty cycle for Motor 1 Reverse (0-100%)
 * @param  motor2_in3: PWM duty cycle for Motor 2 Forward (0-100%)
 * @param  motor2_in4: PWM duty cycle for Motor 2 Reverse (0-100%)
 * @retval None
 */
void Motor_SetSpeed(uint8_t motor1_in1, uint8_t motor1_in2, uint8_t motor2_in3, uint8_t motor2_in4);

/**
 * @brief  Set motor speeds with smooth acceleration/deceleration
 * @param  target_m1_in1: Target PWM duty cycle for Motor 1 Forward (0-100%)
 * @param  target_m1_in2: Target PWM duty cycle for Motor 1 Reverse (0-100%)
 * @param  target_m2_in3: Target PWM duty cycle for Motor 2 Forward (0-100%)
 * @param  target_m2_in4: Target PWM duty cycle for Motor 2 Reverse (0-100%)
 * @retval None
 */
void Motor_SetSpeed_Smooth(uint8_t target_m1_in1, uint8_t target_m1_in2, uint8_t target_m2_in3, uint8_t target_m2_in4);

/**
 * @brief  Motor Forward - Both motors forward at specified speed
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Forward(uint8_t speed);

/**
 * @brief  Motor Reverse - Both motors backward at specified speed
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Reverse(uint8_t speed);

/**
 * @brief  Motor Left - Left motor reverse, right motor forward (spot turn)
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Left(uint8_t speed);

/**
 * @brief  Motor Right - Left motor forward, right motor reverse (spot turn)
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Right(uint8_t speed);

/**
 * @brief  Forward with differential wheel speeds (arc steering)
 * @param  left_speed: Left wheel forward PWM (0-100)
 * @param  right_speed: Right wheel forward PWM (0-100)
 * @retval None
 */
void Motor_ForwardDifferential(uint8_t left_speed, uint8_t right_speed);

/**
 * @brief  Motor Stop - All motors off (instant stop)
 * @retval None
 */
void Motor_Stop(void);

/**
 * @brief  Motor Stop - All motors off with smooth deceleration
 * @retval None
 */
void Motor_Stop_Smooth(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
