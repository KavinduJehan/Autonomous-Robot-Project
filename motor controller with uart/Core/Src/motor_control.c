/**
  ******************************************************************************
  * @file           : motor_control.c
  * @brief          : Motor control module implementation
  * @description    : PWM-based motor control with smooth acceleration/deceleration
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim5;
volatile uint8_t current_speed = SPEED_MEDIUM;
volatile bool accel_enabled = ACCEL_ENABLED;
volatile uint8_t current_m1_in1 = 0;
volatile uint8_t current_m1_in2 = 0;
volatile uint8_t current_m2_in3 = 0;
volatile uint8_t current_m2_in4 = 0;
volatile bool motors_moving = false;
volatile uint8_t last_movement_cmd = CMD_STOP;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize GPIO pins for motor control
 * @retval None
 */
void Motor_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIOA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure Motor Control Pins (PA0-PA3) as TIM5 PWM Output (AF2) */
    GPIO_InitStruct.Pin = MOTOR1_IN1_PIN | MOTOR1_IN2_PIN | MOTOR2_IN3_PIN | MOTOR2_IN4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);
}

/**
 * @brief  Initialize TIM5 for PWM generation on motor control pins
 * @retval None
 */
void Motor_TIM5_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* Enable TIM5 clock */
    __HAL_RCC_TIM5_CLK_ENABLE();
    
    /* Configure TIM5 Base */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = PWM_PRESCALER;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = PWM_PERIOD;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Configure PWM channels */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    /* Channel 1 - PA0 (Motor1_IN1) */
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Channel 2 - PA1 (Motor1_IN2) */
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Channel 3 - PA2 (Motor2_IN3) */
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Channel 4 - PA3 (Motor2_IN4) */
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Start PWM on all channels */
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

/**
 * @brief  Set motor speeds using PWM
 * @param  motor1_in1: PWM duty cycle for Motor 1 Forward (0-100%)
 * @param  motor1_in2: PWM duty cycle for Motor 1 Reverse (0-100%)
 * @param  motor2_in3: PWM duty cycle for Motor 2 Forward (0-100%)
 * @param  motor2_in4: PWM duty cycle for Motor 2 Reverse (0-100%)
 * @retval None
 */
void Motor_SetSpeed(uint8_t motor1_in1, uint8_t motor1_in2, uint8_t motor2_in3, uint8_t motor2_in4)
{
    // Convert percentage (0-100) to PWM value (0-PWM_PERIOD)
    uint32_t pwm1_in1 = (motor1_in1 * PWM_PERIOD) / 100;
    uint32_t pwm1_in2 = (motor1_in2 * PWM_PERIOD) / 100;
    uint32_t pwm2_in3 = (motor2_in3 * PWM_PERIOD) / 100;
    uint32_t pwm2_in4 = (motor2_in4 * PWM_PERIOD) / 100;
    
    // Set PWM duty cycles
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm1_in1);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm1_in2);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm2_in3);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm2_in4);
    
    // Update current values
    current_m1_in1 = motor1_in1;
    current_m1_in2 = motor1_in2;
    current_m2_in3 = motor2_in3;
    current_m2_in4 = motor2_in4;
}

/**
 * @brief  Set motor speeds with smooth acceleration/deceleration
 * @param  target_m1_in1: Target PWM duty cycle for Motor 1 Forward (0-100%)
 * @param  target_m1_in2: Target PWM duty cycle for Motor 1 Reverse (0-100%)
 * @param  target_m2_in3: Target PWM duty cycle for Motor 2 Forward (0-100%)
 * @param  target_m2_in4: Target PWM duty cycle for Motor 2 Reverse (0-100%)
 * @retval None
 */
void Motor_SetSpeed_Smooth(uint8_t target_m1_in1, uint8_t target_m1_in2, uint8_t target_m2_in3, uint8_t target_m2_in4)
{
    // Check if motors are currently stopped and target is low speed
    bool currently_stopped = (current_m1_in1 == 0 && current_m1_in2 == 0 && 
                             current_m2_in3 == 0 && current_m2_in4 == 0);
    uint8_t target_total = target_m1_in1 + target_m1_in2 + target_m2_in3 + target_m2_in4;
    bool needs_kickstart = (currently_stopped && target_total > 0 && target_total <= (SPEED_SLOW * 2) && KICKSTART_ENABLED);
    
    // Apply kick-start pulse
    if (needs_kickstart)
    {
        uint8_t kick_m1_in1 = (target_m1_in1 > 0) ? KICKSTART_DUTY : 0;
        uint8_t kick_m1_in2 = (target_m1_in2 > 0) ? KICKSTART_DUTY : 0;
        uint8_t kick_m2_in3 = (target_m2_in3 > 0) ? KICKSTART_DUTY : 0;
        uint8_t kick_m2_in4 = (target_m2_in4 > 0) ? KICKSTART_DUTY : 0;
        
        Motor_SetSpeed(kick_m1_in1, kick_m1_in2, kick_m2_in3, kick_m2_in4);
        osDelay(KICKSTART_DURATION);
    }
    
    // If acceleration is disabled, set speed instantly
    if (!accel_enabled)
    {
        Motor_SetSpeed(target_m1_in1, target_m1_in2, target_m2_in3, target_m2_in4);
        motors_moving = (target_total > 0);
        return;
    }
    
    // Determine if accelerating or decelerating
    uint8_t current_total = current_m1_in1 + current_m1_in2 + current_m2_in3 + current_m2_in4;
    bool is_accelerating = (target_total > current_total);
    uint8_t step = is_accelerating ? ACCEL_STEP : DECEL_STEP;
    uint16_t delay_ms = is_accelerating ? ACCEL_DELAY_MS : DECEL_DELAY_MS;
    
    // Ramp up/down gradually
    while (current_m1_in1 != target_m1_in1 || current_m1_in2 != target_m1_in2 ||
           current_m2_in3 != target_m2_in3 || current_m2_in4 != target_m2_in4)
    {
        // Ramp motor 1 IN1
        if (current_m1_in1 < target_m1_in1)
            current_m1_in1 = (current_m1_in1 + step > target_m1_in1) ? target_m1_in1 : current_m1_in1 + step;
        else if (current_m1_in1 > target_m1_in1)
            current_m1_in1 = (current_m1_in1 < step || current_m1_in1 - step < target_m1_in1) ? target_m1_in1 : current_m1_in1 - step;
        
        // Ramp motor 1 IN2
        if (current_m1_in2 < target_m1_in2)
            current_m1_in2 = (current_m1_in2 + step > target_m1_in2) ? target_m1_in2 : current_m1_in2 + step;
        else if (current_m1_in2 > target_m1_in2)
            current_m1_in2 = (current_m1_in2 < step || current_m1_in2 - step < target_m1_in2) ? target_m1_in2 : current_m1_in2 - step;
        
        // Ramp motor 2 IN3
        if (current_m2_in3 < target_m2_in3)
            current_m2_in3 = (current_m2_in3 + step > target_m2_in3) ? target_m2_in3 : current_m2_in3 + step;
        else if (current_m2_in3 > target_m2_in3)
            current_m2_in3 = (current_m2_in3 < step || current_m2_in3 - step < target_m2_in3) ? target_m2_in3 : current_m2_in3 - step;
        
        // Ramp motor 2 IN4
        if (current_m2_in4 < target_m2_in4)
            current_m2_in4 = (current_m2_in4 + step > target_m2_in4) ? target_m2_in4 : current_m2_in4 + step;
        else if (current_m2_in4 > target_m2_in4)
            current_m2_in4 = (current_m2_in4 < step || current_m2_in4 - step < target_m2_in4) ? target_m2_in4 : current_m2_in4 - step;
        
        // Apply current values
        Motor_SetSpeed(current_m1_in1, current_m1_in2, current_m2_in3, current_m2_in4);
        
        // Delay for smooth ramping
        osDelay(delay_ms);
    }
    
    // Update moving state
    motors_moving = (target_total > 0);
}

/**
 * @brief  Motor Forward - Both motors forward at specified speed
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Forward(uint8_t speed)
{
    Motor_SetSpeed_Smooth(speed, 0, speed, 0);
}

/**
 * @brief  Motor Reverse - Both motors backward at specified speed
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Reverse(uint8_t speed)
{
    Motor_SetSpeed_Smooth(0, speed, 0, speed);
}

/**
 * @brief  Motor Left - Left motor reverse, right motor forward (spot turn)
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Left(uint8_t speed)
{
    Motor_SetSpeed_Smooth(0, speed, speed, 0);
}

/**
 * @brief  Motor Right - Left motor forward, right motor reverse (spot turn)
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Right(uint8_t speed)
{
    Motor_SetSpeed_Smooth(speed, 0, 0, speed);
}

/**
 * @brief  Forward with differential wheel speeds (arc steering)
 * @param  left_speed: Left wheel forward PWM (0-100)
 * @param  right_speed: Right wheel forward PWM (0-100)
 * @retval None
 */
void Motor_ForwardDifferential(uint8_t left_speed, uint8_t right_speed)
{
    Motor_SetSpeed(left_speed, 0, right_speed, 0);
}

/**
 * @brief  Motor Stop - All motors off (instant stop)
 * @retval None
 */
void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0, 0, 0);
}

/**
 * @brief  Motor Stop - All motors off with smooth deceleration
 * @retval None
 */
void Motor_Stop_Smooth(void)
{
    Motor_SetSpeed_Smooth(0, 0, 0, 0);
}
