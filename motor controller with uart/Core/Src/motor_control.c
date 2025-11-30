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

/* Helper: apply channel values respecting SWAP_MOTORS configuration */
static void Motor_ApplyChannels(uint8_t l_fwd, uint8_t l_rev, uint8_t r_fwd, uint8_t r_rev)
{
#if SWAP_MOTORS
    Motor_SetSpeed(r_fwd, r_rev, l_fwd, l_rev);
#else
    Motor_SetSpeed(l_fwd, l_rev, r_fwd, r_rev);
#endif
}

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
    /* Apply per-motor scaling (defined in main.h) to compensate hardware mismatch.
     * Scaling is a percentage where 100 means no change.
     */
    uint32_t m1_in1 = motor1_in1;
    uint32_t m1_in2 = motor1_in2;
    uint32_t m2_in3 = motor2_in3;
    uint32_t m2_in4 = motor2_in4;

#if (MOTOR1_PWM_SCALE != 100) || (MOTOR2_PWM_SCALE != 100)
    /* scale forward/reverse channels independently; clamp to 100 */
    m1_in1 = (m1_in1 * MOTOR1_PWM_SCALE) / 100;
    m1_in2 = (m1_in2 * MOTOR1_PWM_SCALE) / 100;
    if (m1_in1 > 100) m1_in1 = 100;
    if (m1_in2 > 100) m1_in2 = 100;

    m2_in3 = (m2_in3 * MOTOR2_PWM_SCALE) / 100;
    m2_in4 = (m2_in4 * MOTOR2_PWM_SCALE) / 100;
    if (m2_in3 > 100) m2_in3 = 100;
    if (m2_in4 > 100) m2_in4 = 100;
#endif

    /* Convert percentage (0-100) to PWM value (0-PWM_PERIOD) */
    uint32_t pwm1_in1 = (m1_in1 * PWM_PERIOD) / 100;
    uint32_t pwm1_in2 = (m1_in2 * PWM_PERIOD) / 100;
    uint32_t pwm2_in3 = (m2_in3 * PWM_PERIOD) / 100;
    uint32_t pwm2_in4 = (m2_in4 * PWM_PERIOD) / 100;
    
    // Set PWM duty cycles
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm1_in1);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm1_in2);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm2_in3);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm2_in4);
    
    // Update current values (store logical commanded values)
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
    /* Compute per-motor forward/reverse PWM based on inversion flags.
       motor1 = left, motor2 = right (logical). */
    uint8_t m1_fwd = 0, m1_rev = 0, m2_fwd = 0, m2_rev = 0;

    m1_fwd = (INVERT_MOTOR1_DIRECTION) ? 0 : speed;
    m1_rev = (INVERT_MOTOR1_DIRECTION) ? speed : 0;
    m2_fwd = (INVERT_MOTOR2_DIRECTION) ? 0 : speed;
    m2_rev = (INVERT_MOTOR2_DIRECTION) ? speed : 0;

#if SWAP_MOTORS
    /* Swap logical motor order if needed */
    Motor_SetSpeed_Smooth(m2_fwd, m2_rev, m1_fwd, m1_rev);
#else
    Motor_SetSpeed_Smooth(m1_fwd, m1_rev, m2_fwd, m2_rev);
#endif
}

/**
 * @brief  Motor Reverse - Both motors backward at specified speed
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Reverse(uint8_t speed)
{
    uint8_t m1_fwd = 0, m1_rev = 0, m2_fwd = 0, m2_rev = 0;

    /* Reverse = logical reverse -> swap forward/reverse per motor */
    m1_fwd = (INVERT_MOTOR1_DIRECTION) ? speed : 0;
    m1_rev = (INVERT_MOTOR1_DIRECTION) ? 0 : speed;
    m2_fwd = (INVERT_MOTOR2_DIRECTION) ? speed : 0;
    m2_rev = (INVERT_MOTOR2_DIRECTION) ? 0 : speed;

#if SWAP_MOTORS
    Motor_SetSpeed_Smooth(m2_fwd, m2_rev, m1_fwd, m1_rev);
#else
    Motor_SetSpeed_Smooth(m1_fwd, m1_rev, m2_fwd, m2_rev);
#endif
}

/**
 * @brief  Motor Left - Left motor reverse, right motor forward (spot turn)
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Left(uint8_t speed)
{
    /* Spot turn left: left motor reverse, right motor forward */
    uint8_t l_fwd = 0, l_rev = 0, r_fwd = 0, r_rev = 0;

    l_fwd = (INVERT_MOTOR1_DIRECTION) ? speed : 0;
    l_rev = (INVERT_MOTOR1_DIRECTION) ? 0 : speed;
    r_fwd = (INVERT_MOTOR2_DIRECTION) ? 0 : speed;
    r_rev = (INVERT_MOTOR2_DIRECTION) ? speed : 0;

#if SWAP_MOTORS
    Motor_SetSpeed_Smooth(r_fwd, r_rev, l_fwd, l_rev);
#else
    Motor_SetSpeed_Smooth(l_fwd, l_rev, r_fwd, r_rev);
#endif
}

/**
 * @brief  Motor Right - Left motor forward, right motor reverse (spot turn)
 * @param  speed: Motor speed (0-100%)
 * @retval None
 */
void Motor_Right(uint8_t speed)
{
    /* Spot turn right: left motor forward, right motor reverse */
    uint8_t l_fwd = 0, l_rev = 0, r_fwd = 0, r_rev = 0;

    l_fwd = (INVERT_MOTOR1_DIRECTION) ? 0 : speed;
    l_rev = (INVERT_MOTOR1_DIRECTION) ? speed : 0;
    r_fwd = (INVERT_MOTOR2_DIRECTION) ? speed : 0;
    r_rev = (INVERT_MOTOR2_DIRECTION) ? 0 : speed;

#if SWAP_MOTORS
    Motor_SetSpeed_Smooth(r_fwd, r_rev, l_fwd, l_rev);
#else
    Motor_SetSpeed_Smooth(l_fwd, l_rev, r_fwd, r_rev);
#endif
}

/**
 * @brief  Forward with differential wheel speeds (arc steering)
 * @param  left_speed: Left wheel forward PWM (0-100)
 * @param  right_speed: Right wheel forward PWM (0-100)
 * @retval None
 */
void Motor_ForwardDifferential(uint8_t left_speed, uint8_t right_speed)
{
    /* Differential forward with independent left/right speeds */
    uint8_t l_fwd = (INVERT_MOTOR1_DIRECTION) ? 0 : left_speed;
    uint8_t l_rev = (INVERT_MOTOR1_DIRECTION) ? left_speed : 0;
    uint8_t r_fwd = (INVERT_MOTOR2_DIRECTION) ? 0 : right_speed;
    uint8_t r_rev = (INVERT_MOTOR2_DIRECTION) ? right_speed : 0;

#if SWAP_MOTORS
    Motor_SetSpeed(r_fwd, r_rev, l_fwd, l_rev);
#else
    Motor_SetSpeed(l_fwd, l_rev, r_fwd, r_rev);
#endif
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

/**
 * @brief Perform a smooth spot turn: ramp up then ramp down over duration
 * @param speed: peak turn speed (0-100)
 * @param duration_ms: total duration of the turn in milliseconds
 * @param left: true -> turn left, false -> turn right
 * @note This is a blocking call. It uses Motor_SetSpeed to apply per-step
 *       PWM values and respects INVERT/SWAP configuration.
 */
void Motor_SpotTurnSmooth(uint8_t speed, uint32_t duration_ms, bool left)
{
    if (duration_ms < 2 || speed == 0)
    {
        return;
    }

    /* Define explicit ramp-up time (ms) per user's request (first ~300ms) */
    uint32_t ramp_up_ms = (duration_ms > 300) ? 300 : (duration_ms / 2);
    if (ramp_up_ms > duration_ms - 1) ramp_up_ms = duration_ms / 2;
    uint32_t ramp_down_ms = duration_ms - ramp_up_ms;

    /* Granularity: use accel delay as step interval for predictable timing */
    uint32_t up_steps = (ramp_up_ms + ACCEL_DELAY_MS - 1) / ACCEL_DELAY_MS;
    uint32_t down_steps = (ramp_down_ms + DECEL_DELAY_MS - 1) / DECEL_DELAY_MS;
    if (up_steps == 0) up_steps = 1;
    if (down_steps == 0) down_steps = 1;

    /* Compute per-step increment/decrement */
    uint32_t up_step_value = (speed + up_steps - 1) / up_steps; /* ceil */
    uint32_t down_step_value = (speed + down_steps - 1) / down_steps;

        /* Helper to apply logical per-motor channel values (handled by
         * file-scope Motor_ApplyChannels). */

    /* Start from stopped (ensure safe) */
    Motor_Stop_Smooth();

    /* Ramp up */
    uint32_t current = 0;
    for (uint32_t step = 0; step < up_steps; ++step)
    {
        current += up_step_value;
        if (current > speed) current = speed;

        if (left)
        {
            /* left turn: left motor reverse, right motor forward */
            uint8_t l_fwd = (INVERT_MOTOR1_DIRECTION) ? current : 0;
            uint8_t l_rev = (INVERT_MOTOR1_DIRECTION) ? 0 : current;
            uint8_t r_fwd = (INVERT_MOTOR2_DIRECTION) ? 0 : current;
            uint8_t r_rev = (INVERT_MOTOR2_DIRECTION) ? current : 0;
            Motor_ApplyChannels(l_fwd, l_rev, r_fwd, r_rev);
        }
        else
        {
            /* right turn: left motor forward, right motor reverse */
            uint8_t l_fwd = (INVERT_MOTOR1_DIRECTION) ? 0 : current;
            uint8_t l_rev = (INVERT_MOTOR1_DIRECTION) ? current : 0;
            uint8_t r_fwd = (INVERT_MOTOR2_DIRECTION) ? current : 0;
            uint8_t r_rev = (INVERT_MOTOR2_DIRECTION) ? 0 : current;
            Motor_ApplyChannels(l_fwd, l_rev, r_fwd, r_rev);
        }

        osDelay(ACCEL_DELAY_MS);
    }

    /* Maintain peak for a minimal tick if needed */
    if (ramp_down_ms > 0 && down_steps > 0)
    {
        /* Ramp down */
        for (uint32_t step = 0; step < down_steps; ++step)
        {
            if (current <= down_step_value) current = 0;
            else current -= down_step_value;

            if (left)
            {
                uint8_t l_fwd = (INVERT_MOTOR1_DIRECTION) ? current : 0;
                uint8_t l_rev = (INVERT_MOTOR1_DIRECTION) ? 0 : current;
                uint8_t r_fwd = (INVERT_MOTOR2_DIRECTION) ? 0 : current;
                uint8_t r_rev = (INVERT_MOTOR2_DIRECTION) ? current : 0;
                Motor_ApplyChannels(l_fwd, l_rev, r_fwd, r_rev);
            }
            else
            {
                uint8_t l_fwd = (INVERT_MOTOR1_DIRECTION) ? 0 : current;
                uint8_t l_rev = (INVERT_MOTOR1_DIRECTION) ? current : 0;
                uint8_t r_fwd = (INVERT_MOTOR2_DIRECTION) ? current : 0;
                uint8_t r_rev = (INVERT_MOTOR2_DIRECTION) ? 0 : current;
                Motor_ApplyChannels(l_fwd, l_rev, r_fwd, r_rev);
            }

            osDelay(DECEL_DELAY_MS);
        }
    }

    /* Ensure fully stopped at the end */
    Motor_Stop_Smooth();
}
