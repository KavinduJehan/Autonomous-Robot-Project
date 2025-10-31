/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - MX1508 Motor Controller with UART
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
  * 
  * @project       Motor Controller with UART - Autonomous Robot
  * @target        STM32F401RCT6
  * @motor_driver  MX1508 Dual H-Bridge
  * 
  * @description
  * This firmware implements a UART-controlled motor driver system using the 
  * STM32F401RC microcontroller. The robot receives commands via USART1 to 
  * control two DC motors through an MX1508 H-bridge driver with PWM speed control.
  * 
 * @hardware_connections
 * Motor Driver (MX1508) - PWM Control via TIM5:
 *   PA0 -> IN1 (Motor 1 Forward)  - TIM5_CH1
 *   PA1 -> IN2 (Motor 1 Reverse)  - TIM5_CH2
 *   PA2 -> IN3 (Motor 2 Forward)  - TIM5_CH3
 *   PA3 -> IN4 (Motor 2 Reverse)  - TIM5_CH4
  * 
  * UART (USART1):
  *   PA9  -> TX (Connect to RX of USB-Serial adapter)
  *   PA10 -> RX (Connect to TX of USB-Serial adapter)
  * 
  * @uart_configuration
  *   Baud Rate: 9600 bps
  *   APB2 Clock: 16 MHz
  *   BRR Value: 0x0682 (calculated: 16000000/9600 = 1666.67)
  *   Data Bits: 8, Stop Bits: 1, Parity: None
  *   Alternate Function: AF7
  * 
 * @pwm_configuration
 *   Timer: TIM5 (APB1)
  *   PWM Frequency: 1 kHz
  *   Prescaler: 15 (16MHz / 16 = 1MHz timer clock)
  *   Period: 999 (1MHz / 1000 = 1kHz PWM)
  *   Resolution: 0-100% duty cycle
  * 
  * @acceleration_configuration
  *   Acceleration Step: 5% per 20ms (smooth ramp-up)
  *   Deceleration Step: 10% per 15ms (faster ramp-down)
  *   Total Accel Time: ~400ms (0% to 100%)
  *   Total Decel Time: ~150ms (100% to 0%)
  *   Toggle: 'A' enable, 'D' disable
  * 
  * @commands
  *   'F' (0x46) - Forward:  Both motors forward at current speed
  *   'R' (0x52) - Reverse:  Both motors backward at current speed
  *   'L' (0x4C) - Left:     Left motor reverse, right forward (spot turn)
  *   'T' (0x54) - Right:    Left motor forward, right reverse (spot turn)
  *   'S' (0x53) - Stop:     Both motors stop
  *   '1' (0x31) - Speed:    Set speed to SLOW (40%)
  *   '2' (0x32) - Speed:    Set speed to MEDIUM (70%)
  *   '3' (0x33) - Speed:    Set speed to FAST (100%)
  *   'M' (0x4D) - Accel:    Enable smooth acceleration/deceleration
  *   'Z' (0x5A) - Accel:    Disable smooth ramping (instant response)
  *   'D' (0x44) - Accel:    Disable smooth ramping (alternate)
  * 
  * @safety_features
  *   - Emergency timeout: Motors stop if no command for 2 seconds
  *   - UART error handling: ORE, FE, NE, PE detection
  *   - Invalid command protection: Unknown commands trigger stop
  *   - Safe initialization: Motors stopped at startup
  * 
  * @author        Robot Project Team
  * @date          October 31, 2025
  * @version       2.1 - Added Acceleration/Deceleration
  * 
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim5;
osThreadId defaultTaskHandle;
osThreadId motorTaskHandle;
osThreadId ultrasonicTaskHandle;
osMessageQId uartCmdQueueHandle;
/* USER CODE BEGIN PV */
volatile uint8_t rx_data = 0;
volatile bool new_command = false;
volatile uint32_t last_command_time = 0;
volatile bool uart_error = false;
volatile uint8_t current_speed = SPEED_MEDIUM;  // Default speed
volatile bool accel_enabled = ACCEL_ENABLED;    // Acceleration/deceleration enable

// Current PWM values for smooth ramping
volatile uint8_t current_m1_in1 = 0;
volatile uint8_t current_m1_in2 = 0;
volatile uint8_t current_m2_in3 = 0;
volatile uint8_t current_m2_in4 = 0;

// Track if motors are currently moving (for dynamic speed changes)
volatile bool motors_moving = false;
volatile uint8_t last_movement_cmd = CMD_STOP;  // Track last movement command
#if ULTRASONIC_ENABLED
volatile uint16_t ultrasonic_left_cm = 0;  // Sensor A (left)
volatile uint16_t ultrasonic_right_cm = 0; // Sensor B (right)
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void const * argument);
void StartMotorTask(void const * argument);
void StartUltrasonicTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if ULTRASONIC_ENABLED
/* DWT-based microsecond timing utilities */
static inline void DWT_Delay_Init(void)
{
    /* Enable TRC */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* Reset the cycle counter */
    DWT->CYCCNT = 0;
    /* Enable the cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t micros(void)
{
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
}

static inline void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}

static uint16_t Ultrasonic_Measure_Pin(GPIO_TypeDef* trig_port, uint16_t trig_pin,
                                       GPIO_TypeDef* echo_port, uint16_t echo_pin)
{
    /* Ensure trigger low */
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
    delay_us(2);

    /* 10us trigger pulse */
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
    delay_us(ULTRASONIC_TRIGGER_US);
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

    uint32_t timeout_ticks = (ULTRASONIC_TIMEOUT_US * (SystemCoreClock / 1000000U));
    uint32_t tstart, tend;

    /* Wait for echo rising edge */
    uint32_t start_wait = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET)
    {
        if ((DWT->CYCCNT - start_wait) > timeout_ticks) return 0; // timeout
    }
    tstart = DWT->CYCCNT;

    /* Wait for echo falling edge */
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET)
    {
        if ((DWT->CYCCNT - tstart) > timeout_ticks) return 0; // timeout
    }
    tend = DWT->CYCCNT;

    uint32_t pulse_ticks = (tend - tstart);
    uint32_t pulse_us = pulse_ticks / (SystemCoreClock / 1000000U);

    /* Convert to centimeters: distance (cm) = pulse_us / 58 */
    uint32_t dist_cm = pulse_us / 58U;
    if (dist_cm > 400) dist_cm = 400; // clamp to sensor max
    return (uint16_t)dist_cm;
}

/* Public Ultrasonic API implementations */
void Ultrasonic_Init(void)
{
    DWT_Delay_Init();
    /* Triggers already configured in GPIO_Init; ensure low */
    HAL_GPIO_WritePin(US_GPIO_PORT, US_TRIG_A_PIN | US_TRIG_B_PIN, GPIO_PIN_RESET);
}

uint16_t Ultrasonic_MeasureA(void)
{
    return Ultrasonic_Measure_Pin(US_GPIO_PORT, US_TRIG_A_PIN, US_GPIO_PORT, US_ECHO_A_PIN);
}

uint16_t Ultrasonic_MeasureB(void)
{
    return Ultrasonic_Measure_Pin(US_GPIO_PORT, US_TRIG_B_PIN, US_GPIO_PORT, US_ECHO_B_PIN);
}

bool Ultrasonic_CheckCollision(void)
{
    uint16_t a = Ultrasonic_MeasureA();
    uint16_t b = Ultrasonic_MeasureB();
    return (a > 0 && a <= COLLISION_DISTANCE_STOP) || (b > 0 && b <= COLLISION_DISTANCE_STOP);
}
#endif /* ULTRASONIC_ENABLED */

/**
 * @brief  Initialize GPIO pins for motor control
 * @note   Configures PA0-PA3 as PWM output pins (TIM3 alternate function)
 * @retval None
 */
void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
  /* Enable GPIOA, GPIOB and GPIOC clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* Configure Motor Control Pins (PA0-PA3) as TIM5 PWM Output (AF2) */
    GPIO_InitStruct.Pin = MOTOR1_IN1_PIN | MOTOR1_IN2_PIN | MOTOR2_IN3_PIN | MOTOR2_IN4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;          // Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;       // TIM5 Alternate Function
    HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);
    
    /* Configure LED Indicator Pins (PC13, PC14) as Output */
    GPIO_InitStruct.Pin = LED_RX_PIN | LED_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    
    /* Initialize LED pins to OFF */
    HAL_GPIO_WritePin(LED_PORT, LED_RX_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_TX_PIN, GPIO_PIN_RESET);

  /* Configure Heartbeat LED (PB12) as Output */
  GPIO_InitStruct.Pin = HEARTBEAT_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEARTBEAT_LED_PORT, &GPIO_InitStruct);

  /* Initialize Heartbeat LED to OFF */
  HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_PIN_RESET);
    
    /* Configure UART Pins (PA9-TX, PA10-RX) */
    GPIO_InitStruct.Pin = UART_TX_PIN | UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = UART_AF;
    HAL_GPIO_Init(UART_PORT, &GPIO_InitStruct);

#if ULTRASONIC_ENABLED
    /* Configure Ultrasonic Trigger pins (PB0, PB1) as Output */
    GPIO_InitStruct.Pin = US_TRIG_A_PIN | US_TRIG_B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(US_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(US_GPIO_PORT, US_TRIG_A_PIN | US_TRIG_B_PIN, GPIO_PIN_RESET);

    /* Configure Ultrasonic Echo pins (PB6, PB7) as Input with pulldown */
    GPIO_InitStruct.Pin = US_ECHO_A_PIN | US_ECHO_B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(US_GPIO_PORT, &GPIO_InitStruct);
#endif
}

/**
 * @brief  Initialize TIM3 for PWM generation on motor control pins
 * @note   TIM5_CH1 -> PA0 (Motor1_IN1)
 *         TIM5_CH2 -> PA1 (Motor1_IN2)
 *         TIM5_CH3 -> PA2 (Motor2_IN3)
 *         TIM5_CH4 -> PA3 (Motor2_IN4)
 *         PWM Frequency: 1 kHz
 * @retval None
 */
void TIM5_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* Enable TIM5 clock */
    __HAL_RCC_TIM5_CLK_ENABLE();
    
    /* Configure TIM5 Base */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = PWM_PRESCALER;              // 16MHz / (15+1) = 1MHz
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = PWM_PERIOD;                    // 1MHz / (999+1) = 1kHz PWM
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Configure PWM channels */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;                               // Start with 0% duty cycle
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
 * @brief  Initialize USART1 peripheral
 * @note   Configures USART1 with proper baud rate calculation
 *         APB2 Clock = 16 MHz, Baud = 9600
 *         BRR = 16000000 / 9600 = 1666.67 ≈ 0x0682
 * @retval None
 */
void USART1_Init(void)
{
    /* Enable USART1 clock (on APB2) */
    __HAL_RCC_USART1_CLK_ENABLE();
    
    /* Disable USART1 before configuration */
    USART1->CR1 &= ~USART_CR1_UE;
    
    /* Configure USART1 Control Register 1 (CR1) */
    USART1->CR1 = 0;  // Clear register
    USART1->CR1 |= USART_CR1_RE;      // Receiver enable
    USART1->CR1 |= USART_CR1_TE;      // Transmitter enable
    USART1->CR1 |= USART_CR1_RXNEIE;  // RXNE interrupt enable
    // Word length = 8 bits (M bit = 0)
    // Parity control disabled (PCE bit = 0)
    
    /* Configure USART1 Control Register 2 (CR2) */
    USART1->CR2 = 0;  // Clear register
    // 1 Stop bit (STOP[1:0] = 00)
    
    /* Configure USART1 Control Register 3 (CR3) */
    USART1->CR3 = 0;  // Clear register
    USART1->CR3 |= USART_CR3_EIE;  // Error interrupt enable
    // No hardware flow control
    
    /* Configure Baud Rate Register (BRR)
     * Formula: BRR = fCK / Baud
     * fCK = APB2 Clock = 16 MHz
     * Baud = 9600
     * BRR = 16000000 / 9600 = 1666.67
     * BRR (hex) = 0x0682
     * 
     * Actual baud = 16000000 / 1666 = 9603.84 (0.04% error - acceptable)
     */
    USART1->BRR = 0x0682;  // For 9600 baud at 16MHz APB2 clock
    
    /* Enable USART1 */
    USART1->CR1 |= USART_CR1_UE;
    
    /* Configure NVIC for USART1 interrupt */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
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
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm1_in1);  // Motor1 IN1
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm1_in2);  // Motor1 IN2
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm2_in3);  // Motor2 IN3
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm2_in4);  // Motor2 IN4
    
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
 * @note   Gradually ramps from current speed to target speed
 * @retval None
 */
void Motor_SetSpeed_Smooth(uint8_t target_m1_in1, uint8_t target_m1_in2, uint8_t target_m2_in3, uint8_t target_m2_in4)
{
    // Check if motors are currently stopped and target is low speed -> apply kick-start
    bool currently_stopped = (current_m1_in1 == 0 && current_m1_in2 == 0 && 
                             current_m2_in3 == 0 && current_m2_in4 == 0);
    uint8_t target_total = target_m1_in1 + target_m1_in2 + target_m2_in3 + target_m2_in4;
    bool needs_kickstart = (currently_stopped && target_total > 0 && target_total <= (SPEED_SLOW * 2) && KICKSTART_ENABLED);
    
    // Apply kick-start pulse to overcome static friction
    if (needs_kickstart)
    {
        // Brief high-power pulse to break static friction
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
    
    // Determine if we're accelerating or decelerating
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
 */
void Motor_ForwardDifferential(uint8_t left_speed, uint8_t right_speed)
{
    Motor_SetSpeed_Smooth(left_speed, 0, right_speed, 0);
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
 * @brief  Process received command
 * @param  cmd: Command character received via UART
 * @retval None
 */
void Process_Command(uint8_t cmd)
{
    switch(cmd)
    {
        case CMD_FORWARD:
            Motor_Forward(current_speed);
            last_movement_cmd = CMD_FORWARD;
            break;
            
        case CMD_REVERSE:
            Motor_Reverse(current_speed);
            last_movement_cmd = CMD_REVERSE;
            break;
            
        case CMD_LEFT:
            Motor_Left(current_speed);
            last_movement_cmd = CMD_LEFT;
            break;
            
        case CMD_RIGHT:
            Motor_Right(current_speed);
            last_movement_cmd = CMD_RIGHT;
            break;
            
        case CMD_STOP:
            if (accel_enabled)
                Motor_Stop_Smooth();  // Smooth stop if accel enabled
            else
                Motor_Stop();         // Instant stop if accel disabled
            last_movement_cmd = CMD_STOP;
            break;
        
        case 'E': // Emergency stop alias from UI
            Motor_Stop();
            last_movement_cmd = CMD_STOP;
            break;
            
        case CMD_SPEED_SLOW:
            current_speed = SPEED_SLOW;
            // If motors are moving, ramp to new speed smoothly
            if (motors_moving && last_movement_cmd != CMD_STOP)
            {
                // Re-issue last movement command with new speed
                switch(last_movement_cmd)
                {
                    case CMD_FORWARD:
                        Motor_Forward(current_speed);
                        break;
                    case CMD_REVERSE:
                        Motor_Reverse(current_speed);
                        break;
                    case CMD_LEFT:
                        Motor_Left(current_speed);
                        break;
                    case CMD_RIGHT:
                        Motor_Right(current_speed);
                        break;
                }
            }
            break;
            
        case CMD_SPEED_MEDIUM:
            current_speed = SPEED_MEDIUM;
            // If motors are moving, ramp to new speed smoothly
            if (motors_moving && last_movement_cmd != CMD_STOP)
            {
                // Re-issue last movement command with new speed
                switch(last_movement_cmd)
                {
                    case CMD_FORWARD:
                        Motor_Forward(current_speed);
                        break;
                    case CMD_REVERSE:
                        Motor_Reverse(current_speed);
                        break;
                    case CMD_LEFT:
                        Motor_Left(current_speed);
                        break;
                    case CMD_RIGHT:
                        Motor_Right(current_speed);
                        break;
                }
            }
            break;
            
        case CMD_SPEED_FAST:
            current_speed = SPEED_FAST;
            // If motors are moving, ramp to new speed smoothly
            if (motors_moving && last_movement_cmd != CMD_STOP)
            {
                // Re-issue last movement command with new speed
                switch(last_movement_cmd)
                {
                    case CMD_FORWARD:
                        Motor_Forward(current_speed);
                        break;
                    case CMD_REVERSE:
                        Motor_Reverse(current_speed);
                        break;
                    case CMD_LEFT:
                        Motor_Left(current_speed);
                        break;
                    case CMD_RIGHT:
                        Motor_Right(current_speed);
                        break;
                }
            }
            break;
            
        case CMD_ACCEL_ENABLE:
            accel_enabled = true;
            break;
            
        case CMD_ACCEL_DISABLE:
        case CMD_ACCEL_DISABLE_ALT: // 'Z' and 'D' both disable
            accel_enabled = false;
            break;
        
        case CMD_SELF_TEST: // now 'X'
        {
            bool prev_accel = accel_enabled;
            accel_enabled = false; // instant steps during test
            /* Simple on-demand test: forward 40/70/100%, then stop */
            Motor_Forward(40); osDelay(1000);
            Motor_Forward(70); osDelay(1000);
            Motor_Forward(100); osDelay(1000);
            Motor_Stop();
            accel_enabled = prev_accel;
            break;
        }
            
        default:
            // Invalid command - do nothing or stop for safety
            Motor_Stop();
            last_movement_cmd = CMD_STOP;
            break;
    }
    
    // Update last command time
    last_command_time = HAL_GetTick();
}

/**
 * @brief  Safety check - emergency stop if no command received
 * @retval None
 */
void Safety_Check(void)
{
    if ((HAL_GetTick() - last_command_time) > SAFETY_TIMEOUT_MS)
    {
        Motor_Stop();
    }
}

/**
 * @brief  USART1 Interrupt Handler
 * @note   Handles received data and errors
 * @retval None
 */
void USART1_IRQHandler(void)
{
    /* Check for RXNE (Receive Data Register Not Empty) */
    if (USART1->SR & USART_SR_RXNE)
    {
        /* Toggle RX LED to indicate activity */
        HAL_GPIO_TogglePin(LED_PORT, LED_RX_PIN);
        
        /* Read data register (clears RXNE flag) */
    rx_data = (uint8_t)(USART1->DR & 0xFF);
    /* Push received byte into the UART command queue (ISR-safe) */
    if (uartCmdQueueHandle != NULL)
    {
      osMessagePut(uartCmdQueueHandle, rx_data, 0);
    }
    }
    
    /* Check for TXE (Transmit Data Register Empty) - indicates TX activity */
    if (USART1->SR & USART_SR_TXE)
    {
        /* Toggle TX LED to indicate activity (optional) */
        HAL_GPIO_TogglePin(LED_PORT, LED_TX_PIN);
    }
    
    /* Check for errors */
    if (USART1->SR & USART_SR_ORE)  // Overrun error
    {
        uart_error = true;
        volatile uint32_t dummy = USART1->DR;  // Clear ORE flag
        (void)dummy;  // Avoid unused variable warning
    }
    
    if (USART1->SR & USART_SR_FE)   // Framing error
    {
        uart_error = true;
        volatile uint32_t dummy = USART1->DR;  // Clear FE flag
        (void)dummy;
    }
    
    if (USART1->SR & USART_SR_NE)   // Noise error
    {
        uart_error = true;
        volatile uint32_t dummy = USART1->DR;  // Clear NE flag
        (void)dummy;
    }
    
    if (USART1->SR & USART_SR_PE)   // Parity error
    {
        uart_error = true;
        volatile uint32_t dummy = USART1->DR;  // Clear PE flag
        (void)dummy;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  
  /* Initialize GPIO for motor control and UART */
  GPIO_Init();
  
    /* Initialize TIM5 for PWM motor control */
    TIM5_PWM_Init();
  
  /* Initialize USART1 for command reception */
  USART1_Init();

#if ULTRASONIC_ENABLED
    /* Initialize Ultrasonic sensors */
    Ultrasonic_Init();
#endif
  
  /* Initialize safety timer */
  last_command_time = HAL_GetTick();
  
  /* Start with motors stopped */
  Motor_Stop();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* Create UART command queue (stores command bytes) */
  osMessageQDef(uartCmdQueue, 32, uint16_t);
  uartCmdQueueHandle = osMessageCreate(osMessageQ(uartCmdQueue), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of motorTask */
  osThreadDef(motorTask, StartMotorTask, osPriorityAboveNormal, 0, 256);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

#if ULTRASONIC_ENABLED
    /* definition and creation of ultrasonicTask */
    osThreadDef(ultrasonicTask, StartUltrasonicTask, osPriorityNormal, 0, 256);
    ultrasonicTaskHandle = osThreadCreate(osThread(ultrasonicTask), NULL);
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    // Main loop is now managed by FreeRTOS tasks
    osDelay(1000); // Idle loop, should never reach here
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Housekeeping/idle task */
  for(;;)
  {
  /* Blink a heartbeat so you can see the RTOS is running */
  HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
    osDelay(250);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN 4_EXT */
/**
  * @brief  Function implementing the motorTask thread.
  *         Receives UART bytes from queue and drives motors.
  */
void StartMotorTask(void const * argument)
{
    static bool self_test_done = false;
    for(;;)
  {
        /* Optional: run a short PWM self-test once at boot to validate outputs */
        if (!self_test_done && ENABLE_PWM_SELF_TEST)
        {
            bool prev_accel = accel_enabled;
            accel_enabled = false; // make transitions immediate for clear observation

            /* Forward: 40%, 70%, 100% */
            Motor_Forward(40); osDelay(2000);
            Motor_Forward(70); osDelay(2000);
            Motor_Forward(100); osDelay(2000);

            /* Reverse: 40% */
            Motor_Reverse(40); osDelay(2000);

            /* Left and Right spot turns */
            Motor_Left(70); osDelay(2000);
            Motor_Right(70); osDelay(2000);

            Motor_Stop();
            accel_enabled = prev_accel;
            self_test_done = true;
        }
    /* Wait up to 10ms for a new command */
    osEvent evt = osMessageGet(uartCmdQueueHandle, 10);
    if (evt.status == osEventMessage)
    {
        uint8_t cmd = (uint8_t)(evt.value.v & 0xFF);
        Process_Command(cmd);
        uart_error = false; // Clear error on good cmd
    }

    /* Periodic safety check */
    Safety_Check();

    /* If an error was flagged by IRQ, react safely */
    if (uart_error)
    {
        Motor_Stop();
        uart_error = false;
    }
  }
}
/* USER CODE END 4_EXT */

#if ULTRASONIC_ENABLED
/* USER CODE BEGIN Header_StartUltrasonicTask */
/**
    * @brief  Function implementing the ultrasonicTask thread.
    *         Continuously measures side distances and applies wall avoidance.
    */
/* USER CODE END Header_StartUltrasonicTask */
void StartUltrasonicTask(void const * argument)
{
    for(;;)
    {
        /* Measure left (A) and right (B) distances */
        uint16_t left = Ultrasonic_MeasureA();
        uint16_t right = Ultrasonic_MeasureB();
        ultrasonic_left_cm = left;
        ultrasonic_right_cm = right;

        /* Emergency stop if any side is dangerously close */
        if ((left > 0 && left <= COLLISION_DISTANCE_STOP) ||
                (right > 0 && right <= COLLISION_DISTANCE_STOP))
        {
                Motor_Stop();
                last_movement_cmd = CMD_STOP;
                motors_moving = false;
                osDelay(ULTRASONIC_MEASURE_INTERVAL_MS);
                continue;
        }

        /* Apply gentle steering only when moving forward */
        if (last_movement_cmd == CMD_FORWARD && motors_moving)
        {
                int base = (int)current_speed;
                int left_cmd = base;
                int right_cmd = base;

                /* If left wall too close, reduce left speed to steer right */
                if (left > 0 && left < COLLISION_DISTANCE_SLOW)
                {
                        int delta_cm = (int)COLLISION_DISTANCE_SLOW - (int)left; // positive inside zone
                        int corr = delta_cm * WALL_CORR_GAIN_PCT_PER_CM; // % reduction
                        left_cmd -= corr;
                }

                /* If right wall too close, reduce right speed to steer left */
                if (right > 0 && right < COLLISION_DISTANCE_SLOW)
                {
                        int delta_cm = (int)COLLISION_DISTANCE_SLOW - (int)right;
                        int corr = delta_cm * WALL_CORR_GAIN_PCT_PER_CM;
                        right_cmd -= corr;
                }

        /* Clamp speeds */
        if (left_cmd < 0) {
            left_cmd = 0;
        }
        if (left_cmd > 100) {
            left_cmd = 100;
        }
        if (right_cmd < 0) {
            right_cmd = 0;
        }
        if (right_cmd > 100) {
            right_cmd = 100;
        }

                /* Only update if any correction applied */
                if (left_cmd != base || right_cmd != base)
                {
                        Motor_ForwardDifferential((uint8_t)left_cmd, (uint8_t)right_cmd);
                }
        }

        osDelay(ULTRASONIC_MEASURE_INTERVAL_MS);
    }
}
#endif /* ULTRASONIC_ENABLED */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
