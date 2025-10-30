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
  * control two DC motors through an MX1508 H-bridge driver.
  * 
  * @hardware_connections
  * Motor Driver (MX1508):
  *   PA0 -> IN1 (Motor 1 Forward)
  *   PA1 -> IN2 (Motor 1 Reverse)
  *   PA2 -> IN3 (Motor 2 Forward)
  *   PA3 -> IN4 (Motor 2 Reverse)
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
  * @commands
  *   'F' (0x46) - Forward:  Both motors forward
  *   'R' (0x52) - Reverse:  Both motors backward
  *   'L' (0x4C) - Left:     Left motor reverse, right forward (spot turn)
  *   'T' (0x54) - Right:    Left motor forward, right reverse (spot turn)
  *   'S' (0x53) - Stop:     Both motors stop
  * 
  * @safety_features
  *   - Emergency timeout: Motors stop if no command for 2 seconds
  *   - UART error handling: ORE, FE, NE, PE detection
  *   - Invalid command protection: Unknown commands trigger stop
  *   - Safe initialization: Motors stopped at startup
  * 
  * @author        Robot Project Team
  * @date          October 30, 2025
  * @version       1.0
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
osThreadId defaultTaskHandle;
osThreadId motorTaskHandle;
osMessageQId uartCmdQueueHandle;
/* USER CODE BEGIN PV */
volatile uint8_t rx_data = 0;
volatile bool new_command = false;
volatile uint32_t last_command_time = 0;
volatile bool uart_error = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void const * argument);
void StartMotorTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Initialize GPIO pins for motor control
 * @note   Configures PA0-PA3 as output pins for MX1508 motor driver
 * @retval None
 */
void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
  /* Enable GPIOA, GPIOB and GPIOC clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* Configure Motor Control Pins (PA0-PA3) as Output */
    GPIO_InitStruct.Pin = MOTOR1_IN1_PIN | MOTOR1_IN2_PIN | MOTOR2_IN3_PIN | MOTOR2_IN4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);
    
    /* Initialize all motor pins to LOW (stopped state) */
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN4_PIN, GPIO_PIN_RESET);
    
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
 * @brief  Motor Forward - Both motors forward
 * @retval None
 */
void Motor_Forward(void)
{
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN4_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Motor Reverse - Both motors backward
 * @retval None
 */
void Motor_Reverse(void)
{
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN4_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Motor Left - Left motor reverse, right motor forward
 * @retval None
 */
void Motor_Left(void)
{
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN4_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Motor Right - Left motor forward, right motor reverse
 * @retval None
 */
void Motor_Right(void)
{
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN4_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Motor Stop - All motors off
 * @retval None
 */
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_IN4_PIN, GPIO_PIN_RESET);
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
            Motor_Forward();
            break;
            
        case CMD_REVERSE:
            Motor_Reverse();
            break;
            
        case CMD_LEFT:
            Motor_Left();
            break;
            
        case CMD_RIGHT:
            Motor_Right();
            break;
            
        case CMD_STOP:
            Motor_Stop();
            break;
            
        default:
            // Invalid command - do nothing or stop for safety
            Motor_Stop();
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
  
  /* Initialize USART1 for command reception */
  USART1_Init();
  
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
  for(;;)
  {
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
