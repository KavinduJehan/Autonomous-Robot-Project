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
  * @author        Robot Project Team
  * @date          October 31, 2025
  * @version       2.2 - Modular Architecture
  * 
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "uart_comm.h"
#include "led_indicators.h"
#include "ultrasonic.h"
#include "command_processor.h"
#include "wall_avoidance.h"
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
osThreadId ultrasonicTaskHandle;

/* USER CODE BEGIN PV */

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
 * @brief  USART1 Interrupt Handler (redirects to module)
 * @retval None
 */
void USART1_IRQHandler(void)
{
    UART_IRQHandler();
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
    
    /* Initialize GPIO for motors, UART, LEDs, and sensors */
    Motor_GPIO_Init();
    UART_GPIO_Init();
    LED_Init();
    
#if ULTRASONIC_ENABLED
    Ultrasonic_GPIO_Init();
#endif
    
    /* Initialize TIM5 for PWM motor control */
    Motor_TIM5_PWM_Init();
    
    /* Initialize USART1 for command reception */
    UART_Init();

#if DEBUG_BOOT_BANNER
    /* Print boot banner */
    UART_SendCRLF();
    UART_SendString("BOOT: MX1508 Motor Controller v2.2 (STM32F401RC)\r\n");
    UART_SendString("Architecture: Modular Design\r\n");
    UART_SendString("Clock=HSI 16MHz, UART1=9600 8N1\r\n");
#if ULTRASONIC_ENABLED
    UART_SendString("Ultrasonic: ENABLED");
#else
    UART_SendString("Ultrasonic: DISABLED");
#endif
    UART_SendString(", Debug=");
#if ULTRASONIC_DEBUG
    UART_SendString("ON\r\n");
#else
    UART_SendString("OFF\r\n");
#endif
    UART_SendString("Modules: motor_control, uart_comm, led_indicators, ");
    UART_SendString("ultrasonic, command_processor, wall_avoidance\r\n");
#endif

#if ULTRASONIC_ENABLED
    /* Initialize Ultrasonic sensors */
    Ultrasonic_Init();
#endif
    
    /* Run LED self-test */
    LED_SelfTest();
    
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
    /* Create UART command queue */
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
    osThreadDef(ultrasonicTask, WallAvoidance_Task, osPriorityNormal, 0, 256);
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
        osDelay(1000);
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

    /** Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure. */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
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
        /* Blink heartbeat LED */
        LED_Toggle_Heartbeat();
        
#if DEBUG_UART_HEARTBEAT
        static uint32_t hb_counter = 0;
        if (!uart_rx_seen) 
        {
            if ((hb_counter++ % 4U) == 0U)
            {
                UART_SendString("HB t="); 
                UART_SendUInt(HAL_GetTick()); 
                UART_SendCRLF();
            }
        }
#endif
        osDelay(250);
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN 4_EXT */
/**
  * @brief  Function implementing the motorTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartMotorTask(void const * argument)
{
    static bool self_test_done = false;
    
    for(;;)
    {
        /* Optional PWM self-test at boot */
        if (!self_test_done && ENABLE_PWM_SELF_TEST)
        {
            bool prev_accel = accel_enabled;
            accel_enabled = false;

            Motor_Forward(40); osDelay(2000);
            Motor_Forward(70); osDelay(2000);
            Motor_Forward(100); osDelay(2000);
            Motor_Reverse(40); osDelay(2000);
            Motor_Left(70); osDelay(2000);
            Motor_Right(70); osDelay(2000);
            Motor_Stop();
            
            accel_enabled = prev_accel;
            self_test_done = true;
        }
        
        /* Wait for UART command from queue */
        osEvent evt = osMessageGet(uartCmdQueueHandle, 10);
        if (evt.status == osEventMessage)
        {
            uint8_t cmd = (uint8_t)(evt.value.v & 0xFF);
            Command_Process(cmd);
            uart_error = false;
        }

        /* Periodic safety check */
        Command_SafetyCheck();

        /* React to UART errors */
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
    /* USER can add implementation here */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
