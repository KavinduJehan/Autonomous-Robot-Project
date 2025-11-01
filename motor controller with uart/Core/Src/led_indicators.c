/**
  ******************************************************************************
  * @file           : led_indicators.c
  * @brief          : LED indicators module implementation
  * @description    : Control for RX/TX, heartbeat, and wall detection LEDs
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "led_indicators.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize all LED GPIO pins
 * @retval None
 */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
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
    
    /* Configure Wall Debug LEDs (PB14, PB15) as Output */
    GPIO_InitStruct.Pin = WALL_LEFT_LED_PIN | WALL_RIGHT_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WALL_LED_PORT, &GPIO_InitStruct);

    /* Initialize Wall Debug LEDs to OFF */
    HAL_GPIO_WritePin(WALL_LED_PORT, WALL_LEFT_LED_PIN | WALL_RIGHT_LED_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Toggle RX activity LED
 * @retval None
 */
void LED_Toggle_RX(void)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_RX_PIN);
}

/**
 * @brief  Toggle TX activity LED
 * @retval None
 */
void LED_Toggle_TX(void)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_TX_PIN);
}

/**
 * @brief  Toggle heartbeat LED
 * @retval None
 */
void LED_Toggle_Heartbeat(void)
{
    HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
}

/**
 * @brief  Set wall detection LED states
 * @param  left_state: GPIO_PIN_SET or GPIO_PIN_RESET
 * @param  right_state: GPIO_PIN_SET or GPIO_PIN_RESET
 * @retval None
 */
void LED_SetWallIndicators(GPIO_PinState left_state, GPIO_PinState right_state)
{
#if WALL_LED_ACTIVE_LOW
    left_state = (left_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    right_state = (right_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif
    HAL_GPIO_WritePin(WALL_LED_PORT, WALL_LEFT_LED_PIN, left_state);
    HAL_GPIO_WritePin(WALL_LED_PORT, WALL_RIGHT_LED_PIN, right_state);
}

/**
 * @brief  Run LED self-test (brief blink sequence)
 * @retval None
 */
void LED_SelfTest(void)
{
    GPIO_PinState on_state = GPIO_PIN_SET;
    GPIO_PinState off_state = GPIO_PIN_RESET;
#if WALL_LED_ACTIVE_LOW
    on_state = GPIO_PIN_RESET;
    off_state = GPIO_PIN_SET;
#endif
    HAL_GPIO_WritePin(WALL_LED_PORT, WALL_LEFT_LED_PIN | WALL_RIGHT_LED_PIN, on_state);
    HAL_Delay(150);
    HAL_GPIO_WritePin(WALL_LED_PORT, WALL_LEFT_LED_PIN | WALL_RIGHT_LED_PIN, off_state);
}
