/**
  ******************************************************************************
  * @file           : uart_comm.c
  * @brief          : UART communication module implementation
  * @description    : UART initialization, interrupt handling, and debug output
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uart_comm.h"
#include "led_indicators.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint8_t rx_data = 0;
volatile bool uart_error = false;
volatile bool uart_rx_seen = false;
osMessageQId uartCmdQueueHandle;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize GPIO pins for UART
 * @retval None
 */
void UART_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIOA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
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
 * @retval None
 */
void UART_Init(void)
{
    /* Enable USART1 clock (on APB2) */
    __HAL_RCC_USART1_CLK_ENABLE();
    
    /* Disable USART1 before configuration */
    USART1->CR1 &= ~USART_CR1_UE;
    
    /* Configure USART1 Control Register 1 (CR1) */
    USART1->CR1 = 0;
    USART1->CR1 |= USART_CR1_RE;
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_RXNEIE;
    
    /* Configure USART1 Control Register 2 (CR2) */
    USART1->CR2 = 0;
    
    /* Configure USART1 Control Register 3 (CR3) */
    USART1->CR3 = 0;
    USART1->CR3 |= USART_CR3_EIE;
    
    /* Configure Baud Rate Register (BRR) for 9600 baud at 16MHz */
    USART1->BRR = 0x0682;
    
    /* Enable USART1 */
    USART1->CR1 |= USART_CR1_UE;
    
    /* Configure NVIC for USART1 interrupt */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief  USART1 Interrupt Handler
 * @retval None
 */
void UART_IRQHandler(void)
{
    /* Check for RXNE (Receive Data Register Not Empty) */
    if (USART1->SR & USART_SR_RXNE)
    {
        LED_Toggle_RX();
        
        rx_data = (uint8_t)(USART1->DR & 0xFF);
        uart_rx_seen = true;
        
        if (uartCmdQueueHandle != NULL)
        {
            osMessagePut(uartCmdQueueHandle, rx_data, 0);
        }
    }
    
    /* Check for TXE (Transmit Data Register Empty) */
    if (USART1->SR & USART_SR_TXE)
    {
        LED_Toggle_TX();
    }
    
    /* Check for errors */
    if (USART1->SR & USART_SR_ORE)
    {
        uart_error = true;
        volatile uint32_t dummy = USART1->DR;
        (void)dummy;
    }
    
    if (USART1->SR & USART_SR_FE)
    {
        uart_error = true;
        volatile uint32_t dummy = USART1->DR;
        (void)dummy;
    }
    
    if (USART1->SR & USART_SR_NE)
    {
        uart_error = true;
        volatile uint32_t dummy = USART1->DR;
        (void)dummy;
    }
    
    if (USART1->SR & USART_SR_PE)
    {
        uart_error = true;
        volatile uint32_t dummy = USART1->DR;
        (void)dummy;
    }
}

/**
 * @brief  Send string via UART (polling mode)
 * @param  s: Null-terminated string
 * @retval None
 */
void UART_SendString(const char* s)
{
    while (*s)
    {
        while (!(USART1->SR & USART_SR_TXE)) { }
        USART1->DR = (uint8_t)(*s++);
    }
}

/**
 * @brief  Send unsigned integer via UART (polling mode)
 * @param  v: Value to send
 * @retval None
 */
void UART_SendUInt(uint32_t v)
{
    char buf[11];
    int i = 0;
    if (v == 0) 
    { 
        while (!(USART1->SR & USART_SR_TXE)) { } 
        USART1->DR = '0'; 
        return; 
    }
    while (v && i < (int)sizeof(buf)) 
    { 
        buf[i++] = '0' + (v % 10); 
        v /= 10; 
    }
    while (i--) 
    { 
        while (!(USART1->SR & USART_SR_TXE)) { } 
        USART1->DR = (uint8_t)buf[i]; 
    }
}

/**
 * @brief  Send carriage return + line feed
 * @retval None
 */
void UART_SendCRLF(void)
{
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = '\r';
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = '\n';
}
