/*
 * MOTOR CONTROLLER - STM32F401RCT6
 * Simple working UART + GPIO
 */

#include "main.h"
#include "stm32f4xx.h"

volatile uint8_t uart_command = 'S';
volatile uint8_t uart_data_received = 0;

#define MOTOR_FORWARD    0x05
#define MOTOR_BACKWARD   0x0A
#define MOTOR_LEFT       0x04
#define MOTOR_RIGHT      0x01
#define MOTOR_STOP       0x00

typedef enum {
    MOTOR_STATE_STOP = 0,
    MOTOR_STATE_FORWARD,
    MOTOR_STATE_BACKWARD,
    MOTOR_STATE_LEFT,
    MOTOR_STATE_RIGHT
} MotorState;

typedef struct {
    uint32_t sysclk_hz;
    uint32_t hclk_hz;
    uint32_t apb1_hz;
    uint32_t apb2_hz;
} ClockSnapshot;

static volatile MotorState motor_state = MOTOR_STATE_STOP;
static volatile uint8_t uart_ack_pending = 0;
static volatile uint8_t uart_ack_byte = 'K';
static volatile ClockSnapshot clock_snapshot = {0};

static void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    // ===== PA0-PA3: Motor outputs =====
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
    
    // ===== PA9-PA10: UART alternate function =====
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void Clock_DebugSnapshot(void) {
    SystemCoreClockUpdate();
    clock_snapshot.sysclk_hz = HAL_RCC_GetSysClockFreq();
    clock_snapshot.hclk_hz = HAL_RCC_GetHCLKFreq();
    clock_snapshot.apb1_hz = HAL_RCC_GetPCLK1Freq();
    clock_snapshot.apb2_hz = HAL_RCC_GetPCLK2Freq();
}

static void UART_SendByte(uint8_t byte) {
    while(!(USART1->SR & USART_SR_TXE));
    USART1->DR = byte;
}

void UART1_Init(void) {
    // Enable USART1 clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    
    // Baud rate: USARTDIV = fCLK / (16 * Baud) ≈ 8.68 -> mantissa 8, fraction 11 (0x8B = 139)
    USART1->BRR = 139;  // 115200 @ 16MHz (HSI)
    
    // Control registers
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    
    // Enable interrupt
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 0);
}

static void Motor_ApplyState(MotorState state) {
    uint8_t pattern = MOTOR_STOP;

    switch(state) {
        case MOTOR_STATE_FORWARD:  pattern = MOTOR_FORWARD;  break;
        case MOTOR_STATE_BACKWARD: pattern = MOTOR_BACKWARD; break;
        case MOTOR_STATE_LEFT:     pattern = MOTOR_LEFT;     break;
        case MOTOR_STATE_RIGHT:    pattern = MOTOR_RIGHT;    break;
        case MOTOR_STATE_STOP:
        default:                   pattern = MOTOR_STOP;     break;
    }

    GPIOA->ODR = (GPIOA->ODR & ~(0x0FUL)) | pattern;
}

void UART_ProcessCommand(void) {
    if (!uart_data_received) return;
    uart_data_received = 0;
    
    MotorState next_state = motor_state;
    uint8_t ack = 'K';

    switch(uart_command) {
        case 'F': case 'f': next_state = MOTOR_STATE_FORWARD;  break;
        case 'B': case 'b': next_state = MOTOR_STATE_BACKWARD; break;
        case 'L': case 'l': next_state = MOTOR_STATE_LEFT;     break;
        case 'R': case 'r': next_state = MOTOR_STATE_RIGHT;    break;
        case 'S': case 's': next_state = MOTOR_STATE_STOP;     break;
        default:
            ack = '?';
            break;
    }

    motor_state = next_state;
    Motor_ApplyState(motor_state);

    uart_ack_byte = ack;
    uart_ack_pending = 1;
}

int main(void) {
    HAL_Init();
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    
    // Verify SystemCoreClock
    Clock_DebugSnapshot();
    
    GPIO_Init();
    UART1_Init();
    
    // Boot delay
    for(volatile uint32_t i = 0; i < 5000000; i++);
    Motor_ApplyState(MOTOR_STATE_STOP);
    
    while(1) {
        UART_ProcessCommand();
        if (uart_ack_pending && (USART1->SR & USART_SR_TXE)) {
            UART_SendByte(uart_ack_byte);
            uart_ack_pending = 0;
        }
    }
}

void Error_Handler(void) { while(1); }
void SysTick_Handler(void) { HAL_IncTick(); }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { while(1); }
#endif

