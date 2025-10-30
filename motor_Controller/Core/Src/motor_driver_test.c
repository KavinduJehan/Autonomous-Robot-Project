#include "motor_driver_test.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include <stddef.h>

#define MOTOR_FORWARD_PATTERN   0x05u
#define MOTOR_BACKWARD_PATTERN  0x0Au
#define MOTOR_LEFT_PATTERN      0x04u
#define MOTOR_RIGHT_PATTERN     0x01u
#define MOTOR_STOP_PATTERN      0x00u

static void MotorDriver_ConfigPins(void);
static void MotorDriver_WritePattern(uint8_t pattern);
static void MotorDriver_Delay(volatile uint32_t cycles);

void MotorDriver_TestInitialize(void)
{
    HAL_Init();
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {
    }

    MotorDriver_ConfigPins();
    MotorDriver_WritePattern(MOTOR_STOP_PATTERN);
}

void MotorDriver_ApplyState(MotorTestState state)
{
    uint8_t pattern = MOTOR_STOP_PATTERN;

    switch (state) {
        case MOTOR_TEST_FORWARD:
            pattern = MOTOR_FORWARD_PATTERN;
            break;
        case MOTOR_TEST_BACKWARD:
            pattern = MOTOR_BACKWARD_PATTERN;
            break;
        case MOTOR_TEST_LEFT:
            pattern = MOTOR_LEFT_PATTERN;
            break;
        case MOTOR_TEST_RIGHT:
            pattern = MOTOR_RIGHT_PATTERN;
            break;
        case MOTOR_TEST_STOP:
        default:
            pattern = MOTOR_STOP_PATTERN;
            break;
    }

    MotorDriver_WritePattern(pattern);
}

void MotorDriver_TestRunSequence(void)
{
    static const MotorTestState sequence[] = {
        MOTOR_TEST_FORWARD,
        MOTOR_TEST_BACKWARD,
        MOTOR_TEST_LEFT,
        MOTOR_TEST_RIGHT,
        MOTOR_TEST_STOP
    };

    for (size_t i = 0U; i < sizeof(sequence) / sizeof(sequence[0]); ++i) {
        MotorDriver_ApplyState(sequence[i]);
        MotorDriver_Delay(2000000U);
    }
}

static void MotorDriver_ConfigPins(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void MotorDriver_WritePattern(uint8_t pattern)
{
    GPIOA->ODR = (GPIOA->ODR & ~(0x0FU)) | (pattern & 0x0FU);
}

static void MotorDriver_Delay(volatile uint32_t cycles)
{
    while (cycles-- != 0U) {
        __NOP();
    }
}

#ifdef MOTOR_DRIVER_TEST_MAIN
int main(void)
{
    MotorDriver_TestInitialize();

    while (1) {
        MotorDriver_TestRunSequence();
    }
}
#endif
