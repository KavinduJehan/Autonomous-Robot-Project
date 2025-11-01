/**
  ******************************************************************************
  * @file           : ultrasonic.c
  * @brief          : Ultrasonic sensor module implementation
  * @description    : HC-SR04 distance measurement and collision detection
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ultrasonic.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint16_t ultrasonic_left_cm = 0;
volatile uint16_t ultrasonic_right_cm = 0;

/* Private function prototypes -----------------------------------------------*/
#if ULTRASONIC_ENABLED
static inline void DWT_Delay_Init(void);
static inline uint32_t micros(void);
static inline void delay_us(uint32_t us);
static uint16_t Ultrasonic_Measure_Pin(GPIO_TypeDef* trig_port, uint16_t trig_pin,
                                       GPIO_TypeDef* echo_port, uint16_t echo_pin);
#endif

/* Private functions ---------------------------------------------------------*/

#if ULTRASONIC_ENABLED

/**
 * @brief  Initialize DWT cycle counter for microsecond timing
 * @retval None
 */
static inline void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  Get microseconds since startup
 * @retval Microseconds
 */
static inline uint32_t micros(void)
{
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
}

/**
 * @brief  Microsecond delay
 * @param  us: Microseconds to delay
 * @retval None
 */
static inline void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}

/**
 * @brief  Measure distance from ultrasonic sensor
 * @param  trig_port: Trigger pin GPIO port
 * @param  trig_pin: Trigger pin number
 * @param  echo_port: Echo pin GPIO port
 * @param  echo_pin: Echo pin number
 * @retval Distance in centimeters (0-400), 0 on timeout
 */
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
        if ((DWT->CYCCNT - start_wait) > timeout_ticks) return 0;
    }
    tstart = DWT->CYCCNT;

    /* Wait for echo falling edge */
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET)
    {
        if ((DWT->CYCCNT - tstart) > timeout_ticks) return 0;
    }
    tend = DWT->CYCCNT;

    uint32_t pulse_ticks = (tend - tstart);
    uint32_t pulse_us = pulse_ticks / (SystemCoreClock / 1000000U);

    /* Convert to centimeters: distance (cm) = pulse_us / 58 */
    uint32_t dist_cm = pulse_us / 58U;
    if (dist_cm > 400) dist_cm = 400;
    return (uint16_t)dist_cm;
}

#endif /* ULTRASONIC_ENABLED */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Initialize GPIO pins for ultrasonic sensors
 * @retval None
 */
void Ultrasonic_GPIO_Init(void)
{
#if ULTRASONIC_ENABLED
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIOB clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

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
 * @brief  Initialize DWT cycle counter for microsecond timing
 * @retval None
 */
void Ultrasonic_Init(void)
{
#if ULTRASONIC_ENABLED
    DWT_Delay_Init();
    HAL_GPIO_WritePin(US_GPIO_PORT, US_TRIG_A_PIN | US_TRIG_B_PIN, GPIO_PIN_RESET);
#endif
}

/**
 * @brief  Measure distance from left sensor (Sensor A)
 * @retval Distance in centimeters (0-400), 0 on timeout
 */
uint16_t Ultrasonic_MeasureA(void)
{
#if ULTRASONIC_ENABLED
    return Ultrasonic_Measure_Pin(US_GPIO_PORT, US_TRIG_A_PIN, US_GPIO_PORT, US_ECHO_A_PIN);
#else
    return 0;
#endif
}

/**
 * @brief  Measure distance from right sensor (Sensor B)
 * @retval Distance in centimeters (0-400), 0 on timeout
 */
uint16_t Ultrasonic_MeasureB(void)
{
#if ULTRASONIC_ENABLED
    return Ultrasonic_Measure_Pin(US_GPIO_PORT, US_TRIG_B_PIN, US_GPIO_PORT, US_ECHO_B_PIN);
#else
    return 0;
#endif
}

/**
 * @brief  Check if either sensor detects collision threshold
 * @retval true if collision imminent, false otherwise
 */
bool Ultrasonic_CheckCollision(void)
{
#if ULTRASONIC_ENABLED
    uint16_t a = Ultrasonic_MeasureA();
    uint16_t b = Ultrasonic_MeasureB();
    return (a > 0 && a <= COLLISION_DISTANCE_STOP) || (b > 0 && b <= COLLISION_DISTANCE_STOP);
#else
    return false;
#endif
}
