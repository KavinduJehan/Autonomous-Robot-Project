/**
  ******************************************************************************
  * @file           : wall_avoidance.c
  * @brief          : Wall avoidance module implementation
  * @description    : PID-based wall following and collision avoidance
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "wall_avoidance.h"
#include "ultrasonic.h"
#include "motor_control.h"
#include "led_indicators.h"
#include "uart_comm.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  FreeRTOS task for ultrasonic sensing and wall avoidance
 * @param  argument: Not used
 * @retval None
 */
void WallAvoidance_Task(void const * argument)
{
#if ULTRASONIC_ENABLED
    /* PID state for center-seeking control */
    static float pid_i = 0.0f;
    static float last_err = 0.0f;
    const float dt = (float)ULTRASONIC_MEASURE_INTERVAL_MS / 1000.0f;
    /* Debug throttling */
    static uint32_t dbg_counter = 0;

#if DEBUG_BOOT_BANNER
    UART_SendString("Task: ultrasonic started\r\n");
#endif

    for(;;)
    {
        /* Measure left (A) and right (B) distances */
        uint16_t left = Ultrasonic_MeasureA();
        osDelay(5);
        uint16_t right = Ultrasonic_MeasureB();
        ultrasonic_left_cm = left;
        ultrasonic_right_cm = right;

        /* Drive debug LEDs */
        GPIO_PinState left_led = (left > 0 && left <= COLLISION_DISTANCE_SLOW) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        GPIO_PinState right_led = (right > 0 && right <= COLLISION_DISTANCE_SLOW) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        LED_SetWallIndicators(left_led, right_led);

#if ULTRASONIC_DEBUG
        /* Every ~10 cycles (~500ms), print diagnostics */
        if ((dbg_counter++ % 10U) == 0U)
        {
            UART_SendString("US L="); 
            UART_SendUInt(left);
            UART_SendString("cm R="); 
            UART_SendUInt(right);
            UART_SendString("cm LEDL="); 
            UART_SendUInt((uint32_t)(left_led == GPIO_PIN_SET));
            UART_SendString(" LEDR="); 
            UART_SendUInt((uint32_t)(right_led == GPIO_PIN_SET));
            UART_SendString(" CMD=");
            while (!(USART1->SR & USART_SR_TXE)) { }
            USART1->DR = (uint8_t)last_movement_cmd;
            UART_SendCRLF();
        }
#endif

        /* Emergency stop if any side is dangerously close */
        if ((left > 0 && left <= COLLISION_DISTANCE_STOP) ||
            (right > 0 && right <= COLLISION_DISTANCE_STOP))
        {
            Motor_Stop();
            last_movement_cmd = CMD_STOP;
            motors_moving = false;
            pid_i = 0.0f;
            last_err = 0.0f;
            osDelay(ULTRASONIC_MEASURE_INTERVAL_MS);
            continue;
        }

        /* Center-seeking steering only when moving forward */
        if (last_movement_cmd == CMD_FORWARD && motors_moving)
        {
            int base = (int)current_speed;
            int left_cmd = base;
            int right_cmd = base;

#if CENTERING_PID_ENABLED
            bool left_ok = (left > 0);
            bool right_ok = (right > 0);

            if (left_ok && right_ok)
            {
                /* error positive => closer to left => steer right */
                float err = (float)left - (float)right;

                /* deadband to avoid dithering */
                if (err > -CENTER_DEADBAND_CM && err < CENTER_DEADBAND_CM)
                    err = 0.0f;

                /* PID terms */
                float d = (err - last_err) / dt;
                pid_i += err * dt;

                /* anti-windup */
                if (CENTER_PID_KI > 0.0f)
                {
                    float i_max = CENTER_CORR_MAX / CENTER_PID_KI;
                    if (pid_i > i_max) pid_i = i_max;
                    if (pid_i < -i_max) pid_i = -i_max;
                }
                else
                {
                    pid_i = 0.0f;
                }

                float corr = CENTER_PID_KP * err + CENTER_PID_KI * pid_i + CENTER_PID_KD * d;

                /* clamp correction */
                if (corr > CENTER_CORR_MAX) corr = CENTER_CORR_MAX;
                if (corr < -CENTER_CORR_MAX) corr = -CENTER_CORR_MAX;

                /* symmetric differential application */
                left_cmd  = (int)((float)base - corr);
                right_cmd = (int)((float)base + corr);

                last_err = err;
            }
            else
#endif /* CENTERING_PID_ENABLED */
            {
                /* Fallback: single-sensor proportional nudge away */
                if (left > 0 && left < COLLISION_DISTANCE_SLOW)
                {
                    int delta_cm = (int)COLLISION_DISTANCE_SLOW - (int)left;
                    int corr = delta_cm * WALL_CORR_GAIN_PCT_PER_CM;
                    left_cmd -= corr;
                }

                if (right > 0 && right < COLLISION_DISTANCE_SLOW)
                {
                    int delta_cm = (int)COLLISION_DISTANCE_SLOW - (int)right;
                    int corr = delta_cm * WALL_CORR_GAIN_PCT_PER_CM;
                    right_cmd -= corr;
                }
            }

            /* Clamp speeds */
            if (left_cmd < 0) left_cmd = 0;
            if (left_cmd > 100) left_cmd = 100;
            if (right_cmd < 0) right_cmd = 0;
            if (right_cmd > 100) right_cmd = 100;

            /* Only update if any correction applied */
            if (left_cmd != base || right_cmd != base)
            {
                Motor_ForwardDifferential((uint8_t)left_cmd, (uint8_t)right_cmd);
            }
        }
        else
        {
            /* Not actively centering: reset PID state */
            pid_i = 0.0f;
            last_err = 0.0f;
        }

        osDelay(ULTRASONIC_MEASURE_INTERVAL_MS);
    }
#else
    /* Ultrasonic disabled, task does nothing */
    for(;;)
    {
        osDelay(1000);
    }
#endif /* ULTRASONIC_ENABLED */
}
