/**
  ******************************************************************************
  * @file           : command_processor.c
  * @brief          : Command processing module implementation
  * @description    : UART command processing and safety checks
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "command_processor.h"
#include "motor_control.h"
#include "ultrasonic.h"
#include "uart_comm.h"
#include "main.h"
#include "cmsis_os.h"

/* FreeRTOS task: dequeue UART commands and process them */
void Command_Task(void const * argument)
{
    (void) argument;
    for(;;)
    {
        osEvent evt = osMessageGet(uartCmdQueueHandle, osWaitForever);
        if (evt.status == osEventMessage)
        {
            uint32_t val = evt.value.v;
            uint8_t cmd = (uint8_t)(val & 0xFF);
            /* Normalize lowercase to uppercase for robustness */
            if (cmd >= 'a' && cmd <= 'z') cmd -= 32;
            Command_Process(cmd);
        }
    }
}

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint32_t last_command_time = 0;
/* Short debounce window to ignore rapid repeated turn commands that may be
 * queued by the UART input (e.g., host heartbeat re-sending the last command).
 * After performing a spot-turn we set this to HAL_GetTick() + debounce_ms.
 */
static volatile uint32_t ignore_turns_until = 0;

/* Junction coordination flags (used to handshake with Pi) */
volatile bool junction_ack_received = false;      /* Set by Command_Process when 'K' received */
volatile bool junction_mode_active = false;       /* Set by ToF task while handling a junction */
volatile bool junction_direction_received = false;/* Set by Command_Process when L/T received in junction mode */
volatile uint8_t junction_direction_cmd = 0;     /* Stores the direction command received from Pi (e.g., CMD_LEFT/CMD_RIGHT) */

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Process received UART command
 * @param  cmd: Command character received via UART
 * @retval None
 */
void Command_Process(uint8_t cmd)
{
    switch(cmd)
    {
        /* Global ACK for junction: single-byte 'K' acknowledges the notification */
        case 'K':
            junction_ack_received = true;
            /* keep last_command_time updated so safety doesn't trigger */
            last_command_time = HAL_GetTick();
            break;

        case CMD_FORWARD:
            Motor_Forward(current_speed);
            last_movement_cmd = CMD_FORWARD;
            break;
            
        case CMD_REVERSE:
            Motor_Reverse(current_speed);
            last_movement_cmd = CMD_REVERSE;
            break;
            
        case CMD_LEFT:
        case CMD_LEFT_ALT:
            /* If we're in junction mode and waiting for remote direction, accept
             * the received direction but don't immediately execute it here. The
             * ToF/Junction task will perform the turn at the appropriate time.
             */
            if (junction_mode_active)
            {
                junction_direction_received = true;
                junction_direction_cmd = CMD_LEFT;
                break;
            }
            /* Debounce: ignore if we're inside the ignore window */
            if (HAL_GetTick() < ignore_turns_until)
            {
                /* Drop this command - it was likely queued before the previous turn completed */
                break;
            }
            /* If we're currently moving, gently stop first */
            if (motors_moving && last_movement_cmd != CMD_STOP)
            {
                Motor_Stop_Smooth();
            }
            /* Compute scaled duration based on current_speed (inverse scale):
             * higher speed -> shorter turn time, lower speed -> longer time.
             */
            {
                uint32_t turn_ms = TURN_90_MS;
                if (current_speed > 0)
                {
                    turn_ms = (TURN_90_MS * 100U) / (uint32_t)current_speed;
                }
                if (turn_ms < TURN_90_MS_MIN) turn_ms = TURN_90_MS_MIN;
                if (turn_ms > TURN_90_MS_MAX) turn_ms = TURN_90_MS_MAX;

                /* Perform a smooth spot turn (blocks during turn) */
                Motor_SpotTurnSmooth(current_speed, turn_ms, true);
            }
            /* Ensure state shows stopped after the turn */
            motors_moving = false;
            last_movement_cmd = CMD_STOP;
            /* Prevent immediate repeated turns from queued messages (debounce) */
            ignore_turns_until = HAL_GetTick() + 500;
            break;
            
        case CMD_RIGHT:
        case CMD_RIGHT_ALT:
            if (junction_mode_active)
            {
                junction_direction_received = true;
                junction_direction_cmd = CMD_RIGHT;
                break;
            }
            /* Debounce: ignore if we're inside the ignore window */
            if (HAL_GetTick() < ignore_turns_until)
            {
                break;
            }
            if (motors_moving && last_movement_cmd != CMD_STOP)
            {
                Motor_Stop_Smooth();
            }
            {
                uint32_t turn_ms = TURN_90_MS;
                if (current_speed > 0)
                {
                    turn_ms = (TURN_90_MS * 100U) / (uint32_t)current_speed;
                }
                if (turn_ms < TURN_90_MS_MIN) turn_ms = TURN_90_MS_MIN;
                if (turn_ms > TURN_90_MS_MAX) turn_ms = TURN_90_MS_MAX;

                Motor_SpotTurnSmooth(current_speed, turn_ms, false);
            }
            motors_moving = false;
            last_movement_cmd = CMD_STOP;
            /* Debounce further turn commands for a short window */
            ignore_turns_until = HAL_GetTick() + 500;
            break;
            
        case CMD_STOP:
            if (accel_enabled)
                Motor_Stop_Smooth();
            else
                Motor_Stop();
            last_movement_cmd = CMD_STOP;
            break;
        
        case 'E': // Emergency stop alias from UI
            Motor_Stop();
            last_movement_cmd = CMD_STOP;
            break;
            
        case CMD_SPEED_SLOW:
            current_speed = SPEED_SLOW;
            if (motors_moving && last_movement_cmd != CMD_STOP)
            {
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
            if (motors_moving && last_movement_cmd != CMD_STOP)
            {
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
            if (motors_moving && last_movement_cmd != CMD_STOP)
            {
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
        case CMD_ACCEL_DISABLE_ALT:
            accel_enabled = false;
            break;
        
        case CMD_SELF_TEST:
        {
            bool prev_accel = accel_enabled;
            accel_enabled = false;
            Motor_Forward(40); osDelay(1000);
            Motor_Forward(70); osDelay(1000);
            Motor_Forward(100); osDelay(1000);
            Motor_Stop();
            accel_enabled = prev_accel;
            break;
        }

        case CMD_ULTRASONIC_PING:
        {
            /* Immediate ACK so host can verify TX path without waiting */
            UART_SendString("ACK U\r\n");
#if ULTRASONIC_ENABLED
            uint16_t a = Ultrasonic_MeasureA();
            osDelay(5);
            uint16_t b = Ultrasonic_MeasureB();
            osDelay(5);
            uint16_t c = Ultrasonic_MeasureC();

            /* Print all three sensors with units so host parsers can read them */
            UART_SendString("US A=");
            UART_SendUInt(a);
            UART_SendString("cm B=");
            UART_SendUInt(b);
            UART_SendString("cm C=");
            UART_SendUInt(c);
            UART_SendString("cm\r\n");
#else
            UART_SendString("US disabled\r\n");
#endif
            break;
        }
        
        case CMD_TOF_PING:
        {
            /* Immediate ACK for host */
            UART_SendString("ACK O\r\n");
#if defined(TOF_MEASURE_INTERVAL_MS)
            /* Read both ToF sensors single-shot and report in mm */
            uint16_t left = ToF_ReadSensor1();
            osDelay(5);
            uint16_t right = ToF_ReadSensor2();

            /* Print in a concise, parseable format */
            UART_SendString("ToF L=");
            UART_SendUInt(left);
            UART_SendString("mm R=");
            UART_SendUInt(right);
            UART_SendString("mm\r\n");
#else
            UART_SendString("ToF disabled\r\n");
#endif
            break;
        }

        case 'Q':
        {
            /* Diagnostic: report ToF sensor readiness and last-known distances */
            uint16_t l = 0, r = 0;
            bool ready = ToF_SensorsReady();
            ToF_GetDistances(&l, &r);
            UART_SendString("TOFSTAT ");
            UART_SendString(ready ? "ready=1 " : "ready=0 ");
            UART_SendString("L="); UART_SendUInt(l); UART_SendString("mm ");
            UART_SendString("R="); UART_SendUInt(r); UART_SendString("mm\r\n");
            break;
        }
            
        default:
            Motor_Stop();
            last_movement_cmd = CMD_STOP;
            break;
    }
    
    last_command_time = HAL_GetTick();
}

/**
 * @brief  Safety check - emergency stop if no command received
 * @retval None
 */
void Command_SafetyCheck(void)
{
    if ((HAL_GetTick() - last_command_time) > SAFETY_TIMEOUT_MS)
    {
        Motor_Stop();
        motors_moving = false;
        last_movement_cmd = CMD_STOP;
    }
}
/* Command_TurnCheck removed: spot turns are executed synchronously by
 * Command_Process using Motor_SpotTurnSmooth(), so no background timer is
 * required.
 */
