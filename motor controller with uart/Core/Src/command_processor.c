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
#include "tof_sensors.h"
#include "main.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint32_t last_command_time = 0;

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
            UART_SendString("US L="); 
            UART_SendUInt(a); 
            UART_SendString("cm R="); 
            UART_SendUInt(b); 
            UART_SendString("cm F="); 
            UART_SendUInt(c); 
            UART_SendCRLF();
#else
            UART_SendString("US disabled\r\n");
#endif
            break;
        }

        case CMD_FRONT_DISTANCE:
        {
#if ULTRASONIC_ENABLED
            uint16_t front = Ultrasonic_MeasureC();
            UART_SendString("FD=");
            UART_SendUInt(front);
            UART_SendCRLF();
#else
            UART_SendString("FD=0\r\n");
#endif
            break;
        }

        case CMD_STATUS_REPORT:
        {
#if ULTRASONIC_ENABLED
            uint16_t left = Ultrasonic_MeasureA();
            osDelay(5);
            uint16_t right = Ultrasonic_MeasureB();
            osDelay(5);
            uint16_t front = Ultrasonic_MeasureC();
            
            UART_SendString("STATUS L=");
            UART_SendUInt(left);
            UART_SendString(" R=");
            UART_SendUInt(right);
            UART_SendString(" F=");
            UART_SendUInt(front);
            UART_SendString(" SPD=");
            UART_SendUInt(current_speed);
            UART_SendString(" MOV=");
            if (motors_moving) {
                UART_SendString("Y");
            } else {
                UART_SendString("N");
            }
            UART_SendCRLF();
#else
            UART_SendString("STATUS SENSORS=OFF\r\n");
#endif
            break;
        }

        case CMD_TOF_DISTANCES:
        case CMD_TOF_JUNCTIONS:
        case CMD_TOF_OBSTACLES:
        case 'X':  // ToF hardware test
        case 'Y':  // Toggle slow I2C mode
        case 'N':  // Reinitialize ToF sensors
        {
            /* Delegate ToF commands to ToF module */
            ToF_ProcessCommand(cmd);
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
