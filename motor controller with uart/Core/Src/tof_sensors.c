#include "tof_sensors.h"
#include "vl53l0x_pololu.h"
#include "uart_comm.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

/* Sensor instances using Pololu driver */
static VL53L0X_Pololu_t tof_sensor1_pololu;  // Left 45° (address 0x30)
static VL53L0X_Pololu_t tof_sensor2_pololu;  // Right 45° (address 0x31)

/* Global distance readings */
volatile uint16_t tof_left45_mm = 0;
volatile uint16_t tof_right45_mm = 0;

/* Sensor status */
volatile bool tof_sensor1_ready = false;
volatile bool tof_sensor2_ready = false;
volatile bool tof_junction_detected = false;

/* Private junction info cache */
static ToF_JunctionInfo junction_info = {0};

/* Initialize GPIO pins for XSHUT control */
void ToF_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = TOF_XSHUT1_PIN | TOF_XSHUT2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TOF_XSHUT_PORT, &GPIO_InitStruct);
    
    /* Initially disable both sensors */
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT2_PIN, GPIO_PIN_RESET);
}

void ToF_EnableSensor1(void) {
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT1_PIN, GPIO_PIN_SET);
    osDelay(2);
}

void ToF_EnableSensor2(void) {
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT2_PIN, GPIO_PIN_SET);
    osDelay(2);
}

void ToF_DisableSensor1(void) {
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT1_PIN, GPIO_PIN_RESET);
}

void ToF_DisableSensor2(void) {
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT2_PIN, GPIO_PIN_RESET);
}

void ToF_DisableAll(void) {
    ToF_DisableSensor1();
    ToF_DisableSensor2();
}

void ToF_EnableAll(void) {
    ToF_EnableSensor1();
    ToF_EnableSensor2();
}

/**
 * @brief Initialize both sensors with unique addresses using Pololu driver
 */
bool ToF_InitSensors(I2C_HandleTypeDef *hi2c) {
    char msg[80];
    
    UART_SendString("\r\n=== ToF Sensor Initialization (Pololu Driver) ===\r\n");
    
    /* Step 1: Disable both sensors */
    ToF_DisableAll();
    osDelay(10);
    
    /* Step 2: Enable sensor 2 only, keep sensor 1 disabled */
    ToF_EnableSensor2();
    UART_SendString("S2: Enabled (0x29)\r\n");
    osDelay(10);  // Give sensor time to power on
    
    /* Initialize sensor 2 with default address */
    VL53L0X_Pololu_Init(&tof_sensor2_pololu, hi2c, 0x29);
    UART_SendString("S2: Init called\r\n");
    
    /* Run full initialization sequence */
    UART_SendString("S2: Running setup...\r\n");
    if (!VL53L0X_Pololu_Setup(&tof_sensor2_pololu, true)) {
        UART_SendString("ERROR: S2 setup failed!\r\n");
        return false;
    }
    UART_SendString("S2: Setup OK\r\n");
    
    /* Change sensor 2 address to 0x31 */
    VL53L0X_Pololu_SetAddress(&tof_sensor2_pololu, TOF_SENSOR2_ADDR);
    snprintf(msg, sizeof(msg), "S2: Address changed to 0x%02X\r\n", TOF_SENSOR2_ADDR);
    UART_SendString(msg);
    
    /* Small delay to stabilize */
    osDelay(5);
    
    /* Step 3: Enable sensor 1 */
    ToF_EnableSensor1();
    UART_SendString("S1: Enabled (0x29)\r\n");
    osDelay(10);  // Give sensor time to power on
    
    /* Initialize sensor 1 with default address */
    VL53L0X_Pololu_Init(&tof_sensor1_pololu, hi2c, 0x29);
    UART_SendString("S1: Init called\r\n");
    
    /* Run full initialization sequence */
    UART_SendString("S1: Running setup...\r\n");
    if (!VL53L0X_Pololu_Setup(&tof_sensor1_pololu, true)) {
        UART_SendString("ERROR: S1 setup failed!\r\n");
        return false;
    }
    UART_SendString("S1: Setup OK\r\n");
    
    /* Change sensor 1 address to 0x30 */
    VL53L0X_Pololu_SetAddress(&tof_sensor1_pololu, TOF_SENSOR1_ADDR);
    snprintf(msg, sizeof(msg), "S1: Address changed to 0x%02X\r\n", TOF_SENSOR1_ADDR);
    UART_SendString(msg);
    
    /* Mark sensors as ready */
    UART_SendString("=== ToF Init Complete! ===\r\n");
    UART_SendString("S1 (Left 45°):  0x30 @ PB3/PB10\r\n");
    UART_SendString("S2 (Right 45°): 0x31 @ PB3/PB10\r\n");
    UART_SendString("Single-shot ranging mode\r\n\r\n");
    
    tof_sensor1_ready = true;
    tof_sensor2_ready = true;
    
    return true;
}

void ToF_Init(void) {
    extern I2C_HandleTypeDef hi2c2;
    
    ToF_GPIO_Init();
    
    if (!ToF_InitSensors(&hi2c2)) {
        UART_SendString("FATAL: ToF initialization failed!\r\n");
        tof_sensor1_ready = false;
        tof_sensor2_ready = false;
    }
}

/**
 * @brief Read sensor 1 (single-shot)
 */
uint16_t ToF_ReadSensor1(void) {
    uint16_t range_mm = 0;
    static uint32_t s1_error_count = 0;
    char msg[80];
    
    if (!tof_sensor1_ready) {
        return 8191;
    }
    
    range_mm = VL53L0X_Pololu_ReadRangeSingleMillimeters(&tof_sensor1_pololu);
    
    if (range_mm < 8191) {
        tof_left45_mm = range_mm;
        s1_error_count = 0;
        return range_mm;
    }
    
    s1_error_count++;
    if ((s1_error_count % 50) == 0) {
        snprintf(msg, sizeof(msg), "S1: read out of range (count=%lu)\r\n", s1_error_count);
        UART_SendString(msg);
    }
    
    return 8191;
}

/**
 * @brief Read sensor 2 (single-shot)
 */
uint16_t ToF_ReadSensor2(void) {
    uint16_t range_mm = 0;
    static uint32_t s2_error_count = 0;
    char msg[80];
    
    if (!tof_sensor2_ready) {
        return 8191;
    }
    
    range_mm = VL53L0X_Pololu_ReadRangeSingleMillimeters(&tof_sensor2_pololu);
    
    if (range_mm < 8191) {
        tof_right45_mm = range_mm;
        s2_error_count = 0;
        return range_mm;
    }
    
    s2_error_count++;
    if ((s2_error_count % 50) == 0) {
        snprintf(msg, sizeof(msg), "S2: read out of range (count=%lu)\r\n", s2_error_count);
        UART_SendString(msg);
    }
    
    return 8191;
}

bool ToF_ReadSensor1_NonBlocking(uint16_t *range_mm) {
    if (!tof_sensor1_ready) {
        return false;
    }
    
    *range_mm = VL53L0X_Pololu_ReadRangeSingleMillimeters(&tof_sensor1_pololu);
    
    if (*range_mm < 8191) {
        tof_left45_mm = *range_mm;
        return true;
    }
    
    return false;
}

bool ToF_ReadSensor2_NonBlocking(uint16_t *range_mm) {
    if (!tof_sensor2_ready) {
        return false;
    }
    
    *range_mm = VL53L0X_Pololu_ReadRangeSingleMillimeters(&tof_sensor2_pololu);
    
    if (*range_mm < 8191) {
        tof_right45_mm = *range_mm;
        return true;
    }
    
    return false;
}

bool ToF_ReadBothSensors(uint16_t *left_mm, uint16_t *right_mm) {
    if (!tof_sensor1_ready || !tof_sensor2_ready) {
        return false;
    }
    
    *left_mm = ToF_ReadSensor1();
    *right_mm = ToF_ReadSensor2();
    
    return true;
}

void ToF_GetDistances(uint16_t *left_mm, uint16_t *right_mm) {
    *left_mm = tof_left45_mm;
    *right_mm = tof_right45_mm;
}

bool ToF_SensorsReady(void) {
    return (tof_sensor1_ready && tof_sensor2_ready);
}

void ToF_UpdateJunctionDetection(void) {
    junction_info.left_mm = tof_left45_mm;
    junction_info.right_mm = tof_right45_mm;
    
    junction_info.left_open = (tof_left45_mm > TOF_JUNCTION_THRESHOLD);
    junction_info.right_open = (tof_right45_mm > TOF_JUNCTION_THRESHOLD);
    
    junction_info.is_junction = (junction_info.left_open && junction_info.right_open);
    junction_info.is_left_turn = (junction_info.left_open && !junction_info.right_open);
    junction_info.is_right_turn = (!junction_info.left_open && junction_info.right_open);
    
    tof_junction_detected = junction_info.is_junction;
}

bool ToF_DetectJunction(void) {
    return tof_junction_detected;
}

ToF_JunctionInfo ToF_GetJunctionInfo(void) {
    return junction_info;
}

const char* ToF_StatusToString(int status) {
    switch (status) {
        case 0: return "OK";
        case -1: return "TIMEOUT";
        case -2: return "OUT_OF_RANGE";
        default: return "UNKNOWN";
    }
}

/**
 * @brief FreeRTOS Task for ToF sensor reading
 */
void ToF_Task(void const * argument) {
    char msg[128];
    uint16_t left_mm = 0, right_mm = 0;
    uint32_t read_count = 0;
    bool init_success = false;
    
    osDelay(500);
    
    UART_SendString("\r\n=== ToF Task Started ===\r\n");
    
    extern I2C_HandleTypeDef hi2c2;
    init_success = ToF_InitSensors(&hi2c2);
    
    if (!init_success) {
        UART_SendString("ERROR: ToF init failed, task suspended\r\n");
        vTaskSuspend(NULL);
    }
    
    UART_SendString("ToF Task: Running at 50Hz\r\n\r\n");
    
    uint32_t error_count = 0;
    for(;;) {
        /* Read sensor 1 */
        left_mm = ToF_ReadSensor1();
        if (left_mm < 8191) {
            tof_left45_mm = left_mm;
        } else {
            error_count++;
        }
        
        osDelay(10);
        
        /* Read sensor 2 */
        right_mm = ToF_ReadSensor2();
        if (right_mm < 8191) {
            tof_right45_mm = right_mm;
        } else {
            error_count++;
        }
        
        /* Update junction detection */
        ToF_UpdateJunctionDetection();
        
        /* Debug output every 1 second */
        read_count++;
        if ((read_count % 50) == 0) {
            ToF_JunctionInfo info = ToF_GetJunctionInfo();
            
            snprintf(msg, sizeof(msg), 
                     "ToF: L=%4dmm R=%4dmm | ",
                     info.left_mm, info.right_mm);
            UART_SendString(msg);
            
            if (info.is_junction) {
                UART_SendString("JUNCTION (both open)\r\n");
            } else if (info.is_left_turn) {
                UART_SendString("LEFT TURN available\r\n");
            } else if (info.is_right_turn) {
                UART_SendString("RIGHT TURN available\r\n");
            } else {
                UART_SendString("Corridor\r\n");
            }
        }
        
        osDelay(TOF_MEASURE_INTERVAL_MS);
    }
}
