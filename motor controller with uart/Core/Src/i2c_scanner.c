/**
 * @file i2c_scanner.c
 * @brief Simple I2C device scanner for debugging
 * 
 * Scans I2C bus and reports all responding addresses
 */

#include "i2c_scanner.h"
#include "uart_comm.h"
#include <stdio.h>

/**
 * @brief Scan I2C bus for responding devices
 * @param hi2c I2C handle
 */
void I2C_Scanner(I2C_HandleTypeDef *hi2c) {
    char msg[80];
    uint8_t count = 0;
    
    UART_SendString("\r\n=== I2C Bus Scanner ===\r\n");
    UART_SendString("Scanning I2C2 for devices...\r\n");
    
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        /* Try to read 1 byte from this address */
        uint8_t dummy = 0;
        HAL_StatusTypeDef status = HAL_I2C_Master_Receive(hi2c, (addr << 1), &dummy, 1, 10);
        
        if (status == HAL_OK) {
            snprintf(msg, sizeof(msg), "  Found device at 0x%02X\r\n", addr);
            UART_SendString(msg);
            count++;
        }
    }
    
    if (count == 0) {
        UART_SendString("No devices found on I2C bus!\r\n");
    } else {
        snprintf(msg, sizeof(msg), "Total devices found: %d\r\n", count);
        UART_SendString(msg);
    }
    
    UART_SendString("=== Scan Complete ===\r\n\r\n");
}
