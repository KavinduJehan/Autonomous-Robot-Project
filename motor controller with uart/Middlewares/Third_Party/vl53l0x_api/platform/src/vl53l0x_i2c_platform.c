#include "vl53l0x_i2c_platform.h"
#include "tof_sensors.h"
#include "stm32f4xx_hal.h"

#define STATUS_OK   0
#define STATUS_FAIL 1

int32_t VL53L0X_comms_initialise(uint8_t comms_type, uint16_t comms_speed_khz)
{
    (void)comms_type;
    (void)comms_speed_khz;
    return STATUS_OK;
}

int32_t VL53L0X_comms_close(void)
{
    return STATUS_OK;
}

int32_t VL53L0X_cycle_power(void)
{
    return STATUS_OK;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    if ((pdata == NULL) || (count < 0)) {
        return STATUS_FAIL;
    }
    return (ToF_WriteMulti(address, index, pdata, (uint16_t)count) == HAL_OK) ? STATUS_OK : STATUS_FAIL;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    if ((pdata == NULL) || (count < 0)) {
        return STATUS_FAIL;
    }
    return (ToF_ReadMulti(address, index, pdata, (uint16_t)count) == HAL_OK) ? STATUS_OK : STATUS_FAIL;
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    return VL53L0X_write_multi(address, index, &data, 1);
}

int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    uint8_t buffer[BYTES_PER_WORD] = {
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFF)
    };
    return VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    uint8_t buffer[BYTES_PER_DWORD] = {
        (uint8_t)(data >> 24),
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFF)
    };
    return VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);
}

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    return VL53L0X_read_multi(address, index, pdata, 1);
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    uint8_t buffer[BYTES_PER_WORD];
    int32_t status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD);
    if ((status == STATUS_OK) && (pdata != NULL)) {
        *pdata = ((uint16_t)buffer[0] << 8) | buffer[1];
    }
    return status;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    uint8_t buffer[BYTES_PER_DWORD];
    int32_t status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD);
    if ((status == STATUS_OK) && (pdata != NULL)) {
        *pdata = ((uint32_t)buffer[0] << 24) |
                 ((uint32_t)buffer[1] << 16) |
                 ((uint32_t)buffer[2] << 8) |
                 (uint32_t)buffer[3];
    }
    return status;
}

static void busy_wait_ms(uint32_t wait_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < wait_ms) {
        __NOP();
    }
}

int32_t VL53L0X_platform_wait_us(int32_t wait_us)
{
    if (wait_us <= 0) {
        return STATUS_OK;
    }
    uint32_t wait_ms = (uint32_t)((wait_us + 999) / 1000);
    busy_wait_ms(wait_ms);
    return STATUS_OK;
}

int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
    if (wait_ms > 0) {
        HAL_Delay((uint32_t)wait_ms);
    }
    return STATUS_OK;
}

int32_t VL53L0X_set_gpio(uint8_t level)
{
    (void)level;
    return STATUS_OK;
}

int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
    if (plevel != NULL) {
        *plevel = 0;
    }
    return STATUS_OK;
}

int32_t VL53L0X_release_gpio(void)
{
    return STATUS_OK;
}

int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz)
{
    if (ptimer_freq_hz == NULL) {
        return STATUS_FAIL;
    }
    *ptimer_freq_hz = 1000;
    return STATUS_OK;
}

int32_t VL53L0X_get_timer_value(int32_t *ptimer_count)
{
    if (ptimer_count == NULL) {
        return STATUS_FAIL;
    }
    *ptimer_count = (int32_t)HAL_GetTick();
    return STATUS_OK;
}
