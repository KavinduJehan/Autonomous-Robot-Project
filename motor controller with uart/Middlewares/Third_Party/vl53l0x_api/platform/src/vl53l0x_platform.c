/**
 ******************************************************************************
 * @file    vl53l0x_platform.c
 * @brief   Platform glue between ST's VL53L0X API and the custom SW I2C layer.
 ******************************************************************************
 */

#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include "tof_sensors.h"

#define MAX_SW_I2C_LEN  UINT16_MAX

static VL53L0X_Error ToF_MapHalStatus(HAL_StatusTypeDef status)
{
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (count > MAX_SW_I2C_LEN) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
    return ToF_MapHalStatus(ToF_WriteMulti(Dev->I2cDevAddr, index, pdata, (uint16_t)count));
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (count > MAX_SW_I2C_LEN) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
    return ToF_MapHalStatus(ToF_ReadMulti(Dev->I2cDevAddr, index, pdata, (uint16_t)count));
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return ToF_MapHalStatus(ToF_WriteReg(Dev->I2cDevAddr, index, data));
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buffer[2] = {
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFF)
    };
    return VL53L0X_WriteMulti(Dev, index, buffer, sizeof(buffer));
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buffer[4] = {
        (uint8_t)((data >> 24) & 0xFF),
        (uint8_t)((data >> 16) & 0xFF),
        (uint8_t)((data >> 8) & 0xFF),
        (uint8_t)(data & 0xFF)
    };
    return VL53L0X_WriteMulti(Dev, index, buffer, sizeof(buffer));
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    uint8_t data = 0;
    VL53L0X_Error status = VL53L0X_RdByte(Dev, index, &data);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    data = (data & AndData) | OrData;
    return VL53L0X_WrByte(Dev, index, data);
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return ToF_MapHalStatus(ToF_ReadReg(Dev->I2cDevAddr, index, data));
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    uint8_t buffer[2] = {0};
    VL53L0X_Error status = VL53L0X_ReadMulti(Dev, index, buffer, sizeof(buffer));
    if (status == VL53L0X_ERROR_NONE) {
        *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    }
    return status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    uint8_t buffer[4] = {0};
    VL53L0X_Error status = VL53L0X_ReadMulti(Dev, index, buffer, sizeof(buffer));
    if (status == VL53L0X_ERROR_NONE) {
        *data = ((uint32_t)buffer[0] << 24) |
                ((uint32_t)buffer[1] << 16) |
                ((uint32_t)buffer[2] << 8) |
                (uint32_t)buffer[3];
    }
    return status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    (void)Dev;
    HAL_Delay(1);
    return VL53L0X_ERROR_NONE;
}

