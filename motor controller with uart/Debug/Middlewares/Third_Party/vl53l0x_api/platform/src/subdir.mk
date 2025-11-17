################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_platform.c \
../Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_win_serial_comms.c \
../Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform.c \
../Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform_log.c 

OBJS += \
./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_platform.o \
./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_win_serial_comms.o \
./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform.o \
./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform_log.o 

C_DEPS += \
./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_platform.d \
./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_win_serial_comms.d \
./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform.d \
./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/vl53l0x_api/platform/src/%.o Middlewares/Third_Party/vl53l0x_api/platform/src/%.su Middlewares/Third_Party/vl53l0x_api/platform/src/%.cyclo: ../Middlewares/Third_Party/vl53l0x_api/platform/src/%.c Middlewares/Third_Party/vl53l0x_api/platform/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"C:/E Dirve/Projects/Autonomous Car Prototype/motor controller with uart/Middlewares/Third_Party/vl53l0x_api/platform/inc" -I"C:/E Dirve/Projects/Autonomous Car Prototype/motor controller with uart/Middlewares/Third_Party/vl53l0x_api/core/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-vl53l0x_api-2f-platform-2f-src

clean-Middlewares-2f-Third_Party-2f-vl53l0x_api-2f-platform-2f-src:
	-$(RM) ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_platform.cyclo ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_platform.d ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_platform.o ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_platform.su ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_win_serial_comms.cyclo ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_win_serial_comms.d ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_win_serial_comms.o ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_i2c_win_serial_comms.su ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform.cyclo ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform.d ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform.o ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform.su ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform_log.cyclo ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform_log.d ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform_log.o ./Middlewares/Third_Party/vl53l0x_api/platform/src/vl53l0x_platform_log.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-vl53l0x_api-2f-platform-2f-src

