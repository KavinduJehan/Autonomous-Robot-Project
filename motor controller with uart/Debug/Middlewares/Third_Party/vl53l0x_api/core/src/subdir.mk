################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api.c \
../Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_calibration.c \
../Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_core.c \
../Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_ranging.c \
../Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_strings.c 

OBJS += \
./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api.o \
./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_calibration.o \
./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_core.o \
./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_ranging.o \
./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_strings.o 

C_DEPS += \
./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api.d \
./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_calibration.d \
./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_core.d \
./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_ranging.d \
./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_strings.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/vl53l0x_api/core/src/%.o Middlewares/Third_Party/vl53l0x_api/core/src/%.su Middlewares/Third_Party/vl53l0x_api/core/src/%.cyclo: ../Middlewares/Third_Party/vl53l0x_api/core/src/%.c Middlewares/Third_Party/vl53l0x_api/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"C:/E Dirve/Projects/Autonomous Car Prototype/motor controller with uart/Middlewares/Third_Party/vl53l0x_api/platform/inc" -I"C:/E Dirve/Projects/Autonomous Car Prototype/motor controller with uart/Middlewares/Third_Party/vl53l0x_api/core/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-vl53l0x_api-2f-core-2f-src

clean-Middlewares-2f-Third_Party-2f-vl53l0x_api-2f-core-2f-src:
	-$(RM) ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api.cyclo ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api.d ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api.o ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api.su ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_calibration.cyclo ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_calibration.d ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_calibration.o ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_calibration.su ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_core.cyclo ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_core.d ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_core.o ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_core.su ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_ranging.cyclo ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_ranging.d ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_ranging.o ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_ranging.su ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_strings.cyclo ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_strings.d ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_strings.o ./Middlewares/Third_Party/vl53l0x_api/core/src/vl53l0x_api_strings.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-vl53l0x_api-2f-core-2f-src

