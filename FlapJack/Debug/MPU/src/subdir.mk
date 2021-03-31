################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MPU/src/mpu.c 

OBJS += \
./MPU/src/mpu.o 

C_DEPS += \
./MPU/src/mpu.d 


# Each subdirectory must supply rules for building sources it contributes
MPU/src/mpu.o: ../MPU/src/mpu.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/core/inc" -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/platform/inc" -I"/home/martin/STM32CubeIDE/workspace_1.5.1/FlapJack/MPU/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MPU/src/mpu.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

