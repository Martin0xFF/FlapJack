################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L0X_1.0.2/Api/core/src/vl53l0x_api.c \
../VL53L0X_1.0.2/Api/core/src/vl53l0x_api_calibration.c \
../VL53L0X_1.0.2/Api/core/src/vl53l0x_api_core.c \
../VL53L0X_1.0.2/Api/core/src/vl53l0x_api_ranging.c \
../VL53L0X_1.0.2/Api/core/src/vl53l0x_api_strings.c 

OBJS += \
./VL53L0X_1.0.2/Api/core/src/vl53l0x_api.o \
./VL53L0X_1.0.2/Api/core/src/vl53l0x_api_calibration.o \
./VL53L0X_1.0.2/Api/core/src/vl53l0x_api_core.o \
./VL53L0X_1.0.2/Api/core/src/vl53l0x_api_ranging.o \
./VL53L0X_1.0.2/Api/core/src/vl53l0x_api_strings.o 

C_DEPS += \
./VL53L0X_1.0.2/Api/core/src/vl53l0x_api.d \
./VL53L0X_1.0.2/Api/core/src/vl53l0x_api_calibration.d \
./VL53L0X_1.0.2/Api/core/src/vl53l0x_api_core.d \
./VL53L0X_1.0.2/Api/core/src/vl53l0x_api_ranging.d \
./VL53L0X_1.0.2/Api/core/src/vl53l0x_api_strings.d 


# Each subdirectory must supply rules for building sources it contributes
VL53L0X_1.0.2/Api/core/src/vl53l0x_api.o: ../VL53L0X_1.0.2/Api/core/src/vl53l0x_api.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/core/inc" -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/platform/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"VL53L0X_1.0.2/Api/core/src/vl53l0x_api.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
VL53L0X_1.0.2/Api/core/src/vl53l0x_api_calibration.o: ../VL53L0X_1.0.2/Api/core/src/vl53l0x_api_calibration.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/core/inc" -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/platform/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"VL53L0X_1.0.2/Api/core/src/vl53l0x_api_calibration.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
VL53L0X_1.0.2/Api/core/src/vl53l0x_api_core.o: ../VL53L0X_1.0.2/Api/core/src/vl53l0x_api_core.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/core/inc" -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/platform/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"VL53L0X_1.0.2/Api/core/src/vl53l0x_api_core.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
VL53L0X_1.0.2/Api/core/src/vl53l0x_api_ranging.o: ../VL53L0X_1.0.2/Api/core/src/vl53l0x_api_ranging.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/core/inc" -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/platform/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"VL53L0X_1.0.2/Api/core/src/vl53l0x_api_ranging.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
VL53L0X_1.0.2/Api/core/src/vl53l0x_api_strings.o: ../VL53L0X_1.0.2/Api/core/src/vl53l0x_api_strings.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/core/inc" -I"/home/martin/STM32CubeIDE/workspace_1.5.1/Tof/VL53L0X_1.0.2/Api/platform/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"VL53L0X_1.0.2/Api/core/src/vl53l0x_api_strings.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

