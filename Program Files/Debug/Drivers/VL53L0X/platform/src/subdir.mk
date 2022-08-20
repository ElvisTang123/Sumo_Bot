################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/VL53L0X/platform/src/vl53l0x_i2c_comms.cpp \
../Drivers/VL53L0X/platform/src/vl53l0x_platform.cpp 

OBJS += \
./Drivers/VL53L0X/platform/src/vl53l0x_i2c_comms.o \
./Drivers/VL53L0X/platform/src/vl53l0x_platform.o 

CPP_DEPS += \
./Drivers/VL53L0X/platform/src/vl53l0x_i2c_comms.d \
./Drivers/VL53L0X/platform/src/vl53l0x_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L0X/platform/src/%.o Drivers/VL53L0X/platform/src/%.su: ../Drivers/VL53L0X/platform/src/%.cpp Drivers/VL53L0X/platform/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"H:/Sumo_Bot/Drivers/VL53L0X" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L0X-2f-platform-2f-src

clean-Drivers-2f-VL53L0X-2f-platform-2f-src:
	-$(RM) ./Drivers/VL53L0X/platform/src/vl53l0x_i2c_comms.d ./Drivers/VL53L0X/platform/src/vl53l0x_i2c_comms.o ./Drivers/VL53L0X/platform/src/vl53l0x_i2c_comms.su ./Drivers/VL53L0X/platform/src/vl53l0x_platform.d ./Drivers/VL53L0X/platform/src/vl53l0x_platform.o ./Drivers/VL53L0X/platform/src/vl53l0x_platform.su

.PHONY: clean-Drivers-2f-VL53L0X-2f-platform-2f-src

