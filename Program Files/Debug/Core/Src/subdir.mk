################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/HCSR04.c \
../Core/Src/Line_Detect.c \
../Core/Src/MPU6050.c \
../Core/Src/Motor_Driver.c \
../Core/Src/Remote_Control.c \
../Core/Src/USART.c \
../Core/Src/lidar.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

C_DEPS += \
./Core/Src/HCSR04.d \
./Core/Src/Line_Detect.d \
./Core/Src/MPU6050.d \
./Core/Src/Motor_Driver.d \
./Core/Src/Remote_Control.d \
./Core/Src/USART.d \
./Core/Src/lidar.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/HCSR04.o \
./Core/Src/Line_Detect.o \
./Core/Src/MPU6050.o \
./Core/Src/Motor_Driver.o \
./Core/Src/Remote_Control.o \
./Core/Src/USART.o \
./Core/Src/lidar.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"H:/Sumo_Bot/Drivers/VL53L0X" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/HCSR04.d ./Core/Src/HCSR04.o ./Core/Src/HCSR04.su ./Core/Src/Line_Detect.d ./Core/Src/Line_Detect.o ./Core/Src/Line_Detect.su ./Core/Src/MPU6050.d ./Core/Src/MPU6050.o ./Core/Src/MPU6050.su ./Core/Src/Motor_Driver.d ./Core/Src/Motor_Driver.o ./Core/Src/Motor_Driver.su ./Core/Src/Remote_Control.d ./Core/Src/Remote_Control.o ./Core/Src/Remote_Control.su ./Core/Src/USART.d ./Core/Src/USART.o ./Core/Src/USART.su ./Core/Src/lidar.d ./Core/Src/lidar.o ./Core/Src/lidar.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

