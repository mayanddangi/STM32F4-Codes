################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ILI9341/ILI9341_Driver.c 

OBJS += \
./ILI9341/ILI9341_Driver.o 

C_DEPS += \
./ILI9341/ILI9341_Driver.d 


# Each subdirectory must supply rules for building sources it contributes
ILI9341/ILI9341_Driver.o: ../ILI9341/ILI9341_Driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Icons -I../Images -I../ILI9341 -I../Touch -I../UART -I../Display -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"ILI9341/ILI9341_Driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

