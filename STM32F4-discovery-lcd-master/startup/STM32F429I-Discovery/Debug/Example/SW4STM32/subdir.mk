################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
E:/Axlr8r/STM32/STM32F4-discovery-lcd-master/startup/startup_stm32f429xx.s 

OBJS += \
./Example/SW4STM32/startup_stm32f429xx.o 

S_DEPS += \
./Example/SW4STM32/startup_stm32f429xx.d 


# Each subdirectory must supply rules for building sources it contributes
Example/SW4STM32/startup_stm32f429xx.o: E:/Axlr8r/STM32/STM32F4-discovery-lcd-master/startup/startup_stm32f429xx.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Example/SW4STM32/startup_stm32f429xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

