################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Touch/subdir.mk
-include Src/subdir.mk
-include ILI9341/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Display/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
TFTLCD.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
TFTLCD.list \

OBJCOPY_HEX += \
TFTLCD.hex \


# All Target
all: TFTLCD.elf secondary-outputs

# Tool invocations
TFTLCD.elf: $(OBJS) $(USER_OBJS) ../STM32F411VETx_FLASH.ld
	arm-none-eabi-gcc -o "TFTLCD.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"../STM32F411VETx_FLASH.ld" --specs=nosys.specs -Wl,-Map="TFTLCD.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

TFTLCD.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "TFTLCD.list"
	@echo 'Finished building: $@'
	@echo ' '

TFTLCD.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "TFTLCD.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
