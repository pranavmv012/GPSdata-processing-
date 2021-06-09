################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
vasudevan_Assignment5.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
vasudevan_Assignment5.list \

OBJCOPY_BIN += \
vasudevan_Assignment5.bin \


# All Target
all: vasudevan_Assignment5.elf secondary-outputs

# Tool invocations
vasudevan_Assignment5.elf: $(OBJS) $(USER_OBJS) E:\Conestoga\Embedded\ programing\Assignments\assingnment5\vasudevan_Assignment5\STM32L432KCUX_FLASH.ld
	arm-none-eabi-gcc -o "vasudevan_Assignment5.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"E:\Conestoga\Embedded programing\Assignments\assingnment5\vasudevan_Assignment5\STM32L432KCUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="vasudevan_Assignment5.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

vasudevan_Assignment5.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "vasudevan_Assignment5.list"
	@echo 'Finished building: $@'
	@echo ' '

vasudevan_Assignment5.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "vasudevan_Assignment5.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
