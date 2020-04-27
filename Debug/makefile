################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
ThrusterDynamometer_F4_STM32CubeIDE.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ThrusterDynamometer_F4_STM32CubeIDE.list \

OBJCOPY_HEX += \
ThrusterDynamometer_F4_STM32CubeIDE.hex \

OBJCOPY_BIN += \
ThrusterDynamometer_F4_STM32CubeIDE.bin \


# All Target
all: ThrusterDynamometer_F4_STM32CubeIDE.elf secondary-outputs

# Tool invocations
ThrusterDynamometer_F4_STM32CubeIDE.elf: $(OBJS) $(USER_OBJS) E:\ThrusterDynamometer\ThrusterDynamometer_F4_STM32CubeIDE\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "ThrusterDynamometer_F4_STM32CubeIDE.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"E:\ThrusterDynamometer\ThrusterDynamometer_F4_STM32CubeIDE\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ThrusterDynamometer_F4_STM32CubeIDE.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ThrusterDynamometer_F4_STM32CubeIDE.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ThrusterDynamometer_F4_STM32CubeIDE.list"
	@echo 'Finished building: $@'
	@echo ' '

ThrusterDynamometer_F4_STM32CubeIDE.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "ThrusterDynamometer_F4_STM32CubeIDE.hex"
	@echo 'Finished building: $@'
	@echo ' '

ThrusterDynamometer_F4_STM32CubeIDE.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "ThrusterDynamometer_F4_STM32CubeIDE.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
