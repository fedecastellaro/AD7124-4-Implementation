################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: AD7124_MYFUNC.axf

# Tool invocations
AD7124_MYFUNC.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/home/federico/Escritorio/Workspace/LibFreeRTOS_8.2.3/Debug" -L"/home/federico/Escritorio/Workspace/LPC_BaseBoardBB1769_R02/Debug" -L"/home/federico/Escritorio/Workspace/lpc_chip_175x_6x/Debug" -Xlinker -Map="AD7124_MYFUNC.map" -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T "AD7124_MYFUNC_Debug.ld" -o "AD7124_MYFUNC.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) AD7124_MYFUNC.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "AD7124_MYFUNC.axf"; # arm-none-eabi-objcopy -v -O binary "AD7124_MYFUNC.axf" "AD7124_MYFUNC.bin" ; # checksum -p LPC1769 -d "AD7124_MYFUNC.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
