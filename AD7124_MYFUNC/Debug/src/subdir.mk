################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/AD7124_MYFUNC.c \
../src/ad7124.c \
../src/ad7124_regs.c \
../src/cr_startup_lpc175x_6x.c \
../src/crp.c \
../src/sysinit.c 

OBJS += \
./src/AD7124_MYFUNC.o \
./src/ad7124.o \
./src/ad7124_regs.o \
./src/cr_startup_lpc175x_6x.o \
./src/crp.o \
./src/sysinit.o 

C_DEPS += \
./src/AD7124_MYFUNC.d \
./src/ad7124.d \
./src/ad7124_regs.d \
./src/cr_startup_lpc175x_6x.d \
./src/crp.d \
./src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M3 -D__USE_LPCOPEN -D__LPC17XX__ -D__REDLIB__ -I"/home/federico/Escritorio/Workspace/LPC_BaseBoardBB1769_R02/inc" -I"/home/federico/Escritorio/Workspace/LibFreeRTOS_8.2.3/src/demo_code" -I"/home/federico/Escritorio/Workspace/LibFreeRTOS_8.2.3/src/portable/GCC/ARM_CM3" -I"/home/federico/Escritorio/Workspace/LibFreeRTOS_8.2.3/inc" -I"/home/federico/Escritorio/Workspace/lpc_chip_175x_6x/inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


