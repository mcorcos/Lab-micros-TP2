################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/drivers/drv_CAN.c \
../source/drivers/drv_DEVBOARD.c \
../source/drivers/drv_FXOS8700CQ.c \
../source/drivers/drv_I2C.c \
../source/drivers/drv_K64.c \
../source/drivers/drv_UART.c 

OBJS += \
./source/drivers/drv_CAN.o \
./source/drivers/drv_DEVBOARD.o \
./source/drivers/drv_FXOS8700CQ.o \
./source/drivers/drv_I2C.o \
./source/drivers/drv_K64.o \
./source/drivers/drv_UART.o 

C_DEPS += \
./source/drivers/drv_CAN.d \
./source/drivers/drv_DEVBOARD.d \
./source/drivers/drv_FXOS8700CQ.d \
./source/drivers/drv_I2C.d \
./source/drivers/drv_K64.d \
./source/drivers/drv_UART.d 


# Each subdirectory must supply rules for building sources it contributes
source/drivers/%.o: ../source/drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DCPU_MK64FN1M0VLL12 -D__USE_CMSIS -DNDEBUG -I../source -I../ -I../CMSIS -I../startup -Os -fno-common -g -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


