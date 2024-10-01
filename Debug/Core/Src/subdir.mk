################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/RM3100QUAD.c \
../Core/Src/lfs.c \
../Core/Src/lfs_port.c \
../Core/Src/lfs_port_ai.c \
../Core/Src/lfs_util.c \
../Core/Src/littlefs_driver.c \
../Core/Src/main.c \
../Core/Src/main_original.c \
../Core/Src/nor.c \
../Core/Src/nor_ids.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

OBJS += \
./Core/Src/RM3100QUAD.o \
./Core/Src/lfs.o \
./Core/Src/lfs_port.o \
./Core/Src/lfs_port_ai.o \
./Core/Src/lfs_util.o \
./Core/Src/littlefs_driver.o \
./Core/Src/main.o \
./Core/Src/main_original.o \
./Core/Src/nor.o \
./Core/Src/nor_ids.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/RM3100QUAD.d \
./Core/Src/lfs.d \
./Core/Src/lfs_port.d \
./Core/Src/lfs_port_ai.d \
./Core/Src/lfs_util.d \
./Core/Src/littlefs_driver.d \
./Core/Src/main.d \
./Core/Src/main_original.d \
./Core/Src/nor.d \
./Core/Src/nor_ids.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

