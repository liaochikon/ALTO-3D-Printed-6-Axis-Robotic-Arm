################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Emm_can.c \
../Core/Src/FK.c \
../Core/Src/IK.c \
../Core/Src/alto_config.c \
../Core/Src/command.c \
../Core/Src/error.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/matmul.c \
../Core/Src/robot_behavior.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/transform.c 

OBJS += \
./Core/Src/Emm_can.o \
./Core/Src/FK.o \
./Core/Src/IK.o \
./Core/Src/alto_config.o \
./Core/Src/command.o \
./Core/Src/error.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/matmul.o \
./Core/Src/robot_behavior.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/transform.o 

C_DEPS += \
./Core/Src/Emm_can.d \
./Core/Src/FK.d \
./Core/Src/IK.d \
./Core/Src/alto_config.d \
./Core/Src/command.d \
./Core/Src/error.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/matmul.d \
./Core/Src/robot_behavior.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/transform.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Emm_can.cyclo ./Core/Src/Emm_can.d ./Core/Src/Emm_can.o ./Core/Src/Emm_can.su ./Core/Src/FK.cyclo ./Core/Src/FK.d ./Core/Src/FK.o ./Core/Src/FK.su ./Core/Src/IK.cyclo ./Core/Src/IK.d ./Core/Src/IK.o ./Core/Src/IK.su ./Core/Src/alto_config.cyclo ./Core/Src/alto_config.d ./Core/Src/alto_config.o ./Core/Src/alto_config.su ./Core/Src/command.cyclo ./Core/Src/command.d ./Core/Src/command.o ./Core/Src/command.su ./Core/Src/error.cyclo ./Core/Src/error.d ./Core/Src/error.o ./Core/Src/error.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/matmul.cyclo ./Core/Src/matmul.d ./Core/Src/matmul.o ./Core/Src/matmul.su ./Core/Src/robot_behavior.cyclo ./Core/Src/robot_behavior.d ./Core/Src/robot_behavior.o ./Core/Src/robot_behavior.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/transform.cyclo ./Core/Src/transform.d ./Core/Src/transform.o ./Core/Src/transform.su

.PHONY: clean-Core-2f-Src

