################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../YKC/BoardSupport.c \
../YKC/MdlTest.c 

OBJS += \
./YKC/BoardSupport.o \
./YKC/MdlTest.o 

C_DEPS += \
./YKC/BoardSupport.d \
./YKC/MdlTest.d 


# Each subdirectory must supply rules for building sources it contributes
YKC/%.o YKC/%.su YKC/%.cyclo: ../YKC/%.c YKC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I"D:/SourceTree/ykc/Ctr" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Src -I../YKC -I../Drivers/IMU_BMI270 -I"D:/SourceTree/ykc/yLib" -I"D:/SourceTree/ykc/YKC" -I"D:/SourceTree/ykc/Ctr" -I../Drivers/Air_BMP280 -I../Drivers/IMU_ATK_MS601M -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-YKC

clean-YKC:
	-$(RM) ./YKC/BoardSupport.cyclo ./YKC/BoardSupport.d ./YKC/BoardSupport.o ./YKC/BoardSupport.su ./YKC/MdlTest.cyclo ./YKC/MdlTest.d ./YKC/MdlTest.o ./YKC/MdlTest.su

.PHONY: clean-YKC

