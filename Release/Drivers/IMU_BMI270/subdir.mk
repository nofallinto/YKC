################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/IMU_BMI270/BMI270.c 

OBJS += \
./Drivers/IMU_BMI270/BMI270.o 

C_DEPS += \
./Drivers/IMU_BMI270/BMI270.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IMU_BMI270/%.o Drivers/IMU_BMI270/%.su Drivers/IMU_BMI270/%.cyclo: ../Drivers/IMU_BMI270/%.c Drivers/IMU_BMI270/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I"D:/SourceTree/ykc/Ctr" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Src -I../YKC -I../Drivers/IMU_BMI270 -I"D:/SourceTree/ykc/yLib" -I"D:/SourceTree/ykc/YKC" -I"D:/SourceTree/ykc/Ctr" -I../Drivers/Air_BMP280 -I../Drivers/IMU_ATK_MS601M -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-IMU_BMI270

clean-Drivers-2f-IMU_BMI270:
	-$(RM) ./Drivers/IMU_BMI270/BMI270.cyclo ./Drivers/IMU_BMI270/BMI270.d ./Drivers/IMU_BMI270/BMI270.o ./Drivers/IMU_BMI270/BMI270.su

.PHONY: clean-Drivers-2f-IMU_BMI270

