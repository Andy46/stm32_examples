################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/libs/BMI160_SensorAPI/bmi160.c 

OBJS += \
./Core/libs/BMI160_SensorAPI/bmi160.o 

C_DEPS += \
./Core/libs/BMI160_SensorAPI/bmi160.d 


# Each subdirectory must supply rules for building sources it contributes
Core/libs/BMI160_SensorAPI/%.o Core/libs/BMI160_SensorAPI/%.su Core/libs/BMI160_SensorAPI/%.cyclo: ../Core/libs/BMI160_SensorAPI/%.c Core/libs/BMI160_SensorAPI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"/projects/02_STM32/00_STM32_Examples/stm32_examples/01_BMI160/STM32-BMI160-I2C/Core/libs/BMI160_SensorAPI" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-libs-2f-BMI160_SensorAPI

clean-Core-2f-libs-2f-BMI160_SensorAPI:
	-$(RM) ./Core/libs/BMI160_SensorAPI/bmi160.cyclo ./Core/libs/BMI160_SensorAPI/bmi160.d ./Core/libs/BMI160_SensorAPI/bmi160.o ./Core/libs/BMI160_SensorAPI/bmi160.su

.PHONY: clean-Core-2f-libs-2f-BMI160_SensorAPI

