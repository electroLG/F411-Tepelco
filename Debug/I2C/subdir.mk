################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/STM32-IDE/STM32-LIBRARY/I2C/i2c.c 

OBJS += \
./I2C/i2c.o 

C_DEPS += \
./I2C/i2c.d 


# Each subdirectory must supply rules for building sources it contributes
I2C/i2c.o: C:/STM32-IDE/STM32-LIBRARY/I2C/i2c.c I2C/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/STM32-IDE/STM32-LIBRARY/HTTP" -I"C:/STM32-IDE/STM32-LIBRARY/I2C" -I"C:/STM32-IDE/STM32-LIBRARY/STM32_ETH_W5100" -I"C:/STM32-IDE/STM32-LIBRARY/STM32_ETH_W5500" -I"C:/STM32-IDE/STM32-LIBRARY/STRING" -I"C:/STM32-IDE/STM32-LIBRARY/ModBUS" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-I2C

clean-I2C:
	-$(RM) ./I2C/i2c.d ./I2C/i2c.o ./I2C/i2c.su

.PHONY: clean-I2C

