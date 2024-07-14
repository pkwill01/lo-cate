################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../c_rfm95_driver/src/c_rfm95_driver.c 

OBJS += \
./c_rfm95_driver/src/c_rfm95_driver.o 

C_DEPS += \
./c_rfm95_driver/src/c_rfm95_driver.d 


# Each subdirectory must supply rules for building sources it contributes
c_rfm95_driver/src/%.o c_rfm95_driver/src/%.su c_rfm95_driver/src/%.cyclo: ../c_rfm95_driver/src/%.c c_rfm95_driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"/home/kinga/Desktop/lo-cate/firmware/lo_cate_main/c_rfm95_driver/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-c_rfm95_driver-2f-src

clean-c_rfm95_driver-2f-src:
	-$(RM) ./c_rfm95_driver/src/c_rfm95_driver.cyclo ./c_rfm95_driver/src/c_rfm95_driver.d ./c_rfm95_driver/src/c_rfm95_driver.o ./c_rfm95_driver/src/c_rfm95_driver.su

.PHONY: clean-c_rfm95_driver-2f-src

