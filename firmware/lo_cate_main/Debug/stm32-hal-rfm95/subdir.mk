################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32-hal-rfm95/rfm95.c 

OBJS += \
./stm32-hal-rfm95/rfm95.o 

C_DEPS += \
./stm32-hal-rfm95/rfm95.d 


# Each subdirectory must supply rules for building sources it contributes
stm32-hal-rfm95/%.o stm32-hal-rfm95/%.su stm32-hal-rfm95/%.cyclo: ../stm32-hal-rfm95/%.c stm32-hal-rfm95/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"/home/kinga/Desktop/lo-cate/firmware/lo_cate_main/stm32-hal-rfm95" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32-2d-hal-2d-rfm95

clean-stm32-2d-hal-2d-rfm95:
	-$(RM) ./stm32-hal-rfm95/rfm95.cyclo ./stm32-hal-rfm95/rfm95.d ./stm32-hal-rfm95/rfm95.o ./stm32-hal-rfm95/rfm95.su

.PHONY: clean-stm32-2d-hal-2d-rfm95

