################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/so_grice/STM32CubeIDE/workspace_1.10.1/TIM_PWMOutput/Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.c 

OBJS += \
./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.o 

C_DEPS += \
./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.o: C:/Users/so_grice/STM32CubeIDE/workspace_1.10.1/TIM_PWMOutput/Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.c Drivers/BSP/NUCLEO-WB15CC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB15xx -c -I../../Inc -I../../Drivers/STM32WBxx_HAL_Driver/Inc -I../../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/NUCLEO-WB15CC -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-NUCLEO-2d-WB15CC

clean-Drivers-2f-BSP-2f-NUCLEO-2d-WB15CC:
	-$(RM) ./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.cyclo ./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.d ./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.o ./Drivers/BSP/NUCLEO-WB15CC/nucleo_wb15cc.su

.PHONY: clean-Drivers-2f-BSP-2f-NUCLEO-2d-WB15CC

