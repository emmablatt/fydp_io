################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32H735G-DK/stm32h735g_discovery.c \
../Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_audio.c \
../Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_bus.c 

OBJS += \
./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery.o \
./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_audio.o \
./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_bus.o 

C_DEPS += \
./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery.d \
./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_audio.d \
./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_bus.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32H735G-DK/%.o: ../Drivers/BSP/STM32H735G-DK/%.c Drivers/BSP/STM32H735G-DK/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H735xx -c -I../Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32H735G-2d-DK

clean-Drivers-2f-BSP-2f-STM32H735G-2d-DK:
	-$(RM) ./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery.d ./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery.o ./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_audio.d ./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_audio.o ./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_bus.d ./Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_bus.o

.PHONY: clean-Drivers-2f-BSP-2f-STM32H735G-2d-DK

