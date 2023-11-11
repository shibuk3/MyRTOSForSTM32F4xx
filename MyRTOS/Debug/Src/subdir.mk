################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/CRC.c \
../Src/FIFO.c \
../Src/PriorityScheduler.c \
../Src/SporadicScheduler.c \
../Src/bootloader.c \
../Src/gpio.c \
../Src/led.c \
../Src/mailBox.c \
../Src/main.c \
../Src/osKernel.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/timebase.c \
../Src/timer.c \
../Src/usart.c 

OBJS += \
./Src/CRC.o \
./Src/FIFO.o \
./Src/PriorityScheduler.o \
./Src/SporadicScheduler.o \
./Src/bootloader.o \
./Src/gpio.o \
./Src/led.o \
./Src/mailBox.o \
./Src/main.o \
./Src/osKernel.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/timebase.o \
./Src/timer.o \
./Src/usart.o 

C_DEPS += \
./Src/CRC.d \
./Src/FIFO.d \
./Src/PriorityScheduler.d \
./Src/SporadicScheduler.d \
./Src/bootloader.d \
./Src/gpio.d \
./Src/led.d \
./Src/mailBox.d \
./Src/main.d \
./Src/osKernel.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/timebase.d \
./Src/timer.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I../Inc -I"../$(ProjDirPath)/chip_headers/CMSIS/Device/ST/STM32F4xx/Include" -I"../$(ProjDirPath)/chip_headers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/CRC.cyclo ./Src/CRC.d ./Src/CRC.o ./Src/CRC.su ./Src/FIFO.cyclo ./Src/FIFO.d ./Src/FIFO.o ./Src/FIFO.su ./Src/PriorityScheduler.cyclo ./Src/PriorityScheduler.d ./Src/PriorityScheduler.o ./Src/PriorityScheduler.su ./Src/SporadicScheduler.cyclo ./Src/SporadicScheduler.d ./Src/SporadicScheduler.o ./Src/SporadicScheduler.su ./Src/bootloader.cyclo ./Src/bootloader.d ./Src/bootloader.o ./Src/bootloader.su ./Src/gpio.cyclo ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/led.cyclo ./Src/led.d ./Src/led.o ./Src/led.su ./Src/mailBox.cyclo ./Src/mailBox.d ./Src/mailBox.o ./Src/mailBox.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/osKernel.cyclo ./Src/osKernel.d ./Src/osKernel.o ./Src/osKernel.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/timebase.cyclo ./Src/timebase.d ./Src/timebase.o ./Src/timebase.su ./Src/timer.cyclo ./Src/timer.d ./Src/timer.o ./Src/timer.su ./Src/usart.cyclo ./Src/usart.d ./Src/usart.o ./Src/usart.su

.PHONY: clean-Src

