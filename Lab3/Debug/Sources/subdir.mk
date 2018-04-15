################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/Events.c \
../Sources/FIFO.c \
../Sources/Flash.c \
../Sources/LEDs.c \
../Sources/PIT.c \
../Sources/UART.c \
../Sources/main.c \
../Sources/packet.c 

OBJS += \
./Sources/Events.o \
./Sources/FIFO.o \
./Sources/Flash.o \
./Sources/LEDs.o \
./Sources/PIT.o \
./Sources/UART.o \
./Sources/main.o \
./Sources/packet.o 

C_DEPS += \
./Sources/Events.d \
./Sources/FIFO.d \
./Sources/Flash.d \
./Sources/LEDs.d \
./Sources/PIT.d \
./Sources/UART.d \
./Sources/main.d \
./Sources/packet.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:/Users/12403756/git/es18aut13/Lab3/Static_Code/IO_Map" -I"C:/Users/12403756/git/es18aut13/Lab3/Sources" -I"C:/Users/12403756/git/es18aut13/Lab3/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


