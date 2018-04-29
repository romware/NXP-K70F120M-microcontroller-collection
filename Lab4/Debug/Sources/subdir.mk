################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/Events.c \
../Sources/FIFO.c \
../Sources/FTM.c \
../Sources/Flash.c \
../Sources/I2C.c \
../Sources/LEDs.c \
../Sources/PIT.c \
../Sources/RTC.c \
../Sources/UART.c \
../Sources/accel.c \
../Sources/main.c \
../Sources/median.c \
../Sources/packet.c 

OBJS += \
./Sources/Events.o \
./Sources/FIFO.o \
./Sources/FTM.o \
./Sources/Flash.o \
./Sources/I2C.o \
./Sources/LEDs.o \
./Sources/PIT.o \
./Sources/RTC.o \
./Sources/UART.o \
./Sources/accel.o \
./Sources/main.o \
./Sources/median.o \
./Sources/packet.o 

C_DEPS += \
./Sources/Events.d \
./Sources/FIFO.d \
./Sources/FTM.d \
./Sources/Flash.d \
./Sources/I2C.d \
./Sources/LEDs.d \
./Sources/PIT.d \
./Sources/RTC.d \
./Sources/UART.d \
./Sources/accel.d \
./Sources/main.d \
./Sources/median.d \
./Sources/packet.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:/Users/Liam Warburton/AppData/Roaming/SPB_Data/git/es18aut13-4/Lab4/Static_Code/IO_Map" -I"C:/Users/Liam Warburton/AppData/Roaming/SPB_Data/git/es18aut13-4/Lab4/Sources" -I"C:/Users/Liam Warburton/AppData/Roaming/SPB_Data/git/es18aut13-4/Lab4/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


