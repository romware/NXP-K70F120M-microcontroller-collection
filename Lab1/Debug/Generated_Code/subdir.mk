################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Generated_Code/Cpu.c \
../Generated_Code/PE_LDD.c \
../Generated_Code/Vectors.c 

OBJS += \
./Generated_Code/Cpu.o \
./Generated_Code/PE_LDD.o \
./Generated_Code/Vectors.o 

C_DEPS += \
./Generated_Code/Cpu.d \
./Generated_Code/PE_LDD.d \
./Generated_Code/Vectors.d 


# Each subdirectory must supply rules for building sources it contributes
Generated_Code/%.o: ../Generated_Code/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:/Users/PMcL/Documents/Subjects/48434 Embedded Software/4 Labs/Lab 1/Template/Lab1/Static_Code/PDD" -I"C:/Users/PMcL/Documents/Subjects/48434 Embedded Software/4 Labs/Lab 1/Template/Lab1/Static_Code/IO_Map" -I"C:/Users/PMcL/Documents/Subjects/48434 Embedded Software/4 Labs/Lab 1/Template/Lab1/Sources" -I"C:/Users/PMcL/Documents/Subjects/48434 Embedded Software/4 Labs/Lab 1/Template/Lab1/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


