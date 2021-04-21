################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CODE/button.c \
../CODE/buzzer.c \
../CODE/cmd.c \
../CODE/display.c \
../CODE/elec.c \
../CODE/encoder.c \
../CODE/mecanum_chassis.c \
../CODE/motor.c \
../CODE/pid.c \
../CODE/sci_compute.c \
../CODE/timer_pit.c 

OBJS += \
./CODE/button.o \
./CODE/buzzer.o \
./CODE/cmd.o \
./CODE/display.o \
./CODE/elec.o \
./CODE/encoder.o \
./CODE/mecanum_chassis.o \
./CODE/motor.o \
./CODE/pid.o \
./CODE/sci_compute.o \
./CODE/timer_pit.o 

C_DEPS += \
./CODE/button.d \
./CODE/buzzer.d \
./CODE/cmd.d \
./CODE/display.d \
./CODE/elec.d \
./CODE/encoder.d \
./CODE/mecanum_chassis.d \
./CODE/motor.d \
./CODE/pid.d \
./CODE/sci_compute.d \
./CODE/timer_pit.d 


# Each subdirectory must supply rules for building sources it contributes
CODE/%.o: ../CODE/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"../Libraries/wch_libraries/Core" -I"../Libraries/wch_libraries/Peripheral" -I"../Libraries/wch_libraries/Startup" -I"../Libraries/seekfree_libraries" -I"../Libraries/seekfree_peripheral" -I"../Libraries/board" -I"../CODE" -I"../USER" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

