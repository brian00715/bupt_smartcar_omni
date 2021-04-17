################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CODE/button.c \
../CODE/buzzer.c \
../CODE/display.c \
../CODE/elec.c \
../CODE/encoder.c \
../CODE/mecanum_chassis.c \
../CODE/mecanum_motion.c \
../CODE/motor.c \
../CODE/sci_compute.c \
../CODE/timer_pit.c 

OBJS += \
./CODE/button.o \
./CODE/buzzer.o \
./CODE/display.o \
./CODE/elec.o \
./CODE/encoder.o \
./CODE/mecanum_chassis.o \
./CODE/mecanum_motion.o \
./CODE/motor.o \
./CODE/sci_compute.o \
./CODE/timer_pit.o 

C_DEPS += \
./CODE/button.d \
./CODE/buzzer.d \
./CODE/display.d \
./CODE/elec.d \
./CODE/encoder.d \
./CODE/mecanum_chassis.d \
./CODE/mecanum_motion.d \
./CODE/motor.d \
./CODE/sci_compute.d \
./CODE/timer_pit.d 


# Each subdirectory must supply rules for building sources it contributes
CODE/%.o: ../CODE/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -O1 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"../Libraries/wch_libraries/Core" -I"D:\ProgramDev\SmartcarPrograms\Smartcar_Mecanum_Master\Libraries\rtthread_libraries\bsp" -I"D:\ProgramDev\SmartcarPrograms\Smartcar_Mecanum_Master\Libraries\rtthread_libraries\components\finsh" -I"D:\ProgramDev\SmartcarPrograms\Smartcar_Mecanum_Master\Libraries\rtthread_libraries\include" -I"D:\ProgramDev\SmartcarPrograms\Smartcar_Mecanum_Master\Libraries\rtthread_libraries\include\libc" -I"../Libraries/wch_libraries/Peripheral" -I"../Libraries/wch_libraries/Startup" -I"../Libraries/seekfree_libraries" -I"../Libraries/seekfree_peripheral" -I"../Libraries/board" -I"../CODE" -I"../USER" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

