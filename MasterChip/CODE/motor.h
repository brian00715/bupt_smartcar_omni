#ifndef _motor_h
#define _motor_h

#include "headfile.h"

// ======================宏定义=========================
#define MOTOR1_A   C10              //定义1电机正反转引脚
#define MOTOR1_B   PWM4_CH1_B6      //定义1电机PWM引脚

#define MOTOR2_A   C11               //定义2电机正反转引脚
#define MOTOR2_B   PWM4_CH2_B7      //定义2电机PWM引脚

#define MOTOR3_A   B12              //定义3电机正反转引脚
#define MOTOR3_B   PWM4_CH3_B8      //定义3电机PWM引脚

#define MOTOR4_A   A8              //定义4电机正反转引脚
#define MOTOR4_B   PWM4_CH4_B9      //定义4电机PWM引脚

//=================================函数声明=================================

void DriveMotors_SetRpm(float speed[4]);
float SW_Speed2PWM(float speed);
void DriveMotors_LimitSpeed(float speed[4]);
void Motor_Init(void);
void Motor_PIDCtrl(int16 expect_speed);
void Motor_SetDuty(int32 duty_1, int32 duty_2, int32 duty_3, int32 duty_4);
    
#endif
