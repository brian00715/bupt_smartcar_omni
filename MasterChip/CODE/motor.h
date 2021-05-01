#ifndef _motor_h
#define _motor_h

#include "headfile.h"

// ======================�궨��=========================
#define MOTOR1_A C10         //����1�������ת����
#define MOTOR1_B PWM4_CH1_B6 //����1���PWM����

#define MOTOR2_A C11         //����2�������ת����
#define MOTOR2_B PWM4_CH2_B7 //����2���PWM����

#define MOTOR3_A B12         //����3�������ת����
#define MOTOR3_B PWM4_CH3_B8 //����3���PWM����

#define MOTOR4_A A8          //����4�������ת����
#define MOTOR4_B PWM4_CH4_B9 //����4���PWM����

#define FORWARD_ROTATE 0
#define REVERSE_ROTATE 1

typedef struct Motor_t
{
    int16 now_duty;
    int16 last_duty;
    int16 target_duty;
} Motor_t;

//=================================��������=================================

void DriveMotors_SetRpm(float speed[4]);
int SW_Speed2Duty(float speed);
void DriveMotors_LimitSpeed(float speed[4]);
void Motor_Init(void);
void Motor_DutyCtrl();
void Motor_SetDuty(int32_t duty1,int32_t duty2,int32_t duty3,int32_t duty4);

#endif
