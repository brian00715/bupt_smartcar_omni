/**
 * @file motor.c
 * @author simon
 * @brief ����������
 * @version 0.1
 * @date 2021-04-17
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "motor.h"
#include "config.h"
#include "sci_compute.h"



void Motor_Init(void)
{
    //��ʼ�����PWM���źͷ�������

    //�����������У�����������Ƶ��ѡ��13K-17K
    //���ռ�ձ�ֵPWM_DUTY_MAX ������zf_pwm.h�ļ����޸� Ĭ��Ϊ50000
    //����һ��PWMģ�� ����������ͨ��ֻ�����Ƶ��һ�� ռ�ձȲ�һ���� PWM CH32V103R8T6ֻ���ĸ�PWMģ�� ÿ��ģ����4��ͨ��
    gpio_init(MOTOR1_A, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(MOTOR1_B,17000,0);
    gpio_init(MOTOR2_A, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(MOTOR2_B,17000,0);
    gpio_init(MOTOR3_A, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(MOTOR3_B,17000,0);
    gpio_init(MOTOR4_A, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(MOTOR4_B,17000,0);
}

void Motor_PIDCtrl(int16 expect_speed)
{
    
}

/**
 * @brief �������ֵ�������ٶ�����
 *
 * @param
 * @note
 */
void Motor_SetDuty(int32 duty_1, int32 duty_2, int32 duty_3, int32 duty_4)
{
    //��ռ�ձ��޷�
    if(duty_1>PWM_DUTY_MAX) duty_1 = PWM_DUTY_MAX;
    else                    duty_1 = -PWM_DUTY_MAX;

    if(duty_2>PWM_DUTY_MAX) duty_2 = PWM_DUTY_MAX;
    else                    duty_2 = -PWM_DUTY_MAX;

    if(duty_3>PWM_DUTY_MAX) duty_3 = PWM_DUTY_MAX;
    else                    duty_3 = -PWM_DUTY_MAX;
    
    if(duty_4>PWM_DUTY_MAX) duty_4 = PWM_DUTY_MAX;
    else                    duty_4 = -PWM_DUTY_MAX;
    
    if(0<=duty_1) //���1   ��ת ����ռ�ձ�Ϊ �ٷ�֮ (1000/TIMER1_PWM_DUTY_MAX*100)
    {
        gpio_set(MOTOR1_A, 1);
        pwm_duty(MOTOR1_B, duty_1);
    }
    else                //���1   ��ת
    {
        gpio_set(MOTOR1_A, 0);
        pwm_duty(MOTOR1_B, -duty_1);
    }

    if(0<=duty_2) //���2   ��ת
    {
        gpio_set(MOTOR2_A, 1);
        pwm_duty(MOTOR2_B, duty_2);
    }
    else                //���2   ��ת
    {
        gpio_set(MOTOR2_A, 0);
        pwm_duty(MOTOR2_B, -duty_2);
    }

    if(0<=duty_3) //���3   ��ת
    {
        gpio_set(MOTOR3_A, 1);
        pwm_duty(MOTOR3_B, duty_3);
    }
    else                //���3   ��ת
    {
        gpio_set(MOTOR3_A, 0);
        pwm_duty(MOTOR3_B, -duty_3);
    }

    if(0<=duty_4) //���3   ��ת
    {
        gpio_set(MOTOR4_A, 1);
        pwm_duty(MOTOR4_B, duty_4);
    }
    else                //���3   ��ת
    {
        gpio_set(MOTOR4_A, 0);
        pwm_duty(MOTOR4_B, -duty_4);
    }
}


/**
 * @brief �������ٶ�m/sתΪ���͸��������ռ�ձ�
 *
 * @param speed m/s
 * @return
 */
float SW_Speed2PWM(float speed)
{
}

void DriveMotors_LimitSpeed(float speed[4])
{
    for (int i = 0; i < 4; i++)
    {
        LIMIT_FROM_TO(speed[i], DRIVE_WHEEL_MIN_SPEED, DRIVE_WHEEL_MAX_SPEED);
    }
}
