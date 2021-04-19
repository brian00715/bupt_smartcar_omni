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
#include "pid.h"

void
Motor_Init (void)
{
  //��ʼ�����PWM���źͷ�������

  //�����������У�����������Ƶ��ѡ��13K-17K
  //���ռ�ձ�ֵPWM_DUTY_MAX ������zf_pwm.h�ļ����޸� Ĭ��Ϊ50000
  //����һ��PWMģ�� ����������ͨ��ֻ�����Ƶ��һ�� ռ�ձȲ�һ���� PWM CH32V103R8T6ֻ���ĸ�PWMģ�� ÿ��ģ����4��ͨ��
  gpio_init (MOTOR1_A, GPO, 0, GPIO_PIN_CONFIG);
  pwm_init (MOTOR1_B, 17000, 0);
  gpio_init (MOTOR2_A, GPO, 0, GPIO_PIN_CONFIG);
  pwm_init (MOTOR2_B, 17000, 0);
  gpio_init (MOTOR3_A, GPO, 0, GPIO_PIN_CONFIG);
  pwm_init (MOTOR3_B, 17000, 0);
  gpio_init (MOTOR4_A, GPO, 0, GPIO_PIN_CONFIG);
  pwm_init (MOTOR4_B, 17000, 0);
}
// kp ki kd int_duty int_max int_sum last_err last_delta_err
PID_t MotorPID =
  { 0.1, 0.01, 0.0, 10, 10, 0, 0, 0 };
/**
 * @brief ���ռ�ձȻ�
 * 
 * @param pid 
 * @param now_rpm 
 * @param expect_speed 
 */
void
Motor_DutyCtrl (int16 target_duty)
{
//    int16 pwm_out = PID_GetOutput(&MotorPID,target_duty);
//    return pwm_out;
//  Motor_SetDuty(duty_1, duty_2, duty_3, duty_4);
}

/**
 * @brief �������ֵ�������ٶ�����
 *
 * @param
 * @note
 */
void
Motor_SetDuty (int32 duty_1, int32 duty_2, int32 duty_3, int32 duty_4)
{
  //��ռ�ձ��޷�
  LIMIT(duty_1, PWM_DUTY_MAX);
  LIMIT(duty_2, PWM_DUTY_MAX);
  LIMIT(duty_3, PWM_DUTY_MAX);
  LIMIT(duty_4, PWM_DUTY_MAX);

  // ����ռ�ձ�Ϊ �ٷ�֮ (1000/TIMER1_PWM_DUTY_MAX*100)
  if (0 <= duty_1) //���1   ��ת
    {
      gpio_set (MOTOR1_A, FORWARD_ROTATE);
    }
  else //���1   ��ת
    {
      gpio_set (MOTOR1_A, REVERSE_ROTATE);
    }
  pwm_duty (MOTOR1_B, duty_1);

  if (0 <= duty_2) //���2   ��ת
    {
      gpio_set (MOTOR2_A, FORWARD_ROTATE);
    }
  else //���2   ��ת
    {
      gpio_set (MOTOR2_A, REVERSE_ROTATE);
    }
  pwm_duty (MOTOR2_B, duty_2);

  if (0 <= duty_3) //���3   ��ת
    {
      gpio_set (MOTOR3_A, FORWARD_ROTATE);
    }
  else //���3   ��ת
    {
      gpio_set (MOTOR3_A, REVERSE_ROTATE);
    }
  pwm_duty (MOTOR3_B, duty_3);

  if (0 <= duty_4) //���3   ��ת
    {
      gpio_set (MOTOR4_A, FORWARD_ROTATE);
    }
  else //���3   ��ת
    {
      gpio_set (MOTOR4_A, REVERSE_ROTATE);
    }
  pwm_duty (MOTOR4_B, duty_4);
}

/**
 * @brief �������ٶ�m/sתΪ���͸��������ռ�ձ�
 *
 * @param speed m/s
 * @return
 */
float
SW_Speed2PWM (float speed)
{
}

void
DriveMotors_LimitSpeed (float speed[4])
{
  for (int i = 0; i < 4; i++)
    {
      LIMIT_FROM_TO(speed[i], DRIVE_WHEEL_MIN_SPEED, DRIVE_WHEEL_MAX_SPEED);
    }
}
