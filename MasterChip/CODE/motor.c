/**
 * @file motor.c
 * @author simon
 * @brief 电机驱动相关
 * @version 0.1
 * @date 2021-04-17
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "motor.h"
#include "config.h"
#include "sci_compute.h"
#include "mecanum_chassis.h"
#include <math.h>
#include "encoder.h"
#include "slave_comm.h"
#include "isr.h"
#include "cmd.h"

extern BaseChassis_t MecanumChassis;
// kp ki kd int_duty int_max int_sum last_err last_delta_err use_sub_pid sub_pid_kp sub_pid_thres
PID_t MotorPID[4] ={0};

void Motor_Init(void)
{
	//初始化电机PWM引脚和方向引脚
	//桌大大的推文中，建议电磁组电机频率选用13K-17K
	//最大占空比值PWM_DUTY_MAX 可以在zf_pwm.h文件中修改 默认为50000
	//对于一个PWM模块 包含的所有通道只能输出频率一样 占空比不一样的 PWM CH32V103R8T6只有四个PWM模块 每个模块有4个通道
	gpio_init(MOTOR1_A, GPO, 0, GPIO_PIN_CONFIG);
	pwm_init(MOTOR1_B, 17000, 0);
	gpio_init(MOTOR2_A, GPO, 0, GPIO_PIN_CONFIG);
	pwm_init(MOTOR2_B, 17000, 0);
	gpio_init(MOTOR3_A, GPO, 0, GPIO_PIN_CONFIG);
	pwm_init(MOTOR3_B, 17000, 0);
	gpio_init(MOTOR4_A, GPO, 0, GPIO_PIN_CONFIG);
	pwm_init(MOTOR4_B, 17000, 0);

	MotorPID[0].kp = 0.5;
	MotorPID[0].ki = 3.5;
	MotorPID[0].kd = 0.0;
	MotorPID[0].ctrl_max = 340;
//	MotorPID[0].int_duty = 0.032;
	MotorPID[0].int_max = 800;
//	MotorPID[0].use_sub_pid = 1;
//	MotorPID[0].sub_pid_thres = 2;
//	MotorPID[0].sub_pid_kp = 0.5;
	MotorPID[0].dead_th = 3;
//
	MotorPID[1].kp = 0.5;
	MotorPID[1].ki = 3.5;
	MotorPID[1].kd = 0.0;
	MotorPID[1].ctrl_max = 340;
//	MotorPID[1].int_duty = 0.032;
	MotorPID[1].int_max = 800;
//	MotorPID[1].use_sub_pid = 1;
//	MotorPID[1].sub_pid_thres = 2;
//	MotorPID[1].sub_pid_kp = 0.5;
	MotorPID[1].dead_th = 3;
//
	MotorPID[2].kp = 0.5;
	MotorPID[2].ki = 3.5;
	MotorPID[2].kd = 0.0;
	MotorPID[2].ctrl_max = 340;
//	MotorPID[2].int_duty = 0.032;
	MotorPID[2].int_max = 800;
//	MotorPID[2].use_sub_pid = 1;
//	MotorPID[2].sub_pid_thres = 2;
//	MotorPID[2].sub_pid_kp = 0.5;
	MotorPID[2].dead_th = 3;
//
	MotorPID[3].kp = 0.5;
	MotorPID[3].ki = 3.5;
	MotorPID[3].kd = 0.0;
	MotorPID[3].ctrl_max = 340;
//	MotorPID[3].int_duty = 0.032;
	MotorPID[3].int_max = 800;
//	MotorPID[3].use_sub_pid = 1;
//	MotorPID[3].sub_pid_thres = 2;
//	MotorPID[3].sub_pid_kp = 0.5;
	MotorPID[3].dead_th = 3;
	Motor_SetDuty(0, 0, 0, 0);
}

static int start_time = 0;
/**
 * @brief 电机自检，并校准编码器的范围
 * @warning 务必让车轮悬空！！！
 */
void Motor_SelfCheck(void)
{
	MecanumChassis.motor_self_check_ok = 0;
	systick_start();
	start_time = systick_getval_ms();
	uprintf("---------------------------------------------\r\n");
	uprintf("Motor|Start motor self checking! Time:%d\r\n", start_time);
	Motor_SetDuty(3000, 3000, 3000, 3000);
	encoder_max = 0;
	encoder_min = 0;

	while (1)
	{
		int curr_time = systick_getval_ms();
		for (int i = 0; i < 4; i++)
		{
			if (encoder_data[i] > encoder_max)
			{
				encoder_max = encoder_data[i];
			}
			else if (encoder_data[i] < encoder_min) // 可能有正值可能有负值
			{
				encoder_min = encoder_data[i];
			}
		}
		if (curr_time - start_time > 500)
		{
			for (int i = 0; i < 4; i++)
			{
				encoder_coff[i] = encoder_data[i] / abs(encoder_data[i]); // 获取方向系数
			}
			break;
		}
	}

	start_time = systick_getval_ms();
	Motor_SetDuty(-3000, -3000, -3000, -3000);
	while (1)
	{
		int curr_time = systick_getval_ms();
		for (int i = 0; i < 4; i++)
		{
			if (encoder_data[i] > encoder_max)
			{
				encoder_max = encoder_data[i];
			}
			else if (encoder_data[i] < encoder_min) // 可能有正值可能有负值
			{
				encoder_min = encoder_data[i];
			}
		}
		if (curr_time - start_time > 500)
		{
			break;
		}
	}
	Motor_SetDuty(0, 0, 0, 0);
	MecanumChassis.motor_self_check_ok = 1;
	uprintf("Motor|Finished motor self check.\r\n");
	uprintf("Motor|Encoder min value:%d\r\n", encoder_min);
	uprintf("Motor|Encoder max value:%d\r\n", encoder_max);
	uprintf("Motor|Encoder coff [0]:%d [1]:%d [2]:%d [3]:%d\r\n",
			encoder_coff[0], encoder_coff[1], encoder_coff[2], encoder_coff[3]);
	uprintf("---------------------------------------------\r\n");
}

/**
 * @brief 电机占空比环
 * 
 */
void Motor_DutyCtrl()
{
	int duty_ctrl[4] =
		{0};
	for (int i = 0; i < 4; i++)
	{
		duty_ctrl[i] = (int32_t)PID_GetOutput(&MotorPID[i],
											  MecanumChassis.motor[i].target_duty,
											  MecanumChassis.motor[i].now_duty);
	}
	if (MecanumChassis.send_ctrl_msg_flag)
	{
		Motor_SetDuty(duty_ctrl[0], duty_ctrl[1], duty_ctrl[2], duty_ctrl[3]);
		MecanumChassis.send_ctrl_msg_flag = 0;
	}
}

 #define DEBUG
/**
 * @brief 转速环
 */
void Motor_RpmCtrl(void)
{
	if (!MecanumChassis.motor_self_check_ok)
	{
		uprintf("## Please press key1 to take motor self check first! ##\r\n");
		return;
	}
	if (!EncoderDataUpdated) // 从UART3接收到了从单片机采集的后两轮的编码器值
	{
		// uprintf("## Can't get updated encoder value, check uart3 comm! ##\r\n");
		return;
	}
	int rpm_value[4] = {0};
	for (int i = 0; i < 4; i++)
	{
		rpm_value[i] = MecanumChassis.motor[i].now_rpm+ (int32_t)PID_GetIncrementOutput(&MotorPID[i],
											  MecanumChassis.motor[i].target_rpm,
											  MecanumChassis.motor[i].now_rpm);
#ifdef DEBUG
		if (wave_index == i)
		{
			if (TIM1_20ms_Flag)
			{
				uprintf("Tuning|[%d] now:%d target:%d err:%d ctrl:%d\r\n", i,
						MecanumChassis.motor[i].now_rpm,
						MecanumChassis.motor[i].target_rpm,
						MecanumChassis.motor[i].target_rpm - MecanumChassis.motor[i].now_rpm,
						rpm_value[i]);
			}
		}
#endif
	}
	int16 duty_ctrl[4];
	for (int i = 0; i < 4; i++)
	{
		duty_ctrl[i] = rpm_value[i] * (10000.0f / 314.0f);
	}
	if (MecanumChassis.send_ctrl_msg_flag)
	{
		Motor_SetDuty(duty_ctrl[0], duty_ctrl[1], duty_ctrl[2], duty_ctrl[3]);
		MecanumChassis.send_ctrl_msg_flag = 0;
	}
	EncoderDataUpdated = 0;
}

/**
 * @brief 将期望占空比转为期望转速
 */
int16 Motor_TargetDuty2TargetRpm(int16 duty)
{
	int16 result = 0;
	if (duty > 0)
	{
		result = (int16)((duty * 1.0) / 10000 * encoder_max);
	}
	else
	{
		result = (int16)((duty * 1.0) / 10000 * -encoder_min);
	}
	return result;
}

/**
 * @brief 向驱动轮电调发送速度命令
 *
 * @param
 * @note
 */
void Motor_SetDuty(int32_t duty1, int32_t duty2, int32_t duty3, int32_t duty4)
{
	//对占空比限幅
	__LIMIT(duty1, PWM_DUTY_MAX);
	__LIMIT(duty2, PWM_DUTY_MAX);
	__LIMIT(duty3, PWM_DUTY_MAX);
	__LIMIT(duty4, PWM_DUTY_MAX);

	// 设置占空比为 百分之 (1000/TIMER1_PWM_DUTY_MAX*100)
	if (0 <= duty1) //电机1   正转
	{
		gpio_set(MOTOR1_A, FORWARD_ROTATE);
	}
	else //电机1   反转
	{
		gpio_set(MOTOR1_A, REVERSE_ROTATE);
	}
	pwm_duty(MOTOR1_B, abs(duty1));

	if (0 <= duty2) //电机2   正转
	{
		gpio_set(MOTOR2_A, FORWARD_ROTATE);
	}
	else //电机2   反转
	{
		gpio_set(MOTOR2_A, REVERSE_ROTATE);
	}
	pwm_duty(MOTOR2_B, abs(duty2));

	if (0 <= duty3) //电机3   正转
	{
		gpio_set(MOTOR3_A, FORWARD_ROTATE);
	}
	else //电机3   反转
	{
		gpio_set(MOTOR3_A, REVERSE_ROTATE);
	}
	pwm_duty(MOTOR3_B, abs(duty3));

	if (0 <= duty4) //电机4   正转
	{
		gpio_set(MOTOR4_A, FORWARD_ROTATE);
	}
	else //电机4   反转
	{
		gpio_set(MOTOR4_A, REVERSE_ROTATE);
	}
	pwm_duty(MOTOR4_B, abs(duty4));
}

void DriveMotors_LimitSpeed(float speed[4])
{
	for (int i = 0; i < 4; i++)
	{
		__LIMIT_FROM_TO(speed[i], DRIVE_WHEEL_MIN_SPEED, DRIVE_WHEEL_MAX_SPEED);
	}
}
