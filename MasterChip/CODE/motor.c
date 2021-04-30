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
#include "pid.h"
#include "mecanum_chassis.h"

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
}
// kp ki kd int_duty int_max int_sum last_err last_delta_err
PID_t MotorPID = {0.1, 0.01, 0.0, 10, 10, 0, 0, 0};
extern Chassis_t MecanumChassis;
/**
 * @brief 电机占空比环
 * 
 * @param pid 
 * @param now_rpm 
 * @param expect_speed 
 */
void Motor_DutyCtrl(int16 target_duty[4])
{
	int duty_ctrl[4] = {0};
	for (int i = 0; i < 4; i++)
	{
		duty_ctrl[i] = (int32_t)PID_GetOutput(&MotorPID, target_duty[i], MecanumChassis.motor[i].now_duty);
	}
//	Motor_SetDuty(duty_ctrl);
}

/**
 * @brief 向驱动轮电调发送速度命令
 *
 * @param
 * @note
 */
void Motor_SetDuty(int32_t duty1,int32_t duty2,int32_t duty3,int32_t duty4)
{
	//对占空比限幅
	LIMIT(duty1, PWM_DUTY_MAX);
	LIMIT(duty2, PWM_DUTY_MAX);
	LIMIT(duty3, PWM_DUTY_MAX);
	LIMIT(duty4, PWM_DUTY_MAX);

	// 设置占空比为 百分之 (1000/TIMER1_PWM_DUTY_MAX*100)
	if (0 <= duty1) //电机1   正转
	{
		gpio_set(MOTOR1_A, FORWARD_ROTATE);
	}
	else //电机1   反转
	{
		gpio_set(MOTOR1_A, REVERSE_ROTATE);
	}
	pwm_duty(MOTOR1_B, duty1);

	if (0 <= duty2) //电机2   正转
	{
		gpio_set(MOTOR2_A, FORWARD_ROTATE);
	}
	else //电机2   反转
	{
		gpio_set(MOTOR2_A, REVERSE_ROTATE);
	}
	pwm_duty(MOTOR2_B, duty2);

	if (0 <= duty3) //电机3   正转
	{
		gpio_set(MOTOR3_A, FORWARD_ROTATE);
	}
	else //电机3   反转
	{
		gpio_set(MOTOR3_A, REVERSE_ROTATE);
	}
	pwm_duty(MOTOR3_B, duty3);

	if (0 <= duty4) //电机4   正转
	{
		gpio_set(MOTOR4_A, FORWARD_ROTATE);
	}
	else //电机4   反转
	{
		gpio_set(MOTOR4_A, REVERSE_ROTATE);
	}
	pwm_duty(MOTOR4_B,duty4);
}

/**
 * @brief 将底盘速度m/s转为发送给驱动板的占空比
 *
 * @param speed m/s
 * @return
 */
int SW_Speed2PWM(float speed)
{
}

void DriveMotors_LimitSpeed(float speed[4])
{
	for (int i = 0; i < 4; i++)
	{
		LIMIT_FROM_TO(speed[i], DRIVE_WHEEL_MIN_SPEED, DRIVE_WHEEL_MAX_SPEED);
	}
}
