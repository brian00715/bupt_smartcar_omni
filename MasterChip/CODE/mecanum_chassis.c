/**
 * @file chassis.c
 * @author simon
 * @brief 底盘控制相关
 * @version 0.1
 * @date 2021-04-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "mecanum_chassis.h"
#include "config.h"
#include "sci_compute.h"
#include "motor.h"

#define MECANUM_X_ASSM
// ===========================================Private==============================================

//static const int prvOmegaRatio[4][2] =
//	{
//		{-1, -1},
//		{-1, 1},
//		{1, 1},
//		{1, -1}}; // 计算角速度与线速度的换算时所需系数
//#ifdef MECANUM_X_ASSM
//// 麦克纳姆轮辊子轴向方向(X型布置)，顺序顺时针0123
//static const float prvAssembleDir[4][2] =
//	{
//		{0.707, 0.707},
//		{-0.707, 0.707},
//		{0.707, 0.707},
//		{-0.707, 0.707}};
//#endif
//#ifdef MECANUM_O_ASSM
//// 麦克纳姆轮辊子方向(O型布置)
//static const float prvAssembleDir[4][2] =
//	{
//		{0.707, -0.707},
//		{-0.707, -0.707},
//		{-0.707, 0.707},
//		{0.707, 0.707}};
//#endif
// =============================================END================================================

// ===========================================Public==============================================
BaseChassis_t MecanumChassis; // 底盘控制全局结构体

/**
 * @brief 底盘初始化
 * 
 */
void MecanumChassis_Init(void)
{
	Motor_Init();
	for (int i = 0; i < 4; i++)
	{
		MecanumChassis.motor[i].last_duty = 0;
		MecanumChassis.motor[i].now_duty = 0;
		MecanumChassis.motor[i].target_duty = 0;
	}
	MecanumChassis.send_ctrl_msg_flag = 0;
	MecanumChassis.motor_self_check_ok = 0;
	MecanumChassis.target_dir = 0;
	MecanumChassis.target_speed = 0;
	MecanumChassis.target_omega = 0;
	MecanumChassis.target_yaw = 0;
	MecanumChassis.pos_mode = POS_MODE_RELATIVE;
	MecanumChassis.ctrl_mode = CTRL_MODE_NONE;
	PostureStatusInit();
}

/** @brief  底盘位姿状态初始化*/
void PostureStatusInit(void)
{
	MecanumChassis.PostureStatus.omega = 0;
	MecanumChassis.PostureStatus.speed = 0;
	MecanumChassis.PostureStatus.yaw = 0;
}

/**
 * @brief 全向移动运动学
 * @param speed 速度大小(m/s)
 * @param dir 速度方向/rad
 * @param omega 自转角速度(rad/s)，逆时针为正方向
 * @note 根据速度矢量计算出四个轮的转速
 **/
int MecanumChassis_OmniDrive(float speed, float dir, float omega)
{
	// float absolute_angle_offset = 0; //使用绝对坐标时，根据全场定位测得的偏航角进行补偿
	// if (MecanumChassis.pos_mode == POS_MODE_ABSOLUTE)
	// {
	// 	absolute_angle_offset = MecanumChassis.PostureStatus.yaw - 1.5708; // 车头初始方向90°
	// 	if (absolute_angle_offset > PI)
	// 	{
	// 		absolute_angle_offset = -(2 * PI - absolute_angle_offset);
	// 	}
	// }
	// dir -= absolute_angle_offset;

	if (MecanumChassis.pos_mode == POS_MODE_ABSOLUTE)
	{
		dir -= MecanumChassis.PostureStatus.yaw;
	}

	float vx = speed * cos(dir); // 速度分量
	float vy = speed * sin(dir);
	__LIMIT(omega, MAX_ROTATE_VEL); // omega需要参与运算，故提前限制大小
	float target_speed[4]; // 每个车轮的切向速度
	float a = WHEEL_LEFT2RIGHT / 2, b = WHEEL_FRONT2BACK / 2;

	//>>>直接计算法<<<
	target_speed[0] = vx + vy - omega * (a + b);
	target_speed[1] = -vx + vy + omega * (a + b);
	target_speed[2] = vx + vy + omega * (a + b);
	target_speed[3] = -vx + vy - omega * (a + b);

	// >>>公式计算法<<<
	// for (int i = 0; i < 4; i++)
	// {
	// 	float self_turn_vel_x = omega * WHEEL_FRONT2BACK / 2; // 自转切向速度的分量
	// 	float self_turn_vel_y = omega * WHEEL_LEFT2RIGHT / 2;
	// 	float vel_sum_x = vx + prvOmegaRatio[i][0] * self_turn_vel_x; // 速度合成
	// 	float vel_sum_y = vy + prvOmegaRatio[i][1] * self_turn_vel_y;
	// 	float v_roller = vel_sum_x * prvAssembleDir[i][0] + vel_sum_y * prvAssembleDir[i][1]; // 辊子的切向速度
	// 	float v_wheel = v_roller / cos(__ANGLE2RAD(45));										  // 转为轮子的速度
	// 	target_speed[i] = v_wheel;
	// }

	// 数值限制
	DriveMotors_LimitSpeed(target_speed);

	//m/s转为占空比
	for (int i = 0; i < 4; i++)
	{
	    float target_omega = target_speed[i]/DRIVE_WHEEL_RADIUS;// target_speed单位m/s
		MecanumChassis.motor[i].target_rpm = target_omega*30/PI;
	}

	Motor_RpmCtrl();
}

float drift_damp_coff = 0.85; // 遏止转向漂移的阻尼系数
/**
 * @brief 差分运动学
 * @param speed 速度大小
 * @param omega 角速度(rad/s)
 */
void MecanumChassis_DiffDrive(float speed, float omega)
{
	float left_wheel_speed, right_wheel_speed;
	left_wheel_speed = speed - omega * WHEEL_LEFT2RIGHT / 2;
	right_wheel_speed = speed + omega * WHEEL_LEFT2RIGHT / 2;
	float target_speed[4];
	target_speed[0] = left_wheel_speed;
	target_speed[1] = right_wheel_speed;
	target_speed[2] = right_wheel_speed * drift_damp_coff;
	target_speed[3] = left_wheel_speed * drift_damp_coff;
	for (int i = 0; i < 4; i++)
	{
		__LIMIT(target_speed[i], MAX_ROTATE_VEL);
	}

	for (int i = 0; i < 4; i++)
	{
		MecanumChassis.motor[i].target_rpm = target_speed[i] * 100;
	}

	Motor_RpmCtrl();
}

/**
 * @brief 底盘控制状态机
 * 
 */
void MecanumChassis_Exe()
{
	if (!MecanumChassis.motor_self_check_ok)
		return;
	switch (MecanumChassis.ctrl_mode)
	{
	case CTRL_MODE_NONE:
		break;
	case CTRL_MODE_CMD:
		Motor_SetDuty(MecanumChassis.motor[0].target_duty,
					  MecanumChassis.motor[1].target_duty,
					  MecanumChassis.motor[2].target_duty,
					  MecanumChassis.motor[3].target_duty);
		break;
	case CTRL_MODE_OMNI:
		MecanumChassis_OmniDrive(MecanumChassis.target_speed,
								 MecanumChassis.target_dir, MecanumChassis.target_omega);

		break;
	case CTRL_MODE_DIFF:
		MecanumChassis_DiffDrive(MecanumChassis.target_speed, MecanumChassis.target_omega);
		break;
	case CTRL_MODE_TUNING:
		Motor_RpmCtrl();
		break;
	default:
		break;
	}
}

// =============================================END================================================
