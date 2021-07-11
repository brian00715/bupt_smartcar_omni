/**
 * @file path_following.c
 * @author simon
 * @brief 循迹相关
 * @version 0.1
 * @date 2021-05-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "path_following.h"
#include "pid.h"
#include "sci_compute.h"
#include "mecanum_chassis.h"
#include "isr.h"

/*
 两种驱动方案：
 1. 差分驱动
 用赛道斜率与车头正方向的偏差角作为输入量，输出差分驱动角速度
 2. 偏航角驱动
 用赛道斜率与车头正方向的偏差角作为输入量，输出偏航角的角速度
 */

PID_t HeadingAnglePID; // 差分驱动时航向角PID
PID_t YawPID;		   // 偏航角驱动时的PID
PID_t NormalPID;	   // 法向修正PID

void PathFollowing_Init()
{
	HeadingAnglePID.kp = 1.8;
	HeadingAnglePID.ki = 0.0;
	HeadingAnglePID.kd = 0.3;
	HeadingAnglePID.ctrl_max = 10;
	HeadingAnglePID.int_duty = 0.01;
	HeadingAnglePID.int_max = 5;
	HeadingAnglePID.use_sub_pid = 0;
	HeadingAnglePID.sub_pid_thres = 2;
	HeadingAnglePID.sub_pid_kp = 0.8;

	YawPID.kp = 0.2;
	YawPID.ki = 0.0;
	YawPID.kd = 0.05;
	YawPID.ctrl_max = 10;
	YawPID.int_duty = 0.01;
	YawPID.int_max = 5;
	YawPID.use_sub_pid = 0;
	YawPID.sub_pid_thres = 2;
	YawPID.sub_pid_kp = 0.8;
	YawPID.ctrl_max = 5;

	MecanumChassis.PathFollowing.state = PATH_FOLLOW_NONE;
	MecanumChassis.PathFollowing.begin = 0;
	MecanumChassis.PathFollowing.image_process_done = 0;
	MecanumChassis.PathFollowing.curve_speed = 0.2;
	MecanumChassis.PathFollowing.forthright_speed = 0.45;
}

/**
 * @brief 巡线控制状态机
 * TODO:未完成
 */
void PathFollowing_Exe()
{
	if (!(MecanumChassis.PathFollowing.begin && UART3_RxOK))
	{
		MecanumChassis.target_speed = 0;
		MecanumChassis.target_omega = 0;
		return;
	}

	switch (MecanumChassis.PathFollowing.state)
	{
	case PATH_FOLLOW_NONE:
		/* code */
		break;
	case PATH_FOLLOW_NORMAL:
		float omega_ctrl = PID_GetOutput(&HeadingAnglePID, 0,
										 MecanumChassis.PathFollowing.heading_err);
		MecanumChassis.target_omega = omega_ctrl;

		MecanumChassis.target_speed = MecanumChassis.PathFollowing.forthright_speed - fabs(MecanumChassis.PathFollowing.heading_err) / 5.2;
		if (MecanumChassis.target_speed < 0.3)
		{
			MecanumChassis.target_speed = 0.3;
		}
		break;
	case PATH_FOLLOW_MEET_LEFT_BIG_CURVE:
		break;
	case PATH_FOLLOW_MEET_RIGHT_BIG_CURVE:
		break;
	default:
		break;
	}

	UART3_RxOK = 0;
}
