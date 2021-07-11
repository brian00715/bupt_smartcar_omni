/**
 * @file path_following.c
 * @author simon
 * @brief ѭ�����
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
 ��������������
 1. �������
 ������б���복ͷ�������ƫ�����Ϊ���������������������ٶ�
 2. ƫ��������
 ������б���복ͷ�������ƫ�����Ϊ�����������ƫ���ǵĽ��ٶ�
 */

PID_t HeadingAnglePID; // �������ʱ�����PID
PID_t YawPID;		   // ƫ��������ʱ��PID
PID_t NormalPID;	   // ��������PID

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

	YawPID.kp = 0.5;
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

static float now_yaw = 0;
/**
 * @brief Ѳ�߿���״̬��
 * TODO:δ���
 */
void PathFollowing_Exe()
{
	if (!(MecanumChassis.PathFollowing.begin && UART3_RxOK))
	{
		return;
	}

	float omega_ctrl;
	omega_ctrl = PID_GetOutput(&YawPID, 0, MecanumChassis.PathFollowing.heading_err);
	if (fabs(MecanumChassis.PathFollowing.heading_err) < __ANGLE2RAD(1))
	{
		omega_ctrl = 0;
	}
	MecanumChassis.target_omega = omega_ctrl;
	MecanumChassis.target_dir = 1.5708;
	MecanumChassis.target_speed = 0.1;

	if (TIM1_20ms_Flag)
		uprintf("speed:%5.2f omega:%5.2f\r\n", MecanumChassis.target_speed, MecanumChassis.target_omega);

	//	MecanumChassis.target_speed = MecanumChassis.PathFollowing.forthright_speed - fabs(MecanumChassis.PathFollowing.heading_err) / 5.2;
	//	if (MecanumChassis.target_speed < 0.3)
	//	{
	//		MecanumChassis.target_speed = 0.3;
	//	}

	switch (MecanumChassis.PathFollowing.state)
	{
	case PATH_FOLLOW_NONE:
		/* code */
		break;
	case PATH_FOLLOW_NORMAL:

		break;
	case PATH_FOLLOW_MEET_LEFT_BIG_CURVE:
		break;
	case PATH_FOLLOW_MEET_RIGHT_BIG_CURVE:
		break;
	case PATH_FOLLOW_GO_LEFT_FORK:
		MecanumChassis.PathFollowing.state = PATH_FOLLOW_NORMAL;
		now_yaw = MecanumChassis.PostureStatus.yaw;
		MecanumChassis.target_yaw = now_yaw += 1.5708;
		break;
	default:
		break;
	}

	UART3_RxOK = 0;
}
