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

PathFollowStatus_t PathFollowStatus;

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
	YawPID.kd = 0.00;
	YawPID.ctrl_max = 10;
	YawPID.int_duty = 0.01;
	YawPID.int_max = 5;
	YawPID.use_sub_pid = 0;
	YawPID.sub_pid_thres = 2;
	YawPID.sub_pid_kp = 0.8;
	HeadingAnglePID.ctrl_max = 5;

	PathFollowStatus.begin = 0;
	PathFollowStatus.meet_cross = 0;
	PathFollowStatus.meet_garage = 0;
	PathFollowStatus.meet_island_first = 0;
	PathFollowStatus.meet_island_second = 0;
	PathFollowStatus.meet_left_fork = 0;
	PathFollowStatus.meet_right_fork = 0;
	PathFollowStatus.meet_left_big_curve = 0;
	PathFollowStatus.meet_right_big_curve = 0;
	PathFollowStatus.image_process_done = 0;
	PathFollowStatus.curve_speed = 0.2;
	PathFollowStatus.forthright_speed = 0.45;
}

/**
 * @brief Ѳ�߿���״̬��
 * TODO:δ���
 */
void PathFollowing_Exe()
{
	if (!(PathFollowStatus.begin && UART3_RxOK))
	{
		MecanumChassis.target_speed = 0;
		MecanumChassis.target_omega = 0;
		return;
	}
	float omega_ctrl = PID_GetOutput(&HeadingAnglePID, 0,
			PathFollowStatus.path_diff_angle);
	MecanumChassis.target_omega = omega_ctrl;
	//	if (TIM1_100ms_Flag)
	//	{
	//		uprintf("PathFollow|diff_ang:%6.2f omega_ctrl:%5.2f \r\n",
	//				PathFollowStatus.path_diff_angle, MecanumChassis.target_omega);
	//	}
	MecanumChassis.target_speed = PathFollowStatus.forthright_speed
			- fabs(PathFollowStatus.path_diff_angle) / 5.2;
	if (MecanumChassis.target_speed < 0.3)
	{
		MecanumChassis.target_speed = 0.3;
	}
//		if (fabs(PathFollowStatus.path_diff_angle) > PathFollowStatus.angle_thres)
//		{
//			MecanumChassis.target_speed = PathFollowStatus.curve_speed;
//		}
//		else
//		{
//			MecanumChassis.target_speed = PathFollowStatus.forthright_speed;
//		}
	UART3_RxOK = 0;

}