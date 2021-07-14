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

	YawPID.kp = 0.038;
	YawPID.ki = 0.0;
	YawPID.kd = 0.01;
	YawPID.ctrl_max = 10;
	YawPID.int_duty = 0.01;
	YawPID.int_max = 5;
	YawPID.use_sub_pid = 0;
	YawPID.sub_pid_thres = 2;
	YawPID.sub_pid_kp = 0.8;
	YawPID.ctrl_max = 5;

	NormalPID.kp = 0.01;
	NormalPID.ki = 0.001;
	NormalPID.kd = 0.00;
	NormalPID.ctrl_max = 10;
	NormalPID.int_duty = 0.01;
	NormalPID.int_max = 5;
	NormalPID.use_sub_pid = 0;
	NormalPID.sub_pid_thres = 2;
	NormalPID.sub_pid_kp = 0.8;
	NormalPID.ctrl_max = 5;

	MecanumChassis.PathFollowing.state = PATH_FOLLOW_NONE;
	MecanumChassis.PathFollowing.begin = 0;
	MecanumChassis.PathFollowing.image_process_done = 0;
	MecanumChassis.PathFollowing.curve_speed = 0.2;
	MecanumChassis.PathFollowing.forward_speed = 0.45;
}

static float fork_now_yaw = 0;
static uint32 fork_delay_start = 0;
static uint8 fork_count_flag = 0;
static uint8 round_int_flag = 0; // 环岛偏航角积分标志
static float round_int_sum = 0;	 //环岛偏航角积分量
/**
 * @brief 巡线控制状态机
 * TODO:未完成
 */
void PathFollowing_Exe()
{
	if (!(MecanumChassis.PathFollowing.begin && UART3_RxOK))
	{
		return;
	}
	// pwm_duty(PWM1_CH2_A9, MecanumChassis.cam_servo_duty);

	// 偏航角修正
	if (fabs(MecanumChassis.PathFollowing.heading_err) < 5) // 死区
	{
		MecanumChassis.PathFollowing.heading_err = 0;
	}
	float omega_ctrl = PID_GetOutput(&YawPID, 0.0f, MecanumChassis.PathFollowing.heading_err);
	if (fabs(MecanumChassis.PathFollowing.heading_err) < __ANGLE2RAD(1))
	{
		omega_ctrl = 0;
	}

	// if (MecanumChassis.PathFollowing.en_turn_ctrl) // 三岔路偏航角闭环
	// {
	// 	omega_ctrl = PID_GetOutput(&YawPID, MecanumChassis.target_yaw, MecanumChassis.PostureStatus.yaw);
	// 	if (omega_ctrl < __ANGLE2RAD(5)) // 死区
	// 	{
	// 		MecanumChassis.PathFollowing.en_turn_ctrl = 0;
	// 	}
	// }

	// 横向修正
	// float normal_ctrl = -PID_GetOutput(&NormalPID, 0.0f, (float)(MecanumChassis.PathFollowing.normal_err));
	// if (fabs(MecanumChassis.PathFollowing.normal_err) < 2)
	// {
	// 	normal_ctrl = 0;
	// }
	// float spd_y = 0.2 * sin(1.5708);
	// float spd_x = 0.2 * cos(1.5708) + normal_ctrl;
	// // MecanumChassis.target_dir = atan2f(spd_y,spd_x);
	// MecanumChassis.target_dir = spd_x > 0 ? 0 : 3.14;
	// MecanumChassis.target_speed = fabs(spd_x);
	// uprintf("CMD|Normal err:%d ctrl:%.5f\r\n", MecanumChassis.PathFollowing.normal_err, normal_ctrl);

	MecanumChassis.target_omega = omega_ctrl;
	MecanumChassis.target_dir = 1.5708;

	switch (MecanumChassis.PathFollowing.state)
	{
	case PATH_FOLLOW_NONE:
		break;
	case PATH_FOLLOW_NORMAL:
		MecanumChassis.target_speed = 0.55;
		break;

	case PATH_FOLLOW_OUT_GARAGE:
		MecanumChassis.target_dir = 3.14;
		MecanumChassis.target_speed = 0.2;
		MecanumChassis.target_omega = 0;
		break;

	case PATH_FOLLOW_LEFT_FORK: // 岔路两个动作：车身转90°  摄像头转90°
		MecanumChassis.target_yaw = fork_now_yaw -= 1.5708;
		MecanumChassis.PathFollowing.en_turn_ctrl = 1;
		fork_delay_start = systick_getval_ms();
		fork_count_flag = 1;
		break;
	case PATH_FOLLOW_LEFT_FORK_EXPORT:
		// MecanumChassis.PathFollowing.forward_speed = 0;
		MecanumChassis.target_speed = 0.45;
		if (fork_count_flag)
		{
			uint32 now_time = systick_getval_ms();
			if (now_time - fork_delay_start > 200) // 延时200ms
			{
				uprintf("fork delay finished!\r\n");
				fork_count_flag = 0;
			}
		}
		else // 延时结束，开始转车身
		{
			MecanumChassis.target_speed = 0; // 测试用
		}

		MecanumChassis.cam_servo_duty = CAM_SERVO_90_DUTY;
		break;
	case PATH_FOLLOW_LEFT_FORK_OUT:
		MecanumChassis.target_speed = 0.45;
		MecanumChassis.cam_servo_duty = CAM_SERVO_0_DUTY;
		break;

	// 	目前是小环岛的参数
	case PATH_FOLLOW_LEFT_ROUND:
		round_int_flag =1;
		break;
	case PATH_FOLLOW_LEFT_ROUND_IN:
		YawPID.kp = 0.06;
		MecanumChassis.target_speed = 0.4;
		break;
	case PATH_FOLLOW_LEFT_ROUND_EXPORT:		// 环岛出口
		MecanumChassis.target_omega += 0.5; // 补偿，转的再狠一点
		MecanumChassis.target_speed = 0.32;
		YawPID.kp = 0.08;
		MecanumChassis.target_speed = 0.4;
	case PATH_FOLLOW_LEFT_ROUND_OUT_FLAG:
		YawPID.kp = 0.035;
		MecanumChassis.target_speed = 0.52;
		break;

	// 目前是左环岛小，右环岛大，到时候根据赛场布置交换参数
	case PATH_FOLLOW_RIGHT_ROUND_IN:
		YawPID.kp = 0.06;
		MecanumChassis.target_speed = 0.5;
		break;
	case PATH_FOLLOW_RIGHT_ROUND_EXPORT:	// 环岛出口
		MecanumChassis.target_omega -= 0.8; // 补偿，转的再狠一点;右环岛此时omega为负
		YawPID.kp = 0.08;
		MecanumChassis.target_speed = 0.4;
	case PATH_FOLLOW_RIGHT_ROUND_OUT_FLAG: // 出环岛，准备直走
		YawPID.kp = 0.035;
		MecanumChassis.target_speed = 0.52;
		break;
	}

	// MecanumChassis.target_speed = MecanumChassis.PathFollowing.forward_speed;

	UART3_RxOK = 0;
}
