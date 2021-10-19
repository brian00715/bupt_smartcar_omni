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
int32 stop_time_count = 0;
uint8 stop_flag = 0;
uint8 leftround_finish = 0;
uint8 leftround_begin = 0;
uint8 rightround_finish = 0;
uint8 rightround_begin = 0;
uint8 leftsancha_done = 0;
uint8 rightsancha_done = 0;
int32 circle_count = 0;
int32 circle_ass = 0;
/*
 两种驱动方案：
 1. 差分驱动
 用赛道斜率与车头正方向的偏差角作为输入量，输出差分驱动角速度
 2. 偏航角驱动
 用赛道斜率与车头正方向的偏差角作为输入量，输出偏航角的角速度
 */

PID_t HeadingPID; // 差分驱动时航向角PID
PID_t YawPID;	  // 偏航角驱动时的PID
void PathFollowing_Init()
{
	// HeadingAnglePID.kp = 1.8;
	// HeadingAnglePID.ki = 0.0;
	// HeadingAnglePID.kd = 0.3;
	// HeadingAnglePID.ctrl_max = 10;
	// HeadingAnglePID.int_duty = 0.01;
	// HeadingAnglePID.int_max = 5;
	// HeadingAnglePID.use_sub_pid = 0;
	// HeadingAnglePID.sub_pid_thres = 2;
	// HeadingAnglePID.sub_pid_kp = 0.8;

	HeadingPID.kp = 0.038;
	HeadingPID.ki = 0.0;
	HeadingPID.kd = 0.01;
	HeadingPID.ctrl_max = 10;
	HeadingPID.int_duty = 0.01;
	HeadingPID.int_max = 5;
	HeadingPID.use_sub_pid = 0;
	HeadingPID.sub_pid_thres = 2;
	HeadingPID.sub_pid_kp = 0.8;

	YawPID.kp = 1;
	YawPID.ki = 0.03;
	YawPID.kd = 0.0;
	YawPID.ctrl_max = 3;
	YawPID.int_duty = 0.5;
	YawPID.int_max = 5;
	YawPID.use_sub_pid = 0;
	YawPID.sub_pid_thres = 2;
	YawPID.sub_pid_kp = 0.8;

	MecanumChassis.PathFollowing.state = PATH_FOLLOW_NONE;
	MecanumChassis.PathFollowing.begin = 0;
	MecanumChassis.PathFollowing.image_process_done = 0;
	MecanumChassis.PathFollowing.curve_speed = 0.2;
	MecanumChassis.PathFollowing.forward_speed = 0.45;
}
static uint32 fork_delay_start = 0;
static uint8 fork_count_flag = 0;
static uint8 round_int_flag = 0;	  // 环岛偏航角积分标志
static uint8 fork_turn_done_once = 0; // 保证出三叉时yaw只被复位一次
/**
 * @brief 巡线控制状态机
 */
void PathFollowing_Exe()
{
	if (!(MecanumChassis.PathFollowing.begin && UART3_RxOK))
	{
		return;
	}
	if (stop_flag == 0)
	{
		float tmp_speed = 0.5; //应首先赋normal状态下的初值，否则在识别到特殊元素时速度仍为0
		// 偏航角修正
		if (fabs(MecanumChassis.PathFollowing.heading_err) < 5) // 死区
		{
			MecanumChassis.PathFollowing.heading_err = 0;
		}
		float omega_ctrl = PID_GetOutput(&HeadingPID, 0.0f, MecanumChassis.PathFollowing.heading_err);
		if (fabs(MecanumChassis.PathFollowing.heading_err) < __ANGLE2RAD(1))
		{
			omega_ctrl = 0;
		}

		// 三岔路偏航角闭环
		if (MecanumChassis.PathFollowing.en_turn_ctrl)
		{
			if (TIM1_100ms_Flag)
			{
				// uprintf("en turn ctrl! yaw:%6.2f tar_yaw:%6.2f cam_duty:%d\r\n", MecanumChassis.PostureStatus.yaw,
				// MecanumChassis.target_yaw, MecanumChassis.cam_servo_duty);
			}
			omega_ctrl = PID_GetOutput(&YawPID, MecanumChassis.target_yaw, MecanumChassis.PostureStatus.yaw);
			if (fabs(MecanumChassis.target_yaw - MecanumChassis.PostureStatus.yaw) <= __ANGLE2RAD(2)) // 死区
			{
				tmp_speed = 0.3;
				switch (MecanumChassis.PathFollowing.state)
				{
				case PATH_FOLLOW_LEFT_FORK_EXPORT:
					MecanumChassis.target_dir = 3.14;
					break;
				case PATH_FOLLOW_LEFT_FORK_DONE:
					MecanumChassis.target_dir = 3.14;
					break;
				default:
					break;
				}
				MecanumChassis.PathFollowing.en_turn_ctrl = 0;
				MecanumChassis.PathFollowing.fork_turn_done = 1;
				YawPID.int_sum = 0;
				uprintf("turn done.\r\n");
			}
			else
			{
				tmp_speed = 0;
				MecanumChassis.cam_servo_duty = (uint32)(fabs(MecanumChassis.PostureStatus.yaw) * CAM_SERVO_DUTY_PER_RAD) + CAM_SERVO_0_DUTY;
			}
		}

		MecanumChassis.target_omega = omega_ctrl;

		switch (MecanumChassis.PathFollowing.state)
		{
		case PATH_FOLLOW_NONE:
			break;
		case PATH_FOLLOW_NORMAL:
			MecanumChassis.PathFollowing.en_turn_ctrl = 0;
			MecanumChassis.cam_servo_duty = CAM_SERVO_0_DUTY;
			MecanumChassis.target_dir = 1.5708;
			/******左环岛*******/
			if (leftround_begin == 1 && leftround_finish == 0) //左环岛还没转过300度
			{
				/*****************左小环岛******************/
				if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(45) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(15))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.20, 999);
					tmp_speed = 0.45;
				}
				else if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(90) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(45))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.70, 999);
					tmp_speed = 0.45;
				}
				else if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(135) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(90))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.58, 999);
					tmp_speed = 0.43;
				}
				else if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(225) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(135))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.45, 999);
					tmp_speed = 0.45;
				}
				else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(225) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(270))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.75, 999);
					tmp_speed = 0.55;
				}
				else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(270) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(320))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.80, 999);
					tmp_speed = 0.50;
				}
				else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(320) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(350))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.1, 0.30);
					tmp_speed = 0.60;
				}
				/**************是否转过350度***************/
				if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(350))
					leftround_finish = 1;

				/******************左大环岛*********************/
				// if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(135) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(45))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.6, 999);
				// 	tmp_speed = 0.66;
				// }
				// else if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(225) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(135))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.6, 999);
				// 	tmp_speed = 0.66;
				// }
				// else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(225) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(300))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.65, 999);
				// 	tmp_speed = 0.73;
				// }
				// else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(300) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(330))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.10, 999);
				// 	tmp_speed = 0.80;
				// }
				// else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(330) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(350))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.05, 0.35);
				// 	tmp_speed = 0.80;
				// }
				// if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(350))
				// {
				// 	if (circle_ass == 0)
				// 		circle_ass = time_count;
				// }
				// if (time_count - circle_ass <= 200 && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(350) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(360))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -0.05, 0.2);
				// 	tmp_speed = 0.82;
				// }

				// /**************是否转过360度***************/
				// if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(350) && (time_count - circle_ass >= 200 || MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(360)))
				// {
				// 	if (circle_count == 0)
				// 		circle_count = time_count;
				// 	if (time_count - circle_count <= 100)
				// 	{
				// 		tmp_speed = 0.7;
				// 		MecanumChassis.target_omega = 0;
				// 	}
				// 	else
				// 	{
				// 		leftround_finish = 1;
				// 		circle_ass = 0;
				// 		circle_count = 0;
				// 	}
				// }
			}
			else if (leftround_begin == 1 && leftround_finish == 1)
			{
				leftround_begin = 0;
				leftround_finish = 0;
			}

			/******右环岛*******/
			if (rightround_begin == 1 && rightround_finish == 0) //左环岛还没转过300度
			{

				// /*****************右小环岛******************/
				// if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-90) && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-45))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.63);
				// 	tmp_speed = 0.50;
				// }
				// else if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-135) && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-90))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.48);
				// 	tmp_speed = 0.50;
				// }
				// else if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-225) && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-135))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.48);
				// 	tmp_speed = 0.52;
				// }
				// else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-225) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-320))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.78);
				// 	tmp_speed = 0.52;
				// }
				// else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-320) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-350))
				// {
				// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.10);
				// 	tmp_speed = 0.52;
				// }
				/**************是否转过350度***************/
				// if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(-350))
				// 	rightround_finish = 1;

				/******************右大环岛*********************/
				if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-135) && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-45))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.6);
					tmp_speed = 0.66;
				}
				else if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-225) && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-135))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.6);
					tmp_speed = 0.66;
				}
				else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-225) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-300))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.65);
					tmp_speed = 0.73;
				}
				else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-300) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-330))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.10);
					tmp_speed = 0.80;
				}
				else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-330) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-350))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, -0.35, -0.05);
					tmp_speed = 0.80;
				}
				if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(-350))
				{
					if (circle_ass == 0)
						circle_ass = time_count;
				}
				if (time_count - circle_ass <= 200 && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-350) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-360))
				{
					__LIMIT_FROM_TO(MecanumChassis.target_omega, -0.2, 0.05);
					tmp_speed = 0.82;
				}

				/**************是否转过360度***************/
				if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(-350) && (time_count - circle_ass >= 200 || MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(-360)))
				{
					if (circle_count == 0)
						circle_count = time_count;
					if (time_count - circle_count <= 100)
					{
						tmp_speed = 0.7;
						MecanumChassis.target_omega = 0;
					}
					else
					{
						rightround_finish = 1;
						circle_ass = 0;
						circle_count = 0;
					}
				}
			}
			else if (rightround_begin == 1 && rightround_finish == 1)
			{
				rightround_begin = 0;
				rightround_finish = 0;
			}
			if (leftround_begin == 0 && leftround_finish == 0 && rightround_begin == 0 && rightround_finish == 0)
			{
				round_int_flag = 0;
				tmp_speed = 0.52;
			}
			break;

		case PATH_FOLLOW_OUT_GARAGE:
			// 老方法
			// MecanumChassis.target_dir = 3.14;
			// tmp_speed = 0.2;
			// MecanumChassis.target_omega = 0;
			break;

		// 三岔路----------------------------------------------------------------
		//左三岔
		case PATH_FOLLOW_LEFT_FORK: // 岔路两个动作：车身转90°  摄像头转90°
			fork_delay_start = systick_getval_ms();
			fork_count_flag = 1;
			break;
		case PATH_FOLLOW_LEFT_FORK_EXPORT:
			if (fork_count_flag)
			{
				uint32 now_time = systick_getval_ms();
				if (now_time - fork_delay_start > 200) // 延时200ms
				{
					uprintf("fork delay finished!\r\n");
					fork_count_flag = 0;
					tmp_speed = 0; // 测试用
					MecanumChassis.PathFollowing.en_turn_ctrl = 1;
					MecanumChassis.PathFollowing.fork_turn_done = 0;
					MecanumChassis.target_yaw = __ANGLE2RAD(-90);
					MecanumChassis.PostureStatus.yaw = 0; // 直接让当前值为0
					YawPID.int_sum = 0;
				}
			}
			if (MecanumChassis.PathFollowing.fork_turn_done) // 已经完成
			{
				// uprintf("turn done in switch.\r\n");
				tmp_speed = 0.3;
				MecanumChassis.target_dir = 3.14;
				MecanumChassis.cam_servo_duty = CAM_SERVO_90_DUTY; // 不管车身正不正，摄像头要打正
			}
			break;
		case PATH_FOLLOW_LEFT_FORK_DONE:
			tmp_speed = 0; // 测试用
			MecanumChassis.PathFollowing.en_turn_ctrl = 1;
			MecanumChassis.PathFollowing.fork_turn_done = 0;
			MecanumChassis.PostureStatus.yaw = 0; // 直接让当前值为0
			MecanumChassis.target_yaw = __ANGLE2RAD(90);
			YawPID.int_sum = 0;
			if (MecanumChassis.PathFollowing.fork_turn_done) // 已经完成
			{
				tmp_speed = 0.4;
				MecanumChassis.target_dir = 1.5708;
				MecanumChassis.cam_servo_duty = CAM_SERVO_0_DUTY;
			}
			break;
		case PATH_FOLLOW_LEFT_FORK_DONE2:
			if (MecanumChassis.PathFollowing.fork_turn_done) // 已经完成
			{
				tmp_speed = 0.4;
				MecanumChassis.target_dir = 1.5708;
				MecanumChassis.cam_servo_duty = CAM_SERVO_0_DUTY;
			}
			break;
		//右三岔
		case PATH_FOLLOW_RIGHT_FORK: // 岔路两个动作：车身转90°  摄像头转90°
			fork_delay_start = systick_getval_ms();
			fork_count_flag = 1;
			break;
		case PATH_FOLLOW_RIGHT_FORK_EXPORT:
			if (fork_count_flag)
			{
				uint32 now_time = systick_getval_ms();
				if (now_time - fork_delay_start > 200) // 延时200ms
				{
					uprintf("fork delay finished!\r\n");
					fork_count_flag = 0;
					tmp_speed = 0; // 测试用
					MecanumChassis.PathFollowing.en_turn_ctrl = 1;
					MecanumChassis.PathFollowing.fork_turn_done = 0;
					MecanumChassis.target_yaw = __ANGLE2RAD(-90);
					MecanumChassis.PostureStatus.yaw = 0; // 直接让当前值为0
					YawPID.int_sum = 0;
				}
			}
			if (MecanumChassis.PathFollowing.fork_turn_done) // 已经完成
			{
				// uprintf("turn done in switch.\r\n");
				tmp_speed = 0.3;
				MecanumChassis.target_dir = 3.14;
				MecanumChassis.cam_servo_duty = CAM_SERVO_90_DUTY; // 不管车身正不正，摄像头要打正
			}
			break;
		case PATH_FOLLOW_RIGHT_FORK_DONE:
			tmp_speed = 0; // 测试用
			MecanumChassis.PathFollowing.en_turn_ctrl = 1;
			MecanumChassis.PathFollowing.fork_turn_done = 0;
			MecanumChassis.PostureStatus.yaw = 0; // 直接让当前值为0
			MecanumChassis.target_yaw = __ANGLE2RAD(90);
			YawPID.int_sum = 0;
			if (MecanumChassis.PathFollowing.fork_turn_done) // 已经完成
			{
				tmp_speed = 0.4;
				MecanumChassis.target_dir = 1.5708;
				MecanumChassis.cam_servo_duty = CAM_SERVO_0_DUTY;
			}
			break;
		case PATH_FOLLOW_RIGHT_FORK_DONE2:
			if (MecanumChassis.PathFollowing.fork_turn_done) // 已经完成
			{
				tmp_speed = 0.4;
				MecanumChassis.target_dir = 1.5708;
				MecanumChassis.cam_servo_duty = CAM_SERVO_0_DUTY;
			}
			break;

		// 目前是左环岛小，右环岛大，到时候根据赛场布置交换参数
		case PATH_FOLLOW_LEFT_ROUND:
			if (round_int_flag == 0)
			{
				round_int_flag = 1; // 识别到环岛就开始积分
				MecanumChassis.PostureStatus.yaw = 0;
			}
			leftround_begin = 1;
			break;
		case PATH_FOLLOW_LEFT_ROUND_IN:
			/******************左小环岛*********************/
			if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(45) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(15))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.20, 999);
				tmp_speed = 0.45;
			}
			else if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(90) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(45))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.70, 999);
				tmp_speed = 0.45;
			}
			else if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(135) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(90))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.58, 999);
				tmp_speed = 0.43;
			}
			else if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(225) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(135))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.45, 999);
				tmp_speed = 0.45;
			}
			else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(225) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(270))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.75, 999);
				tmp_speed = 0.55;
			}
			else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(270) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(320))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.80, 999);
				tmp_speed = 0.50;
			}
			else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(320) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(350))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.1, 0.30);
				tmp_speed = 0.60;
			}
			/******************左大环岛*********************/
			// if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(135) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(45))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.6, 999);
			// 	tmp_speed = 0.68;
			// }
			// else if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(225) && MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(135))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.6, 999);
			// 	tmp_speed = 0.68;
			// }
			// else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(225) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(300))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.58, 999);
			// 	tmp_speed = 0.75;
			// }
			// else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(300) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(330))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.10, 999);
			// 	tmp_speed = 0.82;
			// }
			// else if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(330) && MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(350))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, 0.05, 0.15);
			// 	tmp_speed = 0.82;
			// }
			/*******************是否转过350度***********************/
			if (MecanumChassis.PostureStatus.yaw > __ANGLE2RAD(350))
				leftround_finish = 1;
			// HeadingPID.kp = 0.06;
			// tmp_speed = 0.48;
			break;
		case PATH_FOLLOW_LEFT_ROUND_ENTRANCE_FLAG:
			tmp_speed = 0.48;
			break;
		case PATH_FOLLOW_LEFT_ROUND_EXPORT: // 环岛出口
			// MecanumChassis.target_omega += 0.4; // 补偿，转的再狠一点
			HeadingPID.kp = 0.06;
			tmp_speed = 0.39;
			break;
		case PATH_FOLLOW_LEFT_ROUND_OUT_FLAG:
			HeadingPID.kp = 0.035;
			tmp_speed = 0.52;
			break;
		case PATH_FOLLOW_RIGHT_ROUND:
			if (round_int_flag == 0)
			{
				round_int_flag = 1; // 识别到环岛就开始积分
				MecanumChassis.PostureStatus.yaw = 0;
			}
			rightround_begin = 1;
			break;
		case PATH_FOLLOW_RIGHT_ROUND_IN:
			// if(TIM1_20ms_Flag)
			// {
			// 	uprintf("yaw:%.2f\r\n",MecanumChassis.PostureStatus.yaw);
			// }
			/*****************右小环岛******************/
			// if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-90) && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-45))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.63);
			// 	tmp_speed = 0.50;
			// }
			// else if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-135) && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-90))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.48);
			// 	tmp_speed = 0.50;
			// }
			// else if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-225) && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-135))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.48);
			// 	tmp_speed = 0.52;
			// }
			// else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-225) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-320))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.78);
			// 	tmp_speed = 0.52;
			// }
			// else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-320) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-350))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.10);
			// 	tmp_speed = 0.52;
			// }
			/******************右大环岛*********************/
			if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-135) && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-45))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.6);
				tmp_speed = 0.66;
			}
			else if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-225) && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-135))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.6);
				tmp_speed = 0.66;
			}
			else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-225) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-300))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.65);
				tmp_speed = 0.73;
			}
			else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-300) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-330))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, -0.10);
				tmp_speed = 0.80;
			}
			else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-330) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-350))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, -0.35, -0.05);
				tmp_speed = 0.80;
			}
			if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(-350))
			{
				if (circle_ass == 0)
					circle_ass = time_count;
			}
			if (time_count - circle_ass <= 200 && MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-350) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-360))
			{
				__LIMIT_FROM_TO(MecanumChassis.target_omega, -0.2, 0.05);
				tmp_speed = 0.82;
			}
			// else if (MecanumChassis.PostureStatus.yaw < __ANGLE2RAD(-350) && MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(-360))
			// {
			// 	__LIMIT_FROM_TO(MecanumChassis.target_omega, -0.2, 0.05);
			// 	tmp_speed = 0.82;
			// }
			/**************是否转过350度***************/
			if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(-350) && (time_count - circle_ass >= 200 || MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(-360)))
			{
				if (circle_count == 0)
					circle_count = time_count;
				if (time_count - circle_count <= 100)
				{
					tmp_speed = 0.7;
					MecanumChassis.target_omega = 0;
				}
				else
				{
					rightround_finish = 1;
					circle_ass = 0;
					circle_count = 0;
				}
			}
			// HeadingPID.kp = 0.06;
			// tmp_speed = 0.5;
			break;
		case PATH_FOLLOW_RIGHT_ROUND_EXPORT:	// 环岛出口
			MecanumChassis.target_omega -= 0.8; // 补偿，转的再狠一点;右环岛此时omega为负
			HeadingPID.kp = 0.08;
			tmp_speed = 0.4;
			break;
		case PATH_FOLLOW_RIGHT_ROUND_OUT_FLAG: // 2220出环岛，准备直走
			HeadingPID.kp = 0.035;
			tmp_speed = 0.52;
			break;
		case PATH_FOLLOW_STOP:
			stop_flag = 1;
			// if (stop_time_count == 0)
			// 	stop_time_count = time_count;
			// if (time_count - stop_time_count <= 10)
			// {
			// 	tmp_speed = 0.52;
			// 	MecanumChassis.target_omega = 0;
			// 	MecanumChassis.target_dir = 1.5708;
			// }
			// else if (time_count - stop_time_count <= 100)
			// {
			// 	tmp_speed = 0.52;
			// 	MecanumChassis.target_omega = -1.61;
			// 	MecanumChassis.target_dir = 1.5708;
			// }
			// else if (time_count - stop_time_count <= 125)
			// {
			// 	tmp_speed = 0.52;
			// 	MecanumChassis.target_omega = 0;
			// 	MecanumChassis.target_dir = 1.5708;
			// }
			// else
			// {
			// 	tmp_speed = 0.0;
			// 	MecanumChassis.target_omega = 0.0;
			// 	MecanumChassis.target_dir = 0;
			// }
			break;
		default:
			break;
		}

		// if (round_int_flag)
		// {
		// 	uprintf("yaw:%6.3f omega:%6.3f\r\n", MecanumChassis.PostureStatus.yaw, MecanumChassis.target_omega);
		// 	if (MecanumChassis.PostureStatus.yaw >= __ANGLE2RAD(365))
		// 	{
		// 		__LIMIT_FROM_TO(MecanumChassis.target_omega, -999, 0);
		// 	}
		// 	if (MecanumChassis.PostureStatus.yaw <= __ANGLE2RAD(-365))
		// 	{
		// 		__LIMIT_FROM_TO(MecanumChassis.target_omega, 0, 999);
		// 	}
		// }

		MecanumChassis.target_speed = tmp_speed;
	}

	UART3_RxOK = 0;
}
