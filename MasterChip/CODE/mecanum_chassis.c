/**
 * @file chassis.c
 * @author simon
 * @brief ���̿������
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
//		{1, -1}}; // ������ٶ������ٶȵĻ���ʱ����ϵ��
//#ifdef MECANUM_X_ASSM
//// �����ķ�ֹ���������(X�Ͳ���)��˳��˳ʱ��0123
//static const float prvAssembleDir[4][2] =
//	{
//		{0.707, 0.707},
//		{-0.707, 0.707},
//		{0.707, 0.707},
//		{-0.707, 0.707}};
//#endif
//#ifdef MECANUM_O_ASSM
//// �����ķ�ֹ��ӷ���(O�Ͳ���)
//static const float prvAssembleDir[4][2] =
//	{
//		{0.707, -0.707},
//		{-0.707, -0.707},
//		{-0.707, 0.707},
//		{0.707, 0.707}};
//#endif
// =============================================END================================================

// ===========================================Public==============================================
BaseChassis_t MecanumChassis; // ���̿���ȫ�ֽṹ��

/**
 * @brief ���̳�ʼ��
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

/** @brief  ����λ��״̬��ʼ��*/
void PostureStatusInit(void)
{
	MecanumChassis.PostureStatus.omega = 0;
	MecanumChassis.PostureStatus.speed = 0;
	MecanumChassis.PostureStatus.yaw = 0;
}

/**
 * @brief ȫ���ƶ��˶�ѧ
 * @param speed �ٶȴ�С(m/s)
 * @param dir �ٶȷ���/rad
 * @param omega ��ת���ٶ�(rad/s)����ʱ��Ϊ������
 * @note �����ٶ�ʸ��������ĸ��ֵ�ת��
 **/
int MecanumChassis_OmniDrive(float speed, float dir, float omega)
{
	// float absolute_angle_offset = 0; //ʹ�þ�������ʱ������ȫ����λ��õ�ƫ���ǽ��в���
	// if (MecanumChassis.pos_mode == POS_MODE_ABSOLUTE)
	// {
	// 	absolute_angle_offset = MecanumChassis.PostureStatus.yaw - 1.5708; // ��ͷ��ʼ����90��
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

	float vx = speed * cos(dir); // �ٶȷ���
	float vy = speed * sin(dir);
	__LIMIT(omega, MAX_ROTATE_VEL); // omega��Ҫ�������㣬����ǰ���ƴ�С
	float target_speed[4]; // ÿ�����ֵ������ٶ�
	float a = WHEEL_LEFT2RIGHT / 2, b = WHEEL_FRONT2BACK / 2;

	//>>>ֱ�Ӽ��㷨<<<
	target_speed[0] = vx + vy - omega * (a + b);
	target_speed[1] = -vx + vy + omega * (a + b);
	target_speed[2] = vx + vy + omega * (a + b);
	target_speed[3] = -vx + vy - omega * (a + b);

	// >>>��ʽ���㷨<<<
	// for (int i = 0; i < 4; i++)
	// {
	// 	float self_turn_vel_x = omega * WHEEL_FRONT2BACK / 2; // ��ת�����ٶȵķ���
	// 	float self_turn_vel_y = omega * WHEEL_LEFT2RIGHT / 2;
	// 	float vel_sum_x = vx + prvOmegaRatio[i][0] * self_turn_vel_x; // �ٶȺϳ�
	// 	float vel_sum_y = vy + prvOmegaRatio[i][1] * self_turn_vel_y;
	// 	float v_roller = vel_sum_x * prvAssembleDir[i][0] + vel_sum_y * prvAssembleDir[i][1]; // ���ӵ������ٶ�
	// 	float v_wheel = v_roller / cos(__ANGLE2RAD(45));										  // תΪ���ӵ��ٶ�
	// 	target_speed[i] = v_wheel;
	// }

	// ��ֵ����
	DriveMotors_LimitSpeed(target_speed);

	//m/sתΪռ�ձ�
	for (int i = 0; i < 4; i++)
	{
	    float target_omega = target_speed[i]/DRIVE_WHEEL_RADIUS;// target_speed��λm/s
		MecanumChassis.motor[i].target_rpm = target_omega*30/PI;
	}

	Motor_RpmCtrl();
}

float drift_damp_coff = 0.85; // ��ֹת��Ư�Ƶ�����ϵ��
/**
 * @brief ����˶�ѧ
 * @param speed �ٶȴ�С
 * @param omega ���ٶ�(rad/s)
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
 * @brief ���̿���״̬��
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
