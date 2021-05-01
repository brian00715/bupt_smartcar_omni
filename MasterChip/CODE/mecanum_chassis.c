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

static const int prvOmegaRatio[4][2] =
	{
		{-1, -1},
		{-1, 1},
		{1, -1},
		{1, 1}}; // ������ٶ������ٶȵĻ���ʱ����ϵ��
#ifdef MECANUM_X_ASSM
// �����ķ�ֹ��ӷ���(X�Ͳ���)��˳��˳ʱ��0123
static const float prvAssembleDir[4][2] =
	{
		{-0.707, 0.707},
		{0.707, 0.707},
		{-0.707, 0.707},
		{0.707, 0.707}};
#endif
#ifdef MECANUM_O_ASSM
// �����ķ�ֹ��ӷ���(O�Ͳ���)
static const float prvAssembleDir[4][2] =
	{
		{0.707, 0.707},
		{-0.707, 0.707},
		{-0.707, 0.707},
		{0.707, 0.707}};
#endif
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
	MecanumChassis.target_dir = 0;
	MecanumChassis.target_speed = 0;
	MecanumChassis.target_omega = 0;
	MecanumChassis.pos_mode = POS_MODE_RELATIVE;
	PostureStatusInit();
}

/** @brief  ����λ��״̬��ʼ��*/
void PostureStatusInit(void)
{
	MecanumChassis.posture_status.omega = 0;
	MecanumChassis.posture_status.speed = 0;
	MecanumChassis.posture_status.x = 0;
	MecanumChassis.posture_status.y = 0;
	MecanumChassis.posture_status.yaw = 0;
	MecanumChassis.posture_status.speed_x = 0;
	MecanumChassis.posture_status.speed_y = 0;
	MecanumChassis.posture_status.last_x = 0;
	MecanumChassis.posture_status.last_y = 0;
	MecanumChassis.posture_status.last_yaw = 0;
}

/**
 * @brief �����ٶ���������,����������ٶ�����
 * @param speed �ٶȴ�С/rpm
 * @param dir �ٶȷ���/rad
 * @param omega ��ת���ٶ�(rad/s)����ʱ��Ϊ������
 **/
int MecanumChassis_Move(float speed, float dir, float omega)
{
	float absolute_angle_offset = 0; //ʹ�þ�������ʱ������ȫ����λ��õ�ƫ���ǽ��в���
	if (MecanumChassis.pos_mode == POS_MODE_ABSOLUTE)
	{
		absolute_angle_offset = MecanumChassis.posture_status.yaw - 1.5708; // ��ͷ��ʼ����90��
		if (absolute_angle_offset > PI)
		{
			absolute_angle_offset = -(2 * PI - absolute_angle_offset);
		}
	}
	dir -= absolute_angle_offset;
	float vx = speed * cos(dir); // �ٶȷ���
	float vy = speed * sin(dir);
	LIMIT(omega, MAX_ROTATE_VEL); // omega��Ҫ�������㣬����ǰ���ƴ�С
	float target_speed[4];

	for (int i = 0; i < 4; i++)
	{
		float self_turn_vel_x = omega * WHEEL_FRONT2BACK / 2; // ��ת�����ٶȵķ���
		float self_turn_vel_y = omega * WHEEL_LEFT2RIGHT / 2;
		float vel_sum_x = vx + prvOmegaRatio[i][0] * self_turn_vel_x; // �ٶȺϳ�
		float vel_sum_y = vy + prvOmegaRatio[i][1] * self_turn_vel_y;
		float v_roller = vel_sum_x * prvAssembleDir[i][0] + vel_sum_y * prvAssembleDir[i][1]; // ���ӵ������ٶ�
		float v_wheel = v_roller / cos(ANGLE2RAD(45));										  // תΪ���ӵ��ٶ�
		target_speed[i] = v_wheel;
	}

	// ��ֵ����
	DriveMotors_LimitSpeed(target_speed);
	for (int i = 0; i < 4; i++)
	{
		uprintf("[%d]:%d", i, (int)(target_speed[i] * 100));
	}
	uprintf("\r\n");

	//m/sתΪռ�ձ�
	for(int i=0;i<4;i++)
	{
		MecanumChassis.motor[i].target_duty = target_speed[i]*10000;
	}
}

/**
 * @brief ���̿���״̬��
 * 
 */
void MecanumChassis_Exe()
{
	MecanumChassis_Move(MecanumChassis.target_speed, MecanumChassis.target_dir,
						MecanumChassis.target_omega);
	Motor_DutyCtrl();
}

// =============================================END================================================
