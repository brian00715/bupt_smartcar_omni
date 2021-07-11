#ifndef MECANUM_CHASSIS_H_
#define MECANUM_CHASSIS_H_

#include "motor.h"
#include "path_following.h"

//=================================���ENUM=================================

typedef enum CtrlMode_e
{
	CTRL_MODE_NONE = 0,
	CTRL_MODE_CMD,	  // CMD����ģʽ
	CTRL_MODE_OMNI,	  // ȫ������ģʽ
	CTRL_MODE_DIFF,	  // �������ģʽ
	CTRL_MODE_TUNING, // ����ģʽ
} CtrlMode_e;

typedef enum PosMode_e
{
	POS_MODE_RELATIVE = 0, // ��ģ����ϵ�µ����λ��
	POS_MODE_ABSOLUTE	   // ��������ϵ�µľ���λ��
} PosMode_e;

//=================================�ṹ��=================================
typedef struct PostureStatus_t // ����λ��״̬�ṹ�壬��ȫ����λģ�����
{
	float yaw;	 // ƫ����/rad
	float speed; // ���ٶ�m/s
	float omega; // ���ٶ�rad/s
} PostureStatus_t;

typedef struct BaseChassis_t // ���̳���ṹ��
{
	float target_speed;	  // Ŀ���ٶȴ�С(m/s)
	float target_dir;	  // Ŀ���ٶȷ���(rad)
	float target_omega;	  // Ŀ����ٶ�(rad/s)
	float target_yaw;	  // Ŀ��ƫ����
	CtrlMode_e ctrl_mode; // ����ģʽ����/�ֱ�/CMD/�Զ�
	PosMode_e pos_mode;	  // ����ģʽ�����/����
	PostureStatus_t PostureStatus;
	PathFollow_t PathFollowing;
	Motor_t motor[4];		  // ������������Ϊ���Ͻǿ�ʼ˳ʱ��0123
	char send_ctrl_msg_flag;  // ���Ʒ���Ƶ��
	char motor_self_check_ok; // ����Լ����
} BaseChassis_t;

//=================================ȫ�ֱ���=================================

extern BaseChassis_t MecanumChassis; // ����ȫ�ֽṹ��

//=================================��������=================================
void PostureStatusInit(void);
void MecanumChassis_Init(void);
int MecanumChassis_OmniDrive(float speed, float dir, float omega);
void MecanumChassis_Exe();
void MecanumChassis_DiffDrive(float speed, float omega);

#endif
