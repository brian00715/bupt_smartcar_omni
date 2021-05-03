#ifndef MECANUM_CHASSIS_H_
#define MECANUM_CHASSIS_H_

#include "motor.h"

//=================================���ENUM=================================

typedef enum CONTROL_MODE
{
	CTRL_MODE_NONE = 0, CTRL_MODE_CMD, CTRL_MODE_AUTO, CTRL_MODE_TUNING,
} CONTROL_MODE;

typedef enum POS_MODE
{
	POS_MODE_RELATIVE = 0, // ��ģ����ϵ�µ����λ��
	POS_MODE_ABSOLUTE	   // ��������ϵ�µľ���λ��
} POS_MODE;

//=================================�ṹ��=================================
typedef struct PostureStatus_t // ����λ��״̬�ṹ�壬��ȫ����λģ�����
{
	float yaw; // ƫ����/rad
	float speed;	  // ���ٶ�m/s
	float omega;	  // ���ٶ�rad/s
} PostureStatus_t;

typedef struct BaseChassis_t // ���̳���ṹ��
{
	float target_speed;		// Ŀ���ٶȴ�С
	float target_speed_err; // Ŀ���ٶȲ�ֵ
	float target_dir;		// Ŀ���ٶȷ���
	float target_omega;		// Ŀ����ٶ�
	float target_yaw;		// Ŀ��ƫ����
	CONTROL_MODE ctrl_mode; // ����ģʽ����/�ֱ�/CMD/�Զ�
	POS_MODE pos_mode;		// ����ģʽ�����/����
	PostureStatus_t posture_status;
	Motor_t motor[4];									// ������������Ϊ���Ͻǿ�ʼ˳ʱ��0123
	char send_ctrl_msg_flag;								 // ���Ʒ���Ƶ��
	char motor_self_check_ok; // ����Լ����
	void (*fChassisMove)(float vel, float dir, float omega); // ���̿��ƺ����ĺ���ָ��
} BaseChassis_t;

//=================================ȫ�ֱ���=================================

extern BaseChassis_t MecanumChassis; // ����ȫ�ֽṹ��

//=================================��������=================================
void PostureStatusInit(void);
void MecanumChassis_Init(void);
int MecanumChassis_Move(float speed, float dir, float omega);
void MecanumChassis_Exe();

#endif