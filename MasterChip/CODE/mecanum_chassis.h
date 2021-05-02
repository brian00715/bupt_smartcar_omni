#ifndef MECANUM_CHASSIS_H_
#define MECANUM_CHASSIS_H_

#include "motor.h"

//=================================���ENUM=================================

typedef enum CONTROL_MODE
{
	CTRL_MODE_NONE = 0,
	CTRL_MODE_CMD,
	CTRL_MODE_AUTO,
	CTRL_MODE_HANDLE,
	CTRL_MODE_TUNING
} CONTROL_MODE;

typedef enum POS_MODE
{
	POS_MODE_RELATIVE = 0, // ��ģ����ϵ�µ����λ��
	POS_MODE_ABSOLUTE	   // ��������ϵ�µľ���λ��
} POS_MODE;

//=================================�ṹ��=================================
typedef struct PostureStatus_t // ����λ��״̬�ṹ�壬��ȫ����λģ�����
{
	float x; // ��λm
	float y; // ��λm
	float last_x;
	float last_y;
	float yaw; // ƫ����/rad
	float last_yaw;
	float speed;	  // ���ٶ�m/s
	float speed_x;	  // x������ٶ�
	float speed_y;	  // y������ٶ�
	float omega;	  // ���ٶ�rad/s
	float acc;		  // ���ٶ� = ��v2-��v1 = v2-2v1+v0
	float speed_err2; // ��v2
	float speed_err1; // ��v1
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
	Motor_t motor[4];										 // ������������Ϊ���Ͻǿ�ʼ˳ʱ��0123
	char send_ctrl_msg_flag;								 // ���Ʒ���Ƶ��
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
