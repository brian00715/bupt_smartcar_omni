#ifndef MECANUM_CHASSIS_H_
#define MECANUM_CHASSIS_H_

#include "motor.h"

//=================================宏和ENUM=================================

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
	POS_MODE_RELATIVE = 0, // 车模坐标系下的相对位姿
	POS_MODE_ABSOLUTE	   // 世界坐标系下的绝对位姿
} POS_MODE;

//=================================结构体=================================
typedef struct PostureStatus_t // 底盘位姿状态结构体，由全场定位模块更新
{
	float x; // 单位m
	float y; // 单位m
	float last_x;
	float last_y;
	float yaw; // 偏航角/rad
	float last_yaw;
	float speed;	  // 线速度m/s
	float speed_x;	  // x方向分速度
	float speed_y;	  // y方向分速度
	float omega;	  // 角速度rad/s
	float acc;		  // 加速度 = Δv2-Δv1 = v2-2v1+v0
	float speed_err2; // Δv2
	float speed_err1; // Δv1
} PostureStatus_t;

typedef struct BaseChassis_t // 底盘抽象结构体
{
	float target_speed;		// 目标速度大小
	float target_speed_err; // 目标速度差值
	float target_dir;		// 目标速度方向
	float target_omega;		// 目标角速度
	float target_yaw;		// 目标偏航角
	CONTROL_MODE ctrl_mode; // 控制模式：无/手柄/CMD/自动
	POS_MODE pos_mode;		// 坐标模式：相对/绝对
	PostureStatus_t posture_status;
	Motor_t motor[4];										 // 驱动电机，编号为左上角开始顺时针0123
	char send_ctrl_msg_flag;								 // 控制发送频率
	void (*fChassisMove)(float vel, float dir, float omega); // 底盘控制函数的函数指针
} BaseChassis_t;

//=================================全局变量=================================

extern BaseChassis_t MecanumChassis; // 底盘全局结构体

//=================================函数声明=================================
void PostureStatusInit(void);
void MecanumChassis_Init(void);
int MecanumChassis_Move(float speed, float dir, float omega);
void MecanumChassis_Exe();

#endif
