#ifndef MECANUM_CHASSIS_H_
#define MECANUM_CHASSIS_H_

#include "motor.h"
#include "path_following.h"

//=================================宏和ENUM=================================

typedef enum CtrlMode_e
{
	CTRL_MODE_NONE = 0,
	CTRL_MODE_CMD,	  // CMD控制模式
	CTRL_MODE_OMNI,	  // 全向驱动模式
	CTRL_MODE_DIFF,	  // 差分驱动模式
	CTRL_MODE_TUNING, // 调试模式
} CtrlMode_e;

typedef enum PosMode_e
{
	POS_MODE_RELATIVE = 0, // 车模坐标系下的相对位姿
	POS_MODE_ABSOLUTE	   // 世界坐标系下的绝对位姿
} PosMode_e;

//=================================结构体=================================
typedef struct PostureStatus_t // 底盘位姿状态结构体，由全场定位模块更新
{
	float yaw;	 // 偏航角/rad
	float speed; // 线速度m/s
	float omega; // 角速度rad/s
} PostureStatus_t;

typedef struct BaseChassis_t // 底盘抽象结构体
{
	float target_speed;	  // 目标速度大小(m/s)
	float target_dir;	  // 目标速度方向(rad)
	float target_omega;	  // 目标角速度(rad/s)
	float target_yaw;	  // 目标偏航角
	CtrlMode_e ctrl_mode; // 控制模式：无/手柄/CMD/自动
	PosMode_e pos_mode;	  // 坐标模式：相对/绝对
	PostureStatus_t PostureStatus;
	PathFollow_t PathFollowing;
	Motor_t motor[4];		  // 驱动电机，编号为左上角开始顺时针0123
	char send_ctrl_msg_flag;  // 控制发送频率
	char motor_self_check_ok; // 电机自检完成
} BaseChassis_t;

//=================================全局变量=================================

extern BaseChassis_t MecanumChassis; // 底盘全局结构体

//=================================函数声明=================================
void PostureStatusInit(void);
void MecanumChassis_Init(void);
int MecanumChassis_OmniDrive(float speed, float dir, float omega);
void MecanumChassis_Exe();
void MecanumChassis_DiffDrive(float speed, float omega);

#endif
