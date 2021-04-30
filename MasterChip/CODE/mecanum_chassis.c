/**
 * @file chassis.c
 * @author simon
 * @brief 底盘控制相关
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

static const int prvOmegaRatio[4][2] = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}}; // 计算角速度与线速度的换算时所需系数
#ifdef MECANUM_X_ASSM
// 麦克纳姆轮辊子方向(X型布置)，顺序顺时针0123
static const int prvAssembleDir[4][2] = {{-0.707, 0.707}, {0.707, 0.707}, {-0.707, 0.707}, {0.707, 0.707}};
#endif
#ifdef MECANUM_O_ASSM
// 麦克纳姆轮辊子方向(O型布置)
static const int prvAssembleDir[4][2] = {{0.707, 0.707}, {-0.707, 0.707}, {-0.707, 0.707}, {0.707, 0.707}};
#endif
// =============================================END================================================

// ===========================================Public==============================================
Chassis_t MecanumChassis; // 底盘控制全局结构体

/**
 * @brief 底盘初始化
 * 
 */
void Chassis_Init(void)
{
    MecanumChassis.target_dir = 0;
    MecanumChassis.target_speed = 0;
    MecanumChassis.target_omega = 0;
    PostureStatusInit();
}

/** @brief  底盘位姿状态初始化*/
void PostureStatusInit(void)
{
    MecanumChassis.PostureStatus.omega = 0;
    MecanumChassis.PostureStatus.speed = 0;
    MecanumChassis.PostureStatus.x = 0;
    MecanumChassis.PostureStatus.y = 0;
    MecanumChassis.PostureStatus.yaw = 0;
    MecanumChassis.PostureStatus.speed_x = 0;
    MecanumChassis.PostureStatus.speed_y = 0;
    MecanumChassis.PostureStatus.last_x = 0;
    MecanumChassis.PostureStatus.last_y = 0;
    MecanumChassis.PostureStatus.last_yaw = 0;
}

/**
 * @brief 底盘速度矢量控制，最底层调用
 * @param speed 速度大小/rpm
 * @param dir 速度方向/rad
 * @param omega 自转角速度(rad/s)，逆时针为正方向
 * @note 控制方法为四个轮子同时转向，始终保持一个方向，并根据速度方向偏差转动每个单独轮以进行补偿
 *       规定dir的范围是[0,pi],[-pi,0]
 **/
int MecanumChassisMove(float speed, float dir, float omega)
{
    float absolute_angle_offset = 0; //使用绝对坐标时，根据全场定位测得的偏航角进行补偿
    if (MecanumChassis.pos_mode == POS_MODE_ABSOLUTE)
    {
        absolute_angle_offset = MecanumChassis.PostureStatus.yaw - 1.5708; // 车头初始方向90°
        if (absolute_angle_offset > PI)
        {
            absolute_angle_offset = -(2 * PI - absolute_angle_offset);
        }
    }
    dir -= absolute_angle_offset;
    float vx = speed * cos(dir); // 速度分量
    float vy = speed * sin(dir);
    LIMIT(omega, MAX_ROTATE_VEL); // omega需要参与运算，故提前限制大小
    float target_speed[4];

    for (int i = 0; i < 4; i++)
    {
        float self_turn_vel_x = omega * WHEEL_FRONT2BACK / 2; // 自转切向速度的分量
        float self_turn_vel_y = omega * WHEEL_LEFT2RIGHT / 2;
        float vel_sum_x = vx + prvOmegaRatio[i][0] * self_turn_vel_x; // 速度合成
        float vel_sum_y = vy + prvOmegaRatio[i][1] * self_turn_vel_y;
        float v_roller = vel_sum_x * prvAssembleDir[i][0] + vel_sum_y * prvAssembleDir[i][1]; // 辊子的切向速度
        float v_wheel = v_roller / cos(ANGLE2RAD(45));                                        // 转为轮子的速度
        target_speed[i] = v_roller;
        // float actual_vel = sqrt(pow(vel_sum_x, 2) + pow(vel_sum_y, 2));
        // target_speed[i] = actual_vel * cos(actual_dir);
    }

    // 数值限制
    DriveMotors_LimitSpeed(target_speed);

    //向驱动板发送命令
    DriveMotors_SetRpm(target_speed);
}

// =============================================END================================================
