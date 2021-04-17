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
// ===========================================Private==============================================

static const int prvOmegaRatio[4][2] = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

// =============================================END================================================

// ===========================================Public==============================================
Chassis_t MecanumChassis;

/**
 * @brief ���̳�ʼ��
 * 
 */
void Chassis_Init(void)
{
    MecanumChassis.target_dir = 0;
    MecanumChassis.target_speed = 0;
    MecanumChassis.target_omega = 0;
    PostureStatusInit();
}

/** @brief  ����λ��״̬��ʼ��*/
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
 * @brief �����ٶ�ʸ�����ƣ���ײ����
 * @param speed �ٶȴ�С/rpm
 * @param dir �ٶȷ���/rad
 * @param omega ��ת���ٶ�(rad/s)����ʱ��Ϊ������
 * @note ���Ʒ���Ϊ�ĸ�����ͬʱת��ʼ�ձ���һ�����򣬲������ٶȷ���ƫ��ת��ÿ���������Խ��в���
 *       �涨dir�ķ�Χ��[0,pi],[-pi,0]
 **/
int MecanumChassisMove(float speed, float dir, float omega)
{
    float absolute_angle_offset = 0; //ʹ�þ�������ʱ������ȫ����λ��õ�ƫ���ǽ��в���
    if (MecanumChassis.pos_mode == SW_ABSOLUTE)
    {
        absolute_angle_offset = MecanumChassis.PostureStatus.yaw - 1.5708;
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
    if (speed != 0 || omega != 0)
    {
        for (int i = 0; i < 4; i++)
        {
            float self_turn_vel_x = omega * WHEEL_FRONT2BACK / 2; // ��ת�����ٶȵķ���
            float self_turn_vel_y = omega * WHEEL_LEFT2RIGHT / 2;
            float vel_sum_x = vx + prvOmegaRatio[i][0] * self_turn_vel_x; // �ٶȺϳɣ����ǵ�����״
            float vel_sum_y = vy + prvOmegaRatio[i][1] * self_turn_vel_y;
            float real_vel = sqrt(pow(vel_sum_x, 2) + pow(vel_sum_y, 2));
            float real_dir = atan2f(vel_sum_y, vel_sum_x); // ���ӽ��ٶȺ����ʵ�ٶȷ���
            target_speed[i] = real_vel;
        }
    }
    else // �ٶȺͽ��ٶȶ�Ϊ0ʱҪ��֤������Ч
    {
        for (int i = 0; i < 4; i++)
        {
            target_speed[i] = 0;
        }
    }

    // ��ֵ����
    DriveMotors_LimitSpeed(target_speed);

    //�������巢������
    DriveMotors_SetRpm(target_speed);
}

// =============================================END================================================
