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

/*
两种驱动方案：
1. 差分驱动
用赛道斜率与车头正方向的偏差角作为输入量，输出差分驱动角速度
2. 偏航角驱动
用赛道斜率与车头正方向的偏差角作为输入量，输出偏航角的角速度
*/

PathFollowStatus_t PathFollowStatus;

PID_t HeadingAnglePID; // 差分驱动时航向角PID
PID_t YawPID;          // 偏航角驱动时的PID
PID_t NormalPID;       // 法向修正PID

void PathFollowing_Init()
{
    HeadingAnglePID.kp = 0.5;
    HeadingAnglePID.ki = 0.0;
    HeadingAnglePID.kd = 0.00;
    HeadingAnglePID.ctrl_max = 10;
    HeadingAnglePID.int_duty = 0.01;
    HeadingAnglePID.int_max = 800;
    HeadingAnglePID.use_sub_pid = 0;
    HeadingAnglePID.sub_pid_thres = 2;
    HeadingAnglePID.sub_pid_kp = 0.8;

    YawPID.kp = 0.5;
    YawPID.ki = 0.0;
    YawPID.kd = 0.00;
    YawPID.ctrl_max = 10;
    YawPID.int_duty = 0.01;
    YawPID.int_max = 800;
    YawPID.use_sub_pid = 0;
    YawPID.sub_pid_thres = 2;
    YawPID.sub_pid_kp = 0.8;

    PathFollowStatus.begin = 0;
    PathFollowStatus.meet_cross = 0;
    PathFollowStatus.meet_garage = 0;
    PathFollowStatus.meet_island_first = 0;
    PathFollowStatus.meet_island_second = 0;
    PathFollowStatus.meet_left_fork = 0;
    PathFollowStatus.meet_right_fork = 0;
}

/**
 * @brief 巡线控制状态机
 * TODO:未完成
 */
void PathFollowing_Exe()
{
    if (!PathFollowStatus.begin)
        return;
    float omega_ctrl = PID_GetOutput(&HeadingAnglePID, 0, PathFollowStatus.path_diff_angle);
    MecanumChassis.target_omega = omega_ctrl;
    if (PathFollowStatus.meet_island_second)
    {
        MecanumChassis.target_speed = 0.5;
    }
    else if (PathFollowStatus.meet_garage)
    {
        MecanumChassis.target_speed = 0.2;
    }
    else
    {
        MecanumChassis.target_speed = 0.8;
    }
}