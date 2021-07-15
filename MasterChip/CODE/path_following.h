#ifndef PATH_FOLLOWING_H_
#define PATH_FOLLOWING_H_

#include "headfile.h"
#include "pid.h"

#define CAM_SERVO_90_DUTY 4000
#define CAM_SERVO_0_DUTY 2000
#define CAM_SERVO_DUTY_PER_RAD (1273.24f) // 1 rad 对应的占空比

// 巡线状态
typedef enum PathFollowStateMachine_e
{
    PATH_FOLLOW_NONE = 0,
    PATH_FOLLOW_NORMAL = 1,     // 正常巡线（直线+小弯）
    PATH_FOLLOW_LEFT_ROUND = 2, // 左环岛
    PATH_FOLLOW_LEFT_ROUND_ENTRANCE_FLAG = 3,
    PATH_FOLLOW_LEFT_ROUND_ENTRANCE = 4,
    PATH_FOLLOW_LEFT_ROUND_IN = 5,
    PATH_FOLLOW_LEFT_ROUND_EXPORT = 6,
    PATH_FOLLOW_LEFT_ROUND_OUT_FLAG = 7,
    PATH_FOLLOW_LEFT_ROUND_OUT = 8,

    PATH_FOLLOW_RIGHT_ROUND,
    PATH_FOLLOW_RIGHT_ROUND_ENTRANCE_FLAG,
    PATH_FOLLOW_RIGHT_ROUND_ENTRANCE,
    PATH_FOLLOW_RIGHT_ROUND_IN,
    PATH_FOLLOW_RIGHT_ROUND_EXPORT,
    PATH_FOLLOW_RIGHT_ROUND_OUT_FLAG,
    PATH_FOLLOW_RIGHT_ROUND_OUT,

    PATH_FOLLOW_LEFT_FORK,        // 刚看到三叉
    PATH_FOLLOW_LEFT_FORK_EXPORT, // 进入三叉
    PATH_FOLLOW_LEFT_FORK_OUT,

    PATH_FOLLOW_RIGHT_FORK,
    PATH_FOLLOW_RIGHT_FORK_EXPORT,
    PATH_FOLLOW_RIGHT_FORK_OUT,

    PATH_FOLLOW_FINALL_FIRST,      // 第一次经过终点
    PATH_FOLLOW_FINALL_SECOND,     // 第二次经过终点
    PATH_FOLLOW_STOP,              // 入库
    PATH_FOLLOW_FINALL_FIRST_WAIT, // 第一次识别到终点的延时标志位

    PATH_FOLLOW_START,      // 出库后识别到赛道
    PATH_FOLLOW_OUT_GARAGE, // 出库状态

    PATH_FOLLOW_LEFT_FORK_DONE, // 已经出三岔，看到正常道路
    PATH_FOLLOW_RIGHT_FORK_DONE,
    PATH_FOLLOW_LEFT_FORK_DONE2,
    PATH_FOLLOW_RIGHT_FORK_DONE2
} PathFollowStateMachine_e;

typedef struct PathFollow_t
{
    PathFollowStateMachine_e state;
    uint8 begin;
    uint8 image_process_done;
    float heading_err;   // 赛道方向与车头方向(90°)的夹角(rad)
    int16 normal_err;    // 赛道线与视野中心的横向偏差
    float forward_speed; // 前进默认速度
    float curve_speed;   // 弯道默认速度
    float angle_thres;
    uint8_t en_turn_ctrl; // 开启自转闭环
    uint8_t fork_turn_done;
} PathFollow_t;

extern PID_t HeadingPID;
extern PID_t YawPID;

void PathFollowing_Init();
void PathFollowing_Exe();

#endif
