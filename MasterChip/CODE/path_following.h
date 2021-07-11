#ifndef PATH_FOLLOWING_H_
#define PATH_FOLLOWING_H_

#include "headfile.h"
#include "pid.h"

// 巡线数据
typedef struct PathFollow_t
{
    PathFollowStateMachine_e state;
    uint8 begin;
    uint8 image_process_done;
    float heading_err;      // 赛道方向与车头方向(90°)的夹角(rad)
    uint8 normal_err;       // 赛道线与视野中心的横向偏差
    float forthright_speed; // 前进默认速度
    float curve_speed;      // 弯道默认速度
    float angle_thres;
} PathFollow_t;

// 巡线状态机
typedef enum PathFollowStateMachine_e
{
    PATH_FOLLOW_NONE = 0,
    PATH_FOLLOW_NORMAL = 1,
    PATH_FOLLOW_MEER_CROSS = 2,
    PATH_FOLLOW_MEET_ISLAND_FIRST = 3,
    PATH_FOLLOW_MEET_ISLAND_SECOND = 4,
    PATH_FOLLOW_GO_LEFT_FORK = 5,  // 左岔路
    PATH_FOLLOW_GO_RIGHT_FORK = 6, // 右岔路
    PATH_FOLLOW_MEET_GARAGE = 7,   // 车库
    PATH_FOLLOW_MEET_LEFT_BIG_CURVE = 8,
    PATH_FOLLOW_MEET_RIGHT_BIG_CURVE = 9,
} PathFollowStateMachine_e;

extern PID_t HeadingAnglePID;
extern PID_t YawPID;

void PathFollowing_Init();
void PathFollowing_Exe();

#endif
