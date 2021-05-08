#ifndef PATH_FOLLOWING_H_
#define PATH_FOLLOWING_H_

#include "headfile.h"
#include "pid.h"

typedef struct PathFollowStatus_t
{
    uint8 meet_cross;         // 十字
    uint8 meet_island_first;  // 第一次识别到环岛
    uint8 meet_island_second; // 第二次识别到环岛
    uint8 meet_left_fork;     // 左岔路
    uint8 meet_right_fork;    // 右岔路
    uint8 meet_garage;        // 车库
    uint8 meet_left_big_curve;
    uint8 meet_right_big_curve;
    uint8 image_process_done;
    uint8 begin;
    float path_diff_angle; //  赛道方向与车头方向(90°)的夹角(rad)

} PathFollowStatus_t;

extern PathFollowStatus_t PathFollowStatus;
extern PID_t HeadingAnglePID;
extern PID_t YawPID;

void PathFollowing_Init();
void PathFollowing_Exe();

#endif
