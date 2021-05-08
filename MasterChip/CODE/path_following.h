#ifndef PATH_FOLLOWING_H_
#define PATH_FOLLOWING_H_

#include "headfile.h"
#include "pid.h"

typedef struct PathFollowStatus_t
{
    uint8 meet_cross;         // ʮ��
    uint8 meet_island_first;  // ��һ��ʶ�𵽻���
    uint8 meet_island_second; // �ڶ���ʶ�𵽻���
    uint8 meet_left_fork;     // ���·
    uint8 meet_right_fork;    // �Ҳ�·
    uint8 meet_garage;        // ����
    uint8 meet_left_big_curve;
    uint8 meet_right_big_curve;
    uint8 image_process_done;
    uint8 begin;
    float path_diff_angle; //  ���������복ͷ����(90��)�ļн�(rad)

} PathFollowStatus_t;

extern PathFollowStatus_t PathFollowStatus;
extern PID_t HeadingAnglePID;
extern PID_t YawPID;

void PathFollowing_Init();
void PathFollowing_Exe();

#endif
