#ifndef PATH_FOLLOWING_H_
#define PATH_FOLLOWING_H_

#include "headfile.h"
#include "pid.h"

// Ѳ������
typedef struct PathFollow_t
{
    PathFollowStateMachine_e state;
    uint8 begin;
    uint8 image_process_done;
    float heading_err;      // ���������복ͷ����(90��)�ļн�(rad)
    uint8 normal_err;       // ����������Ұ���ĵĺ���ƫ��
    float forthright_speed; // ǰ��Ĭ���ٶ�
    float curve_speed;      // ���Ĭ���ٶ�
    float angle_thres;
} PathFollow_t;

// Ѳ��״̬��
typedef enum PathFollowStateMachine_e
{
    PATH_FOLLOW_NONE = 0,
    PATH_FOLLOW_NORMAL = 1,
    PATH_FOLLOW_MEER_CROSS = 2,
    PATH_FOLLOW_MEET_ISLAND_FIRST = 3,
    PATH_FOLLOW_MEET_ISLAND_SECOND = 4,
    PATH_FOLLOW_GO_LEFT_FORK = 5,  // ���·
    PATH_FOLLOW_GO_RIGHT_FORK = 6, // �Ҳ�·
    PATH_FOLLOW_MEET_GARAGE = 7,   // ����
    PATH_FOLLOW_MEET_LEFT_BIG_CURVE = 8,
    PATH_FOLLOW_MEET_RIGHT_BIG_CURVE = 9,
} PathFollowStateMachine_e;

extern PID_t HeadingAnglePID;
extern PID_t YawPID;

void PathFollowing_Init();
void PathFollowing_Exe();

#endif
