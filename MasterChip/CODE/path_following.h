#ifndef PATH_FOLLOWING_H_
#define PATH_FOLLOWING_H_

#include "headfile.h"
#include "pid.h"

#define CAM_SERVO_90_DUTY 4000
#define CAM_SERVO_0_DUTY 2000
#define CAM_SERVO_DUTY_PER_RAD (1273.24f) // 1 rad ��Ӧ��ռ�ձ�

// Ѳ��״̬
typedef enum PathFollowStateMachine_e
{
    PATH_FOLLOW_NONE = 0,
    PATH_FOLLOW_NORMAL = 1,     // ����Ѳ�ߣ�ֱ��+С�䣩
    PATH_FOLLOW_LEFT_ROUND = 2, // �󻷵�
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

    PATH_FOLLOW_LEFT_FORK,        // �տ�������
    PATH_FOLLOW_LEFT_FORK_EXPORT, // ��������
    PATH_FOLLOW_LEFT_FORK_OUT,

    PATH_FOLLOW_RIGHT_FORK,
    PATH_FOLLOW_RIGHT_FORK_EXPORT,
    PATH_FOLLOW_RIGHT_FORK_OUT,

    PATH_FOLLOW_FINALL_FIRST,      // ��һ�ξ����յ�
    PATH_FOLLOW_FINALL_SECOND,     // �ڶ��ξ����յ�
    PATH_FOLLOW_STOP,              // ���
    PATH_FOLLOW_FINALL_FIRST_WAIT, // ��һ��ʶ���յ����ʱ��־λ

    PATH_FOLLOW_START,      // �����ʶ������
    PATH_FOLLOW_OUT_GARAGE, // ����״̬

    PATH_FOLLOW_LEFT_FORK_DONE, // �Ѿ�����������������·
    PATH_FOLLOW_RIGHT_FORK_DONE,
    PATH_FOLLOW_LEFT_FORK_DONE2,
    PATH_FOLLOW_RIGHT_FORK_DONE2
} PathFollowStateMachine_e;

typedef struct PathFollow_t
{
    PathFollowStateMachine_e state;
    uint8 begin;
    uint8 image_process_done;
    float heading_err;   // ���������복ͷ����(90��)�ļн�(rad)
    int16 normal_err;    // ����������Ұ���ĵĺ���ƫ��
    float forward_speed; // ǰ��Ĭ���ٶ�
    float curve_speed;   // ���Ĭ���ٶ�
    float angle_thres;
    uint8_t en_turn_ctrl; // ������ת�ջ�
    uint8_t fork_turn_done;
} PathFollow_t;

extern PID_t HeadingPID;
extern PID_t YawPID;

void PathFollowing_Init();
void PathFollowing_Exe();

#endif
