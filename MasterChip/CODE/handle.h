#ifndef HANDLE_H_
#define HANDLE_H_

#include "headfile.h"

typedef struct HandleRocker_t // ҡ�˽ṹ��
{
    int x;           // ҡ��x����
    int y;           // ҡ��y����
    uint16_t length; // �������
    float dir;       // ҡ�˷���/dir
} HandleRocker_t;

struct Handle_t
{
    HandleRocker_t left_rocker;
    HandleRocker_t right_rocker;
};

extern uint8 Handle_UARTRxOK;
extern uint8 Handle_PrintRockerStatus_Flag;

void Handle_Init();
void Handle_Exe();
void Handle_PrintRockerStatus();
void Handle_UARTCallback();
float Handle_RockerMapping(uint16_t amplitude,
                           uint8_t max_amplitude,
                           float max_val,
                           float b);
char *substring(char *dst,char *src,int start,int len);

#endif
