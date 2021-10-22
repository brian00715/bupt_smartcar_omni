#ifndef HANDLE_H_
#define HANDLE_H_

#include "headfile.h"

typedef struct HandleRocker_t // 摇杆结构体
{
    int x;           // 摇杆x坐标
    int y;           // 摇杆y坐标
    uint16_t length; // 油门深度
    float dir;       // 摇杆方向/dir
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
