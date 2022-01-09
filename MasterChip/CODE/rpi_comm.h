#ifndef RPI_COMM_H_
#define RPI_COMM_H_

#include "headfile.h"

typedef struct RPiMsg_s
{
    uint8_t ctrl_mode;
    float speed, dir, omega;
    float cam_servo_pwm;
} RPiMsg_s;

typedef struct SmartcarMsg_s
{
    float vx, vy, vz, yaw;
    int encoder[4];
} SmartcarMsg_s;

void RPiComm_Init(void);
void RPiComm_Exe(void);
void RPiComm_UARTCallback(void);

#endif
