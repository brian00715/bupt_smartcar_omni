#ifndef PID_H_
#define PID_H_

typedef struct PID_t
{
    float kp;
    float ki;
    float kd;
    float int_duty; // PID周期（实际上决定积分时间，即积分的快慢）
    float int_max; // 积分限额
    float int_sum; // 积分量
    float last_err;
    float last_delta_err;
} PID_t;

float PID_GetOutput(PID_t *PID, float target, float now);
void PID_Reset(PID_t *s);
void PID_Init(PID_t *pid, float kp, float ki, float kd, float int_duty);

#endif
