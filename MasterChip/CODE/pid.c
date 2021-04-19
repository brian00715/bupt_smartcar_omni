/**
 * @file pid.c
 * @author simon
 * @brief PID控制相关
 * @version 0.1
 * @date 2021-04-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "pid.h"
#include "sci_compute.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float int_duty)
{
    pid->kd = kd;
    pid->ki = ki;
    pid->kp = kp;
    pid->int_duty = int_duty;
    pid->int_sum = 0;
}

float PID_GetOutput(PID_t *pid, float target, float now)
{
    float err;
    float delta_err;
    float result;

    err = target - now;
    delta_err = err - pid->last_err;

    delta_err *= 0.384f;
    delta_err += pid->last_delta_err * 0.615f; //低通滤波

    pid->last_err = err;
    pid->last_delta_err = delta_err;

    pid->int_sum += err * pid->int_duty; // 积分量
    LIMIT(pid->int_sum, pid->int_max);   // 限制积分量大小

    result = err * pid->kp + delta_err * pid->kd + pid->int_sum * pid->ki;
    return result;
}

void PID_Reset(PID_t *pid)
{
    pid->int_sum = 0;
    pid->last_err = 0;
    pid->last_delta_err = 0;
}
