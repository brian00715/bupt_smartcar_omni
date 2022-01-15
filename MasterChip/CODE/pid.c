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
#include <math.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd, float int_duty,
			  float int_max, char use_sub_pid, float sub_pid_thres, float output_max)
{
	pid->kd = kd;
	pid->ki = ki;
	pid->kp = kp;
	pid->int_duty = int_duty;
	pid->int_sum = 0;
	pid->use_sub_pid = 0;
	pid->sub_pid_kp = 0.1;
	pid->sub_pid_thres = 100;
	pid->delta = 0;
}

float PID_GetOutput(PID_t *pid, float target, float now)
{
	float err;
	float delta_err;
	float result;

	err = target - now;
	delta_err = err - pid->last_err;

	//	delta_err *= 0.384f;
	//	delta_err += pid->last_delta_err * 0.615f; //低通滤波

	pid->last_err = err;
	pid->last_delta_err = delta_err;

	pid->int_sum += err * pid->int_duty; // 积分量
	__LIMIT(pid->int_sum, pid->int_max); // 限制积分量大小

	result = err * pid->kp + delta_err * pid->kd + pid->int_sum * pid->ki;
	if (pid->use_sub_pid)
	{
		if (fabs(err) < fabs(pid->sub_pid_thres))
		{
			result = err * pid->sub_pid_kp + pid->int_sum * pid->ki;
		}
	}
	__LIMIT(result, pid->ctrl_max);
	return result;
}

float PID_GetIncrementOutput(PID_t *PID, float target, float now)
{
	float err = target - now;
	// if (!(fabs(err) < PID->dead_th)) // 进入死区后不再更新delta即可
	{
		PID->delta = PID->kp * (err - PID->last_err) +
					 PID->ki * err +
					 PID->kd * (err - 2 * PID->last_err + PID->last_last_err);
		__LIMIT(PID->delta, PID->ctrl_max);
	}

	PID->last_last_err = PID->last_err;
	PID->last_err = err;

	return PID->delta;
}

void PID_Reset(PID_t *pid)
{
	pid->int_sum = 0;
	pid->last_err = 0;
	pid->last_delta_err = 0;
}
