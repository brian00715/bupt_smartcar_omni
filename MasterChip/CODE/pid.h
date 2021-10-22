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
	float last_last_err;
	float last_delta_err;
	char use_sub_pid; // 是否开启分段PID
	float sub_pid_kp; // 分段PID的kp
	float sub_pid_thres; // 分段PID偏差量阈值
	float ctrl_max; // 控制量最大值
	float dead_th; // 死区阈值
	float dead_delta; // 进入死区后增量式pid的静态值
} PID_t;

float PID_GetOutput(PID_t *PID, float target, float now);
void PID_Reset(PID_t *s);
void PID_Init(PID_t *pid, float kp, float ki, float kd, float int_duty,
		float int_max, char use_sub_pid, float sub_pid_thres, float output_max);
float PID_GetIncrementOutput(PID_t *PID, float target, float now);

#endif
