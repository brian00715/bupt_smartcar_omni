#ifndef PID_H_
#define PID_H_

typedef struct PID_t
{
	float kp;
	float ki;
	float kd;
	float int_duty; // PID���ڣ�ʵ���Ͼ�������ʱ�䣬�����ֵĿ�����
	float int_max; // �����޶�
	float int_sum; // ������
	float last_err;
	float last_last_err;
	float last_delta_err;
	char use_sub_pid; // �Ƿ����ֶ�PID
	float sub_pid_kp; // �ֶ�PID��kp
	float sub_pid_thres; // �ֶ�PIDƫ������ֵ
	float ctrl_max; // ���������ֵ
	float dead_th; // ������ֵ
	float dead_delta; // ��������������ʽpid�ľ�ֵ̬
} PID_t;

float PID_GetOutput(PID_t *PID, float target, float now);
void PID_Reset(PID_t *s);
void PID_Init(PID_t *pid, float kp, float ki, float kd, float int_duty,
		float int_max, char use_sub_pid, float sub_pid_thres, float output_max);
float PID_GetIncrementOutput(PID_t *PID, float target, float now);

#endif
