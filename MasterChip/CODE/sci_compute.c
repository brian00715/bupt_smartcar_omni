/**
 * @file sci_compute.c
 * @author simon
 * @brief ��ѧ�������
 * @version 0.1
 * @date 2021-04-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

//----------------------------Kalman Filter-----------------------------------------------
float K1 = 0.02;
static float angle = 0, angle_dot = 0;
float Q_angle = 0.001; // ����������Э����
float Q_gyro = 0.003;  //0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
float R_angle = 0.5;   //����������Э���� ������ƫ��
float dt = 0.005;	   //
char C_0 = 1;
float Q_bias = 0, Angle_err = 0;
float PCt_0 = 0, PCt_1 = 0, E = 0;
float K_0 = 0, K_1 = 0, t_0 = 0, t_1 = 0;
float Pdot[4] = {0, 0, 0, 0};
float PP[2][2] = {{1, 0}, {0, 1}};
float KalmanFilter(float Accel, float Gyro)
{
	angle += (Gyro - Q_bias) * dt;			 // �������
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt; // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt; // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - angle; //zk-�������

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0; //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	angle += K_0 * Angle_err;  //�������
	Q_bias += K_1 * Angle_err; //�������
	angle_dot = Gyro - Q_bias; //���ֵ(�������)��΢��=���ٶ�

	return angle;
}
