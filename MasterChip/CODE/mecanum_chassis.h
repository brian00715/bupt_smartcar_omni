#ifndef MECANUM_CHASSIS_H_
#define MECANUM_CHASSIS_H_
//=================================���ENUM=================================

typedef enum CONTROL_MODE
{
    SW_MODE_NONE = 0,
    SW_HANDLE,
    SW_CMD,
    SW_TRACK,
    SW_TUNING
} CONTROL_MODE;

typedef enum POS_MODE
{
    SW_RELATIVE = 0, // ��ģ����ϵ�µ����λ��
    SW_ABSOLUTE      // ��������ϵ�µľ���λ��
} POS_MODE;

//=================================�ṹ��=================================
typedef struct PostureStatus_t // ����λ��״̬�ṹ�壬��ȫ����λģ�����
{
    float x; // ��λm
    float y; // ��λm
    float last_x;
    float last_y;
    float yaw; // ƫ����/rad
    float last_yaw;
    float speed;      // ���ٶ�m/s
    float speed_x;    // x������ٶ�
    float speed_y;    // y������ٶ�
    float omega;      // ���ٶ�rad/s
    float acc;        // ���ٶ� = ��v2-��v1 = v2-2v1+v0
    float speed_err2; // ��v2
    float speed_err1; // ��v1
} PostureStatus_t;

typedef struct Chassis_t // ���̳���ṹ��
{
    float target_speed;     // Ŀ���ٶȴ�С
    float target_speed_err; // Ŀ���ٶȲ�ֵ
    float target_dir;       // Ŀ���ٶȷ���
    float target_omega;     // Ŀ����ٶ�
    float target_yaw;       // Ŀ��ƫ����
    CONTROL_MODE ctrl_mode; // ����ģʽ����/�ֱ�/CMD/�Զ�
    POS_MODE pos_mode;      // ����ģʽ�����/����
    PostureStatus_t PostureStatus;
    void (*fChassisMove)(float vel, float dir, float omega); // ���̿��ƺ����ĺ���ָ��
}Chassis_t;

//=================================ȫ�ֱ���=================================

extern Chassis_t MecanumChassis; // ����ȫ�ֽṹ��

//=================================��������=================================
void PostureStatusInit(void);
void Chassis_Init(void);
int MecanumChassisMove(float speed, float dir, float omega);

#endif
