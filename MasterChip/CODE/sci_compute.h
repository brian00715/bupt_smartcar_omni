#ifndef SCI_COMPUTE_H_
#define SCI_COMPUTE_H_

#include "math.h"

//=================================���ENUM=================================

#define abs(x) ((x)>0?(x):(-x))

#define PI 3.14159265358979323846
/**
 * @brief	�����ݴ�����
 * @note	�����ڴ�����и����õĲ���.e.g. *p++
 */
#define __LIMIT(value, max)  \
    if (value > max)       \
        value = max;       \
    else if (value < -max) \
    value = -max

#define __LIMIT_FROM_TO(value, min, max) \
    {                                  \
        if ((value) > (max))           \
            value = max;               \
        if ((value) < (min))           \
            value = min;               \
    }

/**
 * @brief ��������ͣ�����ָ����������
 * @param result ��Ҫ����float����ָ�룬Ȼ���ں��ⲿʹ��ǿ������ת��
 */
#define __SUM_OF_AR(AR, N, RESULT)     \
		{\
        float MACRO_SUM_TEMP = 0;   \
        for (int i = 0; i < N; i++)  \
        {                            \
            MACRO_SUM_TEMP += AR[i]; \
        }                            \
        *RESULT = MACRO_SUM_TEMP;    \
		}


/**
 * @brief ��������ƽ����������ָ����������
 * @param result ��Ҫ����float����ָ��
 */
#define __AVE_OF_AR(AR, N, RESULT)      \
{\
        float MACRO_SUM_TEMP = 0;       \
        for (int i = 0; i < N; i++)   \
        {                             \
            MACRO_SUM_TEMP += AR[i];  \
        }                             \
        *RESULT = MACRO_SUM_TEMP / N; \
}


/**
 * @brief ��ȡ����������Ԫ��
 * @param MAX ���
 */
#define __MAX_OF_AR(AR, N, MAX)       \
        MAX = AR[0];                \
        for (int i = 0; i < N; i++) \
        {                           \
            if (AR[i] > MAX)        \
            {                       \
                MAX = AR[i];        \
            }                       \
        }                           \


#define __MIN_OF_AR(AR, N, MIN)       \
        MIN = AR[0];                \
        for (int i = 0; i < N; i++) \
        {                           \
            if (AR[i] > MIN)        \
            {                       \
                MIN = AR[i];        \
            }                       \
        }                           \


#define Min(A, B) ((A) <= (B) ? (A) : (B))
#define Max(A, B) ((A) >= (B) ? (A) : (B))
#define SL_OK 0
#define SL_ERROR 1
//�Ƕ���ת��Ϊ������
#define ANGLE2RAD(x) (((x)*1.0) / 180.0f * PI)
//������ת��Ϊ�Ƕ���
#define RAD2ANGLE(x) (((x)*1.0) / PI * 180.0f)

//=================================�ṹ��=================================

//=================================��������=================================

float AngleLimitPI(float angle);
float AngleLimitDiff(float a, float b);
float KalmanFilter(float Accel, float Gyro);

#endif
