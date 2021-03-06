#ifndef SCI_COMPUTE_H_
#define SCI_COMPUTE_H_

#include "math.h"

//=================================宏和ENUM=================================

#define abs(x) ((x)>0?(x):(-x))

#define PI 3.14159265358979323846
/**
 * @brief	宏数据处理函数
 * @note	切勿在传入带有副作用的参数.e.g. *p++
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
 * @brief 计算数组和，无需指定数组类型
 * @param result 需要填入float类型指针，然后在宏外部使用强制类型转换
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
 * @brief 计算数组平均数，无需指定数组类型
 * @param result 需要填入float类型指针
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
 * @brief 获取数组中最大的元素
 * @param MAX 结果
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


#define __MIN(A, B) ((A) <= (B) ? (A) : (B))
#define __MAX(A, B) ((A) >= (B) ? (A) : (B))
//角度制转化为弧度制
#define __ANGLE2RAD(x) (((x)*1.0) / 180.0f * PI)
//弧度制转换为角度制
#define __RAD2ANGLE(x) (((x)*1.0) / PI * 180.0f)

//=================================结构体=================================

//=================================函数声明=================================

float AngleLimitPI(float angle);
float AngleLimitDiff(float a, float b);
float KalmanFilter(float Accel, float Gyro);

#endif
