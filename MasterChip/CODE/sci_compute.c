/**
 * @file sci_compute.c
 * @author simon
 * @brief 科学计算相关
 * @version 0.1
 * @date 2021-04-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "sci_compute.h"

float K1 = 0.02;
static float angle = 0, angle_dot = 0;
float Q_angle = 0.001; // 过程噪声的协方差
float Q_gyro = 0.003;  // 0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle = 0.5;   // 测量噪声的协方差 及测量偏差
float dt = 0.005;      //
char C_0 = 1;
float Q_bias = 0, Angle_err = 0;
float PCt_0 = 0, PCt_1 = 0, E = 0;
float K_0 = 0, K_1 = 0, t_0 = 0, t_1 = 0;
float Pdot[4] = {0, 0, 0, 0};
float PP[2][2] = {{1, 0}, {0, 1}};
float KalmanFilter(float Accel, float Gyro)
{
  angle += (Gyro - Q_bias) * dt;           // 先验估计
  Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

  Pdot[1] = -PP[1][1];
  Pdot[2] = -PP[1][1];
  Pdot[3] = Q_gyro;
  PP[0][0] += Pdot[0] * dt; // Pk-先验估计误差协方差微分的积分
  PP[0][1] += Pdot[1] * dt; // =先验估计误差协方差
  PP[1][0] += Pdot[2] * dt;
  PP[1][1] += Pdot[3] * dt;

  Angle_err = Accel - angle; //zk-先验估计

  PCt_0 = C_0 * PP[0][0];
  PCt_1 = C_0 * PP[1][0];

  E = R_angle + C_0 * PCt_0;

  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;

  t_0 = PCt_0;
  t_1 = C_0 * PP[0][1];

  PP[0][0] -= K_0 * t_0; //后验估计误差协方差
  PP[0][1] -= K_0 * t_1;
  PP[1][0] -= K_1 * t_0;
  PP[1][1] -= K_1 * t_1;

  angle += K_0 * Angle_err;  //后验估计
  Q_bias += K_1 * Angle_err; //后验估计
  angle_dot = Gyro - Q_bias; //输出值(后验估计)的微分=角速度

  return angle;
}

float AngleLimit180(float angle)
{
  while (angle > 180)
  {
    angle -= 360;
  }
  while (angle <= -180)
  {
    angle += 360;
  }
  return angle;
}

float AngleLimitPI(float angle)
{
  while (angle > PI)
  {
    angle -= 2 * PI;
  }
  while (angle <= -PI)
  {
    angle += 2 * PI;
  }
  return angle;
}

/**
 * @brief 两角差值，限制在[0,pi]
 *
 */
float AngleLimitDiff(float a, float b)
{
  float out = a - b;
  return AngleLimitPI(out);
}

/**
 * @brief float（32位）转为uint数组
 */
void float2buffer(float src, uint8 *dst)
{
  UARTMsg_u tmp;
  tmp.fl = src;
  dst[0] = tmp.ui[0];
  dst[1] = tmp.ui[1];
  dst[2] = tmp.ui[2];
  dst[3] = tmp.ui[3];
}

/**
 * @brief uint数组转为float（32位）
 */
float buffer2float(uint8 *src)
{
  UARTMsg_u tmp;
  tmp.ui[0] = src[0];
  tmp.ui[1] = src[1];
  tmp.ui[2] = src[2];
  tmp.ui[3] = src[3];
  return tmp.fl;
}

/**
 * @brief int（16位）转为uint数组
 */
void int2buffer(int src, uint8 *dst)
{
  dst[0] = (src >> 8) & 0xff;
  dst[1] = (src)&0xff;
}

/**
 * @brief 截取字符串
 *
 * @param dst
 * @param src
 * @param start
 * @param len
 * @return char*
 */
char *substring(char *dst, char *src, int start, int len)
{
  char *p = dst;
  char *q = src;
  //    int length = strlen(src);
  //    if (start >= length || start < 0)
  //        return NULL;
  //    if (len > length)
  //        len = length - start;
  q += start;
  while (len--)
  {
    *(p++) = *(q++);
  }
  //    *(p++) = '\0';
  return dst;
}
