/*
 * imageprocessing.h
 *
 *  Created on: 2021��4��12��
 *      Author: Z.yd
 */

#ifndef CODE_IMAGEPROCESSING_H_
#define CODE_IMAGEPROCESSING_H_

#include "headfile.h"

/*
 * ��������ṹ��
 */
typedef struct
{
    uint16 x;
    uint16 y;
} Site_t;

/*
 * ������δ�С�ṹ��
 */
typedef struct
{
    uint16 W;       //��
    uint16 H;       //��
} Size_t;
extern uint8 Image_Process_Flag;
extern uint8 Threshold;
extern uint16 Threshold_ChaHe;
extern uint8 mt9v03x_image_binary[MT9V03X_H][MT9V03X_W];
extern uint8 Image_Lline[60];     //�洢��߽������
extern uint8 Image_Rline[60];     //�洢�ұ߽������
extern uint8 Image_Mline[60];     //�洢���ߵ�����
extern uint8 Image_TuBian_Sum;    //�洢����ͻ�������
extern uint8 Image_TuBian[10];
extern uint8 Image_LBig_Curve_Flag; //������־
extern uint8 Image_RBig_Curve_Flag; //�Ҵ����־
extern float Image_XieLv_float[2];
extern int16 Image_XieLv_int;
extern uint8 Image_Show_Flag;
extern uint8 Image_GuaiDian_Sum;
extern uint8 Image_GuaiDian[10];
extern int32 count;
void Image_Binary(uint8 Threshold);
void Image_Processing(void);
float Image_Regression(int startline,int endline);
uint8 Image_Border_Judge(uint8 Image_1,uint8 Image_2);

#endif /* CODE_IMAGEPROCESSING_H_ */
