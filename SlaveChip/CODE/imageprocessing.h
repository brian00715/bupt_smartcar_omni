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

extern uint8 Threshold;
extern uint8 mt9v03x_image_binary[MT9V03X_H][MT9V03X_W];
extern uint8 Image_Lline[60];     //�洢��߽������
extern uint8 Image_Rline[60];     //�洢�ұ߽������
extern uint8 Image_Mline[60];     //�洢���ߵ�����


void Image_Binary(uint8 Threshold);
void Image_Processing(void);
uint8 Image_Border_Judge(uint8 Image_1,uint8 Image_2);

#endif /* CODE_IMAGEPROCESSING_H_ */
