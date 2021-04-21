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


void Image_Binary(uint8 Threshold);

#endif /* CODE_IMAGEPROCESSING_H_ */
