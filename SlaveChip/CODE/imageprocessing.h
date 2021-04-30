/*
 * imageprocessing.h
 *
 *  Created on: 2021年4月12日
 *      Author: Z.yd
 */

#ifndef CODE_IMAGEPROCESSING_H_
#define CODE_IMAGEPROCESSING_H_

#include "headfile.h"

/*
 * 定义坐标结构体
 */
typedef struct
{
    uint16 x;
    uint16 y;
} Site_t;

/*
 * 定义矩形大小结构体
 */
typedef struct
{
    uint16 W;       //宽
    uint16 H;       //高
} Size_t;

extern uint8 Threshold;
extern uint8 mt9v03x_image_binary[MT9V03X_H][MT9V03X_W];
extern uint8 Image_Lline[60];     //存储左边界的数组
extern uint8 Image_Rline[60];     //存储右边界的数组
extern uint8 Image_Mline[60];     //存储中线的数组


void Image_Binary(uint8 Threshold);
void Image_Processing(void);
uint8 Image_Border_Judge(uint8 Image_1,uint8 Image_2);

#endif /* CODE_IMAGEPROCESSING_H_ */
