/*
 * imageprocessing.c
 * 摄像头图像处理函数
 *
 *  Created on: 2021年4月12日
 *      Author: Z.yd
 */

#include "headfile.h"
#include "isr.h"
uint8 Threshold = 123;
uint16 Threshold_ChaHe = 400;
uint8 mt9v03x_image_binary[MT9V03X_H][MT9V03X_W];
uint8_t Image_Show_Flag = 1;

