/*
 * imageprocessing.c
 * 摄像头图像处理函数
 *
 *  Created on: 2021年4月12日
 *      Author: Z.yd
 */

#include "headfile.h"





/*****************************************************************************
  * @brief       摄像头数据二值化,用于图像处理
  * @param       Threshold            二值化的阈值设置
  * @return      void
  ***************************************************************************/
void Image_Binary(uint8 Threshold)
{
    uint8 i=0,j=0;
    for (i=0;i<MT9V03X_W;i++)
    {
        for(j=0;j<MT9V03X_H;j++)
        {
            if (mt9v03x_image[i][j]<=Threshold)
            {
                mt9v03x_image[i][j]=0;
            }
            else if (mt9v03x_image[i][j]>Threshold)
            {
                mt9v03x_image[i][j]=255;                  //用于二值化后通过串口显示
                //mt9v03x_image[i][j]=1;                  //用于二值化后进行图像处理
            }
        }
    }
}


