/*
 * imageprocessing.c
 * ����ͷͼ������
 *
 *  Created on: 2021��4��12��
 *      Author: Z.yd
 */

#include "headfile.h"





/*****************************************************************************
  * @brief       ����ͷ���ݶ�ֵ��,����ͼ����
  * @param       Threshold            ��ֵ������ֵ����
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
                mt9v03x_image[i][j]=255;                  //���ڶ�ֵ����ͨ��������ʾ
                //mt9v03x_image[i][j]=1;                  //���ڶ�ֵ�������ͼ����
            }
        }
    }
}


