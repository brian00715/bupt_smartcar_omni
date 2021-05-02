/*
 * imageprocessing.c
 * 摄像头图像处理函数
 *
 *  Created on: 2021年4月12日
 *      Author: Z.yd
 */

#include "headfile.h"

uint8 Threshold = 123;
uint16 Threshold_ChaHe=270;
uint8 mt9v03x_image_binary[MT9V03X_H][MT9V03X_W];

uint8 Image_Process_Flag = 0;    //是否进行了图像处理的标志位，发送时用于告知主片是否进行了图像处理
uint8 Image_Lline[60] = {0};     //存储左边界的数组
uint8 Image_Rline[60] = {0};     //存储右边界的数组
uint8 Image_Mline[60] = {0};     //存储中线的数组
uint8 Image_Mline2[60] = {0};    //存储第二条中线数组，用于三岔路口时的情况
uint8 Image_SpecialLine_Sum = 0; //特殊边界数量，即由白变黑
uint8 Image_Llost_Sum = 0;       //左边丢线数量
uint8 Image_RLost_Sum = 0;       //右边丢线数量
uint8 Image_ShiZi_Flag = 0;      //十字路口标志
uint8 Image_LRing_In_Flag = 0;   //左环岛进入标志
uint8 Image_LRing_Out_Flag = 0;  //出左环岛标志
uint8 Image_RRing_In_Flag = 0;   //进右环岛标志
uint8 Image_RRing_Out_Flag = 0;  //出右环岛标志
uint8 Image_LBig_Curve_Flag = 0; //左大弯标志
uint8 Image_RBig_Curve_Flag = 0; //右大弯标志
uint8 Image_SanCha_Flag = 0;     //三岔路口标志
uint8 Image_TuBian = -1;         //记录中线发生突变时的高度
int Image_XieLv = 0;             //根据中线计算出的斜率,放大1000倍，相当于小数点后三位精度
static int i = 0, j = 0;         //辅助循环变量

/*****************************************************************************
  * @brief       摄像头数据二值化,用于图像处理
  * @param       Threshold            二值化的阈值设置
  * @return      void
  ***************************************************************************/
void Image_Binary(uint8 Threshold)
{
    uint8 i = 0, j = 0;
    for (i = 0; i < MT9V03X_H; i++)
    {
        for (j = 0; j < MT9V03X_W; j++)
        {
            if (mt9v03x_image[i][j] <= Threshold)
            {
                mt9v03x_image_binary[i][j] = 0;
                //mt9v03x_image[i][j] = 0;
            }
            else if (mt9v03x_image[i][j] > Threshold)
            {
                mt9v03x_image_binary[i][j] = 255;
                //mt9v03x_image[i][j] = 255; //用于二值化后通过串口显示
                //mt9v03x_image[i][j]=1;                  //用于二值化后进行图像处理
            }
        }
    }
}

/*****************************************************************************
  * @brief       差和比判断边界
  * @param       像素点1
  * @param       像素点2
  * @return      1（像素点1为黑，2为白），-1（像素点1为白，2为黑），0（不是边界）
  * Sample usage:
  ***************************************************************************/
uint8 Image_Border_Judge(uint8 Image_1, uint8 Image_2)
{
    if(Image_1<Image_2)
    {
        if ((1000 * (Image_2 - Image_1)) / (Image_1 + Image_2) >= Threshold_ChaHe)
        return 1;
        else return 0;
    }
    else if(Image_1>Image_2)
    {
        if ((1000 * (Image_1 - Image_2)) / (Image_1 + Image_2) >= Threshold_ChaHe)
        return -1;
        else return 0;
    }
    else return 0;
}

/*****************************************************************************
  * @brief       摄像头图像处理
  * @param       NULL
  * @return      void
  ***************************************************************************/
void Image_Processing(void)
{
    if (mt9v03x_finish_flag == 0) //若图像未采集完成，则不进行图像处理，且令是否进行图像处理标志为0
    {
        Image_Process_Flag = 0;
    }
    else if (mt9v03x_finish_flag == 1) //若图像采集完成，则进行图像处理
    {
        //mt9v03x_finish_flag = 0; //图像采集完成标志清零，以便下次判断是否采集完成
        Image_Process_Flag = 1; //若图像采集完成，则图像处理标志置1
        /************************变量清零*********************************/
        memset(Image_Lline, 0, sizeof(Image_Lline));
        memset(Image_Rline, 79, sizeof(Image_Rline));
        memset(Image_Mline, 0, sizeof(Image_Mline));
        memset(Image_Mline2, 0, sizeof(Image_Mline2));
        Image_SpecialLine_Sum = 0;
        Image_Llost_Sum = 0;
        Image_RLost_Sum = 0;
        Image_ShiZi_Flag = 0;
        Image_LRing_In_Flag = 0;
        Image_LRing_Out_Flag = 0;
        Image_RRing_In_Flag = 0;
        Image_RRing_Out_Flag = 0;
        Image_LBig_Curve_Flag = 0;
        Image_RBig_Curve_Flag = 0;
        Image_SanCha_Flag = 0;
        Image_TuBian = -1;
        Image_XieLv = 0;

        /************************开始图像处理****************************/
        for (i = 59; i >= 55; i--) //前五行
        {
            /**********************找左边线******************************/
            for (j = 40; j >= 0; j--)
            {
                if (Image_Border_Judge(mt9v03x_image[i][j], mt9v03x_image[i][j + 3]) == 1)
                {
                    Image_Lline[i] = j;
                    break;
                }
                else if (Image_Border_Judge(mt9v03x_image[i][j], mt9v03x_image[i][j + 3]) == -1)
                {
                    Image_Lline[i] = j;
                    Image_SpecialLine_Sum++; //如果是黑变白的边界,即对应三叉路口的情况
                    break;
                }
            }
            if (j < 0) //如果左边没检测到边线
            {
                Image_Llost_Sum++;
            }
            /**********************找右边线******************************/
            for (j = 40; j <= 79; j++)
            {
                if (Image_Border_Judge(mt9v03x_image[i][j], mt9v03x_image[i][j - 3]) == 1)
                {
                    Image_Rline[i] = j;
                    break;
                }
                else if (Image_Border_Judge(mt9v03x_image[i][j], mt9v03x_image[i][j - 3]) == -1)
                {
                    Image_Rline[i] = j;
                    Image_SpecialLine_Sum++;
                    break;
                }
            }
            if (j > 79) //如果右边没检测到边线
            {
                Image_RLost_Sum++;
            }
            /*********************中线计算*************************/
            Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;
        }

        /********************后55行处理***********************/
        for (i = 54; i >= 0; i--) //后55行
        {
            /**********************找左边线******************************/
            for (j = Image_Mline[i+1]; j >= 0; j--)
            {
                if (Image_Border_Judge(mt9v03x_image[i][j], mt9v03x_image[i][j + 3]) == 1)
                {
                    Image_Lline[i] = j;
                    break;
                }
                else if (Image_Border_Judge(mt9v03x_image[i][j], mt9v03x_image[i][j + 3]) == -1)
                {
                    Image_Lline[i] = j;
                    Image_SpecialLine_Sum++; //如果是黑变白的边界,即对应三叉路口的情况
                    break;
                }
            }
            if (j < 0) //如果左边没检测到边线
            {
                Image_Llost_Sum++;
            }
            /**********************找右边线******************************/
            for (j = Image_Mline[i+1]; j <= 79; j++)
            {
                if (Image_Border_Judge(mt9v03x_image[i][j], mt9v03x_image[i][j - 3]) == 1)
                {
                    Image_Rline[i] = j;
                    break;
                }
                else if (Image_Border_Judge(mt9v03x_image[i][j], mt9v03x_image[i][j - 3]) == -1)
                {
                    Image_Rline[i] = j;
                    Image_SpecialLine_Sum++;
                    break;
                }
            }
            if (j > 79) //如果右边没检测到边线
            {
                Image_RLost_Sum++;
            }
            /*********************中线计算*************************/
            Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;

            // if (Image_Mline[i] - Image_Mline[i + 1] >= 5 || Image_Mline[i] - Image_Mline[i + 1] <= -5)
            //     Image_TuBian = i;                           //记录中线突变的地点
            // if (Image_Mline[i] >= 77 && Image_TuBian == -1) //若中线到达右边界，且在此之前未发生中线突变，则为大右弯
            // {
            //     Image_RBig_Curve_Flag = 1;
            //     break;
            // }
            // if (Image_Mline[i] <= 2 && Image_TuBian == -1) //若中线到达左边界，且在此之前未发生中线突变，则为大左弯
            // {
            //     Image_LBig_Curve_Flag = 1;
            //     break;
            // }
        }
        
        //        if(Image_RBig_Curve_Flag==1||Image_LBig_Curve_Flag==1)          //如果为大弯时计算斜率
        //        {
        //            Image_XieLv=1000*(Image_Mline[i]-Image_Mline[59])/(59-i);
        //        }
        //        else
        //        {
        //
        //        }
        //mt9v03x_finish_flag = 0;
    }
}
