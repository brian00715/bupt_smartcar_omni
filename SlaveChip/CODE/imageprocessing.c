/*
 * imageprocessing.c
 * 摄像头图像处理函数
 *
 *  Created on: 2021年4月12日
 *      Author: Z.yd
 */

#include "headfile.h"

uint8 Threshold = 123;
uint16 Threshold_ChaHe = 280;
uint8 mt9v03x_image_binary[MT9V03X_H][MT9V03X_W];

uint8 Image_Process_Flag = 0;    //是否进行了图像处理的标志位，发送时用于告知主片是否进行了图像处理
uint8 Image_Lline[50] = {0};     //存储左边界的数组
uint8 Image_Rline[50] = {0};     //存储右边界的数组
uint8 Image_Mline[50] = {0};     //存储中线的数组
uint8 Image_Mline2[50] = {0};    //存储第二条中线数组，用于三岔路口时的情况
uint8 Image_SpecialLine_Sum = 0; //特殊边界数量，即由白变黑

uint8 Image_Llost_Sum = 0;   //左边丢线数量
uint8 Image_Rlost_Sum = 0;   //右边丢线数量
uint8 Image_Llost[50] = {0}; //记录左边丢线位置
uint8 Image_Rlost[50] = {0}; //记录右边丢线位置

uint8 Image_ShiZi_Flag = 0; //十字路口标志

uint8 Image_LIsland_Meet_Flag = 0; //左环岛进入标志
uint8 Image_RIsland_Meet_Flag = 0; //出左环岛标志
uint8 Image_LIsland_In_Flag = 0;   //左环岛进入标志
uint8 Image_LIsland_Out_Flag = 0;  //出左环岛标志
uint8 Image_RIsland_In_Flag = 0;   //进右环岛标志
uint8 Image_RIsland_Out_Flag = 0;  //出右环岛标志

uint8 Image_LBig_Curve_Flag = 0; //左大弯标志
uint8 Image_RBig_Curve_Flag = 0; //右大弯标志

uint8 Image_SanCha_Flag = 0; //三岔路口标志

uint8 Image_MTuBian_Cnt = 0;     //突变计数，避免因图像采集产生的个别点的突变
uint8 Image_MTuBian_Sum = 0;     //记录突变产生的总数
uint8 Image_MTuBian[10] = {0};   //记录中线发生突变时的高度
uint8 Image_MGuaiDian[10] = {0}; //记录产生拐点的位置
uint8 Image_MGuaiDian_Sum = 0;   //记录拐点数量

uint8 Image_LTuBian_Cnt = 0;     //突变计数，避免因图像采集产生的个别点的突变
uint8 Image_LTuBian_Sum = 0;     //记录突变产生的总数
uint8 Image_LTuBian[10] = {0};   //记录中线发生突变时的高度
uint8 Image_LGuaiDian[10] = {0}; //记录产生拐点的位置
uint8 Image_LGuaiDian_Sum = 0;   //记录拐点数量

uint8 Image_RTuBian_Cnt = 0;     //突变计数，避免因图像采集产生的个别点的突变
uint8 Image_RTuBian_Sum = 0;     //记录突变产生的总数
uint8 Image_RTuBian[10] = {0};   //记录中线发生突变时的高度
uint8 Image_RGuaiDian[10] = {0}; //记录产生拐点的位置
uint8 Image_RGuaiDian_Sum = 0;   //记录拐点数量

float Image_MAdd_float = 0;
int16 Image_MAdd_int = 0;

float Image_XieLv_float[2];
int16 Image_XieLv_int = 0; //根据中线计算出的斜率,放大1000倍，相当于小数点后三位精度
static int i = 0, j = 0;   //辅助循环变量

uint8 Image_Show_Flag = 1; //图像在屏幕显示标志

int32 count = 0;

/*****************************************************************************
 * @brief       最小二乘法拟合直线求斜率
 * @param       startline            开始的行数
 * @param       endline              结束的行数0
 * @param       n=-1,0,1            -1为左边线，0为中线，1为右边线
 * @return      float             拟合出的直线斜率
 ***************************************************************************/
float Image_Regression(int startline, int endline, int n)
{

    int i = 0, SumX = 0, SumY = 0, SumLines = 0;
    float SumUp = 0, SumDown = 0, avrX = 0, avrY = 0, B; //,A;
    SumLines = startline - endline;                      // startline 为开始行， //endline 结束行 //SumLines
    if (n == 0)
    {
        for (i = startline; i > endline; i--)
        {
            SumX += 49 - i;
            SumY += Image_Mline[i]; //这里Middle_black为存放中线的数组
        }
        avrX = 1.0 * SumX / SumLines; //X的平均值
        avrY = 1.0 * SumY / SumLines; //Y的平均值
        SumUp = 0;
        SumDown = 0;
        for (i = startline; i > endline; i--)
        {
            SumUp += (Image_Mline[i] - avrY) * (49 - i - avrX);
            SumDown += (49 - i - avrX) * (49 - i - avrX);
        }
        if (SumDown == 0)
            B = 0;
        else
            B = SumUp / SumDown; //斜率
                                 //    A=(SumY-B*SumX)/SumLines;  //截距
        return B;                //返回斜率
    }
    else if(n==-1)
    {
        for (i = startline; i > endline; i--)
        {
            SumX += 49 - i;
            SumY += Image_Lline[i]; //这里Middle_black为存放中线的数组
        }
        avrX = 1.0 * SumX / SumLines; //X的平均值
        avrY = 1.0 * SumY / SumLines; //Y的平均值
        SumUp = 0;
        SumDown = 0;
        for (i = startline; i > endline; i--)
        {
            SumUp += (Image_Lline[i] - avrY) * (49 - i - avrX);
            SumDown += (49 - i - avrX) * (49 - i - avrX);
        }
        if (SumDown == 0)
            B = 0;
        else
            B = SumUp / SumDown; //斜率
                                 //    A=(SumY-B*SumX)/SumLines;  //截距
        return B;                //返回斜率
    }
    else if(n==1)
    {
        for (i = startline; i > endline; i--)
        {
            SumX += 49 - i;
            SumY += Image_Rline[i]; //这里Middle_black为存放中线的数组
        }
        avrX = 1.0 * SumX / SumLines; //X的平均值
        avrY = 1.0 * SumY / SumLines; //Y的平均值
        SumUp = 0;
        SumDown = 0;
        for (i = startline; i > endline; i--)
        {
            SumUp += (Image_Rline[i] - avrY) * (49 - i - avrX);
            SumDown += (49 - i - avrX) * (49 - i - avrX);
        }
        if (SumDown == 0)
            B = 0;
        else
            B = SumUp / SumDown; //斜率
                                 //    A=(SumY-B*SumX)/SumLines;  //截距
        return B;                //返回斜率
    }
}

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
    if (Image_1 < Image_2)
    {
        if ((1000 * (Image_2 - Image_1)) / (Image_1 + Image_2) >= Threshold_ChaHe)
            return 1;
        else
            return 0;
    }
    else if (Image_1 > Image_2)
    {
        if ((1000 * (Image_1 - Image_2)) / (Image_1 + Image_2) >= Threshold_ChaHe)
            return -1;
        else
            return 0;
    }
    else
        return 0;
}

/*****************************************************************************
 * @brief       求中线差赋权值后的和
 * @param       startline            开始的行数
 * @param       endline              结束的行数
 * @return      float             
 ***************************************************************************/
float Image_MAdd(int startline, int endline)
{
    int k = startline - endline + 1, m = 0;
    float sum = 0;
    for (m = 0; m <= k - 1; m++)
    {
        sum += 0.005 * (Image_Mline[endline + m] - 97) * m / k;
    }
    return sum;
}

/*****************************************************************************
 * @brief       摄像头图像处理
 * @param       NULL
 * @return      void
 ***************************************************************************/
// void Image_Processing(void)
// {
//     if (mt9v03x_finish_flag == 0) //若图像未采集完成，则不进行图像处理，且令是否进行图像处理标志为0
//     {
// //        Image_Process_Flag = 0;
//     }
//     else if (mt9v03x_finish_flag == 1) //若图像采集完成，则进行图像处理
//     {
//         //mt9v03x_finish_flag = 0; //图像采集完成标志清零，以便下次判断是否采集完成
//         //Image_Process_Flag = 1; //若图像采集完成，则图像处理标志置1
//         /************************变量清零*********************************/
//         memset(Image_Lline, 0, sizeof(Image_Lline));
//         memset(Image_Rline, 187, sizeof(Image_Rline));
//         memset(Image_Mline, 94, sizeof(Image_Mline));
//         memset(Image_Mline2, 0, sizeof(Image_Mline2));
//         memset(Image_TuBian, 0, sizeof(Image_TuBian));
//         memset(Image_GuaiDian, 0, sizeof(Image_GuaiDian));
//         Image_SpecialLine_Sum = 0;
//         Image_Llost_Sum = 0;
//         Image_RLost_Sum = 0;
//         Image_ShiZi_Flag = 0;
//         Image_LIsland_In_Flag = 0;
//         Image_LIsland_Out_Flag = 0;
//         Image_RIsland_In_Flag = 0;
//         Image_RIsland_Out_Flag = 0;
//         Image_LBig_Curve_Flag = 0;
//         Image_RBig_Curve_Flag = 0;
//         Image_SanCha_Flag = 0;
//         Image_TuBian_Cnt = 0;
//         Image_TuBian_Sum = 0;
//         Image_XieLv_int = 0;
//         Image_XieLv_float[0] = 0;
//         Image_XieLv_float[1] = 0;
//         Image_GuaiDian_Sum = 0;

//         /************************开始图像处理****************************/
//         for (i = 49; i >= 45; i--) //前五行
//         {
//             /**********************找左边线******************************/
//             for (j = 94; j >= 0; j--)
//             {
//                 if (Image_Border_Judge(mt9v03x_image[i][j],
//                                        mt9v03x_image[i][j + 3]) == 1)
//                 {
//                     Image_Lline[i] = j;
//                     break;
//                 }
//                 else if (Image_Border_Judge(mt9v03x_image[i][j],
//                                             mt9v03x_image[i][j + 3]) == -1)
//                 {
//                     Image_Lline[i] = j;
//                     Image_SpecialLine_Sum++; //如果是黑变白的边界,即对应三叉路口的情况
//                     break;
//                 }
//             }
//             if (j < 0) //如果左边没检测到边线
//             {
//                 Image_Llost_Sum++;
//             }
//             /**********************找右边线******************************/
//             for (j = 94; j <= 187; j++)
//             {
//                 if (Image_Border_Judge(mt9v03x_image[i][j],
//                                        mt9v03x_image[i][j - 3]) == 1)
//                 {
//                     Image_Rline[i] = j;
//                     break;
//                 }
//                 else if (Image_Border_Judge(mt9v03x_image[i][j],
//                                             mt9v03x_image[i][j - 3]) == -1)
//                 {
//                     Image_Rline[i] = j;
//                     Image_SpecialLine_Sum++;
//                     break;
//                 }
//             }
//             if (j > 187) //如果右边没检测到边线
//             {
//                 Image_RLost_Sum++;
//             }
//             /*********************中线计算*************************/
//             Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;
//         }

//         /********************后40行处理***********************/
//         for (i = 44; i >= 5; i--) //后40行
//         {
//             /**********************找左边线******************************/
//             for (j = Image_Mline[i + 1]; j >= 0; j--)
//             {
//                 if (Image_Border_Judge(mt9v03x_image[i][j],
//                                        mt9v03x_image[i][j + 3]) == 1)
//                 {
//                     Image_Lline[i] = j;
//                     break;
//                 }
//                 else if (Image_Border_Judge(mt9v03x_image[i][j],
//                                             mt9v03x_image[i][j + 3]) == -1)
//                 {
//                     Image_Lline[i] = j;
//                     Image_SpecialLine_Sum++; //如果是黑变白的边界,即对应三叉路口的情况
//                     break;
//                 }
//             }
//             if (j < 0) //如果左边没检测到边线
//             {
//                 Image_Llost_Sum++;
//             }
//             /**********************找右边线******************************/
//             for (j = Image_Mline[i + 1]; j <= 187; j++)
//             {
//                 if (Image_Border_Judge(mt9v03x_image[i][j],
//                                        mt9v03x_image[i][j - 3]) == 1)
//                 {
//                     Image_Rline[i] = j;
//                     break;
//                 }
//                 else if (Image_Border_Judge(mt9v03x_image[i][j],
//                                             mt9v03x_image[i][j - 3]) == -1)
//                 {
//                     Image_Rline[i] = j;
//                     Image_SpecialLine_Sum++;
//                     break;
//                 }
//             }
//             if (j > 187) //如果右边没检测到边线
//             {
//                 Image_RLost_Sum++;
//             }
//             /*********************中线计算*************************/
//             Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;

//             if (Image_Mline[i] - Image_Mline[i + 1 + Image_TuBian_Cnt] >= 5 || Image_Mline[i] - Image_Mline[i + 1 + Image_TuBian_Cnt] <= -5)
//                 Image_TuBian_Cnt++; //记录中线突变的地点
//             else
//                 Image_TuBian_Cnt = 0;
//             if (Image_TuBian_Cnt >= 3)
//             {
//                 Image_TuBian[Image_TuBian_Sum] = i + Image_TuBian_Cnt; //记录中线产生突变时的高度
//                 Image_TuBian_Sum++;
//                 Image_TuBian_Cnt = 0;
//             }
//             if (Image_Mline[i] >= 180 && Image_TuBian_Sum == 0) //若中线到达右边界，且在此之前未发生中线突变，则为大右弯
//             {
//                 Image_RBig_Curve_Flag = 1;
//                 break;
//             }
//             if (Image_Mline[i] <= 7 && Image_TuBian_Sum == 0) //若中线到达左边界，且在此之前未发生中线突变，则为大左弯
//             {
//                 Image_LBig_Curve_Flag = 1;
//                 break;
//             }
//         }
//         //开始判断特殊道路和计算斜率
//         if (Image_RBig_Curve_Flag == 1 || Image_LBig_Curve_Flag == 1) //如果为大弯时计算斜率
//         {
//             //                Image_XieLv=1000*(Image_Mline[i]-Image_Mline[59])/(59-i);
//             Image_XieLv_float[0] = 1.0 * (Image_Mline[i] - 94) / (49 - i);
//             Image_XieLv_int = (int16)(1000 * Image_XieLv_float[0]);
//             //            Image_XieLv_float[1]=Image_Regression(59, i,0);
//         }
//         else
//         {
//             //            Image_XieLv_float[0]=1.0*(Image_Mline[5]-Image_Mline[49])/(49-5);
//             // if (Image_TuBian_Sum != 0) //有突变，抛弃突变后的中线
//             // {
//             //     for (i = 58; i < Image_TuBian[0]; i--)
//             //     {
//             //         if (Image_Regression(59, i) >= 0.001 || Image_Regression(59, i) <= -0.001)
//             //             break;
//             //     }
//             //     Image_XieLv_float[1] = Image_Regression(i, Image_TuBian[0]);
//             // }

//             /***********************判断中线拐点*************************/
//             for (i = 44; i >= 10; i--)
//             {
//                 if (Image_Mline[i] <= Image_Mline[i + 1] && Image_Mline[i] <= Image_Mline[i + 2] && Image_Mline[i] <= Image_Mline[i + 3] && Image_Mline[i] <= Image_Mline[i + 4] && Image_Mline[i] <= Image_Mline[i + 5] && Image_Mline[i] <= Image_Mline[i - 1] && Image_Mline[i] <= Image_Mline[i - 2] && Image_Mline[i] <= Image_Mline[i - 3] && Image_Mline[i] <= Image_Mline[i - 4] && Image_Mline[i] <= Image_Mline[i - 5])
//                 {
//                     if (Image_Regression(i + 5, i,0) * Image_Regression(i, i - 5,0) < 0)
//                     {
//                         Image_GuaiDian[Image_GuaiDian_Sum] = i;
//                         Image_GuaiDian_Sum++;
//                         i -= 5;
//                     }
//                 }
//                 else if (Image_Mline[i] >= Image_Mline[i + 1] && Image_Mline[i] >= Image_Mline[i + 2] && Image_Mline[i] >= Image_Mline[i + 3] && Image_Mline[i] >= Image_Mline[i + 4] && Image_Mline[i] >= Image_Mline[i + 5] && Image_Mline[i] >= Image_Mline[i - 1] && Image_Mline[i] >= Image_Mline[i - 2] && Image_Mline[i] >= Image_Mline[i - 3] && Image_Mline[i] >= Image_Mline[i - 4] && Image_Mline[i] >= Image_Mline[i - 5])
//                 {
//                     if (Image_Regression(i + 5, i,0) * Image_Regression(i, i - 5,0) < 0)
//                     {
//                         Image_GuaiDian[Image_GuaiDian_Sum] = i;
//                         Image_GuaiDian_Sum++;
//                         i -= 5;
//                     }
//                 }
//             }
//             /***********************处理拐点和突变**************************/
//             if (Image_TuBian_Sum != 0 && Image_GuaiDian_Sum != 0)
//             {
//                 if (Image_TuBian[0] <= Image_GuaiDian[0])
//                 {
//                     if (Image_GuaiDian_Sum <= 5)
//                     {
//                         for (i = 48; i < Image_GuaiDian[0]; i--)
//                         {
//                             if (Image_Regression(49, i,0) >= 0.0001 || Image_Regression(49, i,0) <= -0.0001)
//                                 break;
//                         }
//                         if(Image_Regression(i, Image_GuaiDian[0],0)>0.1||Image_Regression(i, Image_GuaiDian[0],0)<-0.1)
//                         {
//                             Image_XieLv_float[1] = 5.0*Image_Regression(i, Image_GuaiDian[0],0);
//                         }
//                         else Image_XieLv_float[1] = 5.0*Image_Regression(i, Image_GuaiDian[0],0);
//                     }
//                     else
//                     {
//                         for (i = 48; i < 10; i--)
//                         {
//                             if (Image_Regression(49, i,0) >= 0.0001 || Image_Regression(49, i,0) <= -0.0001)
//                                 break;
//                         }
//                         Image_XieLv_float[1] = Image_Regression(i, 5,0);
//                     }
//                 }
//                 else
//                 {
//                     for (i = 48; i < Image_TuBian[0]; i--)
//                     {
//                         if (Image_Regression(49, i,0) >= 0.0001 || Image_Regression(49, i,0) <= -0.0001)
//                             break;
//                     }
//                     Image_XieLv_float[1] = Image_Regression(i, Image_TuBian[0],0);
//                 }
//             }
//             else if (Image_TuBian_Sum != 0 && Image_GuaiDian_Sum == 0)
//             {
//                 for (i = 48; i < Image_TuBian[0]; i--)
//                 {
//                     if (Image_Regression(49, i,0) >= 0.0001 || Image_Regression(49, i,0) <= -0.0001)
//                         break;
//                 }
//                 Image_XieLv_float[1] = Image_Regression(i, Image_TuBian[0],0);
//             }
//             else if (Image_TuBian_Sum == 0 && Image_GuaiDian_Sum != 0)
//             {
//                 if (Image_GuaiDian_Sum <= 5)
//                 {
//                     for (i = 48; i < Image_GuaiDian[0]; i--)
//                     {
//                         if (Image_Regression(49, i,0) >= 0.0001 || Image_Regression(49, i,0) <= -0.0001)
//                             break;
//                     }
//                     if(Image_Regression(i, Image_GuaiDian[0],0)>0.1||Image_Regression(i, Image_GuaiDian[0],0)<-0.1)
//                     {
//                         Image_XieLv_float[1] = 5.0*Image_Regression(i, Image_GuaiDian[0],0);
//                     }
//                     else Image_XieLv_float[1] = 5.0*Image_Regression(i, Image_GuaiDian[0],0);
//                 }
//                 else
//                 {
//                     for (i = 48; i < 10; i--)
//                     {
//                         if (Image_Regression(49, i,0) >= 0.0001 || Image_Regression(49, i,0) <= -0.0001)
//                             break;
//                     }
//                     Image_XieLv_float[1] = Image_Regression(i, 5,0);
//                 }
//             }
//             else
//             {
//                 for (i = 48; i < 10; i--)
//                 {
//                     if (Image_Regression(49, i,0) >= 0.0001 || Image_Regression(49, i,0) <= -0.0001)
//                         break;
//                 }
//                 Image_XieLv_float[1] = Image_Regression(i, 5,0);
//             }
//             Image_XieLv_int = (int16)(1000 * Image_XieLv_float[1]);
//         }
//     }
//     mt9v03x_finish_flag = 0;
//     Image_Process_Flag = 1; //若图像处理完成，则图像处理标志置1
//     count++;
// }

void Image_Processing(void)
{
    if (mt9v03x_finish_flag == 0) //若图像未采集完成，则不进行图像处理，且令是否进行图像处理标志为0
    {
        //        Image_Process_Flag = 0;
    }
    else if (mt9v03x_finish_flag == 1) //若图像采集完成，则进行图像处理
    {
        //mt9v03x_finish_flag = 0; //图像采集完成标志清零，以便下次判断是否采集完成
        //Image_Process_Flag = 1; //若图像采集完成，则图像处理标志置1
        /************************变量清零*********************************/
        memset(Image_Lline, 0, sizeof(Image_Lline));
        memset(Image_Rline, 187, sizeof(Image_Rline));
        memset(Image_Mline, 94, sizeof(Image_Mline));
        memset(Image_Mline2, 0, sizeof(Image_Mline2));
        memset(Image_MTuBian, 0, sizeof(Image_MTuBian));
        memset(Image_MGuaiDian, 0, sizeof(Image_MGuaiDian));
        memset(Image_LTuBian, 0, sizeof(Image_LTuBian));
        memset(Image_LGuaiDian, 0, sizeof(Image_LGuaiDian));
        memset(Image_RTuBian, 0, sizeof(Image_RTuBian));
        memset(Image_RGuaiDian, 0, sizeof(Image_RGuaiDian));
        memset(Image_Llost, 0, sizeof(Image_Llost));
        memset(Image_Rlost, 0, sizeof(Image_Rlost));

        Image_SpecialLine_Sum = 0;
        Image_Llost_Sum = 0;
        Image_Rlost_Sum = 0;
        Image_ShiZi_Flag = 0;
        // Image_LIsland_In_Flag = 0;
        // Image_LIsland_Out_Flag = 0;
        // Image_RIsland_In_Flag = 0;
        // Image_RIsland_Out_Flag = 0;
        Image_LBig_Curve_Flag = 0;
        Image_RBig_Curve_Flag = 0;
        Image_SanCha_Flag = 0;
        Image_MTuBian_Cnt = 0;
        Image_MTuBian_Sum = 0;
        Image_XieLv_int = 0;
        Image_XieLv_float[0] = 0;
        Image_XieLv_float[1] = 0;
        Image_MAdd_float = 0;
        Image_MAdd_int = 0;
        Image_MGuaiDian_Sum = 0;

        /************************开始图像处理****************************/
        for (i = 49; i >= 45; i--) //前五行
        {
            /**********************找左边线******************************/
            for (j = 94; j >= 0; j--)
            {
                if (Image_Border_Judge(mt9v03x_image[i][j],
                                       mt9v03x_image[i][j + 3]) == 1)
                {
                    Image_Lline[i] = j;
                    break;
                }
                else if (Image_Border_Judge(mt9v03x_image[i][j],
                                            mt9v03x_image[i][j + 3]) == -1)
                {
                    Image_Lline[i] = j;
                    Image_SpecialLine_Sum++; //如果是黑变白的边界,即对应三叉路口的情况
                    break;
                }
            }
            if (j <= 0) //如果左边没检测到边线
            {
                Image_Llost[Image_Llost_Sum] = i;
                Image_Llost_Sum++;
            }
            /**********************找右边线******************************/
            for (j = 94; j <= 187; j++)
            {
                if (Image_Border_Judge(mt9v03x_image[i][j],
                                       mt9v03x_image[i][j - 3]) == 1)
                {
                    Image_Rline[i] = j;
                    break;
                }
                else if (Image_Border_Judge(mt9v03x_image[i][j],
                                            mt9v03x_image[i][j - 3]) == -1)
                {
                    Image_Rline[i] = j;
                    Image_SpecialLine_Sum++;
                    break;
                }
            }
            if (j >= 187) //如果右边没检测到边线
            {
                Image_Rlost[Image_Rlost_Sum] = i;
                Image_Rlost_Sum++;
            }
            /*********************中线计算*************************/
            Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;
        }

        /********************后40行处理***********************/
        for (i = 44; i >= 5; i--) //后40行
        {
            /**********************找左边线******************************/
            for (j = Image_Mline[i + 1]; j >= 0; j--)
            {
                if (Image_Border_Judge(mt9v03x_image[i][j],
                                       mt9v03x_image[i][j + 3]) == 1)
                {
                    Image_Lline[i] = j;
                    break;
                }
                else if (Image_Border_Judge(mt9v03x_image[i][j],
                                            mt9v03x_image[i][j + 3]) == -1)
                {
                    Image_Lline[i] = j;
                    Image_SpecialLine_Sum++; //如果是黑变白的边界,即对应三叉路口的情况
                    break;
                }
            }
            if (j <= 0) //如果左边没检测到边线
            {
                Image_Llost[Image_Llost_Sum] = i;
                Image_Llost_Sum++;
            }
            /**********************找右边线******************************/
            for (j = Image_Mline[i + 1]; j <= 187; j++)
            {
                if (Image_Border_Judge(mt9v03x_image[i][j],
                                       mt9v03x_image[i][j - 3]) == 1)
                {
                    Image_Rline[i] = j;
                    break;
                }
                else if (Image_Border_Judge(mt9v03x_image[i][j],
                                            mt9v03x_image[i][j - 3]) == -1)
                {
                    Image_Rline[i] = j;
                    Image_SpecialLine_Sum++;
                    break;
                }
            }
            if (j >= 187) //如果右边没检测到边线
            {
                Image_Rlost[Image_Rlost_Sum] = i;
                Image_Rlost_Sum++;
            }
            /*********************中线计算*************************/
            Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;

            if (Image_Mline[i] - Image_Mline[i + 1 + Image_MTuBian_Cnt] >= 5 || Image_Mline[i] - Image_Mline[i + 1 + Image_MTuBian_Cnt] <= -5)
                Image_MTuBian_Cnt++; //记录中线突变的地点
            else
                Image_MTuBian_Cnt = 0;
            if (Image_MTuBian_Cnt >= 3)
            {
                Image_MTuBian[Image_MTuBian_Sum] = i + Image_MTuBian_Cnt; //记录中线产生突变时的高度
                Image_MTuBian_Sum++;
                Image_MTuBian_Cnt = 0;
            }

            if (Image_Lline[i] - Image_Lline[i + 1 + Image_LTuBian_Cnt] >= 5 || Image_Lline[i] - Image_Lline[i + 1 + Image_LTuBian_Cnt] <= -5)
                Image_LTuBian_Cnt++; //记录左边线突变的地点
            else
                Image_LTuBian_Cnt = 0;
            if (Image_LTuBian_Cnt >= 3)
            {
                Image_LTuBian[Image_LTuBian_Sum] = i + Image_LTuBian_Cnt; //记录左边线产生突变时的高度
                Image_LTuBian_Sum++;
                Image_LTuBian_Cnt = 0;
            }

            if (Image_Rline[i] - Image_Rline[i + 1 + Image_RTuBian_Cnt] >= 5 || Image_Rline[i] - Image_Rline[i + 1 + Image_RTuBian_Cnt] <= -5)
                Image_RTuBian_Cnt++; //记录右边线突变的地点
            else
                Image_RTuBian_Cnt = 0;
            if (Image_RTuBian_Cnt >= 3)
            {
                Image_RTuBian[Image_RTuBian_Sum] = i + Image_RTuBian_Cnt; //记录右边线产生突变时的高度
                Image_RTuBian_Sum++;
                Image_RTuBian_Cnt = 0;
            }

            // if (Image_Mline[i] >= 180 && Image_MTuBian_Sum == 0) //若中线到达右边界，且在此之前未发生中线突变，则为大右弯
            // {
            //     Image_RBig_Curve_Flag = 1;
            //     break;
            // }
            // if (Image_Mline[i] <= 7 && Image_MTuBian_Sum == 0) //若中线到达左边界，且在此之前未发生中线突变，则为大左弯
            // {
            //     Image_LBig_Curve_Flag = 1;
            //     break;
            // }
        }
        //开始判断特殊道路和计算斜率
        if (Image_Llost_Sum >= 3 && Image_Rlost_Sum < 3)
        {
            Image_LBig_Curve_Flag = 1;
        }
        else if (Image_Rlost_Sum >= 3 && Image_Llost_Sum < 3)
        {
            Image_RBig_Curve_Flag = 1;
        }
        else if (Image_Rlost_Sum >= 3 && Image_Llost_Sum >= 3)
        {
            Image_ShiZi_Flag = 1;
        }

        if (Image_RBig_Curve_Flag == 1 || Image_LBig_Curve_Flag == 1) //如果为大弯时计算斜率
        {
            if (Image_RBig_Curve_Flag == 1)
            {
                for (i = 49; i >= (Image_Rlost[0] + Image_Rlost[Image_Rlost_Sum - 1]) / 2; i--)
                {
                    Image_Lline[i] = Image_Rline[i] - (uint8)((1.0 * (i - (Image_Rlost[0] + Image_Rlost[Image_Rlost_Sum - 1]) / 2) / (50 - (Image_Rlost[0] + Image_Rlost[Image_Rlost_Sum - 1]) / 2)) * (Image_Rline[49] - Image_Lline[49]));
                    Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;
                }
                Image_MAdd_float = Image_MAdd(49, (Image_Rlost[0] + Image_Rlost[Image_Rlost_Sum - 1]) / 2);
                Image_MAdd_int = Image_MAdd_float * 1000;
            }
            else
            {
                for (i = 49; i >= (Image_Llost[0] + Image_Llost[Image_Llost_Sum - 1]) / 2; i--)
                {
                    Image_Rline[i] = Image_Lline[i] + (uint8)((1.0 * (i - (Image_Llost[0] + Image_Llost[Image_Llost_Sum - 1]) / 2) / (50 - (Image_Llost[0] + Image_Llost[Image_Llost_Sum - 1]) / 2)) * (Image_Rline[49] - Image_Lline[49]));
                    Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;
                }
                Image_MAdd_float = Image_MAdd(49, (Image_Llost[0] + Image_Llost[Image_Llost_Sum - 1]) / 2);
                Image_MAdd_int = Image_MAdd_float * 1000;
            }
        }
        else
        {
            /***********************判断中线拐点*************************/
            for (i = 44; i >= 10; i--)
            {
                if (Image_Mline[i] <= Image_Mline[i + 1] && Image_Mline[i] <= Image_Mline[i + 2] && Image_Mline[i] <= Image_Mline[i + 3] && Image_Mline[i] <= Image_Mline[i + 4] && Image_Mline[i] <= Image_Mline[i + 5] && Image_Mline[i] <= Image_Mline[i - 1] && Image_Mline[i] <= Image_Mline[i - 2] && Image_Mline[i] <= Image_Mline[i - 3] && Image_Mline[i] <= Image_Mline[i - 4] && Image_Mline[i] <= Image_Mline[i - 5])
                {
                    if (Image_Regression(i + 5, i,0) * Image_Regression(i, i - 5,0) < 0)
                    {
                        Image_MGuaiDian[Image_MGuaiDian_Sum] = i;
                        Image_MGuaiDian_Sum++;
                        i -= 5;
                    }
                }
                else if (Image_Mline[i] >= Image_Mline[i + 1] && Image_Mline[i] >= Image_Mline[i + 2] && Image_Mline[i] >= Image_Mline[i + 3] && Image_Mline[i] >= Image_Mline[i + 4] && Image_Mline[i] >= Image_Mline[i + 5] && Image_Mline[i] >= Image_Mline[i - 1] && Image_Mline[i] >= Image_Mline[i - 2] && Image_Mline[i] >= Image_Mline[i - 3] && Image_Mline[i] >= Image_Mline[i - 4] && Image_Mline[i] >= Image_Mline[i - 5])
                {
                    if (Image_Regression(i + 5, i,0) * Image_Regression(i, i - 5,0) < 0)
                    {
                        Image_MGuaiDian[Image_MGuaiDian_Sum] = i;
                        Image_MGuaiDian_Sum++;
                        i -= 5;
                    }
                }
            }

            /***********************判断左边线拐点*************************/
            for (i = 44; i >= 10; i--)
            {
                if (Image_Lline[i] <= Image_Lline[i + 1] && Image_Lline[i] <= Image_Lline[i + 2] && Image_Lline[i] <= Image_Lline[i + 3] && Image_Lline[i] <= Image_Lline[i + 4] && Image_Lline[i] <= Image_Lline[i + 5] && Image_Lline[i] <= Image_Lline[i - 1] && Image_Lline[i] <= Image_Lline[i - 2] && Image_Lline[i] <= Image_Lline[i - 3] && Image_Lline[i] <= Image_Lline[i - 4] && Image_Lline[i] <= Image_Lline[i - 5])
                {
                    if (Image_Regression(i + 5, i,-1) * Image_Regression(i, i - 5,-1) < 0)
                    {
                        Image_LGuaiDian[Image_LGuaiDian_Sum] = i;
                        Image_LGuaiDian_Sum++;
                        i -= 5;
                    }
                }
                else if (Image_Lline[i] >= Image_Lline[i + 1] && Image_Lline[i] >= Image_Lline[i + 2] && Image_Lline[i] >= Image_Lline[i + 3] && Image_Lline[i] >= Image_Lline[i + 4] && Image_Lline[i] >= Image_Lline[i + 5] && Image_Lline[i] >= Image_Lline[i - 1] && Image_Lline[i] >= Image_Lline[i - 2] && Image_Lline[i] >= Image_Lline[i - 3] && Image_Lline[i] >= Image_Lline[i - 4] && Image_Lline[i] >= Image_Lline[i - 5])
                {
                    if (Image_Regression(i + 5, i,-1) * Image_Regression(i, i - 5,-1) < 0)
                    {
                        Image_LGuaiDian[Image_LGuaiDian_Sum] = i;
                        Image_LGuaiDian_Sum++;
                        i -= 5;
                    }
                }
            }

            /***********************判断右线拐点*************************/
            for (i = 44; i >= 10; i--)
            {
                if (Image_Rline[i] <= Image_Rline[i + 1] && Image_Rline[i] <= Image_Rline[i + 2] && Image_Rline[i] <= Image_Rline[i + 3] && Image_Rline[i] <= Image_Rline[i + 4] && Image_Rline[i] <= Image_Rline[i + 5] && Image_Rline[i] <= Image_Rline[i - 1] && Image_Rline[i] <= Image_Rline[i - 2] && Image_Rline[i] <= Image_Rline[i - 3] && Image_Rline[i] <= Image_Rline[i - 4] && Image_Rline[i] <= Image_Rline[i - 5])
                {
                    if (Image_Regression(i + 5, i,1) * Image_Regression(i, i - 5,1) < 0)
                    {
                        Image_RGuaiDian[Image_RGuaiDian_Sum] = i;
                        Image_RGuaiDian_Sum++;
                        i -= 5;
                    }
                }
                else if (Image_Rline[i] >= Image_Rline[i + 1] && Image_Rline[i] >= Image_Rline[i + 2] && Image_Rline[i] >= Image_Rline[i + 3] && Image_Rline[i] >= Image_Rline[i + 4] && Image_Rline[i] >= Image_Rline[i + 5] && Image_Rline[i] >= Image_Rline[i - 1] && Image_Rline[i] >= Image_Rline[i - 2] && Image_Rline[i] >= Image_Rline[i - 3] && Image_Rline[i] >= Image_Rline[i - 4] && Image_Rline[i] >= Image_Rline[i - 5])
                {
                    if (Image_Regression(i + 5, i,1) * Image_Regression(i, i - 5,1) < 0)
                    {
                        Image_RGuaiDian[Image_RGuaiDian_Sum] = i;
                        Image_RGuaiDian_Sum++;
                        i -= 5;
                    }
                }
            }

            /***********************处理拐点和突变**************************/
            
            Image_MAdd_float = Image_MAdd(49, 5);
            Image_MAdd_int = Image_MAdd_float * 1000;
        }
    }
    mt9v03x_finish_flag = 0;
    Image_Process_Flag = 1; //若图像处理完成，则图像处理标志置1
    count++;
}
