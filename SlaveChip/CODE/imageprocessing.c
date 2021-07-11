/*
 * imageprocessing.c
 * ����ͷͼ������
 *
 *  Created on: 2021��4��12��
 *      Author: Z.yd
 */

#include "headfile.h"

uint8 Threshold = 123;
uint16 Threshold_ChaHe = 280;
uint8 mt9v03x_image_binary[MT9V03X_H][MT9V03X_W];

uint8 Image_Process_Flag = 0;    //�Ƿ������ͼ����ı�־λ������ʱ���ڸ�֪��Ƭ�Ƿ������ͼ����
uint8 Image_Lline[50] = {0};     //�洢��߽������
uint8 Image_Rline[50] = {0};     //�洢�ұ߽������
uint8 Image_Mline[50] = {0};     //�洢���ߵ�����
uint8 Image_Mline2[50] = {0};    //�洢�ڶ����������飬��������·��ʱ�����
uint8 Image_SpecialLine_Sum = 0; //����߽����������ɰױ��

uint8 Image_Llost_Sum = 0;   //��߶�������
uint8 Image_Rlost_Sum = 0;   //�ұ߶�������
uint8 Image_Llost[50] = {0}; //��¼��߶���λ��
uint8 Image_Rlost[50] = {0}; //��¼�ұ߶���λ��

uint8 Image_ShiZi_Flag = 0; //ʮ��·�ڱ�־

uint8 Image_LIsland_Meet_Flag = 0; //�󻷵������־
uint8 Image_RIsland_Meet_Flag = 0; //���󻷵���־
uint8 Image_LIsland_In_Flag = 0;   //�󻷵������־
uint8 Image_LIsland_Out_Flag = 0;  //���󻷵���־
uint8 Image_RIsland_In_Flag = 0;   //���һ�����־
uint8 Image_RIsland_Out_Flag = 0;  //���һ�����־

uint8 Image_LBig_Curve_Flag = 0; //������־
uint8 Image_RBig_Curve_Flag = 0; //�Ҵ����־

uint8 Image_SanCha_Flag = 0; //����·�ڱ�־

uint8 Image_MTuBian_Cnt = 0;     //ͻ�������������ͼ��ɼ������ĸ�����ͻ��
uint8 Image_MTuBian_Sum = 0;     //��¼ͻ�����������
uint8 Image_MTuBian[10] = {0};   //��¼���߷���ͻ��ʱ�ĸ߶�
uint8 Image_MGuaiDian[10] = {0}; //��¼�����յ��λ��
uint8 Image_MGuaiDian_Sum = 0;   //��¼�յ�����

uint8 Image_LTuBian_Cnt = 0;     //ͻ�������������ͼ��ɼ������ĸ�����ͻ��
uint8 Image_LTuBian_Sum = 0;     //��¼ͻ�����������
uint8 Image_LTuBian[10] = {0};   //��¼���߷���ͻ��ʱ�ĸ߶�
uint8 Image_LGuaiDian[10] = {0}; //��¼�����յ��λ��
uint8 Image_LGuaiDian_Sum = 0;   //��¼�յ�����

uint8 Image_RTuBian_Cnt = 0;     //ͻ�������������ͼ��ɼ������ĸ�����ͻ��
uint8 Image_RTuBian_Sum = 0;     //��¼ͻ�����������
uint8 Image_RTuBian[10] = {0};   //��¼���߷���ͻ��ʱ�ĸ߶�
uint8 Image_RGuaiDian[10] = {0}; //��¼�����յ��λ��
uint8 Image_RGuaiDian_Sum = 0;   //��¼�յ�����

float Image_MAdd_float = 0;
int16 Image_MAdd_int = 0;

float Image_XieLv_float[2];
int16 Image_XieLv_int = 0; //�������߼������б��,�Ŵ�1000�����൱��С�������λ����
static int i = 0, j = 0;   //����ѭ������

uint8 Image_Show_Flag = 1; //ͼ������Ļ��ʾ��־

int32 count = 0;

/*****************************************************************************
 * @brief       ��С���˷����ֱ����б��
 * @param       startline            ��ʼ������
 * @param       endline              ����������0
 * @param       n=-1,0,1            -1Ϊ����ߣ�0Ϊ���ߣ�1Ϊ�ұ���
 * @return      float             ��ϳ���ֱ��б��
 ***************************************************************************/
float Image_Regression(int startline, int endline, int n)
{

    int i = 0, SumX = 0, SumY = 0, SumLines = 0;
    float SumUp = 0, SumDown = 0, avrX = 0, avrY = 0, B; //,A;
    SumLines = startline - endline;                      // startline Ϊ��ʼ�У� //endline ������ //SumLines
    if (n == 0)
    {
        for (i = startline; i > endline; i--)
        {
            SumX += 49 - i;
            SumY += Image_Mline[i]; //����Middle_blackΪ������ߵ�����
        }
        avrX = 1.0 * SumX / SumLines; //X��ƽ��ֵ
        avrY = 1.0 * SumY / SumLines; //Y��ƽ��ֵ
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
            B = SumUp / SumDown; //б��
                                 //    A=(SumY-B*SumX)/SumLines;  //�ؾ�
        return B;                //����б��
    }
    else if(n==-1)
    {
        for (i = startline; i > endline; i--)
        {
            SumX += 49 - i;
            SumY += Image_Lline[i]; //����Middle_blackΪ������ߵ�����
        }
        avrX = 1.0 * SumX / SumLines; //X��ƽ��ֵ
        avrY = 1.0 * SumY / SumLines; //Y��ƽ��ֵ
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
            B = SumUp / SumDown; //б��
                                 //    A=(SumY-B*SumX)/SumLines;  //�ؾ�
        return B;                //����б��
    }
    else if(n==1)
    {
        for (i = startline; i > endline; i--)
        {
            SumX += 49 - i;
            SumY += Image_Rline[i]; //����Middle_blackΪ������ߵ�����
        }
        avrX = 1.0 * SumX / SumLines; //X��ƽ��ֵ
        avrY = 1.0 * SumY / SumLines; //Y��ƽ��ֵ
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
            B = SumUp / SumDown; //б��
                                 //    A=(SumY-B*SumX)/SumLines;  //�ؾ�
        return B;                //����б��
    }
}

/*****************************************************************************
 * @brief       ����ͷ���ݶ�ֵ��,����ͼ����
 * @param       Threshold            ��ֵ������ֵ����
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
                //mt9v03x_image[i][j] = 255; //���ڶ�ֵ����ͨ��������ʾ
                //mt9v03x_image[i][j]=1;                  //���ڶ�ֵ�������ͼ����
            }
        }
    }
}

/*****************************************************************************
 * @brief       ��ͱ��жϱ߽�
 * @param       ���ص�1
 * @param       ���ص�2
 * @return      1�����ص�1Ϊ�ڣ�2Ϊ�ף���-1�����ص�1Ϊ�ף�2Ϊ�ڣ���0�����Ǳ߽磩
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
 * @brief       �����߲Ȩֵ��ĺ�
 * @param       startline            ��ʼ������
 * @param       endline              ����������
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
 * @brief       ����ͷͼ����
 * @param       NULL
 * @return      void
 ***************************************************************************/
// void Image_Processing(void)
// {
//     if (mt9v03x_finish_flag == 0) //��ͼ��δ�ɼ���ɣ��򲻽���ͼ���������Ƿ����ͼ�����־Ϊ0
//     {
// //        Image_Process_Flag = 0;
//     }
//     else if (mt9v03x_finish_flag == 1) //��ͼ��ɼ���ɣ������ͼ����
//     {
//         //mt9v03x_finish_flag = 0; //ͼ��ɼ���ɱ�־���㣬�Ա��´��ж��Ƿ�ɼ����
//         //Image_Process_Flag = 1; //��ͼ��ɼ���ɣ���ͼ�����־��1
//         /************************��������*********************************/
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

//         /************************��ʼͼ����****************************/
//         for (i = 49; i >= 45; i--) //ǰ����
//         {
//             /**********************�������******************************/
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
//                     Image_SpecialLine_Sum++; //����Ǻڱ�׵ı߽�,����Ӧ����·�ڵ����
//                     break;
//                 }
//             }
//             if (j < 0) //������û��⵽����
//             {
//                 Image_Llost_Sum++;
//             }
//             /**********************���ұ���******************************/
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
//             if (j > 187) //����ұ�û��⵽����
//             {
//                 Image_RLost_Sum++;
//             }
//             /*********************���߼���*************************/
//             Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;
//         }

//         /********************��40�д���***********************/
//         for (i = 44; i >= 5; i--) //��40��
//         {
//             /**********************�������******************************/
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
//                     Image_SpecialLine_Sum++; //����Ǻڱ�׵ı߽�,����Ӧ����·�ڵ����
//                     break;
//                 }
//             }
//             if (j < 0) //������û��⵽����
//             {
//                 Image_Llost_Sum++;
//             }
//             /**********************���ұ���******************************/
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
//             if (j > 187) //����ұ�û��⵽����
//             {
//                 Image_RLost_Sum++;
//             }
//             /*********************���߼���*************************/
//             Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;

//             if (Image_Mline[i] - Image_Mline[i + 1 + Image_TuBian_Cnt] >= 5 || Image_Mline[i] - Image_Mline[i + 1 + Image_TuBian_Cnt] <= -5)
//                 Image_TuBian_Cnt++; //��¼����ͻ��ĵص�
//             else
//                 Image_TuBian_Cnt = 0;
//             if (Image_TuBian_Cnt >= 3)
//             {
//                 Image_TuBian[Image_TuBian_Sum] = i + Image_TuBian_Cnt; //��¼���߲���ͻ��ʱ�ĸ߶�
//                 Image_TuBian_Sum++;
//                 Image_TuBian_Cnt = 0;
//             }
//             if (Image_Mline[i] >= 180 && Image_TuBian_Sum == 0) //�����ߵ����ұ߽磬���ڴ�֮ǰδ��������ͻ�䣬��Ϊ������
//             {
//                 Image_RBig_Curve_Flag = 1;
//                 break;
//             }
//             if (Image_Mline[i] <= 7 && Image_TuBian_Sum == 0) //�����ߵ�����߽磬���ڴ�֮ǰδ��������ͻ�䣬��Ϊ������
//             {
//                 Image_LBig_Curve_Flag = 1;
//                 break;
//             }
//         }
//         //��ʼ�ж������·�ͼ���б��
//         if (Image_RBig_Curve_Flag == 1 || Image_LBig_Curve_Flag == 1) //���Ϊ����ʱ����б��
//         {
//             //                Image_XieLv=1000*(Image_Mline[i]-Image_Mline[59])/(59-i);
//             Image_XieLv_float[0] = 1.0 * (Image_Mline[i] - 94) / (49 - i);
//             Image_XieLv_int = (int16)(1000 * Image_XieLv_float[0]);
//             //            Image_XieLv_float[1]=Image_Regression(59, i,0);
//         }
//         else
//         {
//             //            Image_XieLv_float[0]=1.0*(Image_Mline[5]-Image_Mline[49])/(49-5);
//             // if (Image_TuBian_Sum != 0) //��ͻ�䣬����ͻ��������
//             // {
//             //     for (i = 58; i < Image_TuBian[0]; i--)
//             //     {
//             //         if (Image_Regression(59, i) >= 0.001 || Image_Regression(59, i) <= -0.001)
//             //             break;
//             //     }
//             //     Image_XieLv_float[1] = Image_Regression(i, Image_TuBian[0]);
//             // }

//             /***********************�ж����߹յ�*************************/
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
//             /***********************����յ��ͻ��**************************/
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
//     Image_Process_Flag = 1; //��ͼ������ɣ���ͼ�����־��1
//     count++;
// }

void Image_Processing(void)
{
    if (mt9v03x_finish_flag == 0) //��ͼ��δ�ɼ���ɣ��򲻽���ͼ���������Ƿ����ͼ�����־Ϊ0
    {
        //        Image_Process_Flag = 0;
    }
    else if (mt9v03x_finish_flag == 1) //��ͼ��ɼ���ɣ������ͼ����
    {
        //mt9v03x_finish_flag = 0; //ͼ��ɼ���ɱ�־���㣬�Ա��´��ж��Ƿ�ɼ����
        //Image_Process_Flag = 1; //��ͼ��ɼ���ɣ���ͼ�����־��1
        /************************��������*********************************/
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

        /************************��ʼͼ����****************************/
        for (i = 49; i >= 45; i--) //ǰ����
        {
            /**********************�������******************************/
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
                    Image_SpecialLine_Sum++; //����Ǻڱ�׵ı߽�,����Ӧ����·�ڵ����
                    break;
                }
            }
            if (j <= 0) //������û��⵽����
            {
                Image_Llost[Image_Llost_Sum] = i;
                Image_Llost_Sum++;
            }
            /**********************���ұ���******************************/
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
            if (j >= 187) //����ұ�û��⵽����
            {
                Image_Rlost[Image_Rlost_Sum] = i;
                Image_Rlost_Sum++;
            }
            /*********************���߼���*************************/
            Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;
        }

        /********************��40�д���***********************/
        for (i = 44; i >= 5; i--) //��40��
        {
            /**********************�������******************************/
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
                    Image_SpecialLine_Sum++; //����Ǻڱ�׵ı߽�,����Ӧ����·�ڵ����
                    break;
                }
            }
            if (j <= 0) //������û��⵽����
            {
                Image_Llost[Image_Llost_Sum] = i;
                Image_Llost_Sum++;
            }
            /**********************���ұ���******************************/
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
            if (j >= 187) //����ұ�û��⵽����
            {
                Image_Rlost[Image_Rlost_Sum] = i;
                Image_Rlost_Sum++;
            }
            /*********************���߼���*************************/
            Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;

            if (Image_Mline[i] - Image_Mline[i + 1 + Image_MTuBian_Cnt] >= 5 || Image_Mline[i] - Image_Mline[i + 1 + Image_MTuBian_Cnt] <= -5)
                Image_MTuBian_Cnt++; //��¼����ͻ��ĵص�
            else
                Image_MTuBian_Cnt = 0;
            if (Image_MTuBian_Cnt >= 3)
            {
                Image_MTuBian[Image_MTuBian_Sum] = i + Image_MTuBian_Cnt; //��¼���߲���ͻ��ʱ�ĸ߶�
                Image_MTuBian_Sum++;
                Image_MTuBian_Cnt = 0;
            }

            if (Image_Lline[i] - Image_Lline[i + 1 + Image_LTuBian_Cnt] >= 5 || Image_Lline[i] - Image_Lline[i + 1 + Image_LTuBian_Cnt] <= -5)
                Image_LTuBian_Cnt++; //��¼�����ͻ��ĵص�
            else
                Image_LTuBian_Cnt = 0;
            if (Image_LTuBian_Cnt >= 3)
            {
                Image_LTuBian[Image_LTuBian_Sum] = i + Image_LTuBian_Cnt; //��¼����߲���ͻ��ʱ�ĸ߶�
                Image_LTuBian_Sum++;
                Image_LTuBian_Cnt = 0;
            }

            if (Image_Rline[i] - Image_Rline[i + 1 + Image_RTuBian_Cnt] >= 5 || Image_Rline[i] - Image_Rline[i + 1 + Image_RTuBian_Cnt] <= -5)
                Image_RTuBian_Cnt++; //��¼�ұ���ͻ��ĵص�
            else
                Image_RTuBian_Cnt = 0;
            if (Image_RTuBian_Cnt >= 3)
            {
                Image_RTuBian[Image_RTuBian_Sum] = i + Image_RTuBian_Cnt; //��¼�ұ��߲���ͻ��ʱ�ĸ߶�
                Image_RTuBian_Sum++;
                Image_RTuBian_Cnt = 0;
            }

            // if (Image_Mline[i] >= 180 && Image_MTuBian_Sum == 0) //�����ߵ����ұ߽磬���ڴ�֮ǰδ��������ͻ�䣬��Ϊ������
            // {
            //     Image_RBig_Curve_Flag = 1;
            //     break;
            // }
            // if (Image_Mline[i] <= 7 && Image_MTuBian_Sum == 0) //�����ߵ�����߽磬���ڴ�֮ǰδ��������ͻ�䣬��Ϊ������
            // {
            //     Image_LBig_Curve_Flag = 1;
            //     break;
            // }
        }
        //��ʼ�ж������·�ͼ���б��
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

        if (Image_RBig_Curve_Flag == 1 || Image_LBig_Curve_Flag == 1) //���Ϊ����ʱ����б��
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
            /***********************�ж����߹յ�*************************/
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

            /***********************�ж�����߹յ�*************************/
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

            /***********************�ж����߹յ�*************************/
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

            /***********************����յ��ͻ��**************************/
            
            Image_MAdd_float = Image_MAdd(49, 5);
            Image_MAdd_int = Image_MAdd_float * 1000;
        }
    }
    mt9v03x_finish_flag = 0;
    Image_Process_Flag = 1; //��ͼ������ɣ���ͼ�����־��1
    count++;
}
