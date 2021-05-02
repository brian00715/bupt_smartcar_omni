/*
 * imageprocessing.c
 * ����ͷͼ������
 *
 *  Created on: 2021��4��12��
 *      Author: Z.yd
 */

#include "headfile.h"

uint8 Threshold = 123;
uint16 Threshold_ChaHe=270;
uint8 mt9v03x_image_binary[MT9V03X_H][MT9V03X_W];

uint8 Image_Process_Flag = 0;    //�Ƿ������ͼ����ı�־λ������ʱ���ڸ�֪��Ƭ�Ƿ������ͼ����
uint8 Image_Lline[60] = {0};     //�洢��߽������
uint8 Image_Rline[60] = {0};     //�洢�ұ߽������
uint8 Image_Mline[60] = {0};     //�洢���ߵ�����
uint8 Image_Mline2[60] = {0};    //�洢�ڶ����������飬��������·��ʱ�����
uint8 Image_SpecialLine_Sum = 0; //����߽����������ɰױ��
uint8 Image_Llost_Sum = 0;       //��߶�������
uint8 Image_RLost_Sum = 0;       //�ұ߶�������
uint8 Image_ShiZi_Flag = 0;      //ʮ��·�ڱ�־
uint8 Image_LRing_In_Flag = 0;   //�󻷵������־
uint8 Image_LRing_Out_Flag = 0;  //���󻷵���־
uint8 Image_RRing_In_Flag = 0;   //���һ�����־
uint8 Image_RRing_Out_Flag = 0;  //���һ�����־
uint8 Image_LBig_Curve_Flag = 0; //������־
uint8 Image_RBig_Curve_Flag = 0; //�Ҵ����־
uint8 Image_SanCha_Flag = 0;     //����·�ڱ�־
uint8 Image_TuBian = -1;         //��¼���߷���ͻ��ʱ�ĸ߶�
int Image_XieLv = 0;             //�������߼������б��,�Ŵ�1000�����൱��С�������λ����
static int i = 0, j = 0;         //����ѭ������

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
  * @brief       ����ͷͼ����
  * @param       NULL
  * @return      void
  ***************************************************************************/
void Image_Processing(void)
{
    if (mt9v03x_finish_flag == 0) //��ͼ��δ�ɼ���ɣ��򲻽���ͼ���������Ƿ����ͼ�����־Ϊ0
    {
        Image_Process_Flag = 0;
    }
    else if (mt9v03x_finish_flag == 1) //��ͼ��ɼ���ɣ������ͼ����
    {
        //mt9v03x_finish_flag = 0; //ͼ��ɼ���ɱ�־���㣬�Ա��´��ж��Ƿ�ɼ����
        Image_Process_Flag = 1; //��ͼ��ɼ���ɣ���ͼ�����־��1
        /************************��������*********************************/
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

        /************************��ʼͼ����****************************/
        for (i = 59; i >= 55; i--) //ǰ����
        {
            /**********************�������******************************/
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
                    Image_SpecialLine_Sum++; //����Ǻڱ�׵ı߽�,����Ӧ����·�ڵ����
                    break;
                }
            }
            if (j < 0) //������û��⵽����
            {
                Image_Llost_Sum++;
            }
            /**********************���ұ���******************************/
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
            if (j > 79) //����ұ�û��⵽����
            {
                Image_RLost_Sum++;
            }
            /*********************���߼���*************************/
            Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;
        }

        /********************��55�д���***********************/
        for (i = 54; i >= 0; i--) //��55��
        {
            /**********************�������******************************/
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
                    Image_SpecialLine_Sum++; //����Ǻڱ�׵ı߽�,����Ӧ����·�ڵ����
                    break;
                }
            }
            if (j < 0) //������û��⵽����
            {
                Image_Llost_Sum++;
            }
            /**********************���ұ���******************************/
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
            if (j > 79) //����ұ�û��⵽����
            {
                Image_RLost_Sum++;
            }
            /*********************���߼���*************************/
            Image_Mline[i] = (Image_Lline[i] + Image_Rline[i]) / 2;

            // if (Image_Mline[i] - Image_Mline[i + 1] >= 5 || Image_Mline[i] - Image_Mline[i + 1] <= -5)
            //     Image_TuBian = i;                           //��¼����ͻ��ĵص�
            // if (Image_Mline[i] >= 77 && Image_TuBian == -1) //�����ߵ����ұ߽磬���ڴ�֮ǰδ��������ͻ�䣬��Ϊ������
            // {
            //     Image_RBig_Curve_Flag = 1;
            //     break;
            // }
            // if (Image_Mline[i] <= 2 && Image_TuBian == -1) //�����ߵ�����߽磬���ڴ�֮ǰδ��������ͻ�䣬��Ϊ������
            // {
            //     Image_LBig_Curve_Flag = 1;
            //     break;
            // }
        }
        
        //        if(Image_RBig_Curve_Flag==1||Image_LBig_Curve_Flag==1)          //���Ϊ����ʱ����б��
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
