/*
 * inducercontrol.c
 * ��д�����
 *
 *  Created on: 2021��4��13��
 *      Author: Z.yd
 */

#include "headfile.h"

uint8 InducerMax_Get_Start_Flag=1;         //������ֵ��ȡ��ʼ��־,��ͨ���������Ƹñ�־λʵ�������ռ�������ֵ
uint8 InducerMax_Get_End_Flag=0;           //������ֵ��ȡ������־�����ñ�־Ϊ1ʱ��������ݴ������Żᴦ������
uint16 Inducer_Get_Count=1000;             //ÿ����л�ȡ���ֵʱget������
uint16 InducerMax[7]={0,0,0,0,0,0,0};      //���ڴ�ŵ�����ֵ�Խ��й�һ������
//uint16 Inducer_Get_Last[7]={0,0,0,0,0,0,0};
uint16 Inducer_Get[7]={0,0,0,0,0,0,0};           //ADC��ȡ�ĵ��ֵ
float Inducer_Normalize[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  //��һ����ĵ��ֵ��ΪInducer_Get[i]/InducerMax[i]
//float Inducer_Duty[7]={-10,-9,-8,0,8,9,10};                //����ÿ����е�Ȩֵ,ÿ����й�һ�����ֵ����Ȩֵ��Ϊ����x�����ϵ�ֵ(������������⴦��)
float Inducer_Duty[5]={-10,-8,0,8,10};
float Inducer_Sum_x=0.0;                                   //��кϳɺ��x����ֵ,Inducer_Normalize[i]*Inducer_Duty[i]�ĺ�;
float Inducer_Sum_y=3.0;                                   //�Զ���ĵ��y����ֵ
float Inducer_Slope_Float=0.0;                             //������ݴ����õ���б�ʣ�Inducer_Sum_x/Inducer_Sum_y
char Inducer_Slope_Str[20];                                //������ݴ����õ���б��ת��Ϊ�ַ��������ڴ���,snprintf(Inducer_Slope_Str,15,"%f",Inducer_Slope_Float);
static int i=0,j=0;    //ѭ����������

/************************************************************************************
  * @brief        ��4�����ֵ��oled��ʵʱ��ʾ
  * @param        NULL
  * @return       void
  * Sample usage:
  **********************************************************************************/
void Inducer_Show_Oled(void)
{
    oled_fill(0x00);
    for(i=0;i<4;i++)
    {
        Inducer_Get[i]=0;
    }
    //InducerMax_Get();                              //�ռ�������ֵ���Խ��й�һ��
    //Inducer_Get[0]=adc_mean_filter(ADC_IN0_A0,ADC_12BIT,10);        //12BIT�ֱ���,ȡ10��ƽ��
    Inducer_Get[0]=adc_mean_filter(ADC_IN4_A4,ADC_12BIT,10);
    Inducer_Get[1]=adc_mean_filter(ADC_IN6_A6,ADC_12BIT,10);
    Inducer_Get[2]=adc_mean_filter(ADC_IN8_B0,ADC_12BIT,10);
    Inducer_Get[3]=adc_mean_filter(ADC_IN9_B1,ADC_12BIT,10);
    //Inducer_Get[5]=adc_mean_filter(ADC_IN5_A5,ADC_12BIT,5);        //12BIT�ֱ���,ȡ10��ƽ��
    //Inducer_Get[6]=adc_mean_filter(ADC_IN6_A6,ADC_12BIT,5);        //12BIT�ֱ���,ȡ10��ƽ��

    for(i=0;i<4;i++)                 //��ʾADC��ȡԭ���ֵ
    {
        oled_uint16(0, i,Inducer_Get[i]);
       // printf("[%d]%d ",i,Inducer_Get[i]);
    }
    //printf("\r\n");

    if(InducerMax_Get_End_Flag==1)       //��ʾ��һ����ĵ��ֵ
    {
        for(i=0;i<4;i++)
        {
            Inducer_Normalize[i]=1.0*Inducer_Get[i]/InducerMax[i];
            oled_printf_float(60, i, Inducer_Normalize[i], 1,6);
        }
    }
}

/************************************************************************************
  * @brief        ��4�����ֵ��IPS��ʵʱ��ʾ
  * @param        NULL
  * @return       void
  * Sample usage:
  **********************************************************************************/
void Inducer_Show_Ips(void)
{
    //ips114_clear(BLUE);
    //oled_fill(0x00);
    for(i=0;i<4;i++)
    {
        Inducer_Get[i]=0;
    }
    InducerMax_Get();                              //�ռ�������ֵ���Խ��й�һ��
    //Inducer_Get[0]=adc_mean_filter(ADC_IN0_A0,ADC_12BIT,10);        //12BIT�ֱ���,ȡ10��ƽ��
    Inducer_Get[0]=adc_mean_filter(ADC_IN4_A4,ADC_12BIT,10);
    Inducer_Get[1]=adc_mean_filter(ADC_IN6_A6,ADC_12BIT,10);
    Inducer_Get[2]=adc_mean_filter(ADC_IN8_B0,ADC_12BIT,10);
    Inducer_Get[3]=adc_mean_filter(ADC_IN9_B1,ADC_12BIT,10);
    //Inducer_Get[5]=adc_mean_filter(ADC_IN5_A5,ADC_12BIT,5);        //12BIT�ֱ���,ȡ10��ƽ��
    //Inducer_Get[6]=adc_mean_filter(ADC_IN6_A6,ADC_12BIT,5);        //12BIT�ֱ���,ȡ10��ƽ��

    for(i=0;i<4;i++)                 //��ʾADC��ȡԭ���ֵ
    {
        ips114_showuint16(0, i, Inducer_Get[i]);
       // printf("[%d]%d ",i,Inducer_Get[i]);
    }
    //printf("\r\n");

    if(InducerMax_Get_End_Flag==1)       //��ʾ��һ����ĵ��ֵ
    {
        for(i=0;i<4;i++)
        {
            Inducer_Normalize[i]=1.0*Inducer_Get[i]/InducerMax[i];
            ips114_showfloat(120, i, Inducer_Normalize[i], 1, 6);
        }
    }
}


/************************************************************************************
  * @brief        ��ȡ�߸�������ֵ���Է��������һ������
  * @param        NULL
  * @return       void
  * Sample usage:          ��ͨ���޸�Inducer_Get_Count�ı�ÿ�����getֵ������
  **********************************************************************************/
void InducerMax_Get(void)
{
    if(InducerMax_Get_Start_Flag==1)        //�����л�ȡֵ��־Ϊ1���ȡ������ֵ
    {
        ips114_clear(BLUE);
        ips114_showstr(0, 0, "Wait 2s...");
        //oled_fill(0x00);
        //oled_p6x8str(0, 0,"Wait 2s...");
        systick_delay_ms(2000);
        InducerMax_Get_End_Flag=0;          //������ֵ��ȡ������־����
        ips114_clear(BLUE);
        ips114_showstr(0, 0, "Inducer Getting:");
        //oled_fill(0x00);
        //oled_p6x8str(0, 0,"Inducer Getting");
        for(i=0;i<5;i++)
        {
            InducerMax[i]=0;Inducer_Get[i]=0;    //��ر�������
        }
        /***************���0���ֵ��ȡ**********************************************************/
        ips114_showstr(0, 1, "Inducer0 Getting:");
        //oled_p6x8str(0, 1,"Inducer0:");
        for(j=0;j<Inducer_Get_Count;j++)      //ÿ����вɼ���������
        {
            Inducer_Get[0]=adc_mean_filter(ADC_IN4_A4,ADC_12BIT, 10);          //12BIT�ֱ���,ȡ10��ƽ��
            if(Inducer_Get[0]>InducerMax[0])
            {
                InducerMax[0]=Inducer_Get[0];
                ips114_showuint16(160, 1, InducerMax[0]);
                //oled_int16(90, 1, InducerMax[0]);
            }
            systick_delay_ms(5);               //��ʱ5ms
        }
        //oled_fill_page(1,0x00);
        //oled_p6x8str(0,1,"Inducer0:");
        //oled_uint16(90,1,InducerMax[0]);
        /***************���1���ֵ��ȡ*********************************************************/
        ips114_showstr(0, 2, "Inducer1 Getting:");
        //oled_p6x8str(0, 2,"Inducer1:");
        for(j=0;j<Inducer_Get_Count;j++)      //ÿ����вɼ���������
        {
            Inducer_Get[1]=adc_mean_filter(ADC_IN6_A6,ADC_12BIT, 10);          //12BIT�ֱ���,ȡ10��ƽ��
            if(Inducer_Get[1]>InducerMax[1])
            {
                InducerMax[1]=Inducer_Get[1];
                ips114_showuint16(160, 2, InducerMax[1]);
                //oled_int16(90, 2, InducerMax[1]);
            }
            systick_delay_ms(5);               //��ʱ5ms
        }
        //oled_fill_page(2,0x00);
        //oled_p6x8str(0,2,"Inducer1:");
        //oled_uint16(90,2,InducerMax[1]);
        /***************���2���ֵ��ȡ*********************************************************/
        ips114_showstr(0, 3, "Inducer2 Getting:");
        //oled_p6x8str(0, 3,"Inducer2:");
        for(j=0;j<Inducer_Get_Count;j++)      //ÿ����вɼ���������
        {
            Inducer_Get[2]=adc_mean_filter(ADC_IN8_B0,ADC_12BIT, 10);          //12BIT�ֱ���,ȡ10��ƽ��
            if(Inducer_Get[2]>InducerMax[2])
            {
                InducerMax[2]=Inducer_Get[2];
                ips114_showuint16(160, 3, InducerMax[2]);
                //oled_int16(90, 3, InducerMax[2]);
            }
            systick_delay_ms(5);               //��ʱ5ms
        }
        //oled_fill_page(3,0x00);
        //oled_p6x8str(0,3,"Inducer2:");
        //oled_uint16(90,3,InducerMax[2]);
        /***************���3���ֵ��ȡ********************************************************/
        ips114_showstr(0, 4, "Inducer3 Getting:");
        //oled_p6x8str(0, 4,"Inducer3:");
        for(j=0;j<Inducer_Get_Count;j++)      //ÿ����вɼ���������
        {
            Inducer_Get[3]=adc_mean_filter(ADC_IN9_B1,ADC_12BIT, 10);          //12BIT�ֱ���,ȡ10��ƽ��
            if(Inducer_Get[3]>InducerMax[3])
            {
                InducerMax[3]=Inducer_Get[3];
                ips114_showuint16(160, 4, InducerMax[3]);
                //oled_int16(90, 4, InducerMax[3]);
            }
            systick_delay_ms(5);               //��ʱ5ms
        }
        //oled_fill_page(4,0x00);
        //oled_p6x8str(0,4,"Inducer3:");
        //oled_uint16(90,4,InducerMax[3]);

        /****************������ֵ��ȡ����**************************************************/
        //systick_delay_ms(2000);                  //��ʱ3�룬����ͨ��oled����
        if(InducerMax[0]!=0&&InducerMax[1]!=0&&InducerMax[2]!=0&&InducerMax[3]!=0)//&&InducerMax[4]!=0)//&&InducerMax[5]!=0&&InducerMax[6]!=0)//��������е��ֵΪ�㣬���������ֵ��ȡ���
        {
            InducerMax_Get_End_Flag=1;                   //������ֵ��ȡ��������־λ��1
            InducerMax_Get_Start_Flag=0;                 //��л�ȡ��ʼ��־����
            //ips114_clear(BLUE);
            ips114_showstr(0, 6, "Getting Complete!!");
            //oled_fill(0x00);
            //oled_p8x16str(0, 3, "Getting Complete!");
            systick_delay_ms(1000);
            ips114_clear(BLUE);
        }
    }
}


/************************************************************************************
  * @brief        ������ֵ����ȡ·��б�ʡ�����·����־����������ͨ�����ڷ��͸���Ƭ
  * @param        NULL
  * @return       void
  * Sample usage:
  **********************************************************************************/
void Inducer_Processing(void)
{
    //InducerMax_Get();                                //����δ��ȡ������ֵ����
    if(InducerMax_Get_End_Flag==1)                   //��л�ȡ���ֵ��־Ϊ1ʱ�Ž������ݴ������������ֵΪ0���������������Σ��
    {
        Inducer_Sum_x=0.0;                           //��ʼ����г���Ȩֵ��ĺ�
        Inducer_Slope_Float=0.0;                     //��ʼ�����ݴ�����б��
        for(i=0;i<4;i++)                             //��ʼ������
        {
            Inducer_Get[i]=0;Inducer_Normalize[i]=0.0;
        }
        //Inducer_Get[0]=adc_mean_filter(ADC_IN0_A0, ADC_12BIT, 10);        //12BIT���ȣ�ȡ10��ADת��ƽ��ֵ
        Inducer_Get[0]=adc_mean_filter(ADC_IN4_A4, ADC_12BIT, 10);        //12BIT���ȣ�ȡ10��ADת��ƽ��ֵ
        Inducer_Get[1]=adc_mean_filter(ADC_IN6_A6, ADC_12BIT, 10);        //12BIT���ȣ�ȡ10��ADת��ƽ��ֵ
        Inducer_Get[2]=adc_mean_filter(ADC_IN8_B0, ADC_12BIT, 10);        //12BIT���ȣ�ȡ10��ADת��ƽ��ֵ
        Inducer_Get[3]=adc_mean_filter(ADC_IN9_B1, ADC_12BIT, 10);        //12BIT���ȣ�ȡ10��ADת��ƽ��ֵ
        //Inducer_Get[5]=adc_mean_filter(ADC_IN5_A5, ADC_12BIT, 5);        //12BIT���ȣ�ȡ10��ADת��ƽ��ֵ
        //Inducer_Get[6]=adc_mean_filter(ADC_IN6_A6, ADC_12BIT, 5);        //12BIT���ȣ�ȡ10��ADת��ƽ��ֵ
        for(i=0;i<4;i++)                 //��5�����ֵ���й�һ������
        {
            Inducer_Normalize[i]=1.0*Inducer_Get[i]/InducerMax[i];
        }


    }
}
