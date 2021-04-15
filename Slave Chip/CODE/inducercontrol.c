/*
 * inducercontrol.c
 * 电感处理函数
 *
 *  Created on: 2021年4月13日
 *      Author: Z.yd
 */

#include "headfile.h"

uint8 InducerMax_Get_Start_Flag=1;         //电感最大值获取开始标志,可通过按键控制该标志位实现重新收集电感最大值
uint8 InducerMax_Get_End_Flag=0;           //电感最大值获取结束标志，当该标志为1时，电感数据处理函数才会处理数据
uint16 Inducer_Get_Count=1000;             //每个电感获取最大值时get的数量
uint16 InducerMax[7]={0,0,0,0,0,0,0};
//uint16 Inducer_Get_Last[7]={0,0,0,0,0,0,0};
uint16 Inducer_Get[7]={0,0,0,0,0,0,0};           //ADC读取的电感值
float Inducer_Normalize[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  //归一化后的电感值，为Inducer_Get[i]/InducerMax[i]
//float Inducer_Duty[7]={-10,-9,-8,0,8,9,10};                //设置每个电感的权值,每个电感归一化后的值乘以权值即为最终x方向上的值(特殊情况需特殊处理)
float Inducer_Duty[5]={-10,-8,0,8,10};
float Inducer_Sum_x=0.0;                                   //电感合成后的x方向值,Inducer_Normalize[i]*Inducer_Duty[i]的和;
float Inducer_Sum_y=3.0;                                   //自定义的电感y方向值
float Inducer_Slope_Float=0.0;                             //电感数据处理后得到的斜率，Inducer_Sum_x/Inducer_Sum_y
char Inducer_Slope_Str[20];                                //电感数据处理后得到的斜率转变为字符串供串口传送,snprintf(Inducer_Slope_Str,15,"%f",Inducer_Slope_Float);
int i=0,j=0;    //循环次数辅助

/************************************************************************************
  * @brief        将七个电感值在oled上实时显示
  * @param        NULL
  * @return       void
  * Sample usage:
  **********************************************************************************/
void Inducer_Show_Oled(void)
{
    oled_fill(0x00);
    for(i=0;i<5;i++)
    {
        Inducer_Get[i]=0;
    }
    //InducerMax_Get();                              //收集电感最大值，以进行归一化
    Inducer_Get[0]=adc_mean_filter(ADC_IN0_A0,ADC_12BIT,10);        //12BIT分辨率,取10次平均
    Inducer_Get[1]=adc_mean_filter(ADC_IN4_A4,ADC_12BIT,10);
    Inducer_Get[2]=adc_mean_filter(ADC_IN6_A6,ADC_12BIT,10);
    Inducer_Get[3]=adc_mean_filter(ADC_IN8_B0,ADC_12BIT,10);
    Inducer_Get[4]=adc_mean_filter(ADC_IN9_B1,ADC_12BIT,10);
    //Inducer_Get[5]=adc_mean_filter(ADC_IN5_A5,ADC_12BIT,5);        //12BIT分辨率,取10次平均
    //Inducer_Get[6]=adc_mean_filter(ADC_IN6_A6,ADC_12BIT,5);        //12BIT分辨率,取10次平均

    for(i=0;i<5;i++)                 //显示ADC读取原电感值
    {
        oled_uint16(0, i,Inducer_Get[i]);
       // printf("[%d]%d ",i,Inducer_Get[i]);
    }
    //printf("\r\n");

    if(InducerMax_Get_End_Flag==1)       //显示归一化后的电感值
    {
        for(i=0;i<5;i++)
        {
            Inducer_Normalize[i]=1.0*Inducer_Get[i]/InducerMax[i];
            oled_printf_float(60, i, Inducer_Normalize[i], 1,6);
        }
    }
}


/************************************************************************************
  * @brief        获取七个电感最大值，以方便后续归一化处理
  * @param        NULL
  * @return       void
  * Sample usage:          可通过修改Inducer_Get_Count改变每个电感get值的数量
  **********************************************************************************/
void InducerMax_Get(void)
{
    if(InducerMax_Get_Start_Flag==1)        //如果电感获取值标志为1则获取电感最大值
    {
        oled_fill(0x00);
        oled_p6x8str(0, 0,"Wait 2s...");
        systick_delay_ms(2000);
        InducerMax_Get_End_Flag=0;          //电感最大值获取结束标志清零
        oled_fill(0x00);
        oled_p6x8str(0, 0,"Inducer Getting");
        for(i=0;i<5;i++)
        {
            InducerMax[i]=0;Inducer_Get[i]=0;    //相关变量清零
        }
        /***************电感0最大值获取**********************************************************/
        oled_p6x8str(0, 1,"Inducer0:");
        for(j=0;j<Inducer_Get_Count;j++)      //每个电感采集数据数量
        {
            Inducer_Get[0]=adc_mean_filter(ADC_IN0_A0,ADC_12BIT, 10);          //12BIT分辨率,取10次平均
            if(Inducer_Get[0]>InducerMax[0])
            {
                InducerMax[0]=Inducer_Get[0];
                oled_int16(90, 1, InducerMax[0]);
            }
            systick_delay_ms(5);               //延时5ms
        }
        oled_fill_page(1,0x00);
        oled_p6x8str(0,1,"Inducer0:");
        oled_uint16(90,1,InducerMax[0]);
        /***************电感1最大值获取*********************************************************/
        oled_p6x8str(0, 2,"Inducer1:");
        for(j=0;j<Inducer_Get_Count;j++)      //每个电感采集数据数量
        {
            Inducer_Get[1]=adc_mean_filter(ADC_IN4_A4,ADC_12BIT, 10);          //12BIT分辨率,取10次平均
            if(Inducer_Get[1]>InducerMax[1])
            {
                InducerMax[1]=Inducer_Get[1];
                oled_int16(90, 2, InducerMax[1]);
            }
            systick_delay_ms(5);               //延时5ms
        }
        oled_fill_page(2,0x00);
        oled_p6x8str(0,2,"Inducer1:");
        oled_uint16(90,2,InducerMax[1]);
        /***************电感2最大值获取*********************************************************/
        oled_p6x8str(0, 3,"Inducer2:");
        for(j=0;j<Inducer_Get_Count;j++)      //每个电感采集数据数量
        {
            Inducer_Get[2]=adc_mean_filter(ADC_IN6_A6,ADC_12BIT, 10);          //12BIT分辨率,取10次平均
            if(Inducer_Get[2]>InducerMax[2])
            {
                InducerMax[2]=Inducer_Get[2];
                oled_int16(90, 3, InducerMax[2]);
            }
            systick_delay_ms(5);               //延时5ms
        }
        oled_fill_page(3,0x00);
        oled_p6x8str(0,3,"Inducer2:");
        oled_uint16(90,3,InducerMax[2]);
        /***************电感3最大值获取********************************************************/
        oled_p6x8str(0, 4,"Inducer3:");
        for(j=0;j<Inducer_Get_Count;j++)      //每个电感采集数据数量
        {
            Inducer_Get[3]=adc_mean_filter(ADC_IN8_B0,ADC_12BIT, 10);          //12BIT分辨率,取10次平均
            if(Inducer_Get[3]>InducerMax[3])
            {
                InducerMax[3]=Inducer_Get[3];
                oled_int16(90, 4, InducerMax[3]);
            }
            systick_delay_ms(5);               //延时5ms
        }
        oled_fill_page(4,0x00);
        oled_p6x8str(0,4,"Inducer3:");
        oled_uint16(90,4,InducerMax[3]);
        /***************电感4最大值获取*******************************************************/
        oled_p6x8str(0, 5,"Inducer4:");
        for(j=0;j<Inducer_Get_Count;j++)      //每个电感采集数据数量
        {
            Inducer_Get[4]=adc_mean_filter(ADC_IN9_B1,ADC_12BIT, 10);          //12BIT分辨率,取10次平均
            if(Inducer_Get[4]>InducerMax[4])
            {
                InducerMax[4]=Inducer_Get[4];
                oled_int16(90, 5, InducerMax[4]);
            }
            systick_delay_ms(5);               //延时5ms
        }
        oled_fill_page(5,0x00);
        oled_p6x8str(0,5,"Inducer4:");
        oled_uint16(90,5,InducerMax[4]);

        /****************电感最大值获取结束**************************************************/
        systick_delay_ms(3000);                  //延时3秒，方便通过oled读数
        if(InducerMax[0]!=0&&InducerMax[1]!=0&&InducerMax[2]!=0&&InducerMax[3]!=0&&InducerMax[4]!=0)//&&InducerMax[5]!=0&&InducerMax[6]!=0)//如果其中有电感值为零，则不算电感最大值获取完成
        {
            InducerMax_Get_End_Flag=1;                   //电感最大值获取结束，标志位置1
            InducerMax_Get_Start_Flag=0;                 //电感获取开始标志清零
            oled_fill(0x00);
            oled_p8x16str(0, 3, "Getting Complete!");
            systick_delay_ms(3000);
        }
    }
}


/************************************************************************************
  * @brief        处理电感值，获取路径斜率、特殊路况标志，并将数据通过串口发送给主片
  * @param        NULL
  * @return       void
  * Sample usage:
  **********************************************************************************/
void Inducer_Processing(void)
{
    //InducerMax_Get();                                //若还未获取电感最大值，则
    if(InducerMax_Get_End_Flag==1)                   //电感获取最大值标志为1时才进行数据处理，避免有最大值为0、后续除法出错的危险
    {
        Inducer_Sum_x=0.0;                           //初始化电感乘以权值后的和
        Inducer_Slope_Float=0.0;                     //初始化数据处理后的斜率
        for(i=0;i<5;i++)                             //初始化数据
        {
            Inducer_Get[i]=0;Inducer_Normalize[i]=0.0;
        }
        Inducer_Get[0]=adc_mean_filter(ADC_IN0_A0, ADC_12BIT, 10);        //12BIT精度，取10次AD转换平均值
        Inducer_Get[1]=adc_mean_filter(ADC_IN4_A4, ADC_12BIT, 10);        //12BIT精度，取10次AD转换平均值
        Inducer_Get[2]=adc_mean_filter(ADC_IN6_A6, ADC_12BIT, 10);        //12BIT精度，取10次AD转换平均值
        Inducer_Get[3]=adc_mean_filter(ADC_IN8_B0, ADC_12BIT, 10);        //12BIT精度，取10次AD转换平均值
        Inducer_Get[4]=adc_mean_filter(ADC_IN9_B1, ADC_12BIT, 10);        //12BIT精度，取10次AD转换平均值
        //Inducer_Get[5]=adc_mean_filter(ADC_IN5_A5, ADC_12BIT, 5);        //12BIT精度，取10次AD转换平均值
        //Inducer_Get[6]=adc_mean_filter(ADC_IN6_A6, ADC_12BIT, 5);        //12BIT精度，取10次AD转换平均值
        for(i=0;i<5;i++)                 //对5个电感值进行归一化处理
        {
            Inducer_Normalize[i]=1.0*Inducer_Get[i]/InducerMax[i];
        }


    }
}
