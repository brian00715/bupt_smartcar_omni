/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            main
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ790875685)
 * @version         查看doc内version文件 版本说明
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
//整套推荐IO查看Projecct文件夹下的TXT文本

//打开新的工程或者工程移动了位置务必执行以下操作
//右键单击工程，选择刷新

#include "headfile.h"

//#if -#elif -#endif
#if 1

//摄像头调试
void Init_Fun(void)
{
    uart_init(UART_3, 256000, UART3_TX_B10, UART3_RX_B11); //uart3初始化传输摄像头数据
    gpio_init(C8, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //初始化C8按钮，用作增加二值化阈值
    gpio_init(C9, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //初始化C9按钮，用作降低二值化阈值
    gpio_init(B2, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //初始化B2按钮，用作控制屏幕显示图像或从机数据发送，两者不同时进行
    mt9v03x_init();                                        //摄像头初始化
    //oled_init();     //oled初始化
    ips114_init(); //初始化ips屏幕
    /**************初始化ADC*************/
    //adc_init(ADC_IN0_A0);
    adc_init(ADC_IN4_A4);
    adc_init(ADC_IN6_A6);
    adc_init(ADC_IN8_B0);
    adc_init(ADC_IN9_B1); //adc_init(ADC_IN5_A5);adc_init(ADC_IN6_A6);
    Encoder_Init();    //编码器初始化
//    pwm_init(PWM2_CH1_A15, 50, 5000);
    timer_pit_interrupt_ms(TIMER_4, 5); //初始化TIME4定时中断
}

int main(void)
{
    DisableGlobalIRQ();
    systick_delay_ms(300); //延时300ms，等待设备上电
    board_init();          //务必保留，本函数用于初始化MPU 时钟 调试串口
    Init_Fun();
    EnableGlobalIRQ(0);
    while (1)
    {
        //Image_Binary(123);                       //二值化
        // Image_Processing();
        Img_Prc();
        ips114_showint32(180, 0, count, 10);
        if (Image_Process_Flag == 1) //摄像头处理完成标志位
        {
            if (Image_Show_Flag == 1)
            {
                //Image_Binary(Threshold);                       //二值化
//                Image_Processing();
//                ips114_showint32(180, 0, count, 10);
                ips114_showuint16(188, 1, Threshold_ChaHe);
                // ips114_showuint8(188, 2, Image_MTuBian_Sum);
                // ips114_showuint8(188, 3, Image_MTuBian[0]);
                // ips114_showuint8(188, 4, Image_MGuaiDian_Sum);
                // ips114_showuint8(188, 5, Image_MGuaiDian[0]);
//                ips114_showuint8(180, 3, Image_LBig_Curve_Flag);
//                ips114_showuint8(180, 4, Image_RBig_Curve_Flag);
//                ips114_showfloat(188, 6, Image_XieLv_float[0], 2, 3);
//                ips114_showfloat(188, 7, Image_XieLv_float[1], 2, 3);
                ips114_showuint8(188, 2, Image_Llost_Sum);
                ips114_showuint8(188, 3, Image_Llost[0]);
                ips114_showuint8(188, 4, Image_Rlost_Sum);
                ips114_showuint8(188, 5, Image_Rlost[0]);
                // ips114_showfloat(188, 7, Image_MAdd_float, 2, 3);
                ips114_showint16(188, 7, Image_Error);
                //Uart_Sendimg(UART_1,camera_buffer_addr,MT9V03X_W, MT9V03X_H);    //图像分辨率在SEEKFREE_MT9V03X.h中查看
                //oled_dis_bmp(MT9V03X_H, MT9V03X_W, camera_buffer_addr,123);      //oled显示摄像头图像，最后个数据为二值化的阈值
                ips114_displayimage032_zoom1(mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 0, 0, MT9V03X_W , MT9V03X_H * 2);
                for (int i = 49; i >= 5; i--)
                {
                    //                ips114_drawpoint(Image_Lline[i],i,RED);
                    //                ips114_drawpoint(Image_Rline[i],i,PINK);
                    //                ips114_drawpoint(Image_Mline[i],i,GREEN);
//                    ips114_drawpoint(Image_Lline[i] * 2, 2 * i, RED);
//                    ips114_drawpoint(Image_Lline[i] * 2, 2 * i + 1, RED);
//                    ips114_drawpoint(Image_Lline[i] * 2 + 1, 2 * i + 1, RED);
//                    ips114_drawpoint(Image_Lline[i] * 2 + 1, 2 * i, RED);
//
//                    ips114_drawpoint(Image_Rline[i] * 2, 2 * i, PINK);
//                    ips114_drawpoint(Image_Rline[i] * 2, 2 * i + 1, PINK);
//                    ips114_drawpoint(Image_Rline[i] * 2 + 1, 2 * i + 1, PINK);
//                    ips114_drawpoint(Image_Rline[i] * 2 + 1, 2 * i, PINK);
//
//                    ips114_drawpoint(Image_Mline[i] * 2, 2 * i, GREEN);
//                    ips114_drawpoint(Image_Mline[i] * 2, 2 * i + 1, GREEN);
//                    ips114_drawpoint(Image_Mline[i] * 2 + 1, 2 * i + 1, GREEN);
//                    ips114_drawpoint(Image_Mline[i] * 2 + 1, 2 * i, GREEN);
                    ips114_drawpoint(Image_Lline[i], 2 * i, RED);
                    ips114_drawpoint(Image_Lline[i], 2 * i + 1, RED);
                    ips114_drawpoint(Image_Rline[i], 2 * i, PINK);
                    ips114_drawpoint(Image_Rline[i], 2 * i + 1, PINK);
                    ips114_drawpoint(Image_Mline[i], 2 * i, GREEN);
                    ips114_drawpoint(Image_Mline[i], 2 * i + 1, GREEN);
                }
//                for(int i=0;i>Image_GuaiDian_Sum;i++)
//                {
//                    ips114_drawpoint(Image_Mline[Image_GuaiDian[i]] * 2, 2 * Image_GuaiDian[i], YELLOW);
//                    ips114_drawpoint(Image_Mline[Image_GuaiDian[i]] * 2, 2 * Image_GuaiDian[i] + 1, YELLOW);
//                    ips114_drawpoint(Image_Mline[Image_GuaiDian[i]] * 2 + 1, 2 * Image_GuaiDian[i] + 1, YELLOW);
//                    ips114_drawpoint(Image_Mline[Image_GuaiDian[i]] * 2 + 1, 2 * Image_GuaiDian[i], YELLOW);
//                }
//                for(int i=0;i>Image_TuBian_Sum;i++)
//                {
//                    ips114_drawpoint(Image_Mline[Image_TuBian[i]] * 2, 2 * Image_TuBian[i], BLUE);
//                    ips114_drawpoint(Image_Mline[Image_TuBian[i]] * 2, 2 * Image_TuBian[i] + 1, BLUE);
//                    ips114_drawpoint(Image_Mline[Image_TuBian[i]] * 2 + 1, 2 * Image_TuBian[i] + 1, BLUE);
//                    ips114_drawpoint(Image_Mline[Image_TuBian[i]] * 2 + 1, 2 * Image_TuBian[i], BLUE);
//                }
            }
            //InducerMax_Get();          //InducerMax_Get_Start==1时获取最大值
            //Inducer_Processing();      //可放入定时中断中
            //Inducer_Show_Oled();      //在oled上实时显示电感值
            //Inducer_Show_Ips();
        }
    }
}
#elif 0

int main(void)
{
    DisableGlobalIRQ();
    board_init(); //务必保留，本函数用于初始化MPU 时钟 调试串口

    //此处编写用户代码(例如：外设初始化代码等)
    //总中断最后开启
    EnableGlobalIRQ(0);
    while (1)
    {
    }
}

#endif
