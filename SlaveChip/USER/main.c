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
    uart_init(UART_1, 115200, UART1_TX_A9, UART1_RX_A10);    //uart1初始化传输摄像头数据
    gpio_init(B4, GPI, 1,SPEED_50MHZ|IN_PULLUP);                       //初始化B4按钮，用作将InudcerMax_Get_Start_Flag置1
    //mt9v03x_init();  //摄像头初始化
    oled_init();     //oled初始化
    /**************初始化ADC*************/
    adc_init(ADC_IN0_A0);adc_init(ADC_IN4_A4);adc_init(ADC_IN6_A6);adc_init(ADC_IN8_B0);adc_init(ADC_IN9_B1);//adc_init(ADC_IN5_A5);adc_init(ADC_IN6_A6);
    pwm_init(PWM2_CH1_A15, 50, 5000);
    timer_pit_interrupt_ms(TIMER_2, 1);                    //初始化TIME2定时中断
}

int main(void)
{
    DisableGlobalIRQ();
    systick_delay_ms(300);         //延时300ms，等待设备上电
    board_init();           //务必保留，本函数用于初始化MPU 时钟 调试串口
    uart_init(UART_3,115200,UART3_TX_B10,UART3_RX_B11);
    Init_Fun();
    EnableGlobalIRQ(0);
    while(1)
    {
        //Image_Binary(123);                       //二值化
        /*if(mt9v03x_finish_flag==1)               //摄像头采集完成标志位
        {
            mt9v03x_finish_flag=0;                 //摄像头采集标志位清零
            Uart_Sendimg(UART_1,camera_buffer_addr,MT9V03X_W, MT9V03X_H);    //图像分辨率在SEEKFREE_MT9V03X.h中查看
            oled_dis_bmp(MT9V03X_H, MT9V03X_W, camera_buffer_addr,123);      //oled显示摄像头图像，最后个数据为二值化的阈值
        }*/
        //InducerMax_Get();          //InducerMax_Get_Start==1时获取最大值
        //Inducer_Processing();      //可放入定时中断中
        Inducer_Show_Oled();      //在oled上实时显示电感值
        //oled_p6x8str(0, 0, "hello world");
        systick_delay_ms(100);     //延时0.1s



    }
}
#elif 0

int main(void)
{
    DisableGlobalIRQ();
    board_init();           //务必保留，本函数用于初始化MPU 时钟 调试串口

    //此处编写用户代码(例如：外设初始化代码等)
    //总中断最后开启
    EnableGlobalIRQ(0);
    while(1)
    {

    }
}



#endif

