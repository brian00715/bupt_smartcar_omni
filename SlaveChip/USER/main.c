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
#include "isr.h"
#include "master_comm.h"

//#if -#elif -#endif
#if 1

//摄像头调试
void Init_Fun(void)
{
    /**************初始化UART3*************/
    uart_init(UART_3, 115200, UART3_TX_B10, UART3_RX_B11); //uart3初始化传输摄像头数据
    // >>>DMA方式接收数据<<<
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); // 开启闲时中断
    nvic_init(USART3_IRQn, 1, 1, ENABLE); // 配置UART NVIC
    UART_DMA_ReceiveInit(USART3, DMA1_Channel3, (u32)(&USART3->DATAR),
                             (uint32)UART3_RxBuffer, UART3_RX_BUFFER_SIZE); // USART DMA初始化
    nvic_init(DMA1_Channel3_IRQn, 1, 2, ENABLE);                        // 配置DMA NVIC

    /**************初始化按键*************/
    gpio_init(C8, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //初始化C8按钮，用作启动电机自检
    gpio_init(C9, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //初始化C9按钮，用作切换IPS内容
    gpio_init(B2, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //初始化B2按钮，用作控制屏幕显示图像或从机数据发送，两者不同时进行
    gpio_init(B15, GPO, 0, GPIO_PIN_CONFIG);

    //    mt9v03x_init();                                        //摄像头初始化
    ips114_init(); //初始化ips屏幕

    /**************初始化ADC*************/
    //adc_init(ADC_IN0_A0);
    adc_init(ADC_IN4_A4);
    adc_init(ADC_IN6_A6);
    adc_init(ADC_IN8_B0);
    adc_init(ADC_IN9_B1); //adc_init(ADC_IN5_A5);adc_init(ADC_IN6_A6);

    Encoder_Init();    //编码器初始化
    Image_Show_Flag = 0; // 上电后就给主片发数据

    /**************初始化定时器*************/
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

    uint8 cnt_50ms=0;
    while (1)
    {
        ips114_showstr(IPS114_X_MAX/4, 0, "---SlaveChip---");
        if (Image_Show_Flag==0)
        {
            ips114_showstr(0, 1, "Transferring to MasterChip...");
            ips114_showstr(0,2,"encoder3:");
            ips114_showint16(73, 2, encoder_data[0]);
            ips114_showstr(0,3,"encoder4:");
            ips114_showint16(73, 3, encoder_data[1]);

            ips114_showstr(0,4,"rpm1:");
            ips114_showfloat(41, 4, wheel_rpm[0],4,2);
            ips114_showstr(0,5,"rpm1:");
            ips114_showfloat(41, 5, wheel_rpm[1],4,2);
            ips114_showstr(0,6,"rpm1:");
            ips114_showfloat(41, 6, wheel_rpm[2],4,2);
            ips114_showstr(0,7,"rpm1:");
            ips114_showfloat(41, 7, wheel_rpm[3],4,2);
        }
        else
        {
            ips114_showstr(0, 1, "Stopped Transferring.");
        }

        cnt_50ms++;
        if (cnt_50ms==5)
        {
            cnt_50ms = 0;
            gpio_toggle(B15);
        }
        systick_delay_ms(50);
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
