/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            main
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ790875685)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
//�����Ƽ�IO�鿴Projecct�ļ����µ�TXT�ı�

//���µĹ��̻��߹����ƶ���λ�����ִ�����²���
//�Ҽ��������̣�ѡ��ˢ��

#include "headfile.h"
#include "isr.h"
#include "master_comm.h"

//#if -#elif -#endif
#if 1

//����ͷ����
void Init_Fun(void)
{
    /**************��ʼ��UART3*************/
    uart_init(UART_3, 115200, UART3_TX_B10, UART3_RX_B11); //uart3��ʼ����������ͷ����
    // >>>DMA��ʽ��������<<<
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); // ������ʱ�ж�
    nvic_init(USART3_IRQn, 1, 1, ENABLE); // ����UART NVIC
    UART_DMA_ReceiveInit(USART3, DMA1_Channel3, (u32)(&USART3->DATAR),
                             (uint32)UART3_RxBuffer, UART3_RX_BUFFER_SIZE); // USART DMA��ʼ��
    nvic_init(DMA1_Channel3_IRQn, 1, 2, ENABLE);                        // ����DMA NVIC

    /**************��ʼ������*************/
    gpio_init(C8, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //��ʼ��C8��ť���������Ӷ�ֵ����ֵ
    gpio_init(C9, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //��ʼ��C9��ť���������Ͷ�ֵ����ֵ
    gpio_init(B2, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //��ʼ��B2��ť������������Ļ��ʾͼ���ӻ����ݷ��ͣ����߲�ͬʱ����
    gpio_init(B15, GPO, 0, GPIO_PIN_CONFIG);

    //    mt9v03x_init();                                        //����ͷ��ʼ��
    ips114_init(); //��ʼ��ips��Ļ

    /**************��ʼ��ADC*************/
    //adc_init(ADC_IN0_A0);
    adc_init(ADC_IN4_A4);
    adc_init(ADC_IN6_A6);
    adc_init(ADC_IN8_B0);
    adc_init(ADC_IN9_B1); //adc_init(ADC_IN5_A5);adc_init(ADC_IN6_A6);

    Encoder_Init();    //��������ʼ��
    Image_Show_Flag = 0; // �ϵ��͸���Ƭ������

    /**************��ʼ����ʱ��*************/
//    pwm_init(PWM2_CH1_A15, 50, 5000);
    timer_pit_interrupt_ms(TIMER_4, 5); //��ʼ��TIME4��ʱ�ж�
}

int main(void)
{
    DisableGlobalIRQ();
    systick_delay_ms(300); //��ʱ300ms���ȴ��豸�ϵ�
    board_init();          //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
    Init_Fun();
    EnableGlobalIRQ(0);

    uint8 cnt_50ms=0;
    while (1)
    {
        ips114_showstr(0, 0, "---SlaveChip---");
        if (Image_Show_Flag==0)
        {
            ips114_showstr(0, 1, "Transferring to MasterChip...");
            ips114_showint16(0, 2, encoder_data[0]);
            ips114_showint16(0, 3, encoder_data[1]);

            ips114_showfloat(0, 4, wheel_rpm[0],4,2);
            ips114_showfloat(0, 5, wheel_rpm[1],4,2);
            ips114_showfloat(0, 6, wheel_rpm[2],4,2);
            ips114_showfloat(0, 7, wheel_rpm[3],4,2);
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
    board_init(); //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���

    //�˴���д�û�����(���磺�����ʼ�������)
    //���ж������
    EnableGlobalIRQ(0);
    while (1)
    {
    }
}

#endif
