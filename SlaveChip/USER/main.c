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

//#if -#elif -#endif
#if 1

//����ͷ����
void Init_Fun(void)
{
    uart_init(UART_3, 256000, UART3_TX_B10, UART3_RX_B11); //uart3��ʼ����������ͷ����
    gpio_init(C8, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //��ʼ��C8��ť���������Ӷ�ֵ����ֵ
    gpio_init(C9, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //��ʼ��C9��ť���������Ͷ�ֵ����ֵ
    gpio_init(B2, GPI, 1, SPEED_50MHZ | IN_PULLUP);        //��ʼ��B2��ť������������Ļ��ʾͼ���ӻ����ݷ��ͣ����߲�ͬʱ����
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
    while (1)
    {
        ips114_showstr(0, 0, "---SlaveChip---");
        if (Image_Show_Flag==0)
        {
            ips114_showstr(0, 1, "Transferring to MasterChip...");
            ips114_showint16(0, 2, encoder_data[0]);
            ips114_showint16(0, 3, encoder_data[1]);
        }
        else
        {
            ips114_showstr(0, 1, "Stopped Transferring.");
        }
        systick_delay_ms(10);
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
