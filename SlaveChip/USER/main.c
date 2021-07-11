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
    mt9v03x_init();                                        //����ͷ��ʼ��
    //oled_init();     //oled��ʼ��
    ips114_init(); //��ʼ��ips��Ļ
    /**************��ʼ��ADC*************/
    //adc_init(ADC_IN0_A0);
    adc_init(ADC_IN4_A4);
    adc_init(ADC_IN6_A6);
    adc_init(ADC_IN8_B0);
    adc_init(ADC_IN9_B1); //adc_init(ADC_IN5_A5);adc_init(ADC_IN6_A6);
    Encoder_Init();    //��������ʼ��
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
        //Image_Binary(123);                       //��ֵ��
        // Image_Processing();
        Img_Prc();
        ips114_showint32(180, 0, count, 10);
        if (Image_Process_Flag == 1) //����ͷ������ɱ�־λ
        {
            if (Image_Show_Flag == 1)
            {
                //Image_Binary(Threshold);                       //��ֵ��
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
                //Uart_Sendimg(UART_1,camera_buffer_addr,MT9V03X_W, MT9V03X_H);    //ͼ��ֱ�����SEEKFREE_MT9V03X.h�в鿴
                //oled_dis_bmp(MT9V03X_H, MT9V03X_W, camera_buffer_addr,123);      //oled��ʾ����ͷͼ����������Ϊ��ֵ������ֵ
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
            //InducerMax_Get();          //InducerMax_Get_Start==1ʱ��ȡ���ֵ
            //Inducer_Processing();      //�ɷ��붨ʱ�ж���
            //Inducer_Show_Oled();      //��oled��ʵʱ��ʾ���ֵ
            //Inducer_Show_Ips();
        }
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
