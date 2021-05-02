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
    uart_init(UART_3, 115200, UART3_TX_B10, UART3_RX_B11);    //uart3��ʼ����������ͷ����
    gpio_init(B4, GPI, 1,SPEED_50MHZ|IN_PULLUP);                       //��ʼ��B4��ť��������InudcerMax_Get_Start_Flag��1
    gpio_init(B5, GPI, 1,SPEED_50MHZ|IN_PULLUP);                       //��ʼ��B5��ť���������Ͷ�ֵ����ֵ
    gpio_init(B2, GPI, 1,SPEED_50MHZ|IN_PULLUP);                       //��ʼ��B2��ť���������Ӷ�ֵ����ֵ
    mt9v03x_init();  //����ͷ��ʼ��
    //oled_init();     //oled��ʼ��
    ips114_init();     //��ʼ��ips��Ļ
    /**************��ʼ��ADC*************/
    //adc_init(ADC_IN0_A0);
    adc_init(ADC_IN4_A4);adc_init(ADC_IN6_A6);adc_init(ADC_IN8_B0);adc_init(ADC_IN9_B1);//adc_init(ADC_IN5_A5);adc_init(ADC_IN6_A6);
    pwm_init(PWM2_CH1_A15, 50, 5000);
    timer_pit_interrupt_ms(TIMER_2, 5);                    //��ʼ��TIME2��ʱ�ж�
}

int main(void)
{
    DisableGlobalIRQ();
    systick_delay_ms(300);         //��ʱ300ms���ȴ��豸�ϵ�
    board_init();           //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
    Init_Fun();
    EnableGlobalIRQ(0);
    int32 count=0;
    while(1)
    {
        //Image_Binary(123);                       //��ֵ��
        if(mt9v03x_finish_flag==1)               //����ͷ�ɼ���ɱ�־λ
        {
            //Image_Binary(Threshold);                       //��ֵ��
            ips114_showint32(180,0,count++,10);
            ips114_showuint16(180,1,Threshold_ChaHe);
            //Uart_Sendimg(UART_1,camera_buffer_addr,MT9V03X_W, MT9V03X_H);    //ͼ��ֱ�����SEEKFREE_MT9V03X.h�в鿴
            //oled_dis_bmp(MT9V03X_H, MT9V03X_W, camera_buffer_addr,123);      //oled��ʾ����ͷͼ����������Ϊ��ֵ������ֵ
            Image_Processing();
            ips114_displayimage032_zoom1(mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 0, 0, MT9V03X_W*2, MT9V03X_H*2);
            for(int i=59;i>=5;i--)
            {
//                ips114_drawpoint(Image_Lline[i],i,RED);
//                ips114_drawpoint(Image_Rline[i],i,PINK);
//                ips114_drawpoint(Image_Mline[i],i,GREEN);

                ips114_drawpoint(Image_Lline[i]*2,2*i,RED);
                ips114_drawpoint(Image_Lline[i]*2,2*i+1,RED);
                ips114_drawpoint(Image_Lline[i]*2+1,2*i+1,RED);
                ips114_drawpoint(Image_Lline[i]*2+1,2*i,RED);

                ips114_drawpoint(Image_Rline[i]*2,2*i,PINK);
                ips114_drawpoint(Image_Rline[i]*2,2*i+1,PINK);
                ips114_drawpoint(Image_Rline[i]*2+1,2*i+1,PINK);
                ips114_drawpoint(Image_Rline[i]*2+1,2*i,PINK);

                ips114_drawpoint(Image_Mline[i]*2,2*i,GREEN);
                ips114_drawpoint(Image_Mline[i]*2,2*i+1,GREEN);
                ips114_drawpoint(Image_Mline[i]*2+1,2*i+1,GREEN);
                ips114_drawpoint(Image_Mline[i]*2+1,2*i,GREEN);


            }
            //ips114_displayimage032_zoom1(mt9v03x_image_binary[0], MT9V03X_W, MT9V03X_H, 0, 65, MT9V03X_W, MT9V03X_H);
//            ips114_displayimage032_zoom(mt9v03x_image_binary[0], MT9V03X_W, MT9V03X_H, MT9V03X_W*2, MT9V03X_H*2);
//            ips114_displayimage032_zoom(mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W*2, MT9V03X_H*2);
            mt9v03x_finish_flag=0;                 //����ͷ�ɼ���־λ����
        }
        //InducerMax_Get();          //InducerMax_Get_Start==1ʱ��ȡ���ֵ
        //Inducer_Processing();      //�ɷ��붨ʱ�ж���
        //Inducer_Show_Oled();      //��oled��ʵʱ��ʾ���ֵ
        //Inducer_Show_Ips();
        //systick_delay_ms(100);     //��ʱ0.1s

    }
}
#elif 0

int main(void)
{
    DisableGlobalIRQ();
    board_init();           //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���

    //�˴���д�û�����(���磺�����ʼ�������)
    //���ж������
    EnableGlobalIRQ(0);
    while(1)
    {

    }
}



#endif

