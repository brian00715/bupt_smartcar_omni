/**
 * @file slave_comm.c
 * @author simon
 * @brief ��ӻ�֮���ͨ��
 * @version 0.1
 * @date 2021-05-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "slave_comm.h"
#include "headfile.h"
#include "isr.h"
#include "board.h"
#include "cmd.h"
#include <stdlib.h>
#include "encoder.h"
#include "mecanum_chassis.h"
#include "sci_compute.h"

void SlaveComm_Init()
{
    uart_init(UART_3, 115200, UART3_TX_B10, UART3_RX_B11);
    // >>>DMA��ʽ��������<<<
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); // ������ʱ�ж�
    nvic_init(USART3_IRQn, 1, 1, ENABLE); // ����UART NVIC
    UART_DMA_ReceiveInit(USART3, DMA1_Channel3, (u32)(&USART3->DATAR),
                         (uint32)UART3_RxBuffer, UART3_RX_BUFFER_SIZE); // USART DMA��ʼ��
    nvic_init(DMA1_Channel3_IRQn, 1, 2, ENABLE);                        // ����DMA NVIC
}

uint8 EncoderDataUpdated = 0; // ���յ��ӻ����ݱ�������ֵ�Ż���£������¾Ͳ������ٶȻ�
/**
 * @brief �ӻ�ͨ�Ŵ����жϻص�����
 * 
 */
void SlaveComm_UARTCallback()
{
    // �����ӻ����ݰ�
    if (!(UART3_RxBuffer[0] == 0x00 && UART3_RxBuffer[1] == 0xff))
    {
        return;
    }
    encoder_data[2] = UART3_RxBuffer[2] << 8 | UART3_RxBuffer[3];
    encoder_data[3] = UART3_RxBuffer[4] << 8 | UART3_RxBuffer[5];
    if (MecanumChassis.motor_self_check_ok)
    {
        encoder_data[2] *= encoder_coff[2];
        encoder_data[3] *= encoder_coff[3];
    }
    float encoder_rpm[4]; // ������ת��
    for(int i=0;i<4;i++)
    {
        encoder_rpm[i] = encoder_data[i]*1.0f/(512*0.005f)*60.0f; // 512�߱���������������5ms
        MecanumChassis.motor[i].now_rpm = encoder_rpm[i]*45.0f/104.0f;// ������45�ݣ�����104��
    }

//    MecanumChassis.motor[0].now_rpm = encoder_data[0]; //ֱ��ʹ�ñ�������ֵ��Ϊ��ǰת��
//    MecanumChassis.motor[1].now_rpm = encoder_data[1];
//    MecanumChassis.motor[2].now_rpm = encoder_data[2];
//    MecanumChassis.motor[3].now_rpm = encoder_data[3];
    EncoderDataUpdated = 1;

    memset(UART3_RxBuffer, 0, sizeof(uint8_t) * UART3_RX_BUFFER_SIZE);
    UART3_RxOK = 1;
}


void SlaveComm_Exe()
{
     if (UART3_RxOK)
     {
         uart_putchar(UART_3, 0x00);
         uart_putchar(UART_3, 0xff);
         // ����ת������
         uint8 transfer_buffer[4];
         for(int i=0;i<4;i++)
         {
             float2buffer(MecanumChassis.motor[i].now_rpm, transfer_buffer);
             uart_putbuff(UART_3, transfer_buffer, 4);
         }
         uart_putchar(UART_3, 0xee);
         uart_putchar(UART_3, 0x11);

         UART3_RxOK = 0;
     }
}
