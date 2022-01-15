/**
 * @file slave_comm.c
 * @author simon
 * @brief 与从机之间的通信
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
    // >>>DMA方式接收数据<<<
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); // 开启闲时中断
    nvic_init(USART3_IRQn, 1, 1, ENABLE);          // 配置UART NVIC
    UART_DMA_ReceiveInit(USART3, DMA1_Channel3, (u32)(&USART3->DATAR),
                         (uint32)UART3_RxBuffer, UART3_RX_BUFFER_SIZE); // USART DMA初始化
    nvic_init(DMA1_Channel3_IRQn, 1, 2, ENABLE);                        // 配置DMA NVIC
}

uint8 EncoderDataUpdated = 0; // 接收到从机数据编码器数值才会更新，不更新就不能跑速度环
uint8 SlaveComm_MotorSelfCheck = 0; // 来自从片的自检命令
/**
 * @brief 从机通信串口中断回调函数
 * 
 */
void SlaveComm_UARTCallback()
{
    // 解析从机数据包
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

    for (int i = 0; i < 4; i++)
    {
        MecanumChassis.motor[i].now_rpm = Motor_Encoder2RPM(encoder_data[i]);
    }

    EncoderDataUpdated = 1;
    if( UART3_RxBuffer[6]==1)
    {
        SlaveComm_MotorSelfCheck = 1; // 外部置0
    }

    memset(UART3_RxBuffer, 0, sizeof(uint8_t) * UART3_RX_BUFFER_SIZE);
    UART3_RxOK = 1;
}

void SlaveComm_Exe()
{
    if (UART3_RxOK)
    {
        uart_putchar(UART_3, 0x00);
        uart_putchar(UART_3, 0xff);
        // 发送转速数据
        uint8 transfer_buffer[4];
        for (int i = 0; i < 4; i++)
        {
            float2buffer(MecanumChassis.motor[i].now_rpm, transfer_buffer);
            uart_putbuff(UART_3, transfer_buffer, 4);
        }
        uart_putchar(UART_3, 0xee);
        uart_putchar(UART_3, 0x11);

        UART3_RxOK = 0;
    }
}
