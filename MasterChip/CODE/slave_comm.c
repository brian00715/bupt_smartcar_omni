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
#include "path_following.h"

void SlaveComm_Init()
{
    uart_init(UART_3, 256000, UART3_TX_B10, UART3_RX_B11);
    nvic_init(USART3_IRQn, 0, 2, ENABLE); // 配置UART NVIC
    // >>>DMA方式接收数据<<<
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); // 开启闲时中断
    UART_DMA_ReceiveInit(USART3, DMA1_Channel3, (u32)(&USART3->DATAR),
                         (uint32)UART3_RxBuffer, UART3_RX_BUFFER_SIZE); // USART DMA初始化
    nvic_init(DMA1_Channel3_IRQn, 0, 0, ENABLE);                        // 配置DMA NVIC
                                                                        // >>>中断方式接收数据<<<
                                                                        //	uart_rx_irq(UART_3, ENABLE); // 使能串口接收中断
}

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
    MecanumChassis.motor[0].now_rpm = encoder_data[0];
    MecanumChassis.motor[1].now_rpm = encoder_data[1];
    MecanumChassis.motor[2].now_rpm = encoder_data[2];
    MecanumChassis.motor[3].now_rpm = encoder_data[3];
    MecanumChassis.PathFollowing.image_process_done = UART3_RxBuffer[6];
    if (MecanumChassis.PathFollowing.image_process_done == 1) // 0:未处理完图像 1: 处理完图像
    {
        MecanumChassis.PathFollowing.heading_err = ((int16)(UART3_RxBuffer[7] << 8 | UART3_RxBuffer[8])) * 1.0;
        MecanumChassis.PathFollowing.heading_err = atan(
            MecanumChassis.PathFollowing.heading_err / 1000.0);
        MecanumChassis.PathFollowing.normal_err = UART3_RxBuffer[9];
        MecanumChassis.PathFollowing.state = UART3_RxBuffer[10]; // 赛道元素标志
    }
    memset(UART3_RxBuffer, 0, sizeof(uint8_t) * UART3_RX_BUFFER_SIZE);
    UART3_RxOK = 1;
}

void SlaveComm_Exe()
{
    // if (UART3_RxOK)
    // {
    //     uprintf(
    //         "SlaveData|encoder-[0]:%4d [1]:%4d [2]:%4d [3]:%4d imageOK:%d angle:%6.2f LCurve:%d RCurve:%d\r\n",
    //         MecanumChassis.motor[0].now_rpm, MecanumChassis.motor[1].now_rpm,
    //         MecanumChassis.motor[2].now_rpm, MecanumChassis.motor[3].now_rpm,
    //         MecanumChassis.PathFollowing.image_process_done,
    //         MecanumChassis.PathFollowing.heading_err,
    //         MecanumChassis.PathFollowing.meet_left_big_curve,
    //         MecanumChassis.PathFollowing.meet_right_big_curve);
    //     UART3_RxOK = 0;
    // }
}
