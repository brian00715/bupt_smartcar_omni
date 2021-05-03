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


void SlaveComm_Init()
{
    uart_init(UART_3, 460800, UART3_TX_B10, UART3_RX_B11);
    nvic_init(USART3_IRQn, 0, 0, ENABLE); // 配置UART NVIC
    // >>>DMA方式接收数据<<<
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);                                   // 开启闲时中断
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);                                   // 使能UART DMA传输
    UART_DMA_Init(DMA1_Channel3, (u32)(&USART3->DATAR), (uint32)UART3_RxBuffer, 20); // USART DMA初始化
    nvic_init(DMA1_Channel3_IRQn, 0, 0, ENABLE);                                     // 配置DMA NVIC
}

/**
 * @brief 从机通信串口中断回调函数
 * 
 */
void SlaveComm_UARTCallback()
{
    UART3_RxOK = 1;
    uprintf("%s", UART3_RxBuffer);
    memset(UART3_RxBuffer, 0, sizeof(uint8_t) * 20);
}
