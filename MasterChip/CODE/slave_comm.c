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


void SlaveComm_Init()
{
    uart_init(UART_3, 460800, UART3_TX_B10, UART3_RX_B11);
    nvic_init(USART3_IRQn, 0, 0, ENABLE); // ����UART NVIC
    // >>>DMA��ʽ��������<<<
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);                                   // ������ʱ�ж�
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);                                   // ʹ��UART DMA����
    UART_DMA_Init(DMA1_Channel3, (u32)(&USART3->DATAR), (uint32)UART3_RxBuffer, 20); // USART DMA��ʼ��
    nvic_init(DMA1_Channel3_IRQn, 0, 0, ENABLE);                                     // ����DMA NVIC
}

/**
 * @brief �ӻ�ͨ�Ŵ����жϻص�����
 * 
 */
void SlaveComm_UARTCallback()
{
    UART3_RxOK = 1;
    uprintf("%s", UART3_RxBuffer);
    memset(UART3_RxBuffer, 0, sizeof(uint8_t) * 20);
}
