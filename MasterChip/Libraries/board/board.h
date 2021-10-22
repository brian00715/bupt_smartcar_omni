#ifndef _zf_board_h
#define _zf_board_h

#include "common.h"
#include "stdio.h"
#include "ch32v10x.h"
#include "zf_uart.h"
#include "ch32v10x_dma.h"

#define PRINTF_ENABLE           1                   //printfʹ��

#define DEBUG_UART              UART_2              //DEBUG����
#define DEBUG_UART_BAUD         115200              //DEBUG���ڲ�����
#define DEBUG_UART_TX_PIN       UART2_TX_A2         //DEBUG����TX����
#define DEBUG_UART_RX_PIN       UART2_RX_A3         //DEBUG����RX����
#define DEBUG_UART_TX_DMA       DMA1_Channel6

void board_init(void);
void uprintf(char *fmt, ...);

#endif
