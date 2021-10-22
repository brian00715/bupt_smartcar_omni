#ifndef _zf_board_h
#define _zf_board_h

#include "common.h"
#include "stdio.h"
#include "ch32v10x.h"
#include "zf_uart.h"
#include "ch32v10x_dma.h"

#define PRINTF_ENABLE           1                   //printf使能

#define DEBUG_UART              UART_2              //DEBUG串口
#define DEBUG_UART_BAUD         115200              //DEBUG串口波特率
#define DEBUG_UART_TX_PIN       UART2_TX_A2         //DEBUG串口TX引脚
#define DEBUG_UART_RX_PIN       UART2_RX_A3         //DEBUG串口RX引脚
#define DEBUG_UART_TX_DMA       DMA1_Channel6

void board_init(void);
void uprintf(char *fmt, ...);

#endif
