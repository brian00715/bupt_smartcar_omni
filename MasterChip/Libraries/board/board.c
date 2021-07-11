#include "ch32v10x_usart.h"
#include "ch32v10x_rcc.h"
#include "board.h"
#include "isr.h"
#include "cmd.h"
#include "stdarg.h"

void board_init(void)
{
	//获取系统主频
	sys_clk = 8000000 * (((RCC->CFGR0 >> 18) & 0x0F) + 2);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      printf重定向
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:              重定向printf到DEBUG串口上
//-------------------------------------------------------------------------------------------------------------------
//#if (1 == PRINTF_ENABLE)
//int _write(int fd, char *buf, int size)
//{
//    int i;
//    for(i=0; i<size; i++)
//    {
//        while (USART_GetFlagStatus((USART_TypeDef*)UARTN[DEBUG_UART], USART_FLAG_TC) == RESET);
//        USART_SendData((USART_TypeDef*)UARTN[DEBUG_UART], *buf++);
//    }
//    return size;
//}
//#endif

void uprintf(char *fmt, ...)
{
	int size;
	va_list arg_ptr;
	va_start(arg_ptr, fmt);
	size = vsnprintf(UART2_TxBuffer, UART2_TX_BUFFER_SIZE, fmt, arg_ptr);
	va_end(arg_ptr);
	 for (int i = 0; i < size; i++)
	 {
	 	while (USART_GetFlagStatus((USART_TypeDef *)UARTN[DEBUG_UART],
	 							   USART_FLAG_TC) == RESET)
	 		;
	 	USART_SendData((USART_TypeDef *)UARTN[DEBUG_UART], UART2_TxBuffer[i]);
	 }
//	UART_DMA_SendData(DMA1_Channel4, UART2_TxBuffer, size);
}
