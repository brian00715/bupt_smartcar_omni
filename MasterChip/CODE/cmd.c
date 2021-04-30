/**
 * @file cmd.c
 * @author simon
 * @brief CMD功能函数相关，让你使用命令行的方式控制单片机
 * @version 0.1
 * @date 2021-04-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ch32v10x_dma.h"
#include "ch32v10x_rcc.h"
#include "ch32v10x_tim.h"
#include "ch32v10x_usart.h"
#include "ch32v10x.h"

#include "zf_gpio.h"
#include "zf_uart.h"
#include "board.h"
#include "zf_nvic.h"
#include "cmd.h"
#include "isr.h"
#include <string.h>
#include <stdlib.h>
#include "mecanum_chassis.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      CMD初始化
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CMD_Init(void)
{
	//    uart_init(DEBUG_UART, DEBUG_UART_BAUD, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN); // 已经在board_init中初始化了
	nvic_init(USART1_IRQn, 0, 0, ENABLE); // 配置UART NVIC

	// >>>DMA方式接收数据<<<
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); // 开启闲时中断
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); // 使能UART DMA传输
	UART_DMA_Init(DMA1_Channel5, (u32) (&USART1->DATAR),
			(uint32) UART1_RxBuffer,
			RX_BUFFER_SIZE); // USART1 DMA初始化
							 	nvic_init(USART1_RX_DMA_CH_IRQN, 0, 0, ENABLE); // 配置DMA NVIC

							 // >>>中断方式接收数据<<<
							 //	uart_rx_irq(UART_1, ENABLE); // 使能串口接收中断
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      串口DMA初始化
//  @param      dma_ch              DAM通道
//  @param      src_addr            源地址
//  @param      des_addr            目标地址
//  @param      size                数据长度
//  @return     void
//  Sample usage:                   uart_dma_init(DMA1_Channel5, GPIOA->ODR, GPIOC->ODR, 8);
//-------------------------------------------------------------------------------------------------------------------
void UART_DMA_Init(DMA_Channel_TypeDef *dma_ch, uint32 src_addr,
		uint32 des_addr, uint32 size)
{
	DMA_InitTypeDef DMA_InitStructure;

	if (DMA1_Channel1 == dma_ch || DMA1_Channel2 == dma_ch
			|| DMA1_Channel3 == dma_ch || DMA1_Channel4 == dma_ch ||
			DMA1_Channel5 == dma_ch || DMA1_Channel6 == dma_ch
			|| DMA1_Channel7 == dma_ch)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //DMA1总线初始化
	}
	else if (DMA2_Channel1 == dma_ch || DMA2_Channel2 == dma_ch
			|| DMA2_Channel3 == dma_ch || DMA2_Channel4 == dma_ch ||
			DMA2_Channel5 == dma_ch)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); //DMA2总线初始化
	}

	DMA_DeInit(dma_ch); // 复位

	//MDA配置初始化
	DMA_InitStructure.DMA_PeripheralBaseAddr = src_addr;				//源地址
	DMA_InitStructure.DMA_MemoryBaseAddr = des_addr;					//目标地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;					//外设作为源
	DMA_InitStructure.DMA_BufferSize = size;						//传输多少个数据
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;			//内存地址依次+1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设每次传输一个字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//内存每次传输一个字节
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;						//非循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;				//优先级最高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;					//非内存到内存模式
	DMA_Init(dma_ch, &DMA_InitStructure);

	DMA_ITConfig(dma_ch, DMA_IT_TC, ENABLE); //配置DMA传输完成中断
	DMA_Cmd(dma_ch, ENABLE);				 //开启DMA1
}

uint8_t *CMD_Buffer[CMD_SIZE_X] =
{ 0 }; // 指针数组，每个元素都指向分割后的元字符串
uint8_t CMD_BufferCnt = 0;
uint8_t CMD_Argc = 0;			  // 指令参数数量
char *CMD_Argv[CMD_SIZE_X] =
{ 0 }; // 指向指令参数的指针数据
/**
 * @brief UART1中断回调函数
 * 
 */
void CMD_UARTCallback(void)
{
	// uint8_t *clr = DMAaRxBuffer;
	// while (*(clr++) == '\0' && clr < DMAaRxBuffer + DMA_BUFFER_SIZE) // 找到开头，避免传输噪声
	// {
	// }
	// strcpy((char *)DMAUSART_RX_BUF, (char *)(clr - 1));
	// if (DMAUSART_RX_BUF[0] != '\0')
	// {
	// 	DMA_RxOK_Flag = 1;
	// }
	// memset(DMAaRxBuffer, 0, 98);
	CMD_Parse((char *) UART1_RxBuffer, &CMD_Argc, CMD_Argv); // 解析指令
//	for (int i = 0; i < CMD_Argc; i++)
//	{
//		uprintf("%s ", CMD_Argv[i]);
//	}
	CMD_Exe(CMD_Argc, CMD_Argv);							// 执行指令
}

/**
 * @brief	将输入分割，并记录参数个数
 * @param	cmd_line	输入指令字符串
 * @param   argc        指令个数
 * @param   argv        分割后参数列表
 * @return	None
 */
int CMD_Parse(char *cmd_line, uint8_t *argc, char *argv[])
{
	char *token = strtok(cmd_line, " ");
	int arg_index = 0;
	while (token)
	{
		argv[arg_index++] = token;
		if (arg_index >= CMD_SIZE_X)
		{
			uprintf("## Too many arguments! ##\r\n");
			return 0;
		}
		token = strtok(NULL, " ");
	}
	*argc = arg_index;
	return 1;
}

/**
 * @brief 指令执行函数
 * @param argc 指令个数
 * @param argv 指令参数，第一个为指令名称
 */
int CMD_Exe(int argc, char **argv)
{
	if (strcmp(argv[0], "SetDuty") == 0)
	{
		for (int i = 0; i < 4; i++)
		{
			MecanumChassis.motor[i].target_duty = atoi(argv[i + 1]);
			uprintf("[%d]:%d", i, MecanumChassis.motor[i].target_duty);
		}
		uprintf("\r\n");
	}
	return 1;
}
