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
#include "path_following.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      CMD初始化
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CMD_Init()
{
	uart_init(DEBUG_UART, DEBUG_UART_BAUD, DEBUG_UART_TX_PIN,
			  DEBUG_UART_RX_PIN);
	nvic_init(USART1_IRQn, 0, 1, ENABLE); // 配置UART NVIC

	// >>>DMA方式接收数据<<<
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); // 开启闲时中断
	UART_DMA_ReceiveInit(USART1, DMA1_Channel5, (u32)(&USART1->DATAR),
						 (u32)UART1_RxBuffer, UART1_RX_BUFFER_SIZE); // USART1 DMA初始化
	nvic_init(DMA1_Channel5_IRQn, 0, 1, ENABLE);					 // 配置DMA NVIC

	// >>>DMA方式发送数据<<< 实测由于DMA太快，串口发送速度跟不上DMA速度，导致发送数据经常被覆盖
	//	USART_ITConfig(USART1, USART_IT_TC, ENABLE); //  开启串口发送完成中断
	//	UART_DMA_SendInit(USART1, DMA1_Channel4,
	//					  (u32)UART1_TxBuffer, (u32)(&USART1->DATAR));
	//	nvic_init(DMA1_Channel4_IRQn, 0, 1, ENABLE); // 配置DMA NVIC

	// >>>中断方式接收数据<<<
	//	uart_rx_irq(UART_1, ENABLE); // 使能串口接收中断
}

/**
 * @brief      串口DMA接收初始化
 * @param      dma_ch              DAM通道
 * @param      src_addr            源地址
 * @param      des_addr            目标地址
 * @param      size                数据长度
 * @sa                  uart_dma_init(DMA1_Channel5, GPIOA->ODR, GPIOC->ODR, 8);
 */
void UART_DMA_ReceiveInit(USART_TypeDef *usart, DMA_Channel_TypeDef *dma_ch,
						  uint32 src_addr, uint32 des_addr, uint32 size)
{
	DMA_InitTypeDef DMA_InitStructure;

	if (DMA1_Channel1 == dma_ch || DMA1_Channel2 == dma_ch || DMA1_Channel3 == dma_ch || DMA1_Channel4 == dma_ch ||
		DMA1_Channel5 == dma_ch || DMA1_Channel6 == dma_ch || DMA1_Channel7 == dma_ch)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //DMA1总线初始化
	}
	else if (DMA2_Channel1 == dma_ch || DMA2_Channel2 == dma_ch || DMA2_Channel3 == dma_ch || DMA2_Channel4 == dma_ch ||
			 DMA2_Channel5 == dma_ch)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); //DMA2总线初始化
	}

	DMA_DeInit(dma_ch); // 复位

	//MDA配置初始化
	DMA_InitStructure.DMA_PeripheralBaseAddr = src_addr;					//源地址
	DMA_InitStructure.DMA_MemoryBaseAddr = des_addr;						//目标地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						//外设作为源
	DMA_InitStructure.DMA_BufferSize = size;								//传输多少个数据
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设地址不增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//内存地址依次+1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设每次传输一个字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//内存每次传输一个字节
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//非循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					//优先级最高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;							//非内存到内存模式
	DMA_Init(dma_ch, &DMA_InitStructure);

	DMA_Cmd(dma_ch, ENABLE);					  //开启DMA
	USART_DMACmd(usart, USART_DMAReq_Rx, ENABLE); // 使能UART DMA接收
}

/**
 * @brief UART DMA发送初始化
 * 
 * @param usart 
 * @param dma_ch 
 * @param src_addr 源地址，buffer
 * @param des_addr 目的地址，UART数据寄存器
 */
void UART_DMA_SendInit(USART_TypeDef *usart, DMA_Channel_TypeDef *dma_ch,
					   uint32 src_addr, uint32 des_addr)
{
	DMA_InitTypeDef DMA_InitStructure;

	if (DMA1_Channel1 == dma_ch || DMA1_Channel2 == dma_ch || DMA1_Channel3 == dma_ch || DMA1_Channel4 == dma_ch ||
		DMA1_Channel5 == dma_ch || DMA1_Channel6 == dma_ch || DMA1_Channel7 == dma_ch)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //DMA1总线初始化
	}
	else if (DMA2_Channel1 == dma_ch || DMA2_Channel2 == dma_ch || DMA2_Channel3 == dma_ch || DMA2_Channel4 == dma_ch ||
			 DMA2_Channel5 == dma_ch)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); //DMA2总线初始化
	}

	DMA_DeInit(dma_ch); // 复位

	//MDA配置初始化
	DMA_InitStructure.DMA_PeripheralBaseAddr = des_addr;
	DMA_InitStructure.DMA_MemoryBaseAddr = src_addr;   // 发送内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; // 外设为传送数据目的地，即发送数据，即快递是发件
	DMA_InitStructure.DMA_BufferSize = 0;			   //发送长度为0，即未有快递需要发送
	DMA_Init(dma_ch, &DMA_InitStructure);			   //初始化

	DMA_ITConfig(dma_ch, DMA_IT_TC, ENABLE);	  //配置DMA传输完成中断
	DMA_Cmd(dma_ch, ENABLE);					  //开启DMA
	USART_DMACmd(usart, USART_DMAReq_Tx, ENABLE); // 使能UART DMA发送
}

/**
 * @brief 串口DMA发送数据
 * 
 * @param dman DMA通道
 * @param send_buffer 发送缓存
 * @param data 数据
 * @param len 数据长度
 */
void UART_DMA_SendData(DMA_Channel_TypeDef *dman, uint8 *data, uint8 len)
{
	while (DMA_GetCurrDataCounter(dman))
		; // 检查DMA发送通道内是否还有数据
	//DMA发送数据-要先关 设置发送长度 开启DMA
	DMA_Cmd(dman, DISABLE);
	dman->CNTR = len;	   // 设置发送长度
	DMA_Cmd(dman, ENABLE); // 启动DMA发送
}

char CMD_RxOK = 0; // 串口接收完成标志，给CMD_Exe用
uint8_t *CMD_Buffer[CMD_SIZE_X] =
	{0}; // 指针数组，每个元素都指向分割后的元字符串
uint8_t CMD_BufferCnt = 0;
uint8_t CMD_Argc = 0; // 指令参数数量
char *CMD_Argv[CMD_SIZE_X] =
	{0}; // 指向指令参数的指针数据
/**
 * @brief CMD的抽象UART中断回调函数
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

	CMD_RxOK = 1;
}

/**
 * @brief CMD总执行函数，在while（1）中运行
 * @note 不在中断函数中执行，避免超时
 * 
 */
void CMD_Exe(void)
{
	if (CMD_RxOK)
	{
		CMD_CommandParse((char *)UART1_RxBuffer, &CMD_Argc, CMD_Argv); // 解析指令
																	   //	for (int i = 0; i < CMD_Argc; i++)
																	   //	{
																	   //		uprintf("%s ", CMD_Argv[i]);
																	   //	}
		CMD_CommandExe(CMD_Argc, CMD_Argv);							   // 执行指令
		memset(UART1_RxBuffer, 0, sizeof(uint8_t) * UART1_RX_BUFFER_SIZE);

		CMD_RxOK = 0;
	}
}

/**
 * @brief	将输入分割，并记录参数个数
 * @param	cmd_line	输入指令字符串
 * @param   argc        指令个数
 * @param   argv        分割后参数列表
 * @return	None
 */
int CMD_CommandParse(char *cmd_line, uint8_t *argc, char *argv[])
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

int wave_index = -1;
/**
 * @brief 指令执行函数
 * @param argc 指令个数
 * @param argv 指令参数，第一个为指令名称
 */
int CMD_CommandExe(int argc, char **argv)
{
	if (strcmp(argv[0], "SD") == 0) //SetDuty
	{
		for (int i = 0; i < 4; i++)
		{
			MecanumChassis.motor[i].target_duty = atoi(argv[i + 1]);
			uprintf("[%d]:%d", i, MecanumChassis.motor[i].target_duty);
		}
		uprintf("\r\n");
	}
	else if (strcmp(argv[0], "SRPM") == 0) //SetRPM
	{
		for (int i = 0; i < 4; i++)
		{
			MecanumChassis.motor[i].target_rpm = atoi(argv[i + 1]);
			uprintf("[%d]:%d", i, MecanumChassis.motor[i].target_rpm);
		}
		uprintf("\r\n");
	}
	else if (strcmp(argv[0], "SCM") == 0) // SetCtrlMode
	{
		MecanumChassis.ctrl_mode = (MecanumChassis.ctrl_mode + 1) % 5;
		uprintf("CMD|Ctrl mode change to %d\r\n", MecanumChassis.ctrl_mode);
	}
	else if (strcmp(argv[0], "DF") == 0) // DiffDrive
	{
		MecanumChassis.target_speed = atof(argv[1]);
		MecanumChassis.target_omega = atof(argv[2]);
		MecanumChassis.target_dir = 0;
		uprintf("Chassis|target_speed:%5.2f target_omega:%5.2f\r\n",
				MecanumChassis.target_speed, MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "FOL") == 0) // PathFollowTuning
	{

		PathFollowStatus.forthright_speed = atof(argv[1]);
		PathFollowStatus.curve_speed = atof(argv[2]);
		PathFollowStatus.angle_thres = atof(argv[3]);
		uprintf("Chassis|forthright_speed:%5.2f curve_speed:%5.2f angle_thres:%5.2f\r\n",
				PathFollowStatus.forthright_speed, PathFollowStatus.curve_speed, PathFollowStatus.angle_thres);
	}
	else if (strcmp(argv[0], "GA") == 0) // Teleop_GoAhead
	{
		MecanumChassis.target_speed = 0.15;
		MecanumChassis.target_dir = 1.57;
		MecanumChassis.target_omega = 0;
		uprintf(
			"Chassis|target_speed:%5.2f target_dir:%5.2f target_omega:%5.2f\r\n",
			MecanumChassis.target_speed, MecanumChassis.target_dir,
			MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "GB") == 0) //Teleop_GoBack
	{
		MecanumChassis.target_speed = 0.15;
		MecanumChassis.target_dir = -1.57;
		MecanumChassis.target_omega = 0;
		uprintf(
			"Chassis|target_speed:%5.2f target_dir:%5.2f target_omega:%5.2f\r\n",
			MecanumChassis.target_speed, MecanumChassis.target_dir,
			MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "TL") == 0) //Teleop_TurnLeft
	{
		MecanumChassis.target_omega += 0.2;
		uprintf(
			"Chassis|target_speed:%5.2f target_dir:%5.2f target_omega:%5.2f\r\n",
			MecanumChassis.target_speed, MecanumChassis.target_dir,
			MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "TR") == 0) //Teleop_TurnRight
	{
		MecanumChassis.target_omega -= 0.2;
		uprintf(
			"Chassis|target_speed:%5.2f target_dir:%5.2f target_omega:%5.2f\r\n",
			MecanumChassis.target_speed, MecanumChassis.target_dir,
			MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "SL") == 0) //Teleop_ShiftLeft
	{
		MecanumChassis.target_dir = 3.14;
		MecanumChassis.target_speed = 0.15;
		uprintf(
			"Chassis|target_speed:%5.2f target_dir:%5.2f target_omega:%5.2f\r\n",
			MecanumChassis.target_speed, MecanumChassis.target_dir,
			MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "SR") == 0) //Teleop_ShiftRight
	{
		MecanumChassis.target_dir = 0;
		MecanumChassis.target_speed = 0.15;
		uprintf(
			"Chassis|target_speed:%5.2f target_dir:%5.2f target_omega:%5.2f\r\n",
			MecanumChassis.target_speed, MecanumChassis.target_dir,
			MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "ST") == 0) //Teleop_Stop
	{
		MecanumChassis.target_speed = 0;
		MecanumChassis.target_omega = 0;
		uprintf(
			"Chassis|target_speed:%5.2f target_dir:%5.2f target_omega:%5.2f\r\n",
			MecanumChassis.target_speed, MecanumChassis.target_dir,
			MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "MPID") == 0) // Motor PID Tuning
	{
		int i = atoi(argv[1]);
		float kp = atof(argv[2]);
		float ki = atof(argv[3]);
		float kd = atof(argv[4]);
		float int_duty = atof(argv[5]);
		float sub_pid_thres = atof(argv[6]);
		float sub_pid_kp = atof(argv[7]);
		MotorPID[i].kp = kp;
		MotorPID[i].ki = ki;
		MotorPID[i].kd = kd;
		MotorPID[i].int_duty = int_duty;
		MotorPID[i].sub_pid_thres = sub_pid_thres;
		MotorPID[i].sub_pid_kp = sub_pid_kp;
		uprintf(
			"MotorPID|[%d] kp:%.3f ki:%.3f kd:%.3f intduty:%.2f thres:%.2f sub_kp:%.2f\r\n",
			i, kp, ki, kd, int_duty, sub_pid_thres, sub_pid_kp);
	}
	else if (strcmp(argv[0], "HPID") == 0) // Heading PID Tuning
	{
		float kp = atof(argv[1]);
		float ki = atof(argv[2]);
		float kd = atof(argv[3]);
		float int_duty = atof(argv[4]);
		HeadingAnglePID.kp = kp;
		HeadingAnglePID.ki = ki;
		HeadingAnglePID.kd = kd;
		HeadingAnglePID.int_duty = int_duty;
		uprintf("HeadingPID|kp:%.3f ki:%.3f kd:%.3f intduty:%.2f\r\n", kp, ki,
				kd, int_duty);
	}
	else if (strcmp(argv[0], "WV") == 0) // 虚拟示波器
	{
		wave_index = atoi(argv[1]);
		uprintf("Opened motor[%d] wave!\r\n", wave_index);
	}
	return 1;
}
