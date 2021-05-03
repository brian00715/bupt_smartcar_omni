/**
 * @file cmd.c
 * @author simon
 * @brief CMD���ܺ�����أ�����ʹ�������еķ�ʽ���Ƶ�Ƭ��
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
//  @brief      CMD��ʼ��
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void CMD_Init()
{
	uart_init(DEBUG_UART, DEBUG_UART_BAUD, DEBUG_UART_TX_PIN,
			  DEBUG_UART_RX_PIN);
	nvic_init(USART1_IRQn, 0, 1, ENABLE); // ����UART NVIC

	// >>>DMA��ʽ��������<<<
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); // ������ʱ�ж�
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); // ʹ��UART DMA����
	UART_DMA_Init(DMA1_Channel5, (u32)(&USART1->DATAR),
				  (uint32)UART1_RxBuffer,
				  RX_BUFFER_SIZE);					// USART1 DMA��ʼ��
	nvic_init(USART1_RX_DMA_CH_IRQN, 0, 1, ENABLE); // ����DMA NVIC

	// >>>�жϷ�ʽ��������<<<
	//	uart_rx_irq(UART_1, ENABLE); // ʹ�ܴ��ڽ����ж�
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����DMA��ʼ��
//  @param      dma_ch              DAMͨ��
//  @param      src_addr            Դ��ַ
//  @param      des_addr            Ŀ���ַ
//  @param      size                ���ݳ���
//  @return     void
//  Sample usage:                   uart_dma_init(DMA1_Channel5, GPIOA->ODR, GPIOC->ODR, 8);
//-------------------------------------------------------------------------------------------------------------------
void UART_DMA_Init(DMA_Channel_TypeDef *dma_ch, uint32 src_addr,
				   uint32 des_addr, uint32 size)
{
	DMA_InitTypeDef DMA_InitStructure;

	if (DMA1_Channel1 == dma_ch || DMA1_Channel2 == dma_ch || DMA1_Channel3 == dma_ch || DMA1_Channel4 == dma_ch ||
		DMA1_Channel5 == dma_ch || DMA1_Channel6 == dma_ch || DMA1_Channel7 == dma_ch)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //DMA1���߳�ʼ��
	}
	else if (DMA2_Channel1 == dma_ch || DMA2_Channel2 == dma_ch || DMA2_Channel3 == dma_ch || DMA2_Channel4 == dma_ch ||
			 DMA2_Channel5 == dma_ch)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); //DMA2���߳�ʼ��
	}

	DMA_DeInit(dma_ch); // ��λ

	//MDA���ó�ʼ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = src_addr;					//Դ��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = des_addr;						//Ŀ���ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						//������ΪԴ
	DMA_InitStructure.DMA_BufferSize = size;								//������ٸ�����
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//�����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//�ڴ��ַ����+1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //����ÿ�δ���һ���ֽ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//�ڴ�ÿ�δ���һ���ֽ�
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//��ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					//���ȼ����
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;							//���ڴ浽�ڴ�ģʽ
	DMA_Init(dma_ch, &DMA_InitStructure);

	DMA_ITConfig(dma_ch, DMA_IT_TC, ENABLE); //����DMA��������ж�
	DMA_Cmd(dma_ch, ENABLE);				 //����DMA
}

char CMD_RxOK = 0; // ���ڽ�����ɱ�־����CMD_Exe��
uint8_t *CMD_Buffer[CMD_SIZE_X] =
	{0}; // ָ�����飬ÿ��Ԫ�ض�ָ��ָ���Ԫ�ַ���
uint8_t CMD_BufferCnt = 0;
uint8_t CMD_Argc = 0; // ָ���������
char *CMD_Argv[CMD_SIZE_X] =
	{0}; // ָ��ָ�������ָ������
/**
 * @brief CMD�ĳ���UART�жϻص�����
 * 
 */
void CMD_UARTCallback(void)
{
	// uint8_t *clr = DMAaRxBuffer;
	// while (*(clr++) == '\0' && clr < DMAaRxBuffer + DMA_BUFFER_SIZE) // �ҵ���ͷ�����⴫������
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
 * @brief CMD��ִ�к�������while��1��������
 * @note �����жϺ�����ִ�У����ⳬʱ
 * 
 */
void CMD_Exe(void)
{
	if (CMD_RxOK)
	{
		CMD_CommandParse((char *)UART1_RxBuffer, &CMD_Argc, CMD_Argv); // ����ָ��
																	   //	for (int i = 0; i < CMD_Argc; i++)
																	   //	{
																	   //		uprintf("%s ", CMD_Argv[i]);
																	   //	}
		CMD_CommandExe(CMD_Argc, CMD_Argv);							   // ִ��ָ��
		memset(UART1_RxBuffer, 0, sizeof(uint8_t) * RX_BUFFER_SIZE);

		CMD_RxOK = 0;
	}
}

/**
 * @brief	������ָ����¼��������
 * @param	cmd_line	����ָ���ַ���
 * @param   argc        ָ�����
 * @param   argv        �ָ������б�
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
 * @brief ָ��ִ�к���
 * @param argc ָ�����
 * @param argv ָ���������һ��Ϊָ������
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
	else if (strcmp(argv[0], "SR") == 0) //SetRPM
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
		MecanumChassis.ctrl_mode = (MecanumChassis.ctrl_mode + 1) % 4;
		uprintf("CMD|Ctrl mode change to %d\r\n", MecanumChassis.ctrl_mode);
	}
	else if (strcmp(argv[0], "GA") == 0) // Teleop_GoAhead
	{
		MecanumChassis.target_speed = 0.15;
		MecanumChassis.target_dir = 1.57;
		MecanumChassis.target_omega = 0;
		uprintf("Chassis|target_speed:%3d target_dir:%3d target_omega:%3d\r\n",
				(int16)MecanumChassis.target_speed,
				(int16)MecanumChassis.target_dir,
				(int16)MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "GB") == 0) //Teleop_GoBack
	{
		MecanumChassis.target_speed = 0.15;
		MecanumChassis.target_dir = -1.57;
		MecanumChassis.target_omega = 0;
		uprintf("Chassis|target_speed:%3d target_dir:%3d target_omega:%3d\r\n",
				(int16)MecanumChassis.target_speed,
				(int16)MecanumChassis.target_dir,
				(int16)MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "TL") == 0) //Teleop_TurnLeft
	{
		MecanumChassis.target_omega += 0.2;
		uprintf("Chassis|target_speed:%3d target_dir:%3d target_omega:%3d\r\n",
				(int16)MecanumChassis.target_speed,
				(int16)MecanumChassis.target_dir,
				(int16)MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "TR") == 0) //Teleop_TurnRight
	{
		MecanumChassis.target_omega -= 0.2;
		uprintf("Chassis|target_speed:%3d target_dir:%3d target_omega:%3d\r\n",
				(int16)MecanumChassis.target_speed,
				(int16)MecanumChassis.target_dir,
				(int16)MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "SL") == 0) //Teleop_ShiftLeft
	{
		MecanumChassis.target_dir = 3.14;
		MecanumChassis.target_speed = 0.15;
		uprintf("Chassis|target_speed:%3d target_dir:%3d target_omega:%3d\r\n",
				(int16)MecanumChassis.target_speed,
				(int16)MecanumChassis.target_dir,
				(int16)MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "SR") == 0) //Teleop_ShiftRight
	{
		MecanumChassis.target_dir = 0;
		MecanumChassis.target_speed = 0.15;
		uprintf("Chassis|target_speed:%3d target_dir:%3d target_omega:%3d\r\n",
				(int16)MecanumChassis.target_speed,
				(int16)MecanumChassis.target_dir,
				(int16)MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "ST") == 0) //Teleop_Stop
	{
		MecanumChassis.target_speed = 0;
		MecanumChassis.target_omega = 0;
		uprintf("Chassis|target_speed:%3d target_dir:%3d target_omega:%3d\r\n",
				(int16)MecanumChassis.target_speed,
				(int16)MecanumChassis.target_dir,
				(int16)MecanumChassis.target_omega);
	}
	else if (strcmp(argv[0], "PID") == 0) // PID Tuning
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
			"PID|[%d] kp:%.3f ki:%.3f kd:%.3f intduty:%.2f thres:%.2f sub_kp:%.2f\r\n",
			i, kp, ki, kd, int_duty, sub_pid_thres, sub_pid_kp);
	}
	else if (strcmp(argv[0], "WV") == 0) // ����ʾ����
	{
		wave_index = atoi(argv[1]);
		uprintf("Opened motor[%d] wave!\r\n", wave_index);
	}
	return 1;
}