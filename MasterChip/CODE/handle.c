/**
 * @file handle.c
 * @author simon
 * @brief �ֱ��������
 * @version 0.1
 * @date 2021-10-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "handle.h"
#include "mecanum_chassis.h"
#include "isr.h"
#include "cmd.h"
#include <stdlib.h>
#include <string.h>
#include "sci_compute.h"

struct Handle_t Handle; // �ֱ����ݽṹ��
uint8 Handle_UARTRxOK = 0;
uint8 Handle_PrintRockerStatus_Flag = 0;

#define HANDLE_LEFT_ROCKER_TH (20)
#define HADNLE_RIGHT_ROCKER_TH (108)
#define HANDLE_RIGHT_ROCKER_SPD_TRANS_RATIO (1.5f)

void Handle_Init()
{
	uart_init(UART_1, 115200, UART1_TX_A9, UART1_RX_A10);
	nvic_init(USART1_IRQn, 1, 2, ENABLE);          // ����UART NVIC
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); // ������ʱ�ж�
	UART_DMA_ReceiveInit(USART1, DMA1_Channel5, (u32) (&USART1->DATAR),
			(u32) UART1_RxBuffer, UART1_RX_BUFFER_SIZE); // USART1 DMA��ʼ��
	nvic_init(DMA1_Channel5_IRQn, 1, 3, ENABLE);                   // ����DMA NVIC

	Handle.right_rocker.y = 0;
	Handle.right_rocker.x = 0;
	Handle.left_rocker.y = 0;
	Handle.left_rocker.x = 0;
}

void Handle_Exe()
{
	if (!Handle_UARTRxOK)
	{
		return;
	}
	Handle.left_rocker.length = (uint16_t) sqrt(
			pow(Handle.left_rocker.x, 2) + pow(Handle.left_rocker.y, 2));
	Handle.right_rocker.length = (uint16_t) sqrt(
			pow(Handle.right_rocker.x, 2) + pow(Handle.right_rocker.y, 2));
	// >>>>>>>>>>>>>>>>>>>>>>>>>>�ٶ�ʸ������<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	if (Handle.left_rocker.length <= HANDLE_LEFT_ROCKER_TH)
	{
		Handle.left_rocker.length = 0;
	}
	else if (Handle.left_rocker.length > HANDLE_LEFT_ROCKER_TH)
	{
		Handle.left_rocker.length -= HANDLE_LEFT_ROCKER_TH;
		MecanumChassis.target_dir = atan2f(Handle.left_rocker.y,
				Handle.left_rocker.x);
	}
	MecanumChassis.target_speed = (Handle.left_rocker.length*1.0f) / 100.0f;

	// >>>>>>>>>>>>>>>>>>>>>>>>ƫ���ǿ���<<<<<<<<<<<<<<<<<<<<<<<<<<
	float angle_offset = 0;
	if (Handle.right_rocker.length > HADNLE_RIGHT_ROCKER_TH) // ҡ�˶���ͷ����Ч
	{
		angle_offset = AngleLimitDiff(
				atan2(Handle.right_rocker.y, Handle.right_rocker.x), PI / 2)
				* (-1); // ��pi/2��ƫ��ֵ����С���ƽ��ٶ�
		MecanumChassis.target_omega =- angle_offset
				* HANDLE_RIGHT_ROCKER_SPD_TRANS_RATIO;
	}
	else {
		MecanumChassis.target_omega = 0;
	}

	Handle_UARTRxOK = 0;
}

void Handle_PrintRockerStatus()
{
	if (!(Handle_PrintRockerStatus_Flag && TIM1_20ms_Flag))
		return;
	uprintf("--lx:%4d ly:%4d rx:%4d ry:%4d | ", Handle.left_rocker.x,
			Handle.left_rocker.y, Handle.right_rocker.x, Handle.right_rocker.y);
	uprintf("llen:%3d ldir:%.2f rlen:%3d rdir:%.2f\r\n",
			Handle.left_rocker.length, Handle.left_rocker.dir,
			Handle.right_rocker.length, Handle.right_rocker.dir);
}

uint8_t Handle_UARTRxData[UART1_RX_BUFFER_SIZE] =
{ 0 };
void Handle_UARTCallback()
{
	if (!(UART1_RxBuffer[0] == 's'&&strlen(UART1_RxBuffer)==23))
	{
		return;
	}

//	strncpy(Handle_UARTRxData, UART1_RxBuffer, UART1_RxLen);

	char numeric[6];
	substring(numeric, UART1_RxBuffer, 2, 4);
	numeric[4] = '\0';
	Handle.left_rocker.x = atoi(numeric);
	memset(numeric, 0, sizeof(char) * 6);

	substring(numeric, UART1_RxBuffer, 7, 4);
	numeric[4] = '\0';
	Handle.left_rocker.y = atoi(numeric);
	memset(numeric, 0, sizeof(char) * 6);

	substring(numeric, UART1_RxBuffer, 12, 4);
	numeric[4] = '\0';
	Handle.right_rocker.x = atoi(numeric);
	memset(numeric, 0, sizeof(char) * 6);

	substring(numeric, UART1_RxBuffer, 17, 4);
	numeric[4] = '\0';
	Handle.right_rocker.y = atoi(numeric);

//	uprintf("|%4d %4d %4d %4d|\r\n", Handle.left_rocker.x,
//					Handle.left_rocker.y, Handle.right_rocker.x,
//					Handle.right_rocker.y);

//	uprintf("%s", UART1_RxBuffer);
	Handle_UARTRxOK = 1;
}

char *substring(char *dst, char *src, int start, int len)
{
	char *p = dst;
	char *q = src;
	int length = strlen(src);
	if (start >= length || start < 0)
		return NULL;
	if (len > length)
		len = length - start;
	q += start;
	while (len--)
	{
		*(p++) = *(q++);
	}
	*(p++) = '\0';
	return dst;
}

/**
 * @brief �ֱ�ҡ�˷�ֵ�����ֵ�������ٶȣ���ӳ���ϵ
 * @param amplitude ��Ҫӳ��ķ�ֵ
 * @param max_amplitude ҡ������ֵ
 * @param max_val ӳ�������ֵ
 * @param y0 ��������
 * @retval ӳ������ֵ
 * @note Ŀǰ���ö��η�ӳ��
 */
float Handle_RockerMapping(uint16_t amplitude, uint8_t max_amplitude,
		float max_val, float b)
{
	// return (1.125 * amplitude * amplitude - 1012);

	// float a = max_val / (max_amplitude * max_amplitude); // y=ax^2+b
	// return a * (amplitude * amplitude) + b;

	// float a = max_val / max_amplitude; // y = ax+b
	// return (a * amplitude + b);

	float x = (-max_amplitude * 1.0 / 2.0 + amplitude * 1.0) * 8.0 / 65.0; // sigmoid����
	float a = 1.0 / (1.0 + exp(-x));
	return a * max_val + b;
}
