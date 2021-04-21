/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            main
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ790875685)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
//�����Ƽ�IO�鿴Projecct�ļ����µ�TXT�ı�
//���µĹ��̻��߹����ƶ���λ�����ִ�����²���
//�Ҽ��������̣�ѡ��ˢ��
#include "headfile.h"
#include "display.h"
#include "timer_pit.h"
#include "encoder.h"
#include "buzzer.h"
#include "button.h"
#include "motor.h"
#include "elec.h"
#include "cmd.h"
#include "isr.h"

extern uint8_t UART1_RxBuffer[RX_BUFFER_SIZE];
extern uint8_t UART1_RxComplete;
extern uint8_t UART1_RxIDLEFlag;
int main(void)
{
	DisableGlobalIRQ();
	board_init();
	CMD_Init();
	//	ips114_init();
	//      mt9v03x_init();
	//    encoder_init();
	//    buzzer_init();
	//    elec_init();
	//  button_init ();
	Motor_Init();
	//  timer_pit_init ();
	//	ips114_clear(WHITE);
	static int16 duty = 0;
	EnableGlobalIRQ(0);

	printf("==Init Done==\r\n");
	while (1)
	{
		duty = (duty + 1000) % 10000;
		Motor_SetDuty(duty, duty, duty, duty);
		//        systick_delay_ms(1000);
		//		ips114_showstr(0, 0, "running");
		//		ips114_showstr(20, 20, UART_RxBuffer);

		//		if (UART1_RxComplete)
		//		{
		//			printf("%s\r\n", UART1_RxBuffer);
		//			UART1_RxComplete = 0;
		//		}

		if (UART1_RxIDLEFlag)
		{
			//			printf("%s\r\n", UART1_RxBuffer);
			UART1_RxIDLEFlag = 0;
			for (int i = 0; i < CMD_BufferCnt - 1; i++)
			{
				//				printf("%s\r\n",CMD_Buffer);
			}
		}
	}
}
