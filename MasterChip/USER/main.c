/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            main
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ790875685)
 * @version         查看doc内version文件 版本说明
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
//整套推荐IO查看Projecct文件夹下的TXT文本
//打开新的工程或者工程移动了位置务必执行以下操作
//右键单击工程，选择刷新
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
