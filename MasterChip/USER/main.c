/**
 * @file main.c
 * @author simon
 * @brief 
 * @version 0.1
 * @date 2021-05-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
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
#include "mecanum_chassis.h"
#include "valuepack.h"
#include "slave_comm.h"
#include "path_following.h"

int main(void)
{
	DisableGlobalIRQ();
	// ---------------------------------------��ʼ��---------------------------------------
	board_init();
	systick_delay_ms(300); //��ʱ300ms���ȴ��������������ϵ�ɹ�
	button_init();		   // ������ʼ��
	CMD_Init();			   // ����CMD��ʼ��
	SlaveComm_Init();	   // �ӻ����ڳ�ʼ��
	// uart_init(UART_2, 115200, UART2_TX_A2, UART2_RX_A3); // ����ʾ��������
	Encoder_Init();		   // ��������ʼ��
	icm20602_init_spi();   // ICM20602Ӳ��SPI��ʼ��
	MecanumChassis_Init(); // ���̳�ʼ��
	PathFollowing_Init();
	//	ips114_init();
	//    elec_init();

	// Timer1��ʼ��
	pwm_init(PWM1_CH2_A9, 200, 2000); // ����ͷ�����ʼ��
	// timer_pit_interrupt_ms(TIMER_1, 5);							   //200HZ, ����һ�е�Ч
	TIM_ITConfig((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update, ENABLE); //ʹ��TIM�ж�,��������ж�
	TIM_ClearITPendingBit((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update);
	nvic_init(TIM1_UP_IRQn, 0, 0, ENABLE); // ��ʱ�ж�������ȼ�
	// ------------------------------------------------------------------------------------

	EnableGlobalIRQ(0);
	uprintf("\r\n==Init Done==\r\n");

	while (1)
	{
		CMD_Exe();
		SlaveComm_Exe();
		PathFollowing_Exe();
		MecanumChassis_Exe();

		extern int wave_index;
		if (TIM1_10ms_Flag)
		{
			//>>>�۲⳵��ת��<<<
			// switch (wave_index)
			// {
			// case -1:
			// 	break;
			// case 0:
			// 	txpack.floats[0] = MecanumChassis.motor[0].target_rpm * 1.0;
			// 	txpack.floats[1] = MecanumChassis.motor[0].now_rpm * 1.0;
			// 	sendValuePack(&txpack);
			// 	break;
			// case 1:
			// 	txpack.floats[0] = MecanumChassis.motor[1].target_rpm * 1.0;
			// 	txpack.floats[1] = MecanumChassis.motor[1].now_rpm * 1.0;
			// 	sendValuePack(&txpack);
			// 	break;
			// case 2:
			// 	txpack.floats[0] = MecanumChassis.motor[2].target_rpm * 1.0;
			// 	txpack.floats[1] = MecanumChassis.motor[2].now_rpm * 1.0;
			// 	sendValuePack(&txpack);
			// 	break;
			// case 3:
			// 	txpack.floats[0] = MecanumChassis.motor[3].target_rpm * 1.0;
			// 	txpack.floats[1] = MecanumChassis.motor[3].now_rpm * 1.0;
			// 	sendValuePack(&txpack);
			// 	break;
			// default:
			// 	break;
			// }
			// >>>�۲�����������<<<
			// txpack.floats[0] = icm_acc_raw_z;
			// txpack.floats[1] = icm_gyro_raw_z;
			// sendValuePack(&txpack);
		}

		if (TIM1_100ms_Flag)
		{
			//	uprintf("%d %d\r\n",MecanumChassis.motor[0].now_rpm,MecanumChassis.motor[1].now_rpm);
			//			uprintf("%5.2f %5.2f \r\n",MecanumChassis.target_speed, MecanumChassis.target_omega);
		}

		if (gpio_get(KEY1) == RESET)
		{
			systick_delay_ms(80);
			if (gpio_get(KEY1) == RESET)
			{
				uprintf("key1 pressed!\r\n");
				Motor_SelfCheck();
			}
		}
		if (gpio_get(KEY2) == RESET) // ��ʼѭ��
		{
			systick_delay_ms(80);
			if (gpio_get(KEY2) == RESET)
			{
				uprintf("key2 pressed!\r\n");
				MecanumChassis.PathFollowing.begin = (MecanumChassis.PathFollowing.begin + 1) % 2;
				uprintf("path follow change to %d\r\n", MecanumChassis.PathFollowing.begin);
				if (MecanumChassis.PathFollowing.begin && UART3_RxOK) // ��ʼѲ��
				{
					MecanumChassis.ctrl_mode = CTRL_MODE_OMNI;
					MecanumChassis.PathFollowing.state = PATH_FOLLOW_NORMAL;
				}
				else
				{
					MecanumChassis.ctrl_mode = CTRL_MODE_CMD;
					MecanumChassis.target_speed = 0;
					MecanumChassis.target_omega = 0;
					if (!UART3_RxOK)
					{
						uprintf("## UART3 didn't get any data! Check slave communication. ##\r\n");
					}
				}
			}
		}
	}
}
