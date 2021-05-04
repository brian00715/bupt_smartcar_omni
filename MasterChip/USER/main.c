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
	button_init();
	CMD_Init();											 // ����CMD��ʼ��
	SlaveComm_Init();									 // �ӻ����ڳ�ʼ��
	uart_init(UART_2, 115200, UART2_TX_A2, UART2_RX_A3); // ����ʾ��������
	Encoder_Init();										 // ��������ʼ��
														 //	icm20602_init_spi();   // ICM20602Ӳ��SPI��ʼ��
	MecanumChassis_Init();								 // ���̳�ʼ��
	//	ips114_init();
	//    elec_init();

	// ����ͷ����˿�
	//	pwm_init(PWM1_CH2_A9, 200, 0);
	timer_pit_interrupt_ms(TIMER_1, 5);							   //200HZ, ����һ�е�Ч
	TIM_ITConfig((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update, ENABLE); //ʹ��TIM�ж�,��������ж�
	TIM_ClearITPendingBit((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update);
	nvic_init(TIM1_UP_IRQn, 0, 1, ENABLE);
	// ------------------------------------------------------------------------------------

	EnableGlobalIRQ(0);
	uprintf("\r\n==Init Done==\r\n");

	while (1)
	{
		CMD_Exe();
		MecanumChassis_Exe();
		PathFollowing_Exe();

		//		uprintf("icm_gyro|x=%5d ,y=%5d ,z=%5d \r\n",
		//				(int16) (icm_gyro_x ),
		//				(int16) (icm_gyro_y ),
		//				(int16) (icm_gyro_z));
		//		uprintf("icm_acc_x = %d\r\n", icm_acc_x);
		//		uprintf("icm_acc_y = %d\r\n", icm_acc_y);
		//		uprintf("icm_acc_z = %d\r\n", icm_acc_z);
		//		uprintf("yaw:%d\r\n",(int16)MecanumChassis.posture_status.yaw %360);

		extern int wave_index;
		if (TIM1_10ms_Flag)
		{
			switch (wave_index)
			{
			case -1:

				break;
			case 0:
				txpack.floats[0] = MecanumChassis.motor[0].target_rpm * 1.0;
				txpack.floats[1] = MecanumChassis.motor[0].now_rpm * 1.0;
				sendValuePack(&txpack);
				break;
			case 1:
				txpack.floats[0] = MecanumChassis.motor[1].target_rpm * 1.0;
				txpack.floats[1] = MecanumChassis.motor[1].now_rpm * 1.0;
				sendValuePack(&txpack);
				break;
			case 2:
				txpack.floats[0] = MecanumChassis.motor[2].target_rpm * 1.0;
				txpack.floats[1] = MecanumChassis.motor[2].now_rpm * 1.0;
				sendValuePack(&txpack);
				break;
			case 3:
				break;
			default:
				break;
			}
		}
		if (TIM1_100ms_Flag)
		{
			//			uprintf("%d %d\r\n",MecanumChassis.motor[0].now_rpm,MecanumChassis.motor[1].now_rpm);
		}

		if (gpio_get(KEY1) == RESET)
		{
			systick_delay_ms(50);
			if (gpio_get(KEY1) == RESET)
			{
				uprintf("key1 pressed!\r\n");
				Motor_SelfCheck();
			}
		}
	}
}
