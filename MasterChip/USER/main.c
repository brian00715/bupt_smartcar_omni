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
#include "encoder.h"
#include "button.h"
#include "motor.h"
#include "cmd.h"
#include "isr.h"
#include "mecanum_chassis.h"
#include "slave_comm.h"
#include "zf_pit.h"
//#include "path_following.h"
#include "handle.h"
static uint32 out_garage_time;
int main(void)
{
	DisableGlobalIRQ();
	// ---------------------------------------��ʼ��---------------------------------------
	board_init();
	systick_delay_ms(300); //��ʱ300ms���ȴ��������������ϵ�ɹ�
	button_init();		   // ������ʼ��
	CMD_Init();			   // ����CMD��ʼ��
	SlaveComm_Init();	   // �ӻ����ڳ�ʼ��
	Handle_Init();		   // �ֱ���ʼ��
	Encoder_Init();		   // ��������ʼ��
	icm20602_init_spi();   // ICM20602Ӳ��SPI��ʼ��
	MecanumChassis_Init(); // ���̳�ʼ��
						   //	PathFollowing_Init();

	// Timer1��ʼ��
	// pwm_init(PWM1_CH2_A9, 200, 2000); // ����ͷ�����ʼ��
	timer_pit_interrupt_ms(TIMER_1, 5);
	TIM_ITConfig((TIM_TypeDef *) TIM1_BASE, TIM_IT_Update, ENABLE); //ʹ��TIM�ж�,��������ж�
	TIM_ClearITPendingBit((TIM_TypeDef *) TIM1_BASE, TIM_IT_Update);
	nvic_init(TIM1_UP_IRQn, 0, 2, ENABLE); // ��ʱ�ж�������ȼ�
	// ------------------------------------------------------------------------------------

	EnableGlobalIRQ(0);
	uprintf("\r\n==Init Done==\r\n");
	MecanumChassis.ctrl_mode = CTRL_MODE_HANDLE;
	MecanumChassis.pos_mode = POS_MODE_ABSOLUTE;

	while (1)
	{
		CMD_Exe();
		SlaveComm_Exe();
		Handle_Exe();
		//		PathFollowing_Exe();
		MecanumChassis_Exe();


		if (MecanumChassis.PathFollowing.begin)
		{
			if ((systick_getval_ms() - out_garage_time)
					>= OUT_GARAGE_DELAY_TIME_MS) // ������λ����������
			{
				MecanumChassis.PathFollowing.state = PATH_FOLLOW_NORMAL;
				MecanumChassis.target_dir = 1.5708;
			}
		}

		if (gpio_get(KEY1) == RESET)
		{
			systick_delay_ms(80);
			if (gpio_get(KEY1) == RESET)
			{
				uprintf("key1 pressed!\r\n");
				Motor_SelfCheck(); // ����Լ죬У׼������
			}
		}
		if (gpio_get(KEY2) == RESET)
		{
			systick_delay_ms(80);
			if (gpio_get(KEY2) == RESET)
			{
				uprintf("key2 pressed!\r\n");
				systick_start();
				out_garage_time = systick_getval_ms();
				MecanumChassis.PathFollowing.out_garage_shift = 1;
				MecanumChassis.PathFollowing.begin =
						(MecanumChassis.PathFollowing.begin + 1) % 2;
				uprintf("path follow change to %d\r\n",
						MecanumChassis.PathFollowing.begin);
				if (MecanumChassis.PathFollowing.begin && UART3_RxOK) // ��ʼѲ��״̬
				{
					MecanumChassis.ctrl_mode = CTRL_MODE_OMNI;
					// MecanumChassis.PathFollowing.state = PATH_FOLLOW_NORMAL;
					// �������
					MecanumChassis.target_speed = 0.2;
					MecanumChassis.target_dir = 3.14; // ����3.14������0
					MecanumChassis.target_omega = 0;
				}
				else
				{
					MecanumChassis.ctrl_mode = CTRL_MODE_CMD;
					MecanumChassis.target_speed = 0;
					MecanumChassis.target_omega = 0;
					// if (!UART3_RxOK)
					// {
					// 	uprintf("## UART3 didn't get any data! Check slave communication. ##\r\n");
					// }
				}
			}
		}
		if (gpio_get(KEY3) == RESET)
		{
			systick_delay_ms(80);
		}
	}
}
