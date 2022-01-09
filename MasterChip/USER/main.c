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

#include "headfile.h"
#include "encoder.h"
#include "button.h"
#include "motor.h"
#include "cmd.h"
#include "isr.h"
#include "mecanum_chassis.h"
#include "slave_comm.h"
#include "zf_pit.h"
#include "rpi_comm.h"


int main(void)
{
	DisableGlobalIRQ();
	// ---------------------------------------��ʼ��---------------------------------------
	board_init();
	systick_delay_ms(300); //��ʱ300ms���ȴ��������������ϵ�ɹ�
	button_init();		   // ������ʼ��
	CMD_Init();			   // ����CMD��ʼ��
	SlaveComm_Init();	   // �ӻ����ڳ�ʼ��
	RPiComm_Init();
	Encoder_Init();		   // ��������ʼ��
	MecanumChassis_Init(); // ���̳�ʼ��
	icm20602_init_spi();
						   //	PathFollowing_Init();

	// Timer1��ʼ��
	// pwm_init(PWM1_CH2_A9, 200, 2000); // ����ͷ�����ʼ��
	timer_pit_interrupt_ms(TIMER_1, 5);
	TIM_ITConfig((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update, ENABLE); //ʹ��TIM�ж�,��������ж�
	TIM_ClearITPendingBit((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update);
	nvic_init(TIM1_UP_IRQn, 0, 2, ENABLE); // ��ʱ�ж�������ȼ�
	// ------------------------------------------------------------------------------------

	EnableGlobalIRQ(0);
	uprintf("\r\n==Init Done==\r\n");
	MecanumChassis.ctrl_mode = CTRL_MODE_NONE;
	MecanumChassis.pos_mode = POS_MODE_ABSOLUTE;

	while (1)
	{
		CMD_Exe();
		SlaveComm_Exe();
		MecanumChassis_Exe();
		RPiComm_Exe();

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
			}
		}
		if (gpio_get(KEY3) == RESET)
		{
			systick_delay_ms(80);
			if (gpio_get(KEY3) == RESET)
			{
			}
		}

        systick_delay_ms(10);
	}
}
