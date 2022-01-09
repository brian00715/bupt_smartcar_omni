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
	// ---------------------------------------初始化---------------------------------------
	board_init();
	systick_delay_ms(300); //延时300ms，等待主板其他外设上电成功
	button_init();		   // 按键初始化
	CMD_Init();			   // 串口CMD初始化
	SlaveComm_Init();	   // 从机串口初始化
	RPiComm_Init();
	Encoder_Init();		   // 编码器初始化
	MecanumChassis_Init(); // 底盘初始化
	icm20602_init_spi();
						   //	PathFollowing_Init();

	// Timer1初始化
	// pwm_init(PWM1_CH2_A9, 200, 2000); // 摄像头舵机初始化
	timer_pit_interrupt_ms(TIMER_1, 5);
	TIM_ITConfig((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update, ENABLE); //使能TIM中断,允许更新中断
	TIM_ClearITPendingBit((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update);
	nvic_init(TIM1_UP_IRQn, 0, 2, ENABLE); // 定时中断最高优先级
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
				Motor_SelfCheck(); // 电机自检，校准编码器
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
