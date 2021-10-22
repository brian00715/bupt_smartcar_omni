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
//整套推荐IO查看Projecct文件夹下的TXT文本
//打开新的工程或者工程移动了位置务必执行以下操作
//右键单击工程，选择刷新
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
	// ---------------------------------------初始化---------------------------------------
	board_init();
	systick_delay_ms(300); //延时300ms，等待主板其他外设上电成功
	button_init();		   // 按键初始化
	CMD_Init();			   // 串口CMD初始化
	SlaveComm_Init();	   // 从机串口初始化
	Handle_Init();		   // 手柄初始化
	Encoder_Init();		   // 编码器初始化
	icm20602_init_spi();   // ICM20602硬件SPI初始化
	MecanumChassis_Init(); // 底盘初始化
						   //	PathFollowing_Init();

	// Timer1初始化
	// pwm_init(PWM1_CH2_A9, 200, 2000); // 摄像头舵机初始化
	timer_pit_interrupt_ms(TIMER_1, 5);
	TIM_ITConfig((TIM_TypeDef *) TIM1_BASE, TIM_IT_Update, ENABLE); //使能TIM中断,允许更新中断
	TIM_ClearITPendingBit((TIM_TypeDef *) TIM1_BASE, TIM_IT_Update);
	nvic_init(TIM1_UP_IRQn, 0, 2, ENABLE); // 定时中断最高优先级
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
					>= OUT_GARAGE_DELAY_TIME_MS) // 车身已位于赛道中心
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
				Motor_SelfCheck(); // 电机自检，校准编码器
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
				if (MecanumChassis.PathFollowing.begin && UART3_RxOK) // 开始巡线状态
				{
					MecanumChassis.ctrl_mode = CTRL_MODE_OMNI;
					// MecanumChassis.PathFollowing.state = PATH_FOLLOW_NORMAL;
					// 出库横移
					MecanumChassis.target_speed = 0.2;
					MecanumChassis.target_dir = 3.14; // 左移3.14，右移0
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
