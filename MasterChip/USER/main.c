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
	// ---------------------------------------初始化---------------------------------------
	board_init();
	systick_delay_ms(300); //延时300ms，等待主板其他外设上电成功
	button_init();
	CMD_Init();											 // 串口CMD初始化
	SlaveComm_Init();									 // 从机串口初始化
	uart_init(UART_2, 115200, UART2_TX_A2, UART2_RX_A3); // 虚拟示波器串口
	Encoder_Init();										 // 编码器初始化
														 //	icm20602_init_spi();   // ICM20602硬件SPI初始化
	MecanumChassis_Init();								 // 底盘初始化
	//	ips114_init();
	//    elec_init();

	// 摄像头舵机端口
	//	pwm_init(PWM1_CH2_A9, 200, 0);
	timer_pit_interrupt_ms(TIMER_1, 5);							   //200HZ, 与上一行等效
	TIM_ITConfig((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update, ENABLE); //使能TIM中断,允许更新中断
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
