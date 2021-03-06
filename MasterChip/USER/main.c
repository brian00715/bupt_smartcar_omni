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
#include "valuepack.h"
#include "slave_comm.h"
#include "path_following.h"
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
	// uart_init(UART_2, 115200, UART2_TX_A2, UART2_RX_A3); // 虚拟示波器串口
	Encoder_Init();		   // 编码器初始化
	icm20602_init_spi();   // ICM20602硬件SPI初始化
	MecanumChassis_Init(); // 底盘初始化
	PathFollowing_Init();
	//	ips114_init();
	//    elec_init();

	// Timer1初始化
	pwm_init(PWM1_CH2_A9, 200, 2000); // 摄像头舵机初始化
	// timer_pit_interrupt_ms(TIMER_1, 5);							   //200HZ, 与上一行等效
	TIM_ITConfig((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update, ENABLE); //使能TIM中断,允许更新中断
	TIM_ClearITPendingBit((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update);
	nvic_init(TIM1_UP_IRQn, 0, 0, ENABLE); // 定时中断最高优先级
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
			pwm_duty(PWM1_CH2_A9, MecanumChassis.cam_servo_duty);
			// uprintf("gyro_z:%6.3f yaw:%6.3f gyro_y:%6.3f acc_z:%6.3f\r\n",
			// 		icm_gyro_z, MecanumChassis.PostureStatus.yaw, icm_gyro_y, icm_acc_z);
			//>>>观测车轮转速<<<
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
			// >>>观测陀螺仪数据<<<
			// txpack.floats[0] = icm_acc_raw_z;
			// txpack.floats[1] = icm_gyro_raw_z;
			// sendValuePack(&txpack);
		}

		if (TIM1_500ms_Flag)
		{
			//	uprintf("%d %d\r\n",MecanumChassis.motor[0].now_rpm,MecanumChassis.motor[1].now_rpm);
			//			uprintf("%5.2f %5.2f \r\n",MecanumChassis.target_speed, MecanumChassis.target_omega);
			// uprintf("tar_yaw:%6.3f now_yaw:%6.3f ctrl:%6.3f\r\n", CMD_TargetYaw, MecanumChassis.PostureStatus.yaw, MecanumChassis.target_omega);
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

		if (MecanumChassis.PathFollowing.begin)
		{
			if ((systick_getval_ms() - out_garage_time) >= OUT_GARAGE_DELAY_TIME_MS) // 车身已位于赛道中心
			{
				MecanumChassis.PathFollowing.state = PATH_FOLLOW_NORMAL;
				MecanumChassis.target_dir = 1.5708;
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
				MecanumChassis.PathFollowing.begin = (MecanumChassis.PathFollowing.begin + 1) % 2;
				uprintf("path follow change to %d\r\n", MecanumChassis.PathFollowing.begin);
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
					if (!UART3_RxOK)
					{
						uprintf("## UART3 didn't get any data! Check slave communication. ##\r\n");
					}
				}
			}
		}
		if (gpio_get(KEY3) == RESET)
		{
			systick_delay_ms(80);
		}
	}
}
