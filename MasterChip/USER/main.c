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
#include "mecanum_chassis.h"

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
	button_init();
	MecanumChassis_Init(); // ���̳�ʼ��
	icm20602_init_spi(); // ICM20602Ӳ��SPI��ʼ��

	// ����ͷ����˿�
	//	pwm_init(PWM1_CH2_A9, 200, 0);
	timer_pit_interrupt_ms(TIMER_1, 5);							   //200HZ, ����һ�е�Ч
	TIM_ITConfig((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update, ENABLE); //ʹ��TIM�ж�,��������ж�
	TIM_ClearITPendingBit((TIM_TypeDef *)TIM1_BASE, TIM_IT_Update);
	nvic_init(TIM1_UP_IRQn, 1, 2, ENABLE);

	EnableGlobalIRQ(0);
	uprintf("\r\n==Init Done==\r\n");

	while (1)
	{
		//		duty = (duty + 1000) % 10000;
		//		Motor_SetDuty(MecanumChassis.motor[0].target_duty,
		//				MecanumChassis.motor[1].target_duty,
		//				MecanumChassis.motor[2].target_duty,
		//				MecanumChassis.motor[3].target_duty);
		//        systick_delay_ms(1000);

		//		Motor_SetDuty(MecanumChassis.motor[0].target_duty,
		//				MecanumChassis.motor[1].target_duty,
		//				MecanumChassis.motor[2].target_duty,
		//				MecanumChassis.motor[3].target_duty);
		//		systick_delay_ms(1000);

		//		uprintf("icm_gyro|x=%5d ,y=%5d ,z=%5d \r\n",
		//				(int16) (icm_gyro_x ),
		//				(int16) (icm_gyro_y ),
		//				(int16) (icm_gyro_z));
		//		uprintf("icm_acc_x = %d\r\n", icm_acc_x);
		//		uprintf("icm_acc_y = %d\r\n", icm_acc_y);
		//		uprintf("icm_acc_z = %d\r\n", icm_acc_z);

		//		uprintf("yaw:%d\r\n",(int16)MecanumChassis.posture_status.yaw %360);

		systick_delay_ms(50); //��ʱ
		MecanumChassis_Exe();
	}
}
