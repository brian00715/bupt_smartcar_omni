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

rt_sem_t camera_sem;

int
main (void)
{
//  display_init ();
  ips114_init ();
//    camera_sem = rt_sem_create("camera", 0, RT_IPC_FLAG_FIFO);
//    mt9v03x_init();
//    icm20602_init_spi();
//    encoder_init();
//    buzzer_init();
  //    elec_init();
  button_init ();
  Motor_Init ();
//  timer_pit_init ();
  ips114_clear (WHITE);
  ips114_showstr (IPS114_X_MAX / 2, IPS114_Y_MAX / 2, "Init Done");

  while (1)
    {
      ips114_showstr (0, 0, "forward rotate");
//      Motor_SetDuty (500, 500, 500, 500);
//      systick_delay_ms(2000);
//      Motor_SetDuty (0, 0, 0, 0);
//      systick_delay_ms(2000);
//      ips114_showstr (0, 0, "reverse rotate");
//      Motor_SetDuty (-500, -500, -500, -500);
//      systick_delay_ms(2000);
      //�ȴ�����ͷ�ɼ����
//        rt_sem_take(camera_sem, RT_WAITING_FOREVER);
//        rt_thread_mdelay(10);
      //��ʼ��������ͷͼ��

      gpio_set (MOTOR1_A, 1);
      pwm_duty (MOTOR1_B, 2000);
      gpio_set (MOTOR2_A, 1);
      pwm_duty (MOTOR2_B, 2000);
      gpio_set (MOTOR3_A, 1);
      pwm_duty (MOTOR3_B, 2000);
      gpio_set (MOTOR4_A, 1);
      pwm_duty (MOTOR4_B, 2000);
      systick_delay_ms(2000);
      gpio_set (MOTOR1_A, 0);
      pwm_duty (MOTOR1_B, 3000);
      gpio_set (MOTOR2_A, 0);
      pwm_duty (MOTOR2_B, 3000);
      gpio_set (MOTOR3_A, 0);
      pwm_duty (MOTOR3_B, 3000);
      gpio_set (MOTOR4_A, 0);
      pwm_duty (MOTOR4_B, 3000);
      systick_delay_ms(2000);
    }
}
