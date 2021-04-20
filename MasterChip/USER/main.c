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

rt_sem_t camera_sem;

int
main (void)
{
  DisableGlobalIRQ ();

  //  display_init ();
  ips114_init ();
  cmd_init ();
  //    camera_sem = rt_sem_create("camera", 0, RT_IPC_FLAG_FIFO);
//      mt9v03x_init();
  //    icm20602_init_spi();
  //    encoder_init();
  //    buzzer_init();
  //    elec_init();
  button_init ();
  Motor_Init ();
  //  timer_pit_init ();
  ips114_clear (WHITE);
  static int16 duty = 0;

  EnableGlobalIRQ (0);
  while (1)
    {
      //�ȴ�����ͷ�ɼ����
      //        rt_sem_take(camera_sem, RT_WAITING_FOREVER);
      //        rt_thread_mdelay(10);
      //��ʼ��������ͷͼ��
//      duty = (duty + 1000) % 10000;
//      Motor_SetDuty (duty, duty, duty, duty);
      systick_delay_ms(1000);

      ips114_showstr (0, 0, "duty:");
      ips114_showint16 (0, 1, duty);

    }
}
