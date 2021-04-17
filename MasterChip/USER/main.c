/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            main
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ790875685)
 * @version         查看doc内version文件 版本说明
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
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
      //等待摄像头采集完毕
//        rt_sem_take(camera_sem, RT_WAITING_FOREVER);
//        rt_thread_mdelay(10);
      //开始处理摄像头图像

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
