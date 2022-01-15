/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            isr
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ790875685)
 * @version         查看doc内version文件 版本说明
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
#ifndef __ISR_H
#define __ISR_H

#include "headfile.h"

#define UART3_RX_BUFFER_SIZE (30)

extern int32 time_count;
extern uint8_t UART3_RxBuffer[UART3_RX_BUFFER_SIZE];
extern uint8_t UART3_RxBufferCnt;
extern uint8_t UART3_RxOK;
extern uint8_t UART3_RxLen;

extern uint8_t SlaveComm_MotorSelfCheck;

#endif


