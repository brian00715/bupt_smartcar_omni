/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            isr
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ790875685)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
#ifndef __ISR_H
#define __ISR_H

#define RX_BUFFER_SIZE 100

extern uint8_t UART1_RxBuffer[RX_BUFFER_SIZE];
extern uint8_t UART1_RxBufferCnt;
extern uint8_t UART1_RxComplete;
extern uint8_t UART1_RxIDLEFlag;
extern uint8_t UART1_RxBufferOverflow; // �������������־

extern uint8_t UART3_RxBuffer[20];
extern uint8_t UART3_RxBufferCnt;
extern uint8_t UART3_RxOK;

extern char TIM1_10ms_Flag;
extern char TIM1_20ms_Flag;
extern char TIM1_100ms_Flag;

#endif
