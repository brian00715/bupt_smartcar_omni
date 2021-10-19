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

#define UART2_RX_BUFFER_SIZE (50)
#define UART2_TX_BUFFER_SIZE (100)
#define UART3_RX_BUFFER_SIZE (30)

extern uint8_t UART2_RxBuffer[UART2_RX_BUFFER_SIZE];
extern uint8_t UART2_RxArray[UART2_RX_BUFFER_SIZE];
extern uint8_t UART2_TxBuffer[UART2_TX_BUFFER_SIZE];
extern uint8_t UART2_RxBufferCnt;
extern uint8_t UART2_RxComplete;
extern uint8_t UART2_RxIDLEFlag;
extern uint8_t UART2_RxBufferOverflow; // �������������־

extern uint8_t UART3_RxBuffer[UART3_RX_BUFFER_SIZE];
extern uint8_t UART3_RxBufferCnt;
extern uint8_t UART3_RxOK;
extern uint8_t UART3_RxLen;

extern char TIM1_10ms_Flag;
extern char TIM1_20ms_Flag;
extern char TIM1_100ms_Flag;
extern uint8 TIM1_500ms_Flag;

extern int32 time_count;

#endif
