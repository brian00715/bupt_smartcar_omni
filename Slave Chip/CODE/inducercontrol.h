/*
 * inducercontrol.h
 *
 *  Created on: 2021��4��13��
 *      Author: Z.yd
 */

#ifndef CODE_INDUCERCONTROL_H_
#define CODE_INDUCERCONTROL_H_
#include "headfile.h"

void InducerMax_Get(void);
void Inducer_Show_Oled(void);
void Inducer_Processing(void);


/*********************ȫ�ֱ���*************************/
extern uint8 InducerMax_Get_Start_Flag;             //��л�ȡ���ֵ��ʼ��־
extern uint8 InducerMax_Get_End_Flag;               //��л�ȡ���ֵ������־

#endif /* CODE_INDUCERCONTROL_H_ */
