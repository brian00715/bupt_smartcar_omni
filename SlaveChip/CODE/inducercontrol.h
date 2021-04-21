/*
 * inducercontrol.h
 *
 *  Created on: 2021年4月13日
 *      Author: Z.yd
 */

#ifndef CODE_INDUCERCONTROL_H_
#define CODE_INDUCERCONTROL_H_
#include "headfile.h"

void InducerMax_Get(void);
void Inducer_Show_Oled(void);
void Inducer_Processing(void);


/*********************全局变量*************************/
extern uint8 InducerMax_Get_Start_Flag;             //电感获取最大值开始标志
extern uint8 InducerMax_Get_End_Flag;               //电感获取最大值结束标志

#endif /* CODE_INDUCERCONTROL_H_ */
