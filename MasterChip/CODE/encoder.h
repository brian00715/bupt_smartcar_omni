#ifndef _encoder_h
#define _encoder_h

#include "headfile.h"

#define ENCODER1_TIM		TIM_1
#define ENCODER1_A			TIM_1_ENC1_A08
#define ENCODER1_B			TIM_1_ENC2_A01

#define ENCODER2_TIM		TIM_8
#define ENCODER2_A			TIM_8_ENC1_C00
#define ENCODER2_B			TIM_8_ENC2_C01

extern uint16 encoder_data[4];

void Encoder_Init(void);
void Encoder_GetSpeed(void);
#endif
