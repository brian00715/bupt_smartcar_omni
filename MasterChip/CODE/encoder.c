#include "encoder.h"

int16 encoder_data[4]={0};
int16 encoder_max=0;
int16 encoder_min=0;
int16 encoder_coff[4]={0}; // �ϵ��ķ���ϵ��

void Encoder_GetSpeed(void)
{
//    encoder_data[0] = encoder1_speed_spi(ABS_ENCODER_SPI_PC1_PIN);          //��ȡ������1������
//    //encoder_data[1] = encoder2_speed_spi(ABS_ENCODER_SPI_PC2_PIN);          //��ȡ������2������
//    //encoder_data[2] = encoder3_speed_spi(ABS_ENCODER_SPI_PC3_PIN);          //��ȡ������3������
//    //encoder_data[3] = encoder4_speed_spi(ABS_ENCODER_SPI_PC4_PIN);          //��ȡ������4������
}

void Encoder_Init(void)
{
	timer_quad_init(TIMER_2, TIMER2_CHA_A15, TIMER2_CHB_B3);
	timer_quad_init(TIMER_3, TIMER3_CHA_B4, TIMER3_CHB_B5);
}

