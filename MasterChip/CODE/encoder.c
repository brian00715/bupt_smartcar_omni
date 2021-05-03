#include "encoder.h"

int16 encoder_data[4]={0};
int16 encoder_max=0;
int16 encoder_min=0;
int16 encoder_coff[4]={0}; // 上电后的方向系数

void Encoder_GetSpeed(void)
{
//    encoder_data[0] = encoder1_speed_spi(ABS_ENCODER_SPI_PC1_PIN);          //获取编码器1的数据
//    //encoder_data[1] = encoder2_speed_spi(ABS_ENCODER_SPI_PC2_PIN);          //获取编码器2的数据
//    //encoder_data[2] = encoder3_speed_spi(ABS_ENCODER_SPI_PC3_PIN);          //获取编码器3的数据
//    //encoder_data[3] = encoder4_speed_spi(ABS_ENCODER_SPI_PC4_PIN);          //获取编码器4的数据
}

void Encoder_Init(void)
{
	timer_quad_init(TIMER_2, TIMER2_CHA_A15, TIMER2_CHB_B3);
	timer_quad_init(TIMER_3, TIMER3_CHA_B4, TIMER3_CHB_B5);
}

