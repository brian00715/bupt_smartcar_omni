/*
 * master_comm.h
 *
 *  Created on: Jan 10, 2022
 *      Author: smkk7
 */

#ifndef CODE_MASTER_COMM_H_
#define CODE_MASTER_COMM_H_

typedef union UARTMsg_u
{
    float fl;
    uint8 ui[4];
    int in[2];
}UARTMsg_u;

extern float wheel_rpm[4];

#define UNUSED(x) (void)(x) // 用于避免GCC变量不使用的警告
void MasterComm_UARTCallback();
void UART_DMA_ReceiveInit(USART_TypeDef *usart, DMA_Channel_TypeDef *dma_ch,
        uint32 src_addr, uint32 des_addr, uint32 size);
void float2buffer(float src,uint8* dst);
void int2buffer(int src,uint8* dst);
float buffer2float(uint8* src);
char *substring(char *dst, char *src, int start, int len);

#endif /* CODE_MASTER_COMM_H_ */
