/*
 * master_comm.c
 *
 *  Created on: Jan 10, 2022
 *      Author: smkk7
 */
#include "isr.h"
#include "master_comm.h"

/**
 * @brief      串口DMA接收初始化
 * @param      dma_ch              DAM通道
 * @param      src_addr            源地址
 * @param      des_addr            目标地址
 * @param      size                数据长度
 * @sa                  uart_dma_init(DMA1_Channel5, GPIOA->ODR, GPIOC->ODR, 8);
 */
void UART_DMA_ReceiveInit(USART_TypeDef *usart, DMA_Channel_TypeDef *dma_ch,
        uint32 src_addr, uint32 des_addr, uint32 size)
{
    USART_DMACmd(usart, USART_DMAReq_Rx, ENABLE); // 使能UART DMA接收
    DMA_InitTypeDef DMA_InitStructure;

    if (DMA1_Channel1 == dma_ch || DMA1_Channel2 == dma_ch
            || DMA1_Channel3 == dma_ch || DMA1_Channel4 == dma_ch ||
            DMA1_Channel5 == dma_ch || DMA1_Channel6 == dma_ch
            || DMA1_Channel7 == dma_ch)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //DMA1总线初始化
    }
    else if (DMA2_Channel1 == dma_ch || DMA2_Channel2 == dma_ch
            || DMA2_Channel3 == dma_ch || DMA2_Channel4 == dma_ch ||
            DMA2_Channel5 == dma_ch)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); //DMA2总线初始化
    }

    DMA_DeInit(dma_ch); // 复位

    //MDA配置初始化
    DMA_InitStructure.DMA_PeripheralBaseAddr = src_addr;                //源地址
    DMA_InitStructure.DMA_MemoryBaseAddr = des_addr;                    //目标地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  //外设作为源
    DMA_InitStructure.DMA_BufferSize = size;                        //传输多少个数据
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         //内存地址依次+1
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设每次传输一个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存每次传输一个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                     //循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;             //优先级最高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                    //非内存到内存模式
    DMA_Init(dma_ch, &DMA_InitStructure);

//  DMA_ITConfig(dma_ch, DMA_IT_TC, ENABLE);      // 开启DMA传输完成中断;其实开了没啥用，可以用阻塞查询方式
    DMA_Cmd(dma_ch, ENABLE); //开启DMA

}

float wheel_rpm[4]={0};
/**
 * @brief 主机通信串口中断回调函数
 *
 */
void MasterComm_UARTCallback()
{
    // 解析从机数据包
    if (!(UART3_RxBuffer[0] == 0x00 && UART3_RxBuffer[1] == 0xff))
    {
        return;
    }
    uint8 sub_buffer[5];
    substring(sub_buffer, UART3_RxBuffer, 2, 4);
    wheel_rpm[0]=buffer2float(sub_buffer);
    substring(sub_buffer, UART3_RxBuffer, 6, 4);
    wheel_rpm[1]=buffer2float(sub_buffer);
    substring(sub_buffer, UART3_RxBuffer, 10, 4);
    wheel_rpm[2]=buffer2float(sub_buffer);
    substring(sub_buffer, UART3_RxBuffer, 14, 4);
    wheel_rpm[3]=buffer2float(sub_buffer);

    memset(UART3_RxBuffer, 0, sizeof(uint8_t) * UART3_RX_BUFFER_SIZE);
    UART3_RxOK = 1;
}

/**
 * @brief float（32位）转为uint数组
 */
void float2buffer(float src,uint8* dst)
{
    UARTMsg_u tmp;
    tmp.fl = src;
    dst[0]=tmp.ui[0];
    dst[1]=tmp.ui[1];
    dst[2]=tmp.ui[2];
    dst[3]=tmp.ui[3];
}

/**
 * @brief uint数组转为float（32位）
 */
float buffer2float(uint8* src)
{
    UARTMsg_u tmp;
    tmp.ui[0]=src[0];
    tmp.ui[1]=src[1];
    tmp.ui[2]=src[2];
    tmp.ui[3]=src[3];
    return tmp.fl;
}


/**
 * @brief int（16位）转为uint数组
 */
void int2buffer(int src,uint8* dst)
{
    dst[0]=(src>>8)&0xff;
    dst[1]=(src)&0xff;
}

/**
 * @brief 截取字符串
 *
 * @param dst
 * @param src
 * @param start
 * @param len
 * @return char*
 */
char* substring(char* dst,char* src, int start, int len)
{
    char *p = dst;
    char *q = src;
//    int length = strlen(src);
//    if (start >= length || start < 0)
//        return NULL;
//    if (len > length)
//        len = length - start;
    q += start;
    while (len--)
    {
        *(p++) = *(q++);
    }
//    *(p++) = '\0';
    return dst;
}
