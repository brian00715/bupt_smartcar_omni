#include "ch32v10x_dma.h"
#include "ch32v10x_rcc.h"
#include "ch32v10x_tim.h"
#include "ch32v10x.h"

#include "zf_gpio.h"
#include "zf_uart.h"
#include "board.h"
#include "zf_nvic.h"
#include "cmd.h"

uint32 CMD_RX_Buffer[CMD_RX_BUFFER_SIZE];

//-------------------------------------------------------------------------------------------------------------------
//  @brief      CMD初始化
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void cmd_init(void)
{
    uart_init(DEBUG_UART, DEBUG_UART_BAUD, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
    //   UART_DMA_Init(USART1_DMA_CH, (uint32)USART1_DR_ADDR, (uint32)&CMD_RX_Buffer, CMD_RX_BUFFER_SIZE); // USART1 DMA初始化
    //   nvic_init(USART1_DMA_CH_IRQN, 0, 0, ENABLE); // 配置DMA NVIC

    uart_rx_irq(DEBUG_UART, ENABLE); // 使能串口接收中断
    EnableGlobalIRQ(0);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      串口DMA初始化
//  @param      dma_ch              DAM通道
//  @param      src_addr            源地址
//  @param      des_addr            目标地址
//  @param      size                数据长度
//  @return     void
//  Sample usage:                   uart_dma_init(DMA1_Channel5, GPIOA->ODR, GPIOC->ODR, 8);
//-------------------------------------------------------------------------------------------------------------------
void UART_DMA_Init(DMA_Channel_TypeDef *dma_ch, uint32 src_addr, uint32 des_addr, uint32 size)
{
    DMA_InitTypeDef DMA_InitStructure;

    if (DMA1_Channel1 == dma_ch || DMA1_Channel2 == dma_ch || DMA1_Channel3 == dma_ch || DMA1_Channel4 == dma_ch ||
        DMA1_Channel5 == dma_ch || DMA1_Channel6 == dma_ch || DMA1_Channel7 == dma_ch)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //DMA1总线初始化
    }
    else if (DMA2_Channel1 == dma_ch || DMA2_Channel2 == dma_ch || DMA2_Channel3 == dma_ch || DMA2_Channel4 == dma_ch ||
             DMA2_Channel5 == dma_ch)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); //DMA2总线初始化
    }

    DMA_DeInit(dma_ch);

    //MDA配置初始化
    DMA_InitStructure.DMA_PeripheralBaseAddr = src_addr;                    //源地址
    DMA_InitStructure.DMA_MemoryBaseAddr = des_addr;                        //目标地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      //外设作为源
    DMA_InitStructure.DMA_BufferSize = size;                                //传输多少个数据
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存地址依次+1
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设每次传输一个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //内存每次传输一个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //非循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 //优先级最高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            //非内存到内存模式
    DMA_Init(dma_ch, &DMA_InitStructure);

    DMA_ITConfig(dma_ch, DMA_IT_TC, ENABLE); //配置DMA传输完成中断
    DMA_Cmd(dma_ch, ENABLE);                 //开启DMA1
}

/**
 * @brief UART接收中断回调函数
 * 
 */
void CMD_UARTCallback(void)
{
}
