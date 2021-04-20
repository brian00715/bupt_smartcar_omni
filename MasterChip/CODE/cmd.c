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
//  @brief      CMD��ʼ��
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void cmd_init(void)
{
    uart_init(DEBUG_UART, DEBUG_UART_BAUD, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
    //   UART_DMA_Init(USART1_DMA_CH, (uint32)USART1_DR_ADDR, (uint32)&CMD_RX_Buffer, CMD_RX_BUFFER_SIZE); // USART1 DMA��ʼ��
    //   nvic_init(USART1_DMA_CH_IRQN, 0, 0, ENABLE); // ����DMA NVIC

    uart_rx_irq(DEBUG_UART, ENABLE); // ʹ�ܴ��ڽ����ж�
    EnableGlobalIRQ(0);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����DMA��ʼ��
//  @param      dma_ch              DAMͨ��
//  @param      src_addr            Դ��ַ
//  @param      des_addr            Ŀ���ַ
//  @param      size                ���ݳ���
//  @return     void
//  Sample usage:                   uart_dma_init(DMA1_Channel5, GPIOA->ODR, GPIOC->ODR, 8);
//-------------------------------------------------------------------------------------------------------------------
void UART_DMA_Init(DMA_Channel_TypeDef *dma_ch, uint32 src_addr, uint32 des_addr, uint32 size)
{
    DMA_InitTypeDef DMA_InitStructure;

    if (DMA1_Channel1 == dma_ch || DMA1_Channel2 == dma_ch || DMA1_Channel3 == dma_ch || DMA1_Channel4 == dma_ch ||
        DMA1_Channel5 == dma_ch || DMA1_Channel6 == dma_ch || DMA1_Channel7 == dma_ch)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //DMA1���߳�ʼ��
    }
    else if (DMA2_Channel1 == dma_ch || DMA2_Channel2 == dma_ch || DMA2_Channel3 == dma_ch || DMA2_Channel4 == dma_ch ||
             DMA2_Channel5 == dma_ch)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); //DMA2���߳�ʼ��
    }

    DMA_DeInit(dma_ch);

    //MDA���ó�ʼ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = src_addr;                    //Դ��ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = des_addr;                        //Ŀ���ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      //������ΪԴ
    DMA_InitStructure.DMA_BufferSize = size;                                //������ٸ�����
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ��ַ����+1
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //����ÿ�δ���һ���ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�ڴ�ÿ�δ���һ���ֽ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //��ѭ��ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 //���ȼ����
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            //���ڴ浽�ڴ�ģʽ
    DMA_Init(dma_ch, &DMA_InitStructure);

    DMA_ITConfig(dma_ch, DMA_IT_TC, ENABLE); //����DMA��������ж�
    DMA_Cmd(dma_ch, ENABLE);                 //����DMA1
}

/**
 * @brief UART�����жϻص�����
 * 
 */
void CMD_UARTCallback(void)
{
}
