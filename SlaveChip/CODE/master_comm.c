/*
 * master_comm.c
 *
 *  Created on: Jan 10, 2022
 *      Author: smkk7
 */
#include "isr.h"
#include "master_comm.h"

/**
 * @brief      ����DMA���ճ�ʼ��
 * @param      dma_ch              DAMͨ��
 * @param      src_addr            Դ��ַ
 * @param      des_addr            Ŀ���ַ
 * @param      size                ���ݳ���
 * @sa                  uart_dma_init(DMA1_Channel5, GPIOA->ODR, GPIOC->ODR, 8);
 */
void UART_DMA_ReceiveInit(USART_TypeDef *usart, DMA_Channel_TypeDef *dma_ch,
        uint32 src_addr, uint32 des_addr, uint32 size)
{
    USART_DMACmd(usart, USART_DMAReq_Rx, ENABLE); // ʹ��UART DMA����
    DMA_InitTypeDef DMA_InitStructure;

    if (DMA1_Channel1 == dma_ch || DMA1_Channel2 == dma_ch
            || DMA1_Channel3 == dma_ch || DMA1_Channel4 == dma_ch ||
            DMA1_Channel5 == dma_ch || DMA1_Channel6 == dma_ch
            || DMA1_Channel7 == dma_ch)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //DMA1���߳�ʼ��
    }
    else if (DMA2_Channel1 == dma_ch || DMA2_Channel2 == dma_ch
            || DMA2_Channel3 == dma_ch || DMA2_Channel4 == dma_ch ||
            DMA2_Channel5 == dma_ch)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); //DMA2���߳�ʼ��
    }

    DMA_DeInit(dma_ch); // ��λ

    //MDA���ó�ʼ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = src_addr;                //Դ��ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = des_addr;                    //Ŀ���ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                  //������ΪԴ
    DMA_InitStructure.DMA_BufferSize = size;                        //������ٸ�����
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         //�ڴ��ַ����+1
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //����ÿ�δ���һ���ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //�ڴ�ÿ�δ���һ���ֽ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                     //ѭ��ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;             //���ȼ����
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                    //���ڴ浽�ڴ�ģʽ
    DMA_Init(dma_ch, &DMA_InitStructure);

//  DMA_ITConfig(dma_ch, DMA_IT_TC, ENABLE);      // ����DMA��������ж�;��ʵ����ûɶ�ã�������������ѯ��ʽ
    DMA_Cmd(dma_ch, ENABLE); //����DMA

}

float wheel_rpm[4]={0};
/**
 * @brief ����ͨ�Ŵ����жϻص�����
 *
 */
void MasterComm_UARTCallback()
{
    // �����ӻ����ݰ�
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
 * @brief float��32λ��תΪuint����
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
 * @brief uint����תΪfloat��32λ��
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
 * @brief int��16λ��תΪuint����
 */
void int2buffer(int src,uint8* dst)
{
    dst[0]=(src>>8)&0xff;
    dst[1]=(src)&0xff;
}

/**
 * @brief ��ȡ�ַ���
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
