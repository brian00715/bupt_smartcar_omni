#ifndef CMD_H_
#define CMD_H_

#define USART1_DMA_CH 		DMA1_Channel5
#define USART1_DMA_CH_IRQN  	DMA1_Channel5_IRQn
#define USART1_DR_ADDR		(0x40013804) // USART1Êý¾Ý¼Ä´æÆ÷

#define CMD_RX_BUFFER_SIZE	8
void cmd_init(void);
void UART_DMA_Init(DMA_Channel_TypeDef* dma_ch,uint32 src_addr, uint32 des_addr, uint32 size);

#endif
