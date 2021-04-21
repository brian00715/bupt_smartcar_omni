#ifndef CMD_H_
#define CMD_H_

#define USART1_RX_DMA_CH DMA1_Channel5
#define USART1_RX_DMA_CH_IRQN DMA1_Channel5_IRQn
#define USART1_DR_ADDR (0x40013804) // USART1数据寄存器

#define CMD_SIZE_X 10  // 一条指令最多10个参数
#define CMD_SIZE_Y 100 // 每个参数最多100个字符

extern uint8_t *CMD_Buffer[CMD_SIZE_X]; // 指针数组，每个元素都指向分割后的元字符串
extern uint8_t CMD_BufferCnt;

void CMD_Init(void);
void UART_DMA_Init(DMA_Channel_TypeDef *dma_ch, uint32 src_addr,
				   uint32 des_addr, uint32 size);
void CMD_UARTCallback(void);
int CMD_Parse(char *cmd_line, uint8_t *argc, char *argv[]);
int CMD_Exe(int argc, char **argv);

#endif
