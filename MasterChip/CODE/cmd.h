#ifndef CMD_H_
#define CMD_H_

#include "isr.h"

#define USART1_RX_DMA_CH DMA1_Channel5
#define USART1_RX_DMA_CH_IRQN DMA1_Channel5_IRQn

#define CMD_SIZE_X 10  // 一条指令最多10个参数
#define CMD_SIZE_Y 100 // 每个参数最多100个字符

extern uint8_t *CMD_Buffer[CMD_SIZE_X]; // 指针数组，每个元素都指向分割后的元字符串
extern uint8_t CMD_BufferCnt;
extern uint8_t UART1_RxBuffer[RX_BUFFER_SIZE];
extern uint8_t UART1_RxComplete;
extern uint8_t UART1_RxIDLEFlag;
extern char CMD_RxOK;

void CMD_Init(void);
void CMD_Exe(void);
void UART_DMA_Init(DMA_Channel_TypeDef *dma_ch, uint32 src_addr,
				   uint32 des_addr, uint32 size);
void CMD_UARTCallback(void);
int CMD_CommandParse(char *cmd_line, uint8_t *argc, char *argv[]);
int CMD_CommandExe(int argc, char **argv);
void ToolBox_Scope(float *dataArray, int dataNum);

#endif
